"""Render original cell artwork using verified public uArm meshes, NumPy and Pillow.

No simulator, browser, external executable or project binary is launched. These
are illustrative software renders, not Isaac Sim screenshots or physics results.
"""

from __future__ import annotations

import argparse
import base64
import hashlib
import io
import math
import sys
import xml.etree.ElementTree as ET
from pathlib import Path

import numpy as np
from PIL import Image, ImageDraw, ImageFilter, ImageFont

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
sys.path.insert(0, str(ROOT / "tools"))

import fetch_swiftpro_meshes as upstream  # noqa: E402
import build_scene  # noqa: E402
import make_readme_art as motion  # noqa: E402
from isaac_uarm_twin import kinematics  # noqa: E402

OUT = ROOT / "docs" / "assets"
GREEN = (0.46, 0.725, 0.02)
SILVER = (0.66, 0.69, 0.72)
GRAPHITE = (0.105, 0.12, 0.135)


def unit(vectors):
    vectors = np.asarray(vectors, dtype=float)
    return vectors / np.maximum(np.linalg.norm(vectors, axis=-1, keepdims=True), 1e-12)


def transform(position=(0, 0, 0), axis=(0, 0, 1), angle=0):
    axis = unit(axis)
    cosine, sine = math.cos(angle), math.sin(angle)
    cross = np.array(((0, -axis[2], axis[1]), (axis[2], 0, -axis[0]), (-axis[1], axis[0], 0)))
    matrix = np.eye(4)
    matrix[:3, :3] = cosine * np.eye(3) + (1 - cosine) * np.outer(axis, axis) + sine * cross
    matrix[:3, 3] = position
    return matrix


class Mesh:
    def __init__(self, vertices, faces, normals=None):
        self.vertices = np.asarray(vertices, dtype=float)
        self.faces = np.asarray(faces, dtype=np.int32)
        self.normals = normals

    def triangles(self, matrix):
        vertices = self.vertices @ matrix[:3, :3].T + matrix[:3, 3]
        triangles = vertices[self.faces]
        if self.normals is None:
            normals = unit(np.cross(triangles[:, 1] - triangles[:, 0], triangles[:, 2] - triangles[:, 0]))
            normals = np.repeat(normals[:, None, :], 3, axis=1)
        else:
            normals = (self.normals @ matrix[:3, :3].T)[self.faces]
        return triangles, normals


def load_stl(path, resolution=0.0006):
    data = path.read_bytes()
    if hashlib.sha256(data).hexdigest() != upstream.MESHES[path.name]:
        raise ValueError(f"Unverified public mesh: {path.name}")
    count = int.from_bytes(data[80:84], "little")
    dtype = np.dtype([("normal", "<f4", (3,)), ("points", "<f4", (3, 3)), ("attr", "<u2")])
    if len(data) != 84 + 50 * count:
        raise ValueError(f"Invalid binary STL: {path.name}")
    points = np.frombuffer(data, dtype, count, 84)["points"].reshape(-1, 3).astype(float)
    cells = np.round(points / resolution).astype(np.int64)
    _, inverse = np.unique(cells, axis=0, return_inverse=True)
    counts = np.bincount(inverse)
    vertices = np.column_stack([np.bincount(inverse, weights=points[:, axis]) / counts for axis in range(3)])
    faces = inverse.reshape(-1, 3)
    valid = (faces[:, 0] != faces[:, 1]) & (faces[:, 1] != faces[:, 2]) & (faces[:, 0] != faces[:, 2])
    faces = faces[valid]
    triangles = vertices[faces]
    face_normals = np.cross(triangles[:, 1] - triangles[:, 0], triangles[:, 2] - triangles[:, 0])
    normals = np.zeros_like(vertices)
    for corner in range(3):
        np.add.at(normals, faces[:, corner], face_normals)
    return Mesh(vertices, faces, unit(normals))


def box_mesh(size, radius=0.002, segments=4):
    half = np.asarray(size) / 2
    radius = min(radius, min(half) * 0.85)
    inner = half - radius
    vertices, normals, faces = [], [], []
    for axis in range(3):
        others = [index for index in range(3) if index != axis]
        tracks = []
        for other in others:
            edge = np.linspace(inner[other], half[other], segments + 1)
            middle = np.linspace(-inner[other], inner[other], max(2, math.ceil(2 * inner[other] / 0.04)))
            tracks.append(np.unique(np.concatenate((-edge[::-1], middle, edge))))
        for sign in (-1, 1):
            start = len(vertices)
            for across in tracks[0]:
                for along in tracks[1]:
                    point = np.zeros(3)
                    point[axis] = sign * half[axis]
                    point[others] = (across, along)
                    core = np.clip(point, -inner, inner)
                    normal = unit(point - core)
                    vertices.append(core + normal * radius)
                    normals.append(normal)
            stride = len(tracks[1])
            for row in range(len(tracks[0]) - 1):
                for column in range(stride - 1):
                    first = start + row * stride + column
                    for face in ((first, first + stride, first + 1), (first + 1, first + stride, first + stride + 1)):
                        face = list(face)
                        triangle = np.array([vertices[index] for index in face])
                        direction = np.cross(triangle[1] - triangle[0], triangle[2] - triangle[0])
                        if np.dot(direction, normals[first]) < 0:
                            face.reverse()
                        faces.append(face)
    return Mesh(vertices, faces, np.asarray(normals))


def cylinder_mesh(radius, length, segments=48):
    vertices, normals, faces = [], [], []
    for height in (-length / 2, length / 2):
        for index in range(segments):
            angle = index * 2 * math.pi / segments
            normal = (math.cos(angle), math.sin(angle), 0)
            vertices.append((radius * normal[0], radius * normal[1], height))
            normals.append(normal)
    for index in range(segments):
        following = (index + 1) % segments
        faces.extend(((index, following, index + segments), (following, following + segments, index + segments)))
    for sign in (-1, 1):
        start = len(vertices)
        vertices.append((0, 0, sign * length / 2))
        normals.append((0, 0, sign))
        for index in range(segments):
            angle = index * 2 * math.pi / segments
            vertices.append((radius * math.cos(angle), radius * math.sin(angle), sign * length / 2))
            normals.append((0, 0, sign))
        for index in range(segments):
            face = (start, start + 1 + index, start + 1 + (index + 1) % segments)
            faces.append(face if sign > 0 else face[::-1])
    return Mesh(vertices, faces, np.asarray(normals))


class Robot:
    def __init__(self, meshes, gripper=False):
        name = "swiftpro_twin_gripper" if gripper else "swiftpro_twin"
        self.root = ET.parse(ROOT / "assets" / "swiftpro" / f"{name}.urdf").getroot()
        self.meshes = meshes

    def matrices(self, base, yaw, joints):
        matrices = {"Base": transform(base, angle=math.radians(yaw))}
        pending = list(self.root.findall("joint"))
        while pending:
            for joint in pending[:]:
                parent = joint.find("parent").get("link")
                if parent not in matrices:
                    continue
                position = tuple(map(float, joint.find("origin").get("xyz").split()))
                axis = tuple(map(float, joint.find("axis").get("xyz").split()))
                angle = math.radians(joints.get(joint.get("name"), 0))
                matrices[joint.find("child").get("link")] = (
                    matrices[parent] @ transform(position) @ transform(axis=axis, angle=angle)
                )
                pending.remove(joint)
        return matrices

    def objects(self, base, yaw, joints):
        matrices = self.matrices(base, yaw, joints)
        objects = []
        rod_links = {"Link4", "Link5", "Link6", "Link7", "Link9"}
        for link in self.root.findall("link"):
            name = link.get("name")
            for visual in link.findall("visual"):
                origin = tuple(map(float, visual.find("origin").get("xyz").split()))
                matrix = matrices[name] @ transform(origin)
                source = visual.find("geometry/mesh")
                if source is not None:
                    mesh = self.meshes[Path(source.get("filename")).name]
                    if name in rod_links:
                        colour, metal = (0.78, 0.81, 0.84), 0.78
                    elif name == "Base":
                        colour, metal = (0.16, 0.18, 0.20), 0.35
                    elif name == "Link8":
                        colour, metal = (0.20, 0.22, 0.25), 0.50
                    else:
                        colour, metal = (0.25, 0.28, 0.31), 0.45
                else:
                    size = tuple(map(float, visual.find("geometry/box").get("size").split()))
                    colour = GREEN if origin[2] > -0.06 else (0.74, 0.77, 0.80)
                    mesh, metal = box_mesh(size, 0.001), 0.55
                objects.append((mesh, matrix, colour, metal))
        return objects


class Renderer:
    def __init__(self, width, height, eye, target, span, bg_top=34, bg_bottom=20):
        self.width, self.height = width, height
        self.bg_top, self.bg_bottom = bg_top, bg_bottom
        backward = unit(np.asarray(eye) - target)
        right = unit(np.cross((0, 0, 1), backward))
        upward = np.cross(backward, right)
        self.basis = np.array((right, upward, backward))
        self.target = np.array(target)
        self.scale = width / span
        self.light = unit((-0.4, -0.7, 1.4))
        self.fill = unit((0.7, 0.9, 0.8))
        self.half = unit(self.light + backward)

    def project(self, points):
        points = (np.asarray(points, dtype=float) - self.target) @ self.basis.T
        return np.stack((self.width / 2 + points[..., 0] * self.scale,
                         self.height / 2 - points[..., 1] * self.scale, points[..., 2]), axis=-1)

    def render(self, objects, shadows=True, depth_only=False):
        triangles, normals, materials, metals = [], [], [], []
        for mesh, matrix, colour, metal in objects:
            geometry, shading = mesh.triangles(matrix)
            triangles.append(geometry)
            normals.append(shading)
            materials.append(np.tile(colour, (len(geometry), 1)))
            metals.append(np.full(len(geometry), metal))
        triangles = np.concatenate(triangles)
        normals = np.concatenate(normals)
        colours = np.concatenate(materials)
        metals = np.concatenate(metals)
        projected = self.project(triangles)
        face = np.cross(triangles[:, 1] - triangles[:, 0], triangles[:, 2] - triangles[:, 0])
        facing = (face @ self.basis[2]) > 0
        diffuse = np.maximum(normals @ self.light, 0) * 0.72 + np.maximum(normals @ self.fill, 0) * 0.32
        ambient = 0.24 + 0.12 * (normals[..., 2] + 1) / 2
        specular = np.maximum(normals @ self.half, 0) ** 48 * (0.15 + metals[:, None] * 0.4)
        linear = np.maximum(colours, 0) ** 2.2
        shaded = np.clip(linear[:, None, :] * (ambient + diffuse)[..., None] + specular[..., None], 0, 1) ** (1 / 2.2)
        background = np.linspace(self.bg_top, self.bg_bottom, self.height)[:, None, None]
        pixels = np.broadcast_to(background, (self.height, self.width, 3)).copy().astype(np.uint8)
        depth = np.full((self.height, self.width), -np.inf, dtype=np.float32)
        self.rasterize(projected, shaded, facing, pixels, depth, depth_only)
        self.depth = depth
        if depth_only:
            return None
        if shadows:
            span = self.width / self.scale * 1.6
            lamp = Renderer(1200, 1200, self.target + self.light * 5, self.target, span)
            lamp.render(objects, shadows=False, depth_only=True)
            valid = np.isfinite(depth)
            rows, columns = np.nonzero(valid)
            camera_points = np.column_stack(((columns + 0.5 - self.width / 2) / self.scale,
                                              (self.height / 2 - rows - 0.5) / self.scale, depth[valid]))
            world = camera_points @ self.basis + self.target
            projected_shadow = lamp.project(world)
            coordinates = np.rint(projected_shadow[:, :2]).astype(int)
            inside = np.all((coordinates >= 0) & (coordinates < 1200), axis=1)
            darkness = np.zeros(len(rows), dtype=np.uint8)
            sample = coordinates[inside]
            blocked = lamp.depth[sample[:, 1], sample[:, 0]] > projected_shadow[inside, 2] + 0.004
            darkness[inside] = blocked * 100
            mask = np.zeros_like(depth, dtype=np.uint8)
            mask[rows, columns] = darkness
            soft = np.asarray(Image.fromarray(mask).filter(ImageFilter.GaussianBlur(self.width / 1000))) / 255
            pixels = np.clip(pixels * (1 - soft[..., None]), 0, 255).astype(np.uint8)
        return Image.fromarray(pixels)

    def rasterize(self, projected, shaded, facing, pixels, depth, depth_only):
        low = np.maximum(np.floor(projected[:, :, :2].min(axis=1)).astype(int), 0)
        high = np.minimum(np.ceil(projected[:, :, :2].max(axis=1)).astype(int), (self.width - 1, self.height - 1))
        first, second, third = (projected[:, corner] for corner in range(3))
        divisor = (second[:, 1] - third[:, 1]) * (first[:, 0] - third[:, 0]) + (third[:, 0] - second[:, 0]) * (first[:, 1] - third[:, 1])
        valid = facing & np.all(high >= low, axis=1) & (np.abs(divisor) > 1e-6)
        exponents = np.ceil(np.log2(np.maximum(high - low + 1, 1))).astype(int)
        buckets = exponents[:, 0] * 32 + exponents[:, 1]
        flat_depth, flat_pixels = depth.reshape(-1), pixels.reshape(-1, 3)
        for bucket in np.unique(buckets[valid]):
            selected = np.flatnonzero(valid & (buckets == bucket))
            columns, rows = 2 ** (int(bucket) // 32), 2 ** (int(bucket) % 32)
            batch_size = max(1, min(512, 700000 // (rows * columns)))
            for offset in range(0, len(selected), batch_size):
                indices = selected[offset:offset + batch_size]
                grid_x = low[indices, 0, None, None] + np.arange(columns)[None, None, :]
                grid_y = low[indices, 1, None, None] + np.arange(rows)[None, :, None]
                vertex0, vertex1, vertex2 = (projected[indices, corner, :, None, None] for corner in range(3))
                denominator = divisor[indices, None, None]
                weight0 = ((vertex1[:, 1] - vertex2[:, 1]) * (grid_x + 0.5 - vertex2[:, 0]) +
                           (vertex2[:, 0] - vertex1[:, 0]) * (grid_y + 0.5 - vertex2[:, 1])) / denominator
                weight1 = ((vertex2[:, 1] - vertex0[:, 1]) * (grid_x + 0.5 - vertex2[:, 0]) +
                           (vertex0[:, 0] - vertex2[:, 0]) * (grid_y + 0.5 - vertex2[:, 1])) / denominator
                weight2 = 1 - weight0 - weight1
                mask = ((weight0 >= -1e-6) & (weight1 >= -1e-6) & (weight2 >= -1e-6) &
                        (grid_x < self.width) & (grid_y < self.height))
                face_index, row_index, column_index = np.nonzero(mask)
                if not len(face_index):
                    continue
                actual = indices[face_index]
                destinations = (low[actual, 1] + row_index) * self.width + low[actual, 0] + column_index
                weights = np.column_stack((weight0[mask], weight1[mask], weight2[mask]))
                distances = np.einsum("ij,ij->i", weights, projected[actual, :, 2]).astype(np.float32)
                np.maximum.at(flat_depth, destinations, distances)
                if not depth_only:
                    keep = distances >= flat_depth[destinations]
                    colour = np.einsum("ij,ijk->ik", weights[keep], shaded[actual[keep]])
                    flat_pixels[destinations[keep]] = np.clip(colour * 255, 0, 255).astype(np.uint8)


class Cell:
    def __init__(self, meshes):
        self.cycle = motion.Cycle()
        self.layout = self.cycle.cell
        self.robots = {name: Robot(meshes, gripper=name == "arm2") for name in self.cycle.arms}
        self.fixed = []
        self.static_geometry()

    def box(self, center, size, colour=GRAPHITE, radius=0.002, metal=0.2):
        self.fixed.append((box_mesh(size, radius), transform(center), colour, metal))

    def cylinder(self, center, radius, length, colour=SILVER, axis=(0, 0, 1), metal=0.6):
        axis = unit(axis)
        angle = math.acos(np.clip(axis[2], -1, 1))
        rotation_axis = np.cross((0, 0, 1), axis)
        if np.linalg.norm(rotation_axis) < 1e-8:
            rotation_axis = (1, 0, 0)
        self.fixed.append((cylinder_mesh(radius, length), transform(center, rotation_axis, angle), colour, metal))

    def static_geometry(self):
        self.box((0.24, 0.20, 0.943), (1.39, 0.91, 0.08), (0.12, 0.14, 0.16), 0.012)
        self.box((0.24, 0.20, 0.982), (1.40, 0.92, 0.014), (0.40, 0.43, 0.45), 0.007, 0.55)
        self.box((0.12, 0.125, 1.062), (0.64, 0.31, 0.147), GRAPHITE, 0.005)
        self.box((0.12, 0.125, 1.136), (0.65, 0.32, 0.005), (0.26, 0.28, 0.29), 0.002, 0.5)
        for horizontal in np.arange(-0.37, 0.88, 0.055):
            for vertical in (-0.175, 0.575):
                self.cylinder((horizontal, vertical, 0.990), 0.0025, 0.0005, (0.035, 0.04, 0.04))
        for horizontal in (-0.37, 0.86):
            for vertical in (-0.16, 0.56):
                self.box((horizontal, vertical, 0.85), (0.042, 0.042, 0.17), SILVER, 0.003, 0.8)
                self.box((horizontal, vertical - 0.021, 0.85), (0.009, 0.001, 0.15), GRAPHITE, 0.0003)
                self.box((horizontal, vertical, 0.761), (0.078, 0.078, 0.012), GRAPHITE, 0.003, 0.3)
        for index, spec in enumerate(build_scene.CONVEYORS.values(), 1):
            y_center = sum(spec["y"]) / 2
            for offset in (-0.044, 0.044):
                self.box((0.112, y_center + offset, 1.148), (0.734, 0.014, 0.037), SILVER, 0.003, 0.8)
                self.box((0.112, y_center + offset - 0.0075, 1.148), (0.68, 0.001, 0.004), GRAPHITE, 0.0003)
                for horizontal in (-0.21, 0.0, 0.224, 0.435):
                    self.cylinder((horizontal, y_center + offset - 0.009, 1.15), 0.003, 0.003, (0.25, 0.27, 0.29), (0, 1, 0))
            self.box((0.112, y_center, 1.165), (0.70, 0.072, 0.010), (0.075, 0.083, 0.085), 0.004, 0.02)
            for horizontal in (-0.238, 0.462):
                self.cylinder((horizontal, y_center, 1.155), 0.015, 0.075, (0.27, 0.29, 0.31), (0, 1, 0))
                self.cylinder((horizontal, y_center - 0.053, 1.155), 0.008, 0.02, SILVER, (0, 1, 0))
                self.box((horizontal, y_center, 1.075), (0.028, 0.055, 0.14), SILVER, 0.003, 0.6)
                self.box((horizontal, y_center, 0.996), (0.060, 0.090, 0.012), GRAPHITE, 0.004)
            motor_x = 0.462 if index == 1 else -0.238
            self.cylinder((motor_x, y_center - 0.085, 1.151), 0.021, 0.07, GRAPHITE, (0, 1, 0))
            self.cylinder((motor_x, y_center - 0.122, 1.151), 0.022, 0.005, GREEN, (0, 1, 0))
            for ring in range(5):
                self.cylinder((motor_x, y_center - 0.10 + ring * 0.007, 1.151), 0.022, 0.002, (0.19, 0.21, 0.22), (0, 1, 0))
        for name, position in self.layout["sensors"].items():
            sensor_x, sensor_y, sensor_z = position
            self.cylinder((sensor_x, sensor_y + 0.024, 1.160), 0.004, 0.076)
            self.box((sensor_x, sensor_y + 0.018, sensor_z), (0.018, 0.024, 0.032), GRAPHITE, 0.003)
            self.cylinder((sensor_x, sensor_y + 0.003, sensor_z), 0.005, 0.007, (0.40, 0.02, 0.025), (0, 1, 0), 0.8)
            self.box((sensor_x + 0.004, sensor_y + 0.018, sensor_z + 0.017), (0.003, 0.006, 0.001), GREEN, 0.0003)
        tray = self.layout["tray"]
        tray_x = sum(tray["x"]) / 2
        tray_y = sum(tray["y"]) / 2
        self.box((tray_x, tray_y, 1.000), (0.164, 0.121, 0.024), GRAPHITE, 0.006)
        for offset in (-0.079, 0.079):
            self.box((tray_x + offset, tray_y, 1.02), (0.006, 0.121, 0.020), SILVER, 0.001)
        for name, robot in self.layout["robots"].items():
            position = robot["base"]
            self.box((position[0], position[1], position[2] - 0.004), (0.15, 0.14, 0.006), SILVER, 0.007)
            for offset_x in (-0.06, 0.06):
                for offset_y in (-0.054, 0.054):
                    self.cylinder((position[0] + offset_x, position[1] + offset_y, position[2] + 0.001), 0.003, 0.002, GRAPHITE)

    def objects(self, seconds):
        objects = list(self.fixed)
        for name, arm in self.cycle.arms.items():
            base, shoulder, crank = kinematics.joints_from_motors(*kinematics.motor_angles(*arm.sdk(seconds)))
            joints = {"Joint1": base, "Joint2": shoulder, "Joint6": crank, "JointGripperExt": arm.wrist(seconds) - 90}
            joints.update(kinematics.solve_linkage(shoulder, crank)["joints"])
            objects.extend(self.robots[name].objects(arm.base, arm.yaw, joints))
        for index, spec in enumerate(build_scene.CONVEYORS.values(), 1):
            beginning, ending = self.cycle.belt1 if index == 1 else self.cycle.belt2
            direction = 1 if index == 1 else -1
            shift = direction * motion.BELT_SPEED * (np.clip(seconds, beginning, ending) - beginning)
            for horizontal in np.arange(-0.238, 0.462, 0.024):
                shifted = -0.238 + (horizontal + 0.238 + shift) % 0.7
                objects.append((box_mesh((0.0013, 0.064, 0.0006), 0.0002, 1),
                                transform((shifted, sum(spec["y"]) / 2, 1.1703)), (0.14, 0.15, 0.16), 0.05))
        for name, size, colour in (("holder", build_scene.HOLDER_SIZE, (0.66, 0.69, 0.71)),
                                    ("pcb", build_scene.PCB_SIZE, (0.08, 0.31, 0.14))):
            center, yaw, opacity = getattr(self.cycle, name)(seconds)
            if opacity < 0.5:
                continue
            matrix = transform(center, angle=math.radians(yaw))
            objects.append((box_mesh(size, 0.003 if name == "holder" else 0.001), matrix, colour, 0.25))
            if name == "pcb":
                for offset in (-0.015, 0.015):
                    objects.append((box_mesh((0.019, 0.022, 0.004), 0.0005), matrix @ transform((0, offset, size[2] / 2 + 0.002)), GRAPHITE, 0.1))
                for offset in np.linspace(-0.022, 0.022, 12):
                    objects.append((box_mesh((0.002, 0.007, 0.001), 0.0002), matrix @ transform((offset, -0.033, size[2] / 2 + 0.0006)), (0.69, 0.55, 0.19), 0.7))
        return objects


def font(size, bold=False, mono=False):
    names = (("CascadiaMono.ttf", "consola.ttf") if mono else
             ("bahnschrift.ttf", "segoeuib.ttf" if bold else "segoeui.ttf"))
    for name in names:
        for folder in (Path("C:/Windows/Fonts"), Path("/usr/share/fonts/truetype/dejavu")):
            candidate = folder / name
            if candidate.is_file():
                return ImageFont.truetype(str(candidate), size)
    return ImageFont.truetype("DejaVuSans.ttf", size)


def pill(draw, xy, text_value, factor, border="#76b900", fill="#121619", ink="#eef2f5", dot=None, mono=True):
    fnt = font(round(13 * factor), mono=mono)
    bbox = draw.textbbox((0, 0), text_value, font=fnt)
    tw, th = bbox[2] - bbox[0], bbox[3] - bbox[1]
    pad_x = round((22 if dot else 10) * factor)
    pad_r = round(10 * factor)
    pad_y = round(6 * factor)
    x0, y0 = round(xy[0]), round(xy[1])
    x1, y1 = x0 + pad_x + tw + pad_r, y0 + th + 2 * pad_y
    draw.rounded_rectangle((x0, y0, x1, y1), radius=round(5 * factor), fill=fill, outline=border,
                           width=max(1, round(1.4 * factor)))
    if dot:
        cy = (y0 + y1) // 2
        r = max(2, round(4 * factor))
        draw.ellipse((x0 + round(10 * factor) - r, cy - r, x0 + round(10 * factor) + r, cy + r), fill=dot)
    draw.text((x0 + pad_x, y0 + pad_y - bbox[1]), text_value, fill=ink, font=fnt)
    return (x0, y0, x1, y1)


def viewport(image, seconds, duration, cell=None):
    draw = ImageDraw.Draw(image)
    width, height = image.size
    factor = width / 1600
    cam = Renderer(width, height, (1.0, -1.65, 2.25), (0.22, 0.21, 1.17), 1.53)

    def label(position, value, size=16, colour="#d4d7d9", mono=False, bold=False):
        draw.text(tuple(round(component * factor) for component in position), value, fill=colour,
                  font=font(round(size * factor), bold=bold, mono=mono))

    if cell is not None:
        for name, (sx, sy, sz) in cell.layout["sensors"].items():
            belt = "Conveyor1" if name in ("S1", "S2") else "Conveyor2"
            y_far = build_scene.CONVEYORS[belt]["y"][0] - 0.008
            pts = cam.project([(sx, sy, sz), (sx, y_far, sz), (sx, sy + 0.02, sz + 0.055)])
            p0, p1, p_lbl = pts[0, :2], pts[1, :2], pts[2, :2]
            active = cell.cycle.sensor_on(name, seconds)
            if active:
                draw.line([tuple(p0), tuple(p1)], fill="#4a7c00", width=max(3, round(6 * factor)))
                draw.line([tuple(p0), tuple(p1)], fill="#c8ff4d", width=max(1, round(2.2 * factor)))
            else:
                for u in np.linspace(0.0, 0.88, 8):
                    q0 = p0 + (p1 - p0) * u
                    q1 = p0 + (p1 - p0) * (u + 0.07)
                    draw.line([tuple(q0), tuple(q1)], fill="#4c6b22", width=max(1, round(1.3 * factor)))
            pill(draw, (p_lbl[0] - 18 * factor, p_lbl[1] - 12 * factor), name, factor,
                 border="#76b900" if active else "#3a4248",
                 fill="#1d2b0a" if active else "#14181c",
                 ink="#d8ff7a" if active else "#b8c0c6")

        arm_specs = (
            ("uarm1", "arm1", "uarm1 · vacuum", (-95, 22)),
            ("uarm2", "arm2", "uarm2 · gripper+wrist", (-55, 26)),
            ("uarm3", "arm3", "uarm3 · vacuum", (-45, -145)),
        )
        for rname, aname, tag, (dx, dy) in arm_specs:
            bx, by, bz = cell.layout["robots"][rname]["base"]
            pt = cam.project([(bx, by, bz)])[0, :2]
            on = cell.cycle.tool_on(aname, seconds)
            suffix = " [ON]" if on else ""
            pill(draw, (pt[0] + dx * factor, pt[1] + dy * factor), tag + suffix, factor,
                 border="#76b900" if on else "#384148",
                 fill="#172408" if on else "#12161a",
                 ink="#e8ffad" if on else "#d8dee3",
                 dot="#76b900" if on else "#606a72")

        hud_x, hud_y = round(1110 * factor), round(66 * factor)
        hud_w, hud_h = round(468 * factor), round(226 * factor)
        draw.rounded_rectangle((hud_x, hud_y, hud_x + hud_w, hud_y + hud_h), radius=round(8 * factor),
                               fill="#12161a", outline="#2e363d", width=max(1, round(1.5 * factor)))
        draw.rectangle((hud_x, hud_y + round(10 * factor), hud_x + round(4 * factor), hud_y + hud_h - round(10 * factor)),
                       fill="#76b900")
        label((1130, 78), "STAGE & LIVE TELEMETRY", 13, "#76b900", mono=True, bold=True)
        label((1416, 78), "GET /api/servo_angles", 12, "#8f99a1", mono=True)
        for idx, (aname, rpath) in enumerate((("arm1", "/World/uarm1"), ("arm2", "/World/uarm2"), ("arm3", "/World/uarm3"))):
            vals = cell.cycle.servo(aname, seconds)
            formatted = ", ".join(f"{v:5.1f}°" for v in vals)
            ry = 106 + idx * 28
            label((1130, ry), f"{rpath}", 13, "#c7d0d6", mono=True)
            label((1256, ry), f"[{formatted}]", 13, "#ffffff", mono=True)
        draw.line((hud_x + round(16 * factor), round(198 * factor), hud_x + hud_w - round(16 * factor), round(198 * factor)),
                  fill="#262d33", width=max(1, round(factor)))
        indicators = [
            ("S1", cell.cycle.sensor_on("S1", seconds)),
            ("S2", cell.cycle.sensor_on("S2", seconds)),
            ("S3", cell.cycle.sensor_on("S3", seconds)),
            ("S4", cell.cycle.sensor_on("S4", seconds)),
            ("BELT 1", cell.cycle.belt_on(1, seconds)),
            ("BELT 2", cell.cycle.belt_on(2, seconds)),
        ]
        for idx, (iname, state) in enumerate(indicators):
            ix = 1132 + idx * 71
            iy = 216
            r = max(2, round(4.5 * factor))
            cx, cy = round(ix * factor), round((iy + 8) * factor)
            draw.ellipse((cx - r, cy - r, cx + r, cy + r), fill="#76b900" if state else "#394147")
            label((ix + 9, iy), iname, 12, "#e5ebf0" if state else "#7f8991", mono=True)
        label((1130, 252), "120 Hz HTTP poll -> 60 Hz PhysX PD targets (EMA α=0.28)", 12, "#97a1a9", mono=True)

    gx, gy = round(58 * factor), height - round(108 * factor)
    for axis_vec, axis_col, axis_lbl in (((1, 0, 0), "#ff5555", "X"), ((0, 1, 0), "#76b900", "Y"), ((0, 0, 1), "#3b82f6", "Z")):
        d2 = (np.asarray(axis_vec, dtype=float) @ cam.basis.T)[:2]
        ex, ey = gx + round(d2[0] * 30 * factor), gy - round(d2[1] * 30 * factor)
        draw.line((gx, gy, ex, ey), fill=axis_col, width=max(2, round(2.5 * factor)))
        label(((ex / factor) + 4, (ey / factor) - 8), axis_lbl, 12, axis_col, mono=True, bold=True)

    draw.rectangle((0, 0, width, round(48 * factor)), fill="#14171a")
    draw.rectangle((0, round(46 * factor), width, round(48 * factor)), fill="#262c32")
    draw.rectangle((round(18 * factor), round(14 * factor), round(24 * factor), round(34 * factor)), fill="#76b900")
    label((34, 14), "NVIDIA Isaac Sim 5.1", 18, "#ffffff", bold=True)
    label((235, 16), "|   scene/cell.usda   ·   Perspective   ·   PhysX 60 Hz   ·   Z-up (metres)", 15, "#b4bdc4", mono=True)
    pill(draw, (1325 * factor, 11 * factor), "FOLLOWER ACTIVE · :8001", factor, border="#76b900",
         fill="#172408", ink="#d8ff7a", dot="#76b900")

    draw.rectangle((0, height - round(58 * factor), width, height), fill="#14171a")
    draw.rectangle((0, height - round(58 * factor), width, height - round(56 * factor)), fill="#262c32")
    caption = "Live digital twin cycle"
    if cell is not None:
        active_captions = [text for start, text in cell.cycle.captions() if start <= seconds]
        caption = active_captions[-1] if active_captions else cell.cycle.captions()[0][1]
    label((24, height / factor - 41), "STEP", 14, "#76b900", mono=True, bold=True)
    label((78, height / factor - 42), caption, 16, "#eef2f5")
    start, end = round(980 * factor), width - round(195 * factor)
    vertical = height - round(29 * factor)
    draw.line((start, vertical, end, vertical), fill="#384047", width=max(2, round(3 * factor)))
    playhead = start + (end - start) * min(max(seconds / duration, 0.0), 1.0)
    draw.line((start, vertical, playhead, vertical), fill="#76b900", width=max(2, round(3 * factor)))
    draw.ellipse((playhead - 5 * factor, vertical - 5 * factor, playhead + 5 * factor, vertical + 5 * factor), fill="#76b900")
    label((1425, height / factor - 41), f"t = {seconds:05.2f} / {duration:04.1f} s", 14, "#c8d0d6", mono=True)
    return image


def cell_image(cell, seconds, width=2400, height=1500):
    renderer = Renderer(width, height, (1.0, -1.65, 2.25), (0.22, 0.21, 1.17), 1.53)
    return renderer.render(cell.objects(seconds))


def embed_image(image):
    buffer = io.BytesIO()
    image.save(buffer, format="PNG", optimize=True)
    return "data:image/png;base64," + base64.b64encode(buffer.getvalue()).decode("ascii")


def write_hero(viewport_img, output, dark):
    W, H = 2400, 760
    bg, ink, muted = ("#0d1013", "#ffffff", "#a7b1b8") if dark else ("#f5f7f9", "#111518", "#4a545c")
    chip_bg, chip_border = ("#161b20", "#2d363f") if dark else ("#ffffff", "#cfd7de")
    grid_col = "#171c21" if dark else "#e6ebf0"
    canvas = Image.new("RGB", (W, H), bg)
    draw = ImageDraw.Draw(canvas)
    for gx in range(0, 1180, 40):
        draw.line((gx, 0, gx, H), fill=grid_col, width=1)
    for gy in range(0, H, 40):
        draw.line((0, gy, 1180, gy), fill=grid_col, width=1)

    draw.rectangle((64, 72, 72, 232), fill="#76b900")
    draw.text((96, 72), "NVIDIA ISAAC SIM 5.x  ·  OPENUSD  ·  PHYSX ARTICULATIONS", font=font(22, mono=True), fill="#76b900")
    draw.text((94, 116), "Isaac uArm Twin", font=font(82, bold=True), fill=ink)
    draw.text((64, 268), "Real-time digital twin of a three-arm robot assembly cell.", font=font(36, bold=True), fill=ink)
    draw.text((64, 330), "Live servo angles from three uArm Swift Pro arms stream over FastAPI into", font=font(25), fill=muted)
    draw.text((64, 368), "calibrated PhysX joint drives, closed-loop linkages, conveyors and light barriers.", font=font(25), fill=muted)

    metrics = [
        ("3× uArm Swift Pro", "2 vacuum + 1 gripper/wrist"),
        ("Closed-loop PhysX", "3 excluded loop joints / arm"),
        ("1 Hz -> 60 Hz", "rate-limited EMA follower"),
    ]
    for idx, (m_title, m_sub) in enumerate(metrics):
        mx = 64 + idx * 356
        my = 446
        draw.rounded_rectangle((mx, my, mx + 338, my + 108), radius=10, fill=chip_bg, outline=chip_border, width=2)
        draw.rectangle((mx, my + 14, mx + 4, my + 94), fill="#76b900")
        draw.text((mx + 20, my + 18), m_title, font=font(24, bold=True), fill=ink)
        draw.text((mx + 20, my + 58), m_sub, font=font(19, mono=True), fill=muted)

    chips = ("Isaac Sim 5.x", "OpenUSD Stage", "PhysX PD Drives", "OmniGraph Sensors", "FastAPI :8001")
    cx = 64
    for label in chips:
        fnt = font(20, mono=True)
        bbox = draw.textbbox((0, 0), label, font=fnt)
        cw = bbox[2] - bbox[0] + 38
        draw.rounded_rectangle((cx, 602, cx + cw, 654), radius=8, fill=chip_bg, outline=chip_border, width=2)
        draw.rectangle((cx, 612, cx + 4, 644), fill="#76b900")
        draw.text((cx + 18, 616), label, font=fnt, fill=ink)
        cx += cw + 14

    card_x, card_y, card_w, card_h = 1170, 42, 1178, 676
    crop = viewport_img.resize((1240, 775), Image.Resampling.LANCZOS).crop((31, 45, 31 + card_w, 45 + card_h))
    canvas.paste(crop, (card_x, card_y))
    draw.rounded_rectangle((card_x, card_y, card_x + card_w, card_y + card_h), radius=14, outline="#76b900", width=3)
    draw.rectangle((card_x, card_y, card_x + card_w, card_y + 44), fill="#14171a")
    draw.ellipse((card_x + 18, card_y + 15, card_x + 32, card_y + 29), fill="#76b900")
    draw.text((card_x + 44, card_y + 11), "scene/cell.usda  ·  3× uArm Swift Pro  ·  PhysX 60 Hz",
              font=font(19, mono=True), fill="#e6edf2")
    draw.text((card_x + card_w - 245, card_y + 11), "GET /api/servo_angles", font=font(18, mono=True), fill="#76b900")
    draw.rectangle((0, H - 6, W, H), fill="#76b900")
    canvas.save(output / f"hero-{'dark' if dark else 'light'}.png", optimize=True)


def write_poster(image, output, name, title):
    image.save(output / f"{name}.png", optimize=True)
    encoded = embed_image(image)
    (output / f"{name}.svg").write_text(
        f'<svg xmlns="http://www.w3.org/2000/svg" viewBox="0 0 {image.width} {image.height}" role="img">'
        f'<title>{title}</title><image width="{image.width}" height="{image.height}" href="{encoded}"/></svg>',
        encoding="utf-8")


def detail_image(meshes, output):
    W, H = 2400, 1260
    joints = {"Joint1": -18, "Joint2": 25, "Joint6": 50}
    solved = kinematics.solve_linkage(joints["Joint2"], joints["Joint6"])
    joints.update(solved["joints"])
    robot = Robot(meshes)
    matrices = robot.matrices((0, 0, 0), 0, joints)
    objects = robot.objects((0, 0, 0), 0, joints)
    objects.append((box_mesh((0.54, 0.44, 0.022), 0.006), transform((0.09, 0, -0.011)), (0.13, 0.15, 0.17), 0.25))
    cam = Renderer(1500, H, (0.58, -1.02, 0.48), (0.075, 0, 0.145), 0.45, bg_top=22, bg_bottom=22)
    left_render = cam.render(objects)

    panel = Image.new("RGB", (W, H), "#161616")
    panel.paste(left_render, (0, 0))
    draw = ImageDraw.Draw(panel)
    draw.rectangle((0, 0, W, 88), fill="#121518")
    draw.rectangle((0, 86, W, 88), fill="#262c32")
    draw.rectangle((36, 22, 42, 66), fill="#76b900")
    draw.text((58, 18), "uArm Swift Pro  ·  Closed-Loop Parallel Linkage & PhysX Rig", font=font(32, bold=True), fill="#ffffff")
    draw.text((58, 54), "2 actuated planar drives + base yaw  |  6 passive linkage joints  |  3 excluded loop-closure revolute joints",
              font=font(18, mono=True), fill="#9aa5ad")

    def world_pt(link_name, local_xyz=(0, 0, 0)):
        mat = matrices[link_name]
        return tuple((np.asarray(local_xyz, dtype=float) @ mat[:3, :3].T) + mat[:3, 3])

    callouts = [
        (world_pt("Link1"), (85, 815), "J1 Base Yaw · Driven (k=10000)", "#76b900"),
        (world_pt("Link2"), (85, 650), "J2 Shoulder + J6 Crank · Driven", "#76b900"),
        (world_pt("Link5"), (85, 495), "J5 Rear Pivot · Passive", "#cbd5e1"),
        (world_pt("Link5", kinematics.LOOP_JOINTS["Loop_Link5_Link4"]["pos1"]), (85, 225),
         "Loop 1 · Link5-Link4 (excludeFromArticulation)", "#38bdf8"),
        (world_pt("Link7", kinematics.LOOP_JOINTS["Loop_Link7_Link3"]["pos1"]), (85, 330),
         "Loop 2 · Link7-Link3 (elbow drive rod)", "#38bdf8"),
        (world_pt("Link3"), (720, 155), "J3 / J4 Elbow · Passive (DriveAPI removed)", "#cbd5e1"),
        (world_pt("Link9", kinematics.LOOP_JOINTS["Loop_Link9_Link4"]["pos1"]), (735, 255),
         "Loop 3 · Link9-Link4 (keeps tool level)", "#38bdf8"),
        (world_pt("Link8"), (880, 445), "J8 / J9 Wrist · Passive", "#cbd5e1"),
        (world_pt("Link8", (0.056381, 0.000173, -0.073327)), (810, 745),
         "AttachPoint0 · Pick-as-is tool reference", "#fbbf24"),
    ]
    for wpt, label_xy, text_value, col in callouts:
        px, py, _ = cam.project([wpt])[0]
        x0, y0, x1, y1 = pill(draw, label_xy, text_value, 1.35, border=col, fill="#111519", ink="#f1f5f9", dot=col)
        anchor_x = x1 if px > (x0 + x1) / 2 else x0
        anchor_y = (y0 + y1) // 2
        draw.line((px, py, anchor_x, anchor_y), fill=col, width=3)
        draw.ellipse((px - 7, py - 7, px + 7, py + 7), fill=col, outline="#0d1013", width=2)
        pill(draw, label_xy, text_value, 1.35, border=col, fill="#111519", ink="#f1f5f9", dot=col)

    rx = 1475
    draw.line((rx - 25, 108, rx - 25, H - 36), fill="#2a3138", width=2)
    cards = [
        ("1. ACTUATED PD DRIVES (SERVO TARGETS)", "#76b900", [
            ("Joint1  (Z yaw)", "limits -90..90°   | k = 10,000 | d = 1,000"),
            ("Joint2  (Y shoulder)", "limits -30..90°   | k = 10,000 | d = 1,000"),
            ("Joint6  (Y crank)", "limits -17..110°  | k = 2,000  | d = 200"),
            ("JointGripperExt (arm 2)", "limits -90..90°   | k = 20,000 | d = 5,000"),
        ]),
        ("2. PASSIVE LINKAGE JOINTS (NO DRIVE)", "#cbd5e1", [
            ("Joint3, Joint4, Joint5", "DriveAPI removed  | joint friction = 0.1"),
            ("Joint7, Joint8, Joint9", "Follow closed loops | J3 limits -83.1..83.1°"),
            ("Validation vs PhysX", "solve_linkage(j2, j6) matches within 0.4°"),
        ]),
        ("3. THREE EXCLUDED LOOP CLOSURE JOINTS", "#38bdf8", [
            ("Loop_Link5_Link4", "Rear parallelogram  | pos0 (-34.7, 16.2, 16.6) mm"),
            ("Loop_Link7_Link3", "Elbow drive rod     | pos0 (-45.0, -3.2, -2.7) mm"),
            ("Loop_Link9_Link4", "Tool-level linkage  | pos0 (27.7, 16.1, 27.1) mm"),
            ("Articulation solver", "32 pos / 1 vel iters | self-collision off"),
        ]),
    ]
    cy = 118
    for heading, col, rows in cards:
        ch = 68 + len(rows) * 62
        draw.rounded_rectangle((rx, cy, W - 44, cy + ch), radius=10, fill="#12161a", outline="#2b333b", width=2)
        draw.rectangle((rx, cy + 12, rx + 5, cy + ch - 12), fill=col)
        draw.text((rx + 24, cy + 18), heading, font=font(22, mono=True, bold=True), fill=col)
        for r_idx, (left_t, right_t) in enumerate(rows):
            ry = cy + 62 + r_idx * 60
            draw.text((rx + 24, ry), left_t, font=font(21, mono=True, bold=True), fill="#ffffff")
            draw.text((rx + 295, ry), right_t, font=font(20, mono=True), fill="#b8c2c9")
        cy += ch + 24

    draw.rectangle((0, H - 5, W, H), fill="#76b900")
    panel.save(output / "linkage.png", optimize=True)


def write_calibration_charts(cell, output):
    import follower

    W, H = 1200, 430
    series = motion.follow(cell.cycle, "arm1", "Joint1", 0)
    dur = cell.cycle.T
    for dark in (True, False):
        bg = "#0d1013" if dark else "#f7f9fa"
        card = "#13181c" if dark else "#ffffff"
        line = "#283038" if dark else "#d5dde3"
        grid = "#1c2228" if dark else "#e9eef2"
        ink = "#ffffff" if dark else "#111518"
        muted = "#99a3ad" if dark else "#525e68"
        svg = [
            f'<svg xmlns="http://www.w3.org/2000/svg" viewBox="0 0 {W} {H}" width="{W}" height="{H}" '
            f'font-family="Segoe UI, Helvetica Neue, Arial, sans-serif" role="img" '
            'aria-label="Two-point servo calibration curves and 1 Hz to 60 Hz follower filter response">',
            "<title>Two-point servo calibration and 60 Hz follower smoothing response</title>",
            f'<rect width="{W}" height="{H}" rx="16" fill="{bg}" stroke="{line}"/>',
            f'<rect x="24" y="22" width="564" height="{H - 44}" rx="12" fill="{card}" stroke="{line}"/>',
            f'<rect x="612" y="22" width="564" height="{H - 44}" rx="12" fill="{card}" stroke="{line}"/>',
            f'<text x="48" y="54" font-family="Cascadia Mono, Consolas, monospace" font-size="13" font-weight="700" fill="#76b900">1 · TWO-POINT SERVO-TO-ISAAC CALIBRATION</text>',
            f'<text x="48" y="74" font-size="12.5" fill="{muted}">Measured extended/retracted servo anchors mapped to PhysX DOF targets (deg)</text>',
            f'<text x="636" y="54" font-family="Cascadia Mono, Consolas, monospace" font-size="13" font-weight="700" fill="#76b900">2 · 1 HZ TELEMETRY TO 60 HZ PHYSX TARGET (ARM 1 BASE)</text>',
            f'<text x="636" y="74" font-size="12.5" fill="{muted}">Rate limiter (360°/s) + exponential smoothing (α = 0.28) + 0.05° deadband</text>',
        ]
        lx0, ly0, lx1, ly1 = 84, 102, 556, 356
        for sv in (-20, 0, 45, 90, 135, 180):
            px = lx0 + (lx1 - lx0) * (sv - (-30)) / 220
            svg.append(f'<line x1="{px:.1f}" y1="{ly0}" x2="{px:.1f}" y2="{ly1}" stroke="{grid}"/>')
            svg.append(f'<text x="{px:.1f}" y="{ly1 + 18}" text-anchor="middle" font-family="Cascadia Mono, Consolas, monospace" font-size="10.5" fill="{muted}">{sv}°</text>')
        for iv in (-90, -45, 0, 45, 90):
            py = ly1 - (ly1 - ly0) * (iv - (-95)) / 215
            svg.append(f'<line x1="{lx0}" y1="{py:.1f}" x2="{lx1}" y2="{py:.1f}" stroke="{grid}"/>')
            svg.append(f'<text x="{lx0 - 8}" y="{py + 4:.1f}" text-anchor="end" font-family="Cascadia Mono, Consolas, monospace" font-size="10.5" fill="{muted}">{iv}°</text>')
        svg.append(f'<text x="{(lx0 + lx1) / 2:.0f}" y="{H - 34}" text-anchor="middle" font-size="11.5" fill="{muted}">Physical uArm servo angle (deg)</text>')
        cfg = follower.ARM_CONFIGS["arm2"]
        palette = (("Joint1", "#76b900"), ("Joint2", "#38bdf8"), ("Joint6", "#f59e0b"), ("JointGripperExt", "#a855f7"))
        for idx, (jname, col) in enumerate(palette):
            pts = []
            for sv in np.linspace(-28, 185, 90):
                iv = follower.servo_to_isaac_deg(cfg, jname, float(sv))
                px = lx0 + (lx1 - lx0) * (sv + 30) / 220
                py = ly1 - (ly1 - ly0) * (iv + 95) / 215
                pts.append(f"{px:.1f},{py:.1f}")
            svg.append(f'<polyline fill="none" stroke="{col}" stroke-width="2.4" points="{" ".join(pts)}"/>')
            cal = follower.CALIBRATION[jname]
            for s_anchor in (cal["servo_ext"], cal["servo_ret"]):
                iv = follower.servo_to_isaac_deg(cfg, jname, s_anchor)
                px = lx0 + (lx1 - lx0) * (s_anchor + 30) / 220
                py = ly1 - (ly1 - ly0) * (iv + 95) / 215
                svg.append(f'<circle cx="{px:.1f}" cy="{py:.1f}" r="4" fill="{col}" stroke="{card}" stroke-width="1.5"/>')
            lx = lx0 + 10 + (idx % 2) * 230
            ly = ly0 + 16 + (idx // 2) * 20
            svg.append(f'<circle cx="{lx}" cy="{ly - 4}" r="4.5" fill="{col}"/>')
            svg.append(f'<text x="{lx + 10}" y="{ly}" font-family="Cascadia Mono, Consolas, monospace" font-size="11" fill="{ink}">{jname}</text>')

        rx0, ry0, rx1, ry1 = 668, 102, 1148, 356
        vals = [v for _, v, _ in series] + [t_v for _, _, t_v in series]
        v_lo, v_hi = min(vals) - 6, max(vals) + 6
        for t_tick in range(0, int(dur) + 1, 5):
            px = rx0 + (rx1 - rx0) * t_tick / dur
            svg.append(f'<line x1="{px:.1f}" y1="{ry0}" x2="{px:.1f}" y2="{ry1}" stroke="{grid}"/>')
            svg.append(f'<text x="{px:.1f}" y="{ry1 + 18}" text-anchor="middle" font-family="Cascadia Mono, Consolas, monospace" font-size="10.5" fill="{muted}">{t_tick}s</text>')
        for y_tick in (-60, -30, 0, 30, 60):
            if v_lo <= y_tick <= v_hi:
                py = ry1 - (ry1 - ry0) * (y_tick - v_lo) / (v_hi - v_lo)
                svg.append(f'<line x1="{rx0}" y1="{py:.1f}" x2="{rx1}" y2="{py:.1f}" stroke="{grid}"/>')
                svg.append(f'<text x="{rx0 - 8}" y="{py + 4:.1f}" text-anchor="end" font-family="Cascadia Mono, Consolas, monospace" font-size="10.5" fill="{muted}">{y_tick}°</text>')
        raw_pts = []
        smooth_pts = []
        for tau, applied, target in series[::2]:
            px = rx0 + (rx1 - rx0) * tau / dur
            py_raw = ry1 - (ry1 - ry0) * (target - v_lo) / (v_hi - v_lo)
            py_sm = ry1 - (ry1 - ry0) * (applied - v_lo) / (v_hi - v_lo)
            raw_pts.append(f"{px:.1f},{py_raw:.1f}")
            smooth_pts.append(f"{px:.1f},{py_sm:.1f}")
        svg.append(f'<polyline fill="none" stroke="{muted}" stroke-width="1.6" stroke-dasharray="4 3" points="{" ".join(raw_pts)}"/>')
        svg.append(f'<polyline fill="none" stroke="#76b900" stroke-width="2.5" points="{" ".join(smooth_pts)}"/>')
        svg.append(f'<line x1="{rx0 + 12}" y1="{ry0 + 14}" x2="{rx0 + 34}" y2="{ry0 + 14}" stroke="{muted}" stroke-width="1.8" stroke-dasharray="4 3"/>')
        svg.append(f'<text x="{rx0 + 42}" y="{ry0 + 18}" font-family="Cascadia Mono, Consolas, monospace" font-size="11" fill="{ink}">1 Hz sampled input</text>')
        svg.append(f'<line x1="{rx0 + 210}" y1="{ry0 + 14}" x2="{rx0 + 232}" y2="{ry0 + 14}" stroke="#76b900" stroke-width="2.6"/>')
        svg.append(f'<text x="{rx0 + 240}" y="{ry0 + 18}" font-family="Cascadia Mono, Consolas, monospace" font-size="11" fill="{ink}">60 Hz smoothed PhysX target</text>')
        svg.append(f'<text x="{(rx0 + rx1) / 2:.0f}" y="{H - 34}" text-anchor="middle" font-size="11.5" fill="{muted}">Cycle time (seconds)</text>')
        svg.append("</svg>")
        (output / f"calibration-{'dark' if dark else 'light'}.svg").write_text("\n".join(svg), encoding="utf-8")


def generate(cell, output, frames=0, animation_cell=None, animation_only=False):
    seconds = 11.1
    if not animation_only:
        raw = cell_image(cell, seconds, 3200, 2000)
        image = raw.resize((2400, 1500), Image.Resampling.LANCZOS)
        vp = viewport(image.copy(), seconds, cell.cycle.T, cell=cell)
        vp.save(output / "twin-viewport.png", optimize=True)
        for dark in (True, False):
            write_hero(image.copy(), output, dark)
        write_calibration_charts(cell, output)
    if frames:
        animation_cell = animation_cell or cell
        images = []
        for index in range(frames):
            seconds = 0.45 + (cell.cycle.T - 0.95) * index / max(frames - 1, 1)
            frame = cell_image(animation_cell, seconds, 1920, 1200).resize((1280, 800), Image.Resampling.LANCZOS)
            images.append(viewport(frame, seconds, cell.cycle.T, cell=animation_cell))
            if index % 10 == 0:
                print(f"Rendered animation {index + 1}/{frames}", flush=True)
        samples = Image.new("RGB", (320 * 6, 200))
        for index, frame in enumerate(images[::max(1, frames // 6)][:6]):
            samples.paste(frame.resize((320, 200)), (index * 320, 0))
        palette = samples.quantize(colors=256, method=Image.Quantize.MEDIANCUT)
        indexed = [frame.quantize(palette=palette, dither=Image.Dither.NONE) for frame in images]
        indexed[0].save(output / "twin-cycle.gif", save_all=True, append_images=indexed[1:], duration=100,
                        loop=0, optimize=True, disposal=1)
    print("Updated hero banners, viewport, linkage and calibration artwork", flush=True)


def load_meshes(source, resolution=0.0006):
    meshes = {name: load_stl(source / name, resolution) for name in upstream.MESHES}
    print(f"Verified {len(meshes)} public meshes; {sum(len(mesh.faces) for mesh in meshes.values()):,} render triangles")
    return meshes


def main():
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--meshes", type=Path, required=True)
    parser.add_argument("--out", type=Path, default=OUT)
    parser.add_argument("--cell", action="store_true")
    parser.add_argument("--frames", type=int, default=0)
    parser.add_argument("--animation-only", action="store_true")
    parser.add_argument("--proof", action="store_true")
    args = parser.parse_args()
    meshes = load_meshes(args.meshes)
    args.out.mkdir(parents=True, exist_ok=True)
    if args.proof:
        cell = Cell(meshes)
        cell_image(cell, 12.0, 1920, 1200).resize((1280, 800), Image.Resampling.LANCZOS).save(args.out / "animation-proof.png")
        print("Rendered animation proof", flush=True)
        return
    if args.cell:
        if not args.animation_only:
            detail_image(meshes, args.out)
        animation_cell = Cell(load_meshes(args.meshes, resolution=0.001)) if args.frames else None
        generate(Cell(meshes), args.out, args.frames, animation_cell, args.animation_only)
        return
    joints = {"Joint1": -20, "Joint2": 25, "Joint6": 50}
    joints.update(kinematics.solve_linkage(joints["Joint2"], joints["Joint6"])["joints"])
    objects = Robot(meshes).objects((0, 0, 0), 0, joints)
    objects.append((box_mesh((0.6, 0.5, 0.025), 0.008), transform((0.1, 0, -0.013)), GRAPHITE, 0.1))
    image = Renderer(1800, 1400, (0.6, -1, 0.5), (0.08, 0, 0.14), 0.48).render(objects)
    image.resize((1200, 934), Image.Resampling.LANCZOS).save(args.out / "robot-detail.png")
    print("Rendered robot-detail.png")


if __name__ == "__main__":
    main()