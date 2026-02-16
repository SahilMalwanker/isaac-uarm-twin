"""Generate ``scene/cell.usda`` - the robot cell of the twin, built from scratch.

Only the three uArm Swift Pro robots come from outside (URDF import, see docs/building-the-twin.md).
Table, riser, conveyors, light barriers, parts, lights, cameras, the physics scene and the OmniGraph
controllers are generated here from boxes, cylinders and spheres. Part, sensor and robot positions are
derived from the robots' real waypoints (``isaac_uarm_twin.sequences``), so every pick, place and hand-off
lines up with the programs that run on the physical cell.

    python tools/build_scene.py           # write scene/cell.usda
    python tools/build_scene.py --check   # exit 1 if scene/cell.usda is out of date
"""

from __future__ import annotations

import argparse
import math
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from isaac_uarm_twin import kinematics, sequences  # noqa: E402

SCENE_PATH = ROOT / "scene" / "cell.usda"
GRAPH_DIR = ROOT / "isaac" / "graphs"

# --- fixed layout (metres, Z up) -------------------------------------------------------------
TABLE_TOP = 0.989
TABLE_X = (-0.42, 0.92)
TABLE_Y = (-0.22, 0.64)
TABLE_THICKNESS = 0.03
RISER_X = (-0.20, 0.44)
RISER_Y = (-0.03, 0.28)
BELT_X = (-0.238, 0.462)
BELT_TOP = 1.170
BELT_THICKNESS = 0.006
BELT_SPEED = 0.15  # m/s
BEAM_Z = 1.1987
BEAM_DIR = (0.0, -1.0, 0.0)
TRAY_TOP = 1.014
HOLDER_SIZE = (0.058, 0.078, 0.040)
PCB_SIZE = (0.055, 0.075, 0.010)
ROBOT_MODELS = {"suction": "swiftpro_twin", "gripper": "swiftpro_twin_gripper"}

# Robot bases as measured on the old layout; arm 3 is solved from the hand-off (see layout()).
ARM1 = {"base": (-0.11088, 0.11747, 1.13573), "yaw": 180.0, "model": "suction"}
ARM2 = {"base": (0.34123, 0.12798, 1.13666), "yaw": 0.0, "model": "gripper"}
ARM3_YAW = -90.0
ARM3_BASE_Z = 0.99073

CONVEYORS = {
    "Conveyor1": {"y": (0.296, 0.367), "velocity": (BELT_SPEED, 0.0, 0.0)},
    "Conveyor2": {"y": (-0.116, -0.045), "velocity": (-BELT_SPEED, 0.0, 0.0)},
}

# --- colours (linear RGB): graphite and greys with NVIDIA green (#76B900) as the only accent ------
MATERIALS = {
    "Floor": {"color": (0.2, 0.2, 0.2), "roughness": 0.9},
    "TableTop": {"color": (0.42, 0.42, 0.42), "roughness": 0.6},
    "Frame": {"color": (0.015, 0.015, 0.015), "roughness": 0.45, "metallic": 0.3},
    "Aluminium": {"color": (0.6, 0.6, 0.62), "roughness": 0.35, "metallic": 0.8},
    "Belt": {"color": (0.03, 0.03, 0.03), "roughness": 0.85},
    "Accent": {"color": (0.181, 0.485, 0.0), "roughness": 0.45},
    "Beam": {"color": (0.181, 0.485, 0.0), "roughness": 0.3, "emissive": (0.36, 0.97, 0.0)},
    "Holder": {"color": (0.85, 0.85, 0.85), "roughness": 0.5},
    "Board": {"color": (0.02, 0.09, 0.04), "roughness": 0.5},
    "Chip": {"color": (0.02, 0.02, 0.02), "roughness": 0.4},
}


def _sdk_point(steps, kind, index=0):
    """``index``-th waypoint of ``kind`` ("move"/"catch") in a step list."""
    points = [value for step_kind, value in steps if step_kind == kind]
    return points[index]


def _last_point(steps):
    return [value for kind, value in steps if kind in ("move", "catch")][-1]


def layout():
    """Every derived position of the cell (pure; used by the builder, the tests and the artwork)."""
    arm1_pick = kinematics.tool_point_world(ARM1["base"], ARM1["yaw"], _sdk_point(sequences.ARM1_SEQ1, "catch", 0))
    arm1_place = kinematics.tool_point_world(ARM1["base"], ARM1["yaw"], _sdk_point(sequences.ARM1_SEQ1, "catch", 2))
    arm2_pick = kinematics.tool_point_world(ARM2["base"], ARM2["yaw"], _sdk_point(sequences.ARM2_SEQ1, "catch", 1))
    arm2_hold = kinematics.tool_point_world(ARM2["base"], ARM2["yaw"], _last_point(sequences.ARM2_SEQ1))
    arm2_place = kinematics.tool_point_world(ARM2["base"], ARM2["yaw"], _sdk_point(sequences.ARM2_SEQ2, "catch", 0))

    # Arm 3 stands where its PCB drop point (catch 3 of ARM3_SEQ1) is above arm 2's hold point.
    drop_sdk = _sdk_point(sequences.ARM3_SEQ1, "catch", 2)
    drop_offset = kinematics.place((0.0, 0.0, 0.0), ARM3_YAW, (drop_sdk[0] / 1000.0, drop_sdk[1] / 1000.0, 0.0))
    arm3 = {"base": (round(arm2_hold[0] - drop_offset[0], 5), round(arm2_hold[1] - drop_offset[1], 5), ARM3_BASE_Z),
            "yaw": ARM3_YAW, "model": "suction"}
    arm3_pick = kinematics.tool_point_world(arm3["base"], arm3["yaw"], _sdk_point(sequences.ARM3_SEQ1, "catch", 0))
    arm3_pick2 = kinematics.tool_point_world(arm3["base"], arm3["yaw"], _sdk_point(sequences.ARM3_SEQ2, "catch", 0))
    arm3_drop = kinematics.tool_point_world(arm3["base"], arm3["yaw"], drop_sdk)

    hx, hy, hz = HOLDER_SIZE
    holder = (arm1_pick[0], arm1_pick[1], TABLE_TOP + hz / 2)
    pcb = (arm3_pick[0], arm3_pick[1], TRAY_TOP + PCB_SIZE[2] / 2)
    # gripper jaws of arm 2 close at the TCP height of its pick waypoint, measured on a holder on belt 1
    jaw_z = round(arm2_pick[2] - (BELT_TOP + hz / 2), 4)

    belt1_y_far = CONVEYORS["Conveyor1"]["y"][1]
    belt2_y_near = CONVEYORS["Conveyor2"]["y"][1]
    sensors = {
        # S1 at the drop zone of arm 1, S2 stops the holder centred under arm 2's jaws,
        # S3 at arm 2's place point, S4 at the outfeed end of belt 2
        "S1": (-0.184, round(belt1_y_far - 0.0045, 4), BEAM_Z),
        "S2": (round(arm2_pick[0] + hx / 2, 4), round(belt1_y_far - 0.0045, 4), BEAM_Z),
        "S3": (round(arm2_place[0], 4), round(belt2_y_near - 0.002, 4), BEAM_Z),
        "S4": (-0.174, round(belt2_y_near - 0.003, 4), BEAM_Z),
    }
    slots = [(p[0], p[1]) for p in (arm3_pick, arm3_pick2)]
    tray_x = (min(x for x, _ in slots) - PCB_SIZE[0] / 2 - 0.012, max(x for x, _ in slots) + PCB_SIZE[0] / 2 + 0.012)
    tray_y = (min(y for _, y in slots) - PCB_SIZE[1] / 2 - 0.012, max(y for _, y in slots) + PCB_SIZE[1] / 2 + 0.012)
    return {
        "robots": {"uarm1": ARM1, "uarm2": ARM2, "uarm3": arm3},
        "waypoints": {
            "arm1_pick": arm1_pick, "arm1_place": arm1_place, "arm2_pick": arm2_pick, "arm2_hold": arm2_hold,
            "arm2_place": arm2_place, "arm3_pick": arm3_pick, "arm3_pick2": arm3_pick2, "arm3_drop": arm3_drop,
        },
        "holder": holder,
        "holder_grip": (holder[0], holder[1], holder[2] + hz / 2),
        "holder_jaw_z": jaw_z,
        "pcb": pcb,
        "pcb_grip": (pcb[0], pcb[1], pcb[2] + PCB_SIZE[2] / 2),
        "sensors": sensors,
        "tray": {"x": tray_x, "y": tray_y},
        "riser_top": min(ARM1["base"][2], ARM2["base"][2]),
    }


# --- tiny USDA writer ------------------------------------------------------------------------
def num(value):
    text = f"{value:.6f}".rstrip("0").rstrip(".")
    return "0" if text in ("-0", "") else text


def vec(values):
    return "(" + ", ".join(num(v) for v in values) + ")"


class Raw(str):
    """Property text whose continuation lines must not be indented (multi-line strings)."""


class Prim:
    def __init__(self, type_name, name, props=(), children=(), schemas=(), meta=()):
        self.type_name, self.name = type_name, name
        self.props, self.children = list(props), list(children)
        self.schemas, self.meta = list(schemas), list(meta)

    def render(self, depth=0):
        pad = "    " * depth
        head = f'{pad}def {self.type_name} "{self.name}"' if self.type_name else f'{pad}def "{self.name}"'
        meta = list(self.meta)
        if self.schemas:
            meta.insert(0, "prepend apiSchemas = [" + ", ".join(f'"{s}"' for s in self.schemas) + "]")
        lines = [head]
        if meta:
            lines[0] += " ("
            lines += [f"{pad}    {m}" for m in meta]
            lines.append(f"{pad})")
        lines.append(pad + "{")
        for prop in self.props:
            if isinstance(prop, Raw):
                lines.append(pad + "    " + prop)
            else:
                lines += [pad + "    " + line for line in prop.split("\n")]
        for index, child in enumerate(self.children):
            if index or self.props:
                lines.append("")
            lines.append(child.render(depth + 1))
        lines.append(pad + "}")
        return "\n".join(lines)


def xform_props(translate=None, rotate_z=None, scale=None):
    props, order = [], []
    if translate is not None:
        props.append(f"double3 xformOp:translate = {vec(translate)}")
        order.append("xformOp:translate")
    if rotate_z is not None:
        props.append(f"float xformOp:rotateZ = {num(rotate_z)}")
        order.append("xformOp:rotateZ")
    if scale is not None:
        props.append(f"float3 xformOp:scale = {vec(scale)}")
        order.append("xformOp:scale")
    if order:
        props.append("uniform token[] xformOpOrder = [" + ", ".join(f'"{o}"' for o in order) + "]")
    return props


def matrix_props(rows):
    text = ", ".join("(" + ", ".join(num(v) for v in row) + ")" for row in rows)
    return [f"matrix4d xformOp:transform = ( {text} )", 'uniform token[] xformOpOrder = ["xformOp:transform"]']


def bind(material):
    return f"rel material:binding = </World/Looks/{material}>"


def collider(schemas, props, offsets=True):
    schemas.append("PhysicsCollisionAPI")
    if offsets:
        schemas.append("PhysxCollisionAPI")
        props += ["float physxCollision:contactOffset = 0.002", "float physxCollision:restOffset = 0"]


def box(name, center, size, material=None, collision=False, invisible=False, extra_schemas=(), extra_props=()):
    schemas, props = list(extra_schemas), ["double size = 1"]
    if collision:
        collider(schemas, props)
    if material:
        schemas.append("MaterialBindingAPI")
        props.append(bind(material))
    if invisible:
        props.append('token visibility = "invisible"')
    props += list(extra_props)
    props += xform_props(translate=center, scale=size)
    return Prim("Cube", name, props, schemas=schemas)


def box_span(name, x, y, z, material=None, collision=False):
    center = ((x[0] + x[1]) / 2, (y[0] + y[1]) / 2, (z[0] + z[1]) / 2)
    return box(name, center, (x[1] - x[0], y[1] - y[0], z[1] - z[0]), material, collision)


def cylinder(name, center, radius, height, axis, material):
    props = [f"double radius = {num(radius)}", f"double height = {num(height)}", f'uniform token axis = "{axis}"',
             bind(material)] + xform_props(translate=center)
    return Prim("Cylinder", name, props, schemas=["MaterialBindingAPI"])


def sphere(name, center, radius, material):
    props = [f"double radius = {num(radius)}", bind(material)] + xform_props(translate=center)
    return Prim("Sphere", name, props, schemas=["MaterialBindingAPI"])


def group(name, children, translate=None, rotate_z=None, type_name="Xform"):
    return Prim(type_name, name, xform_props(translate, rotate_z), children)


def look_at(eye, target, up=(0.0, 0.0, 1.0)):
    """USD row-major transform for a camera/light at ``eye`` looking at ``target`` (-Z forward)."""
    def sub(a, b):
        return tuple(x - y for x, y in zip(a, b))

    def cross(a, b):
        return (a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0])

    def unit(a):
        length = math.sqrt(sum(x * x for x in a))
        return tuple(x / length for x in a)

    forward = unit(sub(target, eye))
    right = unit(cross(forward, up))
    true_up = cross(right, forward)
    back = tuple(-x for x in forward)
    return [(*right, 0.0), (*true_up, 0.0), (*back, 0.0), (*eye, 1.0)]


# --- scene parts -----------------------------------------------------------------------------
def looks():
    children = []
    for name, spec in MATERIALS.items():
        shader = [
            'uniform token info:id = "UsdPreviewSurface"',
            f"color3f inputs:diffuseColor = {vec(spec['color'])}",
            f"float inputs:roughness = {num(spec.get('roughness', 0.5))}",
            f"float inputs:metallic = {num(spec.get('metallic', 0.0))}",
        ]
        if "emissive" in spec:
            shader.append(f"color3f inputs:emissiveColor = {vec(spec['emissive'])}")
        shader.append("token outputs:surface")
        surface = f"token outputs:surface.connect = </World/Looks/{name}/Shader.outputs:surface>"
        children.append(Prim("Material", name, [surface], [Prim("Shader", "Shader", shader)]))
    friction = ["float physics:dynamicFriction = 0.8", "float physics:restitution = 0",
                "float physics:staticFriction = 0.9"]
    children.append(Prim("Material", "BeltRubber", friction, schemas=["PhysicsMaterialAPI"]))
    return Prim("Scope", "Looks", children=children)


def environment():
    sun = Prim("DistantLight", "Sun", ["float inputs:angle = 1", "float inputs:intensity = 2500"]
               + matrix_props(look_at((1.2, -1.0, 2.6), (0.0, 0.0, 0.0))))
    sky = Prim("DomeLight", "Sky", ["float inputs:intensity = 800", 'token inputs:texture:format = "latlong"'])
    floor = box("Floor", (0.25, 0.2, -0.01), (8.0, 8.0, 0.02), "Floor", collision=True)
    overview = Prim("Camera", "Overview", ["float2 clippingRange = (0.01, 100)", "float focalLength = 18"]
                    + matrix_props(look_at((1.55, -1.05, 1.85), (0.22, 0.2, 1.08))))
    top = Prim("Camera", "Top", ["float2 clippingRange = (0.01, 100)", "float focalLength = 18"]
               + matrix_props(look_at((0.25, 0.2, 2.7), (0.25, 0.2, 1.0), up=(0.0, 1.0, 0.0))))
    return group("Environment", [sun, sky, floor, overview, top])


def table(cell):
    x0, x1 = TABLE_X
    y0, y1 = TABLE_Y
    legs = []
    for index, (lx, ly) in enumerate(((x0 + 0.05, y0 + 0.05), (x1 - 0.05, y0 + 0.05), (x1 - 0.05, y1 - 0.05),
                                      (x0 + 0.05, y1 - 0.05)), start=1):
        legs.append(box_span(f"Leg{index}", (lx - 0.025, lx + 0.025), (ly - 0.025, ly + 0.025),
                             (0.0, TABLE_TOP - TABLE_THICKNESS), "Frame"))
    top = box_span("Top", TABLE_X, TABLE_Y, (TABLE_TOP - TABLE_THICKNESS, TABLE_TOP), "TableTop", collision=True)
    riser = box_span("Riser", RISER_X, RISER_Y, (TABLE_TOP, cell["riser_top"]), "Frame", collision=True)
    arm3 = cell["robots"]["uarm3"]["base"]
    plate = box_span("MountPlate3", (arm3[0] - 0.07, arm3[0] + 0.07), (arm3[1] - 0.07, arm3[1] + 0.07),
                     (TABLE_TOP, arm3[2]), "Accent")
    return group("Table", [top, riser, plate] + legs)


def conveyor(name, y_range, velocity, sensors_on_far_side):
    y0, y1 = y_range
    x0, x1 = BELT_X
    yc = (y0 + y1) / 2
    bed_bottom = BELT_TOP - 0.035
    belt_props = ["bool physics:kinematicEnabled = 1",
                  f"vector3f physxSurfaceVelocity:surfaceVelocity = {vec(velocity)}",
                  "bool physxSurfaceVelocity:surfaceVelocityEnabled = 0",
                  "bool physxSurfaceVelocity:surfaceVelocityLocalSpace = 0",
                  "rel material:binding:physics = </World/Looks/BeltRubber>"]
    belt = box_span("Belt", BELT_X, y_range, (BELT_TOP - BELT_THICKNESS, BELT_TOP), "Belt", collision=True)
    belt.schemas = ["PhysicsRigidBodyAPI"] + belt.schemas + ["PhysxSurfaceVelocityAPI"]
    belt.props = belt.props[:1] + belt_props + belt.props[1:]
    parts = [
        belt,
        box_span("Bed", BELT_X, y_range, (bed_bottom, BELT_TOP - BELT_THICKNESS), "Aluminium"),
        box_span("RailNear", BELT_X, (y0 - 0.006, y0), (bed_bottom, BELT_TOP - 0.002), "Aluminium"),
        box_span("RailFar", BELT_X, (y1, y1 + 0.006), (bed_bottom, BELT_TOP - 0.002), "Aluminium"),
        cylinder("RollerIn", (x0, yc, BELT_TOP - 0.016), 0.016, (y1 - y0) + 0.004, "Y", "Aluminium"),
        cylinder("RollerOut", (x1, yc, BELT_TOP - 0.016), 0.016, (y1 - y0) + 0.004, "Y", "Aluminium"),
    ]
    downstream = x1 if velocity[0] > 0 else x0
    motor_y = y1 + 0.03 if sensors_on_far_side else y0 - 0.03
    parts.append(box("Motor", (downstream, motor_y, BELT_TOP - 0.02), (0.045, 0.045, 0.05), "Accent"))
    for index, lx in enumerate((x0 + 0.03, x1 - 0.03), start=1):
        for side, ly in (("Near", y0 + 0.008), ("Far", y1 - 0.008)):
            parts.append(box_span(f"Leg{index}{side}", (lx - 0.01, lx + 0.01), (ly - 0.008, ly + 0.008),
                                  (TABLE_TOP, bed_bottom), "Frame"))
    return group(name, parts)


def light_barrier(name, origin, outward):
    """Emitter housing on the +Y side of a belt; the ``Beam`` sphere marks the sweep origin."""
    x, y, z = origin
    housing_y = y + outward * 0.018
    return group(name, [
        box("Housing", (x, housing_y, z), (0.016, 0.022, 0.03), "Accent"),
        box_span("Post", (x - 0.004, x + 0.004), (housing_y - 0.004, housing_y + 0.004), (BELT_TOP - 0.035, z - 0.015),
                 "Aluminium"),
        sphere("Beam", origin, 0.003, "Beam"),
    ])


def rigid_part(name, center, mass, children):
    props = [f"float physics:mass = {num(mass)}", "float physxRigidBody:angularDamping = 0.1",
             "float physxRigidBody:linearDamping = 0.1", "vector3f physics:velocity = (0, 0, 0)"]
    props += xform_props(translate=center)
    return Prim("Xform", name, props, children,
                schemas=["PhysicsRigidBodyAPI", "PhysicsMassAPI", "PhysxRigidBodyAPI"])


def marker(name, local):
    return group(name, [], translate=local)


def parts(cell):
    hx, hy, hz = HOLDER_SIZE
    holder = rigid_part("Holder", cell["holder"], 0.1, [
        box("Body", (0, 0, 0), HOLDER_SIZE, "Holder", collision=True),
        box("Pocket", (0, 0, hz / 2 + 0.0004), (hx - 0.012, hy - 0.012, 0.001), "Accent"),
        marker("GripPoint", (0, 0, hz / 2)),
        marker("JawPoint", (0, 0, cell["holder_jaw_z"])),
    ])
    px, py, pz = PCB_SIZE
    pcb = rigid_part("PCB", cell["pcb"], 0.1, [
        box("Collider", (0, 0, 0), PCB_SIZE, collision=True, invisible=True),
        box("Board", (0, 0, -pz / 2 + 0.0008), (px, py, 0.0016), "Board"),
        box("Chip1", (0.008, 0.012, -pz / 2 + 0.0031), (0.022, 0.022, 0.003), "Chip"),
        box("Chip2", (-0.014, -0.02, -pz / 2 + 0.0026), (0.012, 0.016, 0.002), "Chip"),
        box("Connector", (0.0, -0.031, -pz / 2 + 0.0046), (0.04, 0.007, 0.006), "Chip"),
        marker("GripPoint", (0, 0, pz / 2)),
    ])
    tray = box_span("Tray", cell["tray"]["x"], cell["tray"]["y"], (TABLE_TOP, TRAY_TOP), "Frame", collision=True)
    return group("Parts", [holder, pcb, tray])


def robot(name, spec):
    model = ROBOT_MODELS[spec["model"]]
    ref = Prim("", "swiftpro", meta=[f"prepend references = @./robot/{model}/{model}.usd@"])
    return group(name, [ref], translate=spec["base"], rotate_z=spec["yaw"])


# --- OmniGraph controllers -------------------------------------------------------------------
def configure(script, **values):
    """Replace assignments inside the ``# --- config ---`` block of a graph script."""
    start = script.index("# --- config ---\n")
    end = script.index("# --- end config ---")
    lines = script[start:end].splitlines(keepends=True)
    missing = set(values)
    for index, line in enumerate(lines):
        name = line.split("=", 1)[0].strip() if "=" in line and not line.startswith("#") else None
        if name in values:
            lines[index] = f"{name} = {values[name]!r}\n"
            missing.discard(name)
    if missing:
        raise KeyError(f"not in config block: {sorted(missing)}")
    return script[:start] + "".join(lines) + script[end:]


def script_graph(name, script):
    if '"""' in script or "\\" in script:
        raise ValueError(f"{name}: graph scripts must not contain triple quotes or backslashes")
    graph = f"/World/Graphs/{name}"
    execution = "(\n    customData = {\n        bool isExecution = 1\n    }\n)"
    tick = Prim("OmniGraphNode", "tick", [
        "custom uint inputs:framePeriod = 0",
        "custom bool inputs:onlyPlayback = 1",
        'token node:type = "omni.graph.action.OnTick"',
        "int node:typeVersion = 2",
        f"custom uint outputs:tick {execution}",
    ])
    node = Prim("OmniGraphNode", "script", [
        f"custom uint inputs:execIn {execution}",
        f"prepend uint inputs:execIn.connect = <{graph}/tick.outputs:tick>",
        Raw('custom string inputs:script = """\n' + script.rstrip("\n") + '\n"""'),
        'custom token inputs:scriptPath = ""',
        "custom bool inputs:usePath = 0",
        'token node:type = "omni.graph.scriptnode.ScriptNode"',
        "int node:typeVersion = 2",
        f"custom uint outputs:execOut {execution}",
        "custom bool state:omni_initialized",
    ])
    return Prim("OmniGraph", name, [
        'token evaluationMode = "Automatic"',
        'token evaluator:type = "execution"',
        'token fabricCacheBacking = "StageWithoutHistory"',
        "int2 fileFormatVersion = (1, 9)",
        'token pipelineStage = "pipelineStageSimulation"',
    ], [tick, node])


def graphs(cell):
    conveyor_src = (GRAPH_DIR / "conveyor_controller.py").read_text(encoding="utf-8")
    grab_src = (GRAPH_DIR / "vacuum_autograb.py").read_text(encoding="utf-8")
    conveyor_script = configure(
        conveyor_src,
        PARTS=("/World/Cell/Parts/Holder",),
        BELT1_PRIM="/World/Cell/Conveyor1/Belt", BELT1_VEL=CONVEYORS["Conveyor1"]["velocity"],
        BELT2_PRIM="/World/Cell/Conveyor2/Belt", BELT2_VEL=CONVEYORS["Conveyor2"]["velocity"],
        S1_PRIM="/World/Cell/Sensors/S1/Beam", S2_PRIM="/World/Cell/Sensors/S2/Beam",
        S3_PRIM="/World/Cell/Sensors/S3/Beam", S4_PRIM="/World/Cell/Sensors/S4/Beam",
        BEAM_DIR=BEAM_DIR,
    )

    def tool(robot_id, tool_name, arm, hold_link, part, point, threshold, drop):
        link8 = f"/World/{arm}/swiftpro/Link8"
        return configure(
            grab_src, ROBOT_ID=robot_id, TOOL_NAME=tool_name, LINK8_PATH=link8,
            HOLD_LINK_PATH=f"/World/{arm}/swiftpro/{hold_link}", ATTACH_PATH=f"{link8}/GripperFrame/AttachPoint0",
            OBJ_PATH=f"/World/Cell/Parts/{part}", GRIPPOINT_PATH=f"/World/Cell/Parts/{part}/{point}",
            JOINT_PATH=f"{link8}/GripperFrame/PickAsIsFixedJoint", THRESH_MM=threshold, DROP_VELOCITY=drop,
        )

    return Prim("Scope", "Graphs", children=[
        script_graph("ConveyorController", conveyor_script),
        script_graph("VacuumArm1", tool("UARM1", "vacuum", "uarm1", "Link8", "Holder", "GripPoint", 10.0,
                                        (0.0, 0.0, -0.005))),
        script_graph("GripperArm2", tool("UARM2", "gripper", "uarm2", "GripperExt", "Holder", "JawPoint", 15.0,
                                         (0.0, 0.0, -0.005))),
        script_graph("VacuumArm3", tool("UARM3", "vacuum", "uarm3", "Link8", "PCB", "GripPoint", 10.0,
                                        (0.0, 0.0, -0.02))),
    ])


def build_usda(cell=None):
    cell = cell or layout()
    sensors = cell["sensors"]
    cell_prim = group("Cell", [
        table(cell),
        conveyor("Conveyor1", CONVEYORS["Conveyor1"]["y"], CONVEYORS["Conveyor1"]["velocity"], True),
        conveyor("Conveyor2", CONVEYORS["Conveyor2"]["y"], CONVEYORS["Conveyor2"]["velocity"], False),
        group("Sensors", [light_barrier(name, sensors[name], 1.0) for name in ("S1", "S2", "S3", "S4")]),
        parts(cell),
    ])
    physics = Prim("PhysicsScene", "PhysicsScene", [
        "vector3f physics:gravityDirection = (0, 0, -1)",
        "float physics:gravityMagnitude = 9.81",
        "bool physxScene:enableSceneQuerySupport = 1",
    ], schemas=["PhysxSceneAPI"])
    robots = [robot(name, spec) for name, spec in cell["robots"].items()]
    world = Prim("Xform", "World", children=[looks(), physics, environment(), cell_prim] + robots + [graphs(cell)])
    header = "\n".join([
        "#usda 1.0",
        "(",
        '    defaultPrim = "World"',
        '    doc = """Isaac uArm Twin - robot cell, generated by tools/build_scene.py (do not edit by hand).',
        "    Robots are referenced from ./robot (import assets/swiftpro/*.urdf with the Isaac Sim URDF importer).",
        '    Everything else is built from primitive shapes."""',
        "    endTimeCode = 1000000",
        "    metersPerUnit = 1",
        "    startTimeCode = 0",
        "    timeCodesPerSecond = 60",
        '    upAxis = "Z"',
        ")",
    ])
    return header + "\n\n" + world.render() + "\n"


def main(argv=None):
    parser = argparse.ArgumentParser(description="Generate scene/cell.usda")
    parser.add_argument("--check", action="store_true", help="exit 1 if the committed scene is out of date")
    parser.add_argument("--out", type=Path, default=SCENE_PATH)
    args = parser.parse_args(argv)
    text = build_usda()
    if args.check:
        current = args.out.read_text(encoding="utf-8") if args.out.is_file() else ""
        if current != text:
            print(f"{args.out} is out of date - run python tools/build_scene.py")
            return 1
        print(f"{args.out} is up to date")
        return 0
    args.out.parent.mkdir(parents=True, exist_ok=True)
    args.out.write_text(text, encoding="utf-8", newline="\n")
    print(f"wrote {args.out} ({len(text.splitlines())} lines)")
    return 0


if __name__ == "__main__":
    sys.exit(main())
