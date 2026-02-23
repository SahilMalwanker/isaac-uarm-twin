import xml.etree.ElementTree as ET

import numpy as np
import pytest
from PIL import Image

import render_readme as render


def test_rounded_box_retains_dimensions_and_unit_normals():
    mesh = render.box_mesh((0.2, 0.1, 0.06), radius=0.006)
    assert np.ptp(mesh.vertices, axis=0) == pytest.approx((0.2, 0.1, 0.06))
    assert np.linalg.norm(mesh.normals, axis=1) == pytest.approx(np.ones(len(mesh.vertices)))


def test_box_faces_point_outward():
    mesh = render.box_mesh((0.2, 0.1, 0.06))
    triangles, normals = mesh.triangles(np.eye(4))
    geometric = np.cross(triangles[:, 1] - triangles[:, 0], triangles[:, 2] - triangles[:, 0])
    assert np.all(np.einsum("ij,ij->i", geometric, normals[:, 0]) >= -1e-12)


def test_transform_rotates_and_translates():
    matrix = render.transform((1, 2, 3), angle=np.pi / 2)
    assert matrix @ np.array((1, 0, 0, 1)) == pytest.approx((1, 3, 3, 1))


def test_depth_test_is_independent_of_draw_order():
    mesh = render.box_mesh((0.6, 0.6, 0.2))
    near = (mesh, render.transform((0, 0, 0.2)), (0.7, 0.1, 0.1), 0)
    far = (mesh, render.transform((0, 0, -0.2)), (0.1, 0.7, 0.1), 0)
    camera = render.Renderer(80, 80, (0, -0.01, 2), (0, 0, 0), 1)
    first = np.asarray(camera.render([near, far], shadows=False))
    second = np.asarray(camera.render([far, near], shadows=False))
    assert np.array_equal(first, second)
    assert first[40, 40, 0] > first[40, 40, 1]


def test_unknown_mesh_is_rejected(tmp_path):
    candidate = tmp_path / "Base.STL"
    candidate.write_bytes(b"unverified geometry")
    with pytest.raises(ValueError, match="Unverified public mesh"):
        render.load_stl(candidate)


def test_poster_embeds_raster_and_labels_it(tmp_path):
    render.write_poster(Image.new("RGB", (40, 30)), tmp_path, "sample", "Illustrative render")
    tree = ET.parse(tmp_path / "sample.svg")
    image = tree.find("{http://www.w3.org/2000/svg}image")
    assert image.get("href").startswith("data:image/png;base64,")
    assert tree.getroot().get("viewBox") == "0 0 40 30"


def test_schematic_generator_does_not_replace_public_renders(tmp_path, monkeypatch):
    monkeypatch.setattr(render.motion, "ROOT", tmp_path)
    monkeypatch.setattr(render.motion, "OUT", tmp_path / "public")
    monkeypatch.setattr(render.motion, "PREVIEWS", tmp_path / "previews")
    render.motion.write("hero-dark.svg", "schematic")
    render.motion.write("architecture.svg", "vector diagram")
    assert not (tmp_path / "public" / "hero-dark.svg").exists()
    assert (tmp_path / "previews" / "hero-dark.svg").read_text() == "schematic"
    assert (tmp_path / "public" / "architecture.svg").is_file()