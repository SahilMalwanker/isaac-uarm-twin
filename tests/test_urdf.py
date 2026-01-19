import math
import xml.etree.ElementTree as ET
from pathlib import Path

import pytest

import build_scene
import fetch_swiftpro_meshes
import follower
from isaac_uarm_twin import kinematics

ASSETS = Path(__file__).resolve().parents[1] / "assets" / "swiftpro"
URDFS = {"swiftpro_twin": ASSETS / "swiftpro_twin.urdf", "swiftpro_twin_gripper": ASSETS / "swiftpro_twin_gripper.urdf"}


def load(name):
    return ET.parse(URDFS[name]).getroot()


def joints(root):
    out = {}
    for joint in root.findall("joint"):
        origin = joint.find("origin")
        limit = joint.find("limit")
        out[joint.get("name")] = {
            "parent": joint.find("parent").get("link"),
            "child": joint.find("child").get("link"),
            "xyz": tuple(float(v) for v in origin.get("xyz").split()),
            "axis": tuple(float(v) for v in joint.find("axis").get("xyz").split()),
            "limits": (math.degrees(float(limit.get("lower"))), math.degrees(float(limit.get("upper")))),
        }
    return out


@pytest.mark.parametrize("name", sorted(URDFS))
def test_robot_names_match_the_scene_references(name):
    assert load(name).get("name") == name
    assert name in build_scene.ROBOT_MODELS.values()


@pytest.mark.parametrize("name", sorted(URDFS))
def test_meshes_are_the_fetched_upstream_files(name):
    for mesh in load(name).iter("mesh"):
        filename = mesh.get("filename")
        assert filename.startswith("meshes/")
        assert filename.split("/", 1)[1] in fetch_swiftpro_meshes.MESHES


@pytest.mark.parametrize("name", sorted(URDFS))
def test_joint_tree(name):
    found = joints(load(name))
    links = {link.get("name") for link in load(name).findall("link")}
    children = [spec["child"] for spec in found.values()]
    assert len(children) == len(set(children)) == len(links) - 1  # a tree: every link but Base has one parent
    for joint_name in follower.SERVO_TO_DOF.values():
        assert joint_name in found


def test_linkage_geometry_matches_the_kinematics():
    j = joints(load("swiftpro_twin"))
    assert j["Joint1"]["xyz"][2] == pytest.approx(kinematics.BASE_HEIGHT)
    shoulder = (j["Joint2"]["xyz"][0], kinematics.BASE_HEIGHT + j["Joint2"]["xyz"][2])
    assert shoulder == pytest.approx(kinematics.SHOULDER)
    rear = (j["Joint5"]["xyz"][0], kinematics.BASE_HEIGHT + j["Joint5"]["xyz"][2])
    assert rear == pytest.approx(kinematics.REAR_PIVOT)
    assert (j["Joint3"]["xyz"][0], j["Joint3"]["xyz"][2]) == pytest.approx(kinematics.LOWER_ARM)
    assert (j["Joint7"]["xyz"][0], j["Joint7"]["xyz"][2]) == pytest.approx(kinematics.CRANK)
    assert (j["Joint8"]["xyz"][0], j["Joint8"]["xyz"][2]) == pytest.approx(kinematics.FOREARM)
    assert (j["Joint9"]["xyz"][0], j["Joint9"]["xyz"][2]) == pytest.approx(kinematics.WRIST_TO_JOINT9)


def test_driven_joint_limits_match_the_follower():
    j = joints(load("swiftpro_twin_gripper"))
    for name in ("Joint1", "Joint6", "JointGripperExt"):
        assert j[name]["limits"] == pytest.approx(follower.JOINT_LIMITS_DEG[name], abs=0.01)
    assert j["Joint2"]["limits"] == pytest.approx((-29.972, 89.954), abs=0.01)


def test_gripper_variant_only_adds_the_wrist():
    suction, gripper = joints(load("swiftpro_twin")), joints(load("swiftpro_twin_gripper"))
    assert set(gripper) - set(suction) == {"JointGripperExt"}
    wrist = gripper["JointGripperExt"]
    assert wrist["parent"] == "Link8" and wrist["axis"] == (0.0, 0.0, 1.0)
    for name in suction:
        assert suction[name] == gripper[name]
