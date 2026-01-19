import math

import pytest

from isaac_uarm_twin import kinematics, sequences

WAYPOINTS = sorted({value for name in ("ARM1_SEQ1", "ARM1_SEQ2", "ARM2_SEQ1", "ARM2_SEQ2", "ARM3_SEQ1", "ARM3_SEQ2")
                    for kind, value in getattr(sequences, name) if kind in ("move", "catch")})

# Passive joint states that PhysX settled to in the original twin with Joint2 = 14.798 and Joint6 = 36.026
PHYSX_PASSIVE = {"Joint3": 21.51, "Joint4": -14.79, "Joint5": 14.81, "Joint7": -21.17, "Joint8": -35.99,
                 "Joint9": 36.06}


def test_loop_closure_matches_the_physics_engine():
    joints = kinematics.solve_linkage(14.798, 36.026)["joints"]
    for name, expected in PHYSX_PASSIVE.items():
        assert joints[name] == pytest.approx(expected, abs=0.5), name


@pytest.mark.parametrize("waypoint", WAYPOINTS)
def test_tool_point_reaches_the_sdk_waypoint(waypoint):
    j1, j2, j6 = kinematics.joints_from_motors(*kinematics.motor_angles(*waypoint))
    tool = kinematics.arm_points_3d(j1, j2, j6)["tool"]
    error_mm = 1000.0 * math.dist(tool, tuple(v / 1000.0 for v in waypoint))
    assert error_mm < 2.5


def test_end_effector_stays_level():
    angles = []
    for waypoint in WAYPOINTS:
        j1, j2, j6 = kinematics.joints_from_motors(*kinematics.motor_angles(*waypoint))
        angles.append(kinematics.solve_linkage(j2, j6)["effector"])
    assert max(angles) - min(angles) < 2.5


def test_loop_pins_close():
    """Each loop pin is the same point seen from both links it connects."""
    for j2, j6 in ((0.0, 20.0), (30.0, 60.0), (70.0, 100.0)):
        points = kinematics.solve_linkage(j2, j6)["points"]
        assert math.dist(points["pin_a"], kinematics.REAR_PIVOT) == pytest.approx(math.hypot(*kinematics.PIN_A_LINK5))
        assert math.dist(points["pin_b"], points["crank_pin"]) == pytest.approx(math.hypot(*kinematics.PIN_B_LINK7))
        assert math.dist(points["joint9"], points["pin_c"]) == pytest.approx(math.hypot(*kinematics.PIN_C_LINK9))


def test_loop_joint_anchors_match_the_planar_pins():
    loops = kinematics.LOOP_JOINTS
    pairs = {
        "Loop_Link5_Link4": (kinematics.PIN_A_LINK4, kinematics.PIN_A_LINK5),
        "Loop_Link7_Link3": (kinematics.PIN_B_LINK3, kinematics.PIN_B_LINK7),
        "Loop_Link9_Link4": (kinematics.PIN_C_LINK4, kinematics.PIN_C_LINK9),
    }
    for name, (pin0, pin1) in pairs.items():
        assert (loops[name]["pos0"][0], loops[name]["pos0"][2]) == pin0
        assert (loops[name]["pos1"][0], loops[name]["pos1"][2]) == pin1
        assert loops[name]["pos0"][1] == loops[name]["pos1"][1]


def test_base_rotation_follows_the_sdk_frame():
    assert kinematics.joints_from_motors(*kinematics.motor_angles(200, 0, 100))[0] == pytest.approx(0.0)
    j1 = kinematics.joints_from_motors(*kinematics.motor_angles(200, 200, 100))[0]
    assert j1 == pytest.approx(45.0)


def test_unreachable_target():
    assert kinematics.motor_angles(900, 0, 0) is None


def test_place_rotates_about_z():
    x, y, z = kinematics.place((1.0, 2.0, 3.0), 90.0, (0.1, 0.0, 0.2))
    assert (x, y, z) == pytest.approx((1.0, 2.1, 3.2))
