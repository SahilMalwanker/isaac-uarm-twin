"""uArm Swift Pro kinematics for scene layout, tests and the README artwork.

Two models are combined:

* the firmware inverse kinematics (Cartesian SDK coordinates -> motor angles), matching the equations
  in the uArm Swift Pro ROS package (uArm-Developer/RosForSwiftAndSwiftPro, BSD header in swiftpro/src), and
* the planar linkage of the Swift Pro URDF: shoulder (Joint2) and elbow crank (Joint6) are driven,
  three closed loops (rear parallelogram, elbow drive rod, level-keeping parallelogram) decide every
  passive joint. ``solve_linkage`` closes those loops the same way PhysX does in the twin.

Lengths are metres in the robot base frame (x forward, z up) unless a name says ``_mm``.
"""

import math

# --- firmware inverse kinematics (millimetres) ------------------------------------------------
FW_SHOULDER_HEIGHT_MM = 106.6
FW_SHOULDER_OFFSET_MM = 13.2
FW_LOWER_ARM_MM = 142.07
FW_UPPER_ARM_MM = 158.81
FW_Z_OFFSET_MM = 74.55
FW_FRONT_MM = 56.55

# --- linkage geometry from the URDF (x, z), metres -------------------------------------------
SHOULDER = (0.0132, 0.1056)  # Joint2 and Joint6 axis in the base frame
REAR_PIVOT = (-0.0215, 0.12231)  # Joint5 axis in the base frame
LOWER_ARM = (0.0, 0.14207)  # Link2: shoulder -> elbow (Joint3 / Joint4)
CRANK = (-0.0455, -0.00301)  # Link6: Joint6 -> Joint7
FOREARM = (0.15852, 0.0)  # Link3: elbow -> wrist (Joint8)
# loop-closure pins, each expressed in the frame of both links it connects
PIN_A_LINK4 = (-0.0347, 0.01661)
PIN_A_LINK5 = (0.0, 0.14197)
PIN_B_LINK3 = (-0.04501, -0.00271)
PIN_B_LINK7 = (0.00049, 0.14237)
PIN_C_LINK4 = (0.02765, 0.02708)
PIN_C_LINK9 = (-0.15828, 0.00005)
WRIST_TO_JOINT9 = (0.02741, 0.02703)  # Link8: Joint8 -> Joint9
TOOL_POINT = (0.056381, -0.073327)  # Link8: suction tip (AttachPoint0 of the rig)
BASE_HEIGHT = 0.0723  # Joint1 above the base origin

# 3D anchors of the three loop joints authored by isaac/tools/rig_swiftpro.py: (x, y, z) per link
LOOP_JOINTS = {
    "Loop_Link5_Link4": {"parent": "Link5", "body0": "Link4", "body1": "Link5",
                         "pos0": (-0.0347, 0.01619, 0.01661), "pos1": (0.0, 0.01619, 0.14197)},
    "Loop_Link7_Link3": {"parent": "Link7", "body0": "Link3", "body1": "Link7",
                         "pos0": (-0.04501, -0.00322, -0.00271), "pos1": (0.00049, -0.00322, 0.14237)},
    "Loop_Link9_Link4": {"parent": "Link9", "body0": "Link4", "body1": "Link9",
                         "pos0": (0.02765, 0.01614, 0.02708), "pos1": (-0.15828, 0.01614, 0.00005)},
}


def wrap(deg):
    """Wrap an angle to [-180, 180)."""
    return (deg + 180.0) % 360.0 - 180.0


def motor_angles(x, y, z):
    """Firmware inverse kinematics: SDK coordinates (mm) -> (base, left, right) motor angles in degrees.

    Returns ``None`` for unreachable targets.
    """
    z_in = (z + FW_Z_OFFSET_MM - FW_SHOULDER_HEIGHT_MM) / FW_LOWER_ARM_MM
    x = max(x, 0.1)
    if y == 0:
        base = 90.0
    elif y < 0:
        base = -math.degrees(math.atan(x / y))
    else:
        base = 180.0 - math.degrees(math.atan(x / y))
    reach = math.hypot(x, y)
    x_in = (reach - FW_SHOULDER_OFFSET_MM - FW_FRONT_MM) / FW_LOWER_ARM_MM
    ratio = FW_UPPER_ARM_MM / FW_LOWER_ARM_MM
    span = math.hypot(z_in, x_in)
    if span == 0:
        return None
    phi = math.degrees(math.atan2(z_in, x_in))
    cos_right = (span * span + ratio * ratio - 1) / (2 * ratio * span)
    cos_left = (span * span + 1 - ratio * ratio) / (2 * span)
    if not (-1.0 <= cos_right <= 1.0 and -1.0 <= cos_left <= 1.0):
        return None
    left = math.degrees(math.acos(cos_left)) + phi
    right = math.degrees(math.acos(cos_right)) - phi
    return base, left, right


def _rot(v, deg):
    """Rotate (x, z) about +Y (URDF joint axis) by ``deg``."""
    p = math.radians(deg)
    c, s = math.cos(p), math.sin(p)
    return (v[0] * c + v[1] * s, -v[0] * s + v[1] * c)


def _add(a, b):
    return (a[0] + b[0], a[1] + b[1])


def _sub(a, b):
    return (a[0] - b[0], a[1] - b[1])


def _norm(v):
    return math.hypot(v[0], v[1])


def _angle_to(v, w):
    """Rotation (degrees, about +Y) that turns direction ``v`` into direction ``w``."""
    return wrap(math.degrees(math.atan2(v[1], v[0]) - math.atan2(w[1], w[0])))


def _circle_hit(c0, r0, c1, r1, guess):
    d = _norm(_sub(c1, c0))
    a = (r0 * r0 - r1 * r1 + d * d) / (2 * d)
    h = math.sqrt(max(r0 * r0 - a * a, 0.0))
    ex = ((c1[0] - c0[0]) / d, (c1[1] - c0[1]) / d)
    m = (c0[0] + a * ex[0], c0[1] + a * ex[1])
    p1 = (m[0] - h * ex[1], m[1] + h * ex[0])
    p2 = (m[0] + h * ex[1], m[1] - h * ex[0])
    return p1 if _norm(_sub(p1, guess)) <= _norm(_sub(p2, guess)) else p2


def solve_linkage(j2, j6):
    """Close the three loops for shoulder ``j2`` and elbow crank ``j6`` (degrees).

    Returns the side-plane points (x, z) and the passive joint angles Joint3, 4, 5, 7, 8, 9.
    """
    s = SHOULDER
    elbow = _add(s, _rot(LOWER_ARM, j2))
    pin_a = _circle_hit(elbow, _norm(PIN_A_LINK4), REAR_PIVOT, _norm(PIN_A_LINK5), _add(elbow, _sub(REAR_PIVOT, s)))
    abs4 = _angle_to(PIN_A_LINK4, _sub(pin_a, elbow))
    abs5 = _angle_to(PIN_A_LINK5, _sub(pin_a, REAR_PIVOT))
    crank_pin = _add(s, _rot(CRANK, j6))
    pin_b = _circle_hit(crank_pin, _norm(PIN_B_LINK7), elbow, _norm(PIN_B_LINK3), _add(elbow, _rot(PIN_B_LINK3, j6)))
    abs3 = _angle_to(PIN_B_LINK3, _sub(pin_b, elbow))
    abs7 = _angle_to(PIN_B_LINK7, _sub(pin_b, crank_pin))
    wrist = _add(elbow, _rot(FOREARM, abs3))
    pin_c = _add(elbow, _rot(PIN_C_LINK4, abs4))
    joint9 = _circle_hit(wrist, _norm(WRIST_TO_JOINT9), pin_c, _norm(PIN_C_LINK9),
                         _add(wrist, _rot(WRIST_TO_JOINT9, abs4)))
    abs8 = _angle_to(WRIST_TO_JOINT9, _sub(joint9, wrist))
    abs9 = _angle_to(PIN_C_LINK9, _sub(pin_c, joint9))
    tool = _add(wrist, _rot(TOOL_POINT, abs8))
    return {
        "points": {
            "base": (0.0, BASE_HEIGHT), "shoulder": s, "rear_pivot": REAR_PIVOT, "elbow": elbow, "pin_a": pin_a,
            "crank_pin": crank_pin, "pin_b": pin_b, "wrist": wrist, "pin_c": pin_c, "joint9": joint9, "tool": tool,
        },
        "joints": {
            "Joint3": wrap(abs3 - j2), "Joint4": wrap(abs4 - j2), "Joint5": abs5, "Joint7": wrap(abs7 - j6),
            "Joint8": wrap(abs8 - abs3), "Joint9": wrap(abs9 - abs8),
        },
        "forearm": abs3,
        "effector": abs8,
    }


def joints_from_motors(base, left, right):
    """Nominal URDF joints (Joint1, Joint2, Joint6) for firmware motor angles, in degrees.

    Uses the mapping of the uArm ROS package (forearm angle follows the right motor) and solves the
    elbow crank so that the closed loop reproduces that forearm angle.
    """
    j1 = base - 90.0
    j2 = 90.0 - left
    forearm = right + 0.09
    j6 = forearm
    for _ in range(12):
        j6 += wrap(forearm - solve_linkage(j2, j6)["forearm"])
    return j1, j2, j6


def arm_points_3d(j1, j2, j6):
    """3D points of the linkage in the robot base frame (metres) for Joint1/2/6 in degrees."""
    plane = solve_linkage(j2, j6)["points"]
    yaw = math.radians(j1)
    c, s = math.cos(yaw), math.sin(yaw)
    return {name: (x * c, x * s, z) for name, (x, z) in plane.items()}


def place(base_xyz, yaw_deg, local_xyz):
    """Robot base frame -> world (Z-up, metres) for a robot standing at ``base_xyz`` turned by ``yaw_deg``."""
    yaw = math.radians(yaw_deg)
    c, s = math.cos(yaw), math.sin(yaw)
    x, y, z = local_xyz
    return (base_xyz[0] + x * c - y * s, base_xyz[1] + x * s + y * c, base_xyz[2] + z)


def tool_point_world(base_xyz, yaw_deg, sdk_xyz_mm):
    """World position of the suction tip for an SDK waypoint (x, y, z in mm, robot frame).

    Through the firmware IK and the URDF linkage the tip lands on the SDK point itself, within about
    2 mm (checked in ``tests/test_kinematics.py``), so the waypoint is simply converted to metres.
    """
    x, y, z = sdk_xyz_mm
    return place(base_xyz, yaw_deg, (x / 1000.0, y / 1000.0, z / 1000.0))
