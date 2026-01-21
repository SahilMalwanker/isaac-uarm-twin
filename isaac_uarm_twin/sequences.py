"""Waypoint sequences of the three arms as run by the standalone cell demo.

Coordinates are uArm SDK millimetres in each robot's own frame (x forward, y left, z up). Every
waypoint was tuned on the physical cell. ``move`` uses the transfer speed, ``catch`` the slow
approach speed used near parts; ``wait`` pauses let vacuum and gripper settle.
"""

import time

MOVE_SPEED = 5_000_000
CATCH_SPEED = 100_000


def move(x, y, z):
    return ("move", (x, y, z))


def catch(x, y, z):
    return ("catch", (x, y, z))


def wait(seconds):
    return ("wait", seconds)


def gripper(closed):
    return ("gripper", closed)


def wrist(angle):
    return ("wrist", angle)


# Arm 1 (vacuum): picks a PCB holder from the supply and sets it on conveyor 1
ARM1_SEQ1 = [
    move(180, 0, 0), wait(1),
    move(180, -40, -80), wait(1),
    catch(180, -40, -115), wait(1),
    catch(180, -40, -50), wait(1),
    move(180, -40, 200), wait(1),
    move(22, -215, 185), wait(1),
    move(50, -200, 100), wait(1),
    catch(50, -200, 70), wait(1),
    move(50, -200, 100), wait(1),
    move(22, -215, 185), wait(1),
    move(180, -40, 200),
]

# Arm 1, alternate cycle: the second holder slot
ARM1_SEQ2 = [
    move(180, 0, 0), wait(1),
    move(180, 60, -80), wait(1),
    catch(180, 60, -115), wait(1),
    catch(180, 60, -50), wait(1),
    move(180, 60, 200), wait(1),
    move(22, -216, 185), wait(1),
    move(50, -216, 100), wait(1),
    catch(50, -216, 80), wait(1),
    move(50, -216, 100), wait(1),
    move(22, -216, 185), wait(1),
    move(180, 60, 200),
]

# Arm 2 (gripper + wrist): takes the holder at the end of conveyor 1 and holds it for assembly
ARM2_SEQ1 = [
    move(160, 0, 80), gripper(False), wrist(180), wait(1),
    move(80, 210, 60), wait(1),
    catch(80, 210, 43), wait(1),
    catch(5, 190, 40), wait(2),
    gripper(True), wait(2),
    move(5, 200, 100), wait(1),
    move(190, 210, 120), wrist(150), wait(1),
    wrist(120), move(160, 65, 200), wait(1),
    wrist(90), move(210, 85, -20), wait(1),
    wrist(50), catch(180, 72, -40),
]

# Arm 2: sets the finished assembly on conveyor 2
ARM2_SEQ2 = [
    move(70, -200, 80), wrist(0), wait(1),
    catch(20, -205, 50), wait(2),
    gripper(False), wait(2),
    catch(50, -205, 55), wait(1),
    catch(70, -200, 80), wait(1),
    move(160, 0, 100), wrist(90),
]

# Arm 3 (vacuum): brings a PCB from the tray onto the held holder
ARM3_SEQ1 = [
    move(130, 0, 120), wait(1),
    move(205, -30, 120), wait(1),
    move(216, -38, 50), wait(1),
    catch(216, -38, 35), wait(1),
    catch(216, -38, 50), wait(1),
    move(205, -30, 120), wait(1),
    move(273, -150, 180), wait(1),
    catch(270, -160, 160), wait(1),
    catch(273, -150, 180), wait(1),
    move(165, -5, 180),
]

# Arm 3, alternate cycle: the second PCB slot
ARM3_SEQ2 = [
    move(205, -30, 120), wait(1),
    move(215, 25, 50), wait(1),
    catch(215, 25, 35), wait(1),
    move(215, 25, 50), wait(1),
    move(205, -30, 120), wait(1),
    move(273, -150, 180), wait(1),
    catch(270, -160, 160), wait(1),
    catch(273, -150, 180), wait(1),
    move(165, -5, 180),
]

PHASES = [
    ("Phase 1: arm1 seq1 -> arm2 seq1 -> arm3 seq1 -> arm2 seq2",
     [("arm1", ARM1_SEQ1), ("arm2", ARM2_SEQ1), ("arm3", ARM3_SEQ1), ("arm2", ARM2_SEQ2)]),
    ("Phase 2: arm1 seq2 -> arm2 seq1 -> arm3 seq2 -> arm2 seq2",
     [("arm1", ARM1_SEQ2), ("arm2", ARM2_SEQ1), ("arm3", ARM3_SEQ2), ("arm2", ARM2_SEQ2)]),
]


def run_steps(arm, steps, sleep=time.sleep):
    """Execute a step list on one SwiftAPI arm (blocking moves, as on the cell)."""
    for kind, value in steps:
        if kind in ("move", "catch"):
            x, y, z = value
            speed = MOVE_SPEED if kind == "move" else CATCH_SPEED
            arm.set_position(x=x, y=y, z=z, speed=speed, wait=True, timeout=None)
        elif kind == "wait":
            sleep(value)
        elif kind == "gripper":
            arm.set_gripper(catch=value)
        elif kind == "wrist":
            arm.set_wrist(value)
        else:
            raise ValueError(f"unknown step {kind!r}")
