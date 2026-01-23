import threading

import pytest

from fakes import FakeSwift
from isaac_uarm_twin import cell_demo, kinematics, sequences

ALL_SEQUENCES = {name: getattr(sequences, name) for name in
                 ("ARM1_SEQ1", "ARM1_SEQ2", "ARM2_SEQ1", "ARM2_SEQ2", "ARM3_SEQ1", "ARM3_SEQ2")}


def test_run_steps_uses_the_cell_speeds_and_tools():
    arm, slept = FakeSwift(), []
    steps = [sequences.move(1, 2, 3), sequences.catch(4, 5, 6), sequences.wait(2), sequences.gripper(True),
             sequences.wrist(120)]
    sequences.run_steps(arm, steps, sleep=slept.append)
    assert arm.calls == [
        ("set_position", {"x": 1, "y": 2, "z": 3, "speed": sequences.MOVE_SPEED, "wait": True, "timeout": None}),
        ("set_position", {"x": 4, "y": 5, "z": 6, "speed": sequences.CATCH_SPEED, "wait": True, "timeout": None}),
        ("set_gripper", True),
        ("set_wrist", 120),
    ]
    assert slept == [2]


def test_unknown_step_is_rejected():
    with pytest.raises(ValueError):
        sequences.run_steps(FakeSwift(), [("teleport", None)], sleep=lambda s: None)


@pytest.mark.parametrize("name", sorted(ALL_SEQUENCES))
def test_every_waypoint_is_reachable(name):
    for kind, value in ALL_SEQUENCES[name]:
        if kind in ("move", "catch"):
            assert kinematics.motor_angles(*value) is not None, (name, value)


def test_phases_alternate_the_holder_and_pcb_slots():
    (title1, phase1), (title2, phase2) = sequences.PHASES
    assert [arm for arm, _ in phase1] == ["arm1", "arm2", "arm3", "arm2"]
    assert [arm for arm, _ in phase2] == ["arm1", "arm2", "arm3", "arm2"]
    assert phase1[0][1] is sequences.ARM1_SEQ1 and phase2[0][1] is sequences.ARM1_SEQ2
    assert phase1[2][1] is sequences.ARM3_SEQ1 and phase2[2][1] is sequences.ARM3_SEQ2


def test_only_arm2_uses_gripper_and_wrist():
    for name, steps in ALL_SEQUENCES.items():
        tools = {kind for kind, _ in steps if kind in ("gripper", "wrist")}
        assert (name.startswith("ARM2") and tools == {"gripper", "wrist"}) or not tools


def test_motion_loop_runs_the_phases_in_order():
    arms = {name: FakeSwift() for name in ("arm1", "arm2", "arm3")}
    order, stop = [], threading.Event()
    for name, arm in arms.items():
        original = arm.set_position

        def record(name=name, original=original, **kwargs):
            if not order or order[-1] != name:
                order.append(name)
            original(**kwargs)

        arm.set_position = record

    def sleep(seconds):
        if seconds == 1 and len(order) >= 8:  # end of the first full cycle
            stop.set()

    cell_demo.motion_loop(arms, stop, sleep=sleep)
    assert order[:8] == ["arm1", "arm2", "arm3", "arm2", "arm1", "arm2", "arm3", "arm2"]


def test_cell_demo_app_serves_telemetry():
    from fastapi.testclient import TestClient

    from isaac_uarm_twin.telemetry import ServoTelemetry

    app = cell_demo.create_app(ServoTelemetry({"arm1": FakeSwift()}))
    assert TestClient(app).get("/api/servo_angles").json() == {"arm1": None}
