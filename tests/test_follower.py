import math
import threading

import pytest

import follower
from fakes import FakeDynamicControl

ARM_DOFS = ["Joint1", "Joint2", "Joint3", "Joint4", "Joint5", "Joint6", "Joint7", "Joint8", "Joint9"]


def make_follower(**kwargs):
    dc = FakeDynamicControl({
        "/World/uarm1/swiftpro/Base": ARM_DOFS,
        "/World/uarm2/swiftpro/Base": ARM_DOFS + ["JointGripperExt"],
        "/World/uarm3/swiftpro/Base": ARM_DOFS,
    })
    return follower.TwinFollower(follower.ARM_CONFIGS, dc, **kwargs), dc


@pytest.mark.parametrize("joint", sorted(follower.CALIBRATION))
def test_calibration_anchors_map_exactly(joint):
    cal = follower.CALIBRATION[joint]
    config = follower.ARM_CONFIGS["arm2"]
    assert follower.servo_to_isaac_deg(config, joint, cal["servo_ext"]) == pytest.approx(cal["isaac_ext"])
    assert follower.servo_to_isaac_deg(config, joint, cal["servo_ret"]) == pytest.approx(cal["isaac_ret"])


def test_base_mapping_is_servo_minus_90():
    config = follower.ARM_CONFIGS["arm1"]
    for servo in (0.0, 45.0, 90.0, 135.0, 180.0):
        assert follower.servo_to_isaac_deg(config, "Joint1", servo) == pytest.approx(servo - 90.0)


def test_servo_outside_the_anchors_is_clamped():
    config = follower.ARM_CONFIGS["arm1"]
    assert follower.servo_to_isaac_deg(config, "Joint2", -50.0) == pytest.approx(89.954)
    assert follower.servo_to_isaac_deg(config, "Joint2", 400.0) == pytest.approx(-29.972)


def test_clamp_accepts_swapped_bounds():
    assert follower.clamp(5.0, 10.0, 0.0) == 5.0
    assert follower.clamp(-3.0, 10.0, 0.0) == 0.0


def test_map_linear_degenerate_anchors():
    assert follower.map_linear(7.0, 1.0, 1.0, 3.0, 9.0) == 3.0


def test_rate_limit():
    assert follower.rate_limit(None, 40.0, 100.0, 0.01) == 40.0
    assert follower.rate_limit(0.0, 40.0, 100.0, 0.05) == pytest.approx(5.0)
    assert follower.rate_limit(0.0, -40.0, 100.0, 0.05) == pytest.approx(-5.0)
    assert follower.rate_limit(0.0, 2.0, 100.0, 0.05) == 2.0
    assert follower.rate_limit(0.0, 40.0, 100.0, 0.0) == 40.0


def test_arm_configs():
    assert follower.ARM_CONFIGS["arm1"]["expected_servo_count"] == 3
    assert follower.ARM_CONFIGS["arm2"]["expected_servo_count"] == 4
    assert follower.ARM_CONFIGS["arm2"]["servo_to_dof"][3] == "JointGripperExt"
    assert follower.ARM_CONFIGS["arm3"]["art_path"] == "/World/uarm3/swiftpro/Base"


def test_ingest_ignores_short_or_missing_samples():
    twin, _ = make_follower()
    twin.ingest({"arm1": [90, 45, 60], "arm2": [90, 45, 60], "arm3": "nonsense"})
    assert twin.latest["arm1"]["servo_deg"] == [90.0, 45.0, 60.0]
    assert twin.latest["arm2"]["servo_deg"] is None  # the wrist arm needs four values
    assert twin.latest["arm3"]["servo_deg"] is None


def test_first_step_applies_the_mapped_targets():
    twin, dc = make_follower()
    twin.ingest({"arm1": [90.0, 45.0, 60.0], "arm2": [90.0, 45.0, 60.0, 90.0], "arm3": [120.0, 30.0, 40.0]})
    twin.on_physics_step(1.0 / 60.0)
    targets = {(art.split("/")[2], dof): value for art, dof, value in dc.targets}
    assert len(targets) == 3 + 4 + 3
    config = follower.ARM_CONFIGS["arm3"]
    assert targets[("uarm3", "Joint1")] == pytest.approx(math.radians(30.0))
    joint2 = follower.servo_to_isaac_deg(config, "Joint2", 30.0)
    assert targets[("uarm3", "Joint2")] == pytest.approx(math.radians(joint2))
    assert targets[("uarm2", "JointGripperExt")] == pytest.approx(0.0)
    assert dc.wakes >= 3


def test_unchanged_sample_hits_the_deadband():
    twin, dc = make_follower()
    twin.ingest({"arm1": [90.0, 45.0, 60.0]})
    twin.on_physics_step(1.0 / 60.0)
    count = len(dc.targets)
    twin.on_physics_step(1.0 / 60.0)
    assert len(dc.targets) == count


def test_new_sample_is_rate_limited_and_smoothed():
    twin, dc = make_follower()
    twin.ingest({"arm1": [90.0, 45.0, 60.0]})
    twin.on_physics_step(1.0 / 60.0)
    twin.ingest({"arm1": [180.0, 45.0, 60.0]})  # base jumps from 0 to +90 deg
    dt = 1.0 / 60.0
    twin.on_physics_step(dt)
    step = follower.RATE_LIMIT_DEG_PER_S["Joint1"] * dt
    expected = 0.0 + follower.ALPHA * step
    last = [value for art, dof, value in dc.targets if art.startswith("/World/uarm1") and dof == "Joint1"][-1]
    assert last == pytest.approx(math.radians(expected))


def test_twin_converges_to_the_target():
    twin, dc = make_follower()
    twin.ingest({"arm1": [90.0, 45.0, 60.0]})
    for _ in range(3):
        twin.on_physics_step(1.0 / 60.0)
    twin.ingest({"arm1": [150.0, 45.0, 60.0]})
    for _ in range(240):
        twin.on_physics_step(1.0 / 60.0)
    last = [value for art, dof, value in dc.targets if art.startswith("/World/uarm1") and dof == "Joint1"][-1]
    assert math.degrees(last) == pytest.approx(60.0, abs=0.1)


def test_missing_articulation_is_skipped():
    dc = FakeDynamicControl({"/World/uarm1/swiftpro/Base": ARM_DOFS})
    twin = follower.TwinFollower(follower.ARM_CONFIGS, dc)
    twin.ingest({"arm1": [90.0, 45.0, 60.0], "arm3": [90.0, 45.0, 60.0]})
    twin.on_physics_step(1.0 / 60.0)
    assert {art for art, _, _ in dc.targets} == {"/World/uarm1/swiftpro/Base"}


def test_late_arm_is_discovered_without_rebinding_ready_arm(monkeypatch):
    config = {name: follower.ARM_CONFIGS[name] for name in ("arm1", "arm3")}
    ready_path, late_path = config["arm1"]["art_path"], config["arm3"]["art_path"]
    dc = FakeDynamicControl({ready_path: ARM_DOFS})
    queries = []
    original = dc.get_articulation

    def lookup(path):
        queries.append(path)
        return original(path)

    monkeypatch.setattr(dc, "get_articulation", lookup)
    twin = follower.TwinFollower(config, dc)
    twin.ingest({"arm1": [90, 45, 60], "arm3": [120, 30, 40]})
    twin.on_physics_step(1 / 60)
    assert set(twin.arts) == {"arm1"}
    twin.ingest({"arm1": [180, 45, 60]})
    twin.on_physics_step(1 / 60)
    previous = twin.smoothed_deg["arm1"]["Joint1"]
    dc.articulations[late_path] = ARM_DOFS
    twin.on_physics_step(1 / 60)
    assert set(twin.arts) == {"arm1", "arm3"}
    assert queries.count(ready_path) == 1
    assert previous < twin.smoothed_deg["arm1"]["Joint1"] < 90
    assert any(path == late_path for path, _, _ in dc.targets)


@pytest.mark.parametrize("invalid", [None, 0])
def test_initially_missing_scene_is_retried(invalid, monkeypatch):
    config = {"arm1": follower.ARM_CONFIGS["arm1"]}
    dc = FakeDynamicControl({})
    monkeypatch.setattr(dc, "get_articulation", lambda path: path if path in dc.articulations else invalid)
    twin = follower.TwinFollower(config, dc)
    twin.ingest({"arm1": [90, 45, 60]})
    twin.on_physics_step(1 / 60)
    assert twin.arts == {}
    assert dc.wakes == 0
    dc.articulations[config["arm1"]["art_path"]] = ARM_DOFS
    twin.on_physics_step(1 / 60)
    assert len(dc.targets) == 3


def test_partially_loaded_dofs_are_retried():
    config = {"arm1": follower.ARM_CONFIGS["arm1"]}
    path = config["arm1"]["art_path"]
    dc = FakeDynamicControl({path: ["Joint1"]})
    twin = follower.TwinFollower(config, dc)
    twin.ingest({"arm1": [90, 45, 60]})
    twin.on_physics_step(1 / 60)
    assert twin.arts == {}
    assert dc.targets == []
    dc.articulations[path] = ARM_DOFS
    twin.on_physics_step(1 / 60)
    assert {name for _, name, _ in dc.targets} == {"Joint1", "Joint2", "Joint6"}


def test_missing_arm_warning_is_not_repeated_each_step(capsys):
    twin = follower.TwinFollower({"arm1": follower.ARM_CONFIGS["arm1"]}, FakeDynamicControl({}))
    for _ in range(5):
        twin.on_physics_step(1 / 60)
    assert capsys.readouterr().out.count("No articulation") == 1


def test_poll_http_feeds_the_follower_until_stopped():
    twin, _ = make_follower()
    stop = threading.Event()

    class Response:
        def json(self):
            return {"arm1": [100.0, 40.0, 50.0]}

    class Session:
        calls = 0

        def get(self, url, timeout):
            Session.calls += 1
            if Session.calls == 2:
                raise ConnectionError("telemetry gap")
            if Session.calls >= 3:
                stop.set()
            return Response()

    follower.poll_http(twin, url="http://127.0.0.1:1/api/servo_angles", hz=1000, stop_event=stop, session=Session())
    assert Session.calls == 3
    assert twin.latest["arm1"]["servo_deg"] == [100.0, 40.0, 50.0]
