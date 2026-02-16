import math

import pytest

import build_scene
from isaac_uarm_twin import kinematics
from usda_lint import lint

CELL = build_scene.layout()
HX, HY, HZ = build_scene.HOLDER_SIZE


@pytest.fixture(scope="module")
def usda():
    return build_scene.build_usda()


def test_committed_scene_is_up_to_date(usda):
    assert build_scene.SCENE_PATH.read_text(encoding="utf-8") == usda, "run: python tools/build_scene.py"


def test_usda_structure(usda):
    paths, targets, errors = lint(usda)
    assert errors == []
    for target in targets:
        assert target.split(".")[0] in paths, target
    required = [
        "/World/PhysicsScene", "/World/Looks/BeltRubber", "/World/Environment/Floor", "/World/Cell/Table/Top",
        "/World/Cell/Table/Riser", "/World/Cell/Conveyor1/Belt", "/World/Cell/Conveyor2/Belt",
        "/World/Cell/Parts/Holder/GripPoint", "/World/Cell/Parts/Holder/JawPoint", "/World/Cell/Parts/PCB/GripPoint",
        "/World/Cell/Parts/Tray", "/World/Graphs/ConveyorController/script", "/World/Graphs/VacuumArm1/tick",
        "/World/Graphs/GripperArm2/script", "/World/Graphs/VacuumArm3/script",
    ]
    required += [f"/World/Cell/Sensors/S{i}/Beam" for i in range(1, 5)]
    required += [f"/World/uarm{i}/swiftpro" for i in range(1, 4)]
    for path in required:
        assert path in paths, path


def test_stage_is_z_up_in_metres(usda):
    header = usda.split("def Xform", 1)[0]
    assert 'upAxis = "Z"' in header and "metersPerUnit = 1" in header and 'defaultPrim = "World"' in header
    assert "vector3f physics:gravityDirection = (0, 0, -1)" in usda


def test_robots_reference_the_imported_urdfs(usda):
    assert usda.count("@./robot/swiftpro_twin/swiftpro_twin.usd@") == 2
    assert usda.count("@./robot/swiftpro_twin_gripper/swiftpro_twin_gripper.usd@") == 1


def test_belts_are_kinematic_conveyors(usda):
    assert usda.count("bool physics:kinematicEnabled = 1") == 2
    assert usda.count("bool physxSurfaceVelocity:surfaceVelocityEnabled = 0") == 2  # the graph switches them
    assert "vector3f physxSurfaceVelocity:surfaceVelocity = (0.15, 0, 0)" in usda
    assert "vector3f physxSurfaceVelocity:surfaceVelocity = (-0.15, 0, 0)" in usda


def test_graphs_are_configured_per_tool(usda):
    assert "HOLD_LINK_PATH = '/World/uarm2/swiftpro/GripperExt'" in usda
    assert "GRIPPOINT_PATH = '/World/Cell/Parts/Holder/JawPoint'" in usda
    assert "OBJ_PATH = '/World/Cell/Parts/PCB'" in usda
    assert "PARTS = ('/World/Cell/Parts/Holder',)" in usda
    assert usda.count('token node:type = "omni.graph.scriptnode.ScriptNode"') == 4


def test_configure_rejects_unknown_names():
    script = "a = 1\n# --- config ---\nX = 1\n# --- end config ---\n"
    expected = "a = 1\n# --- config ---\nX = (1.0, 2.0)\n# --- end config ---\n"
    assert build_scene.configure(script, X=(1.0, 2.0)) == expected
    with pytest.raises(KeyError):
        build_scene.configure(script, Y=2)


def test_suction_tips_reach_the_parts():
    waypoints = CELL["waypoints"]
    assert math.dist(waypoints["arm1_pick"], CELL["holder_grip"]) < 0.010  # vacuum threshold
    assert math.dist(waypoints["arm3_pick"], CELL["pcb_grip"]) < 0.010


def test_gripper_meets_the_holder_stopped_by_s2():
    waypoints = CELL["waypoints"]
    stopped = (CELL["sensors"]["S2"][0] - HX / 2, waypoints["arm1_place"][1],
               build_scene.BELT_TOP + HZ / 2 + CELL["holder_jaw_z"])
    assert math.dist(waypoints["arm2_pick"], stopped) < 0.015  # gripper threshold


def test_pcb_is_dropped_onto_the_held_holder():
    waypoints = CELL["waypoints"]
    assert math.dist(waypoints["arm3_drop"][:2], waypoints["arm2_hold"][:2]) < 1e-4
    holder_top = waypoints["arm2_hold"][2] - CELL["holder_jaw_z"] + HZ / 2
    pcb_bottom = waypoints["arm3_drop"][2] - build_scene.PCB_SIZE[2]
    assert 0.0 < pcb_bottom - holder_top < 0.02


def test_light_barriers_see_the_parts_where_they_are_set_down():
    sensors, waypoints = CELL["sensors"], CELL["waypoints"]
    assert abs(sensors["S1"][0] - waypoints["arm1_place"][0]) < HX / 2
    assert abs(sensors["S3"][0] - waypoints["arm2_place"][0]) < HX / 2
    for name, (x, y, z) in sensors.items():
        belt = "Conveyor1" if name in ("S1", "S2") else "Conveyor2"
        y0, y1 = build_scene.CONVEYORS[belt]["y"]
        assert y0 < y < y1, name
        assert build_scene.BELT_TOP < z < build_scene.BELT_TOP + HZ, name
        assert build_scene.BELT_X[0] < x < build_scene.BELT_X[1], name


def test_belts_carry_parts_from_start_to_stop_barrier():
    s = CELL["sensors"]
    assert build_scene.CONVEYORS["Conveyor1"]["velocity"][0] > 0 and s["S1"][0] < s["S2"][0]
    assert build_scene.CONVEYORS["Conveyor2"]["velocity"][0] < 0 and s["S3"][0] > s["S4"][0]


def test_parts_rest_on_surfaces():
    assert CELL["holder"][2] - HZ / 2 == pytest.approx(build_scene.TABLE_TOP)
    assert CELL["pcb"][2] - build_scene.PCB_SIZE[2] / 2 == pytest.approx(build_scene.TRAY_TOP)
    assert CELL["riser_top"] == pytest.approx(min(spec["base"][2] for spec in (build_scene.ARM1, build_scene.ARM2)))
    assert CELL["robots"]["uarm3"]["base"][2] >= build_scene.TABLE_TOP


def test_arm1_drop_lands_on_belt1():
    x, y, _ = CELL["waypoints"]["arm1_place"]
    y0, y1 = build_scene.CONVEYORS["Conveyor1"]["y"]
    assert y0 < y < y1 and build_scene.BELT_X[0] < x < build_scene.BELT_X[1]
    x, y, _ = CELL["waypoints"]["arm2_place"]
    y0, y1 = build_scene.CONVEYORS["Conveyor2"]["y"]
    assert y0 < y < y1


def test_tool_point_world_matches_place():
    base, yaw = CELL["robots"]["uarm2"]["base"], CELL["robots"]["uarm2"]["yaw"]
    assert kinematics.tool_point_world(base, yaw, (100.0, 0.0, 0.0)) == pytest.approx((base[0] + 0.1, base[1], base[2]))


def test_look_at_is_orthonormal():
    rows = build_scene.look_at((1.0, -1.0, 2.0), (0.0, 0.0, 1.0))
    axes = [row[:3] for row in rows[:3]]
    for i in range(3):
        assert math.fsum(v * v for v in axes[i]) == pytest.approx(1.0)
        for j in range(i):
            assert math.fsum(a * b for a, b in zip(axes[i], axes[j])) == pytest.approx(0.0, abs=1e-12)


def test_check_mode(tmp_path, capsys):
    out = tmp_path / "cell.usda"
    assert build_scene.main(["--check", "--out", str(out)]) == 1
    assert build_scene.main(["--out", str(out)]) == 0
    assert build_scene.main(["--check", "--out", str(out)]) == 0
    capsys.readouterr()
