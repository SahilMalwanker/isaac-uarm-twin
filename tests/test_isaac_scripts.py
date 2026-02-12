import ast
from pathlib import Path

import numpy as np
import pytest

import follower
from isaac_uarm_twin import kinematics

ISAAC = Path(__file__).resolve().parents[1] / "isaac"
SCRIPTS = sorted(ISAAC.rglob("*.py"))
GRAPHS = sorted((ISAAC / "graphs").glob("*.py"))


def module_constant(path, name):
    for node in ast.parse(path.read_text(encoding="utf-8")).body:
        if isinstance(node, ast.Assign) and any(getattr(t, "id", None) == name for t in node.targets):
            return ast.literal_eval(node.value)
    raise KeyError(name)


def load_function(path, name):
    """Execute one top-level function of a script whose imports need Isaac Sim."""
    tree = ast.parse(path.read_text(encoding="utf-8"))
    node = next(n for n in tree.body if isinstance(n, ast.FunctionDef) and n.name == name)
    namespace = {}
    exec(compile(ast.Module(body=[node], type_ignores=[]), str(path), "exec"), namespace)
    return namespace[name]


@pytest.mark.parametrize("path", SCRIPTS, ids=lambda p: p.relative_to(ISAAC).as_posix())
def test_scripts_compile(path):
    compile(path.read_text(encoding="utf-8"), str(path), "exec")


@pytest.mark.parametrize("path", GRAPHS, ids=lambda p: p.name)
def test_graph_scripts_can_be_embedded(path):
    source = path.read_text(encoding="utf-8")
    assert '"""' not in source and "\\" not in source
    assert source.count("# --- config ---") == 1 and source.count("# --- end config ---") == 1
    tree = ast.parse(source)
    functions = {node.name for node in tree.body if isinstance(node, ast.FunctionDef)}
    assert {"setup", "compute"} <= functions


@pytest.mark.parametrize("running, start, stop, expected", [
    (False, False, False, False),
    (False, True, False, True),   # part reaches the start barrier
    (True, False, False, True),   # keeps running between the barriers
    (True, False, True, False),   # part reaches the stop barrier
    (False, False, True, False),  # a part waiting at the stop barrier does not restart the belt
    (False, True, True, False),   # both barriers blocked: stay stopped
])
def test_belt_logic(running, start, stop, expected):
    step_belt = load_function(ISAAC / "graphs" / "conveyor_controller.py", "step_belt")
    assert step_belt(running, start, stop) is expected


def test_rig_uses_the_kinematic_loop_anchors():
    rig = ISAAC / "tools" / "rig_swiftpro.py"
    assert module_constant(rig, "LOOP_JOINTS") == kinematics.LOOP_JOINTS
    tool = module_constant(rig, "TOOL_POINT")
    assert (tool[0], tool[2]) == kinematics.TOOL_POINT


def test_rig_drives_the_joints_the_follower_commands():
    drives = module_constant(ISAAC / "tools" / "rig_swiftpro.py", "DRIVES")
    commanded = set(follower.SERVO_TO_DOF.values()) | set(follower.WRIST_TO_DOF.values())
    assert set(drives) == commanded
    passive = module_constant(ISAAC / "tools" / "rig_swiftpro.py", "PASSIVE_JOINTS")
    assert not set(passive) & commanded


@pytest.mark.parametrize("relative_path, function_name", [
    ("graphs/vacuum_autograb.py", "_try_grab"),
    ("tools/grab_part.py", "grab"),
])
@pytest.mark.parametrize("rotated", [False, True])
@pytest.mark.parametrize("offset", [(0.0, 0.0, 0.0), (0.002, -0.001, 0.0005)])
@pytest.mark.parametrize("meters_per_unit", [1.0, 0.01])
def test_pickup_anchors_coincide_without_moving_the_part(relative_path, function_name, rotated, offset,
                                                       meters_per_unit):
    class RowTransform:
        def __init__(self, matrix):
            self.matrix = matrix

        def GetInverse(self):
            return RowTransform(np.linalg.inv(self.matrix))

        def Transform(self, point):
            return (np.append(point, 1.0) @ self.matrix)[:3]

        def ExtractTranslation(self):
            return self.matrix[3, :3]

        def __mul__(self, other):
            return RowTransform(self.matrix @ other.matrix)

    part = np.eye(4)
    if rotated:
        part[:3, :3] = ((0, 1, 0), (-1, 0, 0), (0, 0, 1))
    part[3, :3] = np.array((0.7, 0.2, 1.1)) / meters_per_unit
    hold = np.eye(4)
    hold[:3, :3] = ((0, 0, -1), (0, 1, 0), (1, 0, 0))
    hold[3, :3] = np.array((0.3, -0.2, 1.4)) / meters_per_unit
    grip = np.eye(4)
    grip[3, :3] = np.array((0.02, 0.003, 0.015)) / meters_per_unit
    namespace = {
        "m_part": RowTransform(part),
        "m_hold": RowTransform(hold),
        "m_grip": RowTransform(grip @ part),
    }
    tool_world = namespace["m_grip"].ExtractTranslation() + np.array(offset) / meters_per_unit
    namespace.update(tool_world=tool_world, local_tool=RowTransform(hold).GetInverse().Transform(tool_world))
    path = ISAAC / relative_path
    tree = ast.parse(path.read_text(encoding="utf-8"))
    function = next(node for node in tree.body if isinstance(node, ast.FunctionDef) and node.name == function_name)
    assignments = [node for node in function.body if isinstance(node, ast.Assign)
                   and any(getattr(target, "id", None) in ("local_hold", "local_part") for target in node.targets)]
    exec(compile(ast.Module(body=assignments, type_ignores=[]), str(path), "exec"), namespace)
    held_anchor = namespace.get("local_hold", namespace["local_tool"])
    assert RowTransform(hold).Transform(held_anchor) == pytest.approx(tool_world)
    assert RowTransform(part).Transform(namespace["local_part"]) == pytest.approx(tool_world)
