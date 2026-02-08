# Conveyor controller - OmniGraph Script Node (On Tick -> Script Node), embedded by tools/build_scene.py.
#
# Each belt is a kinematic rigid body with PhysX surface velocity: parts lying on it are carried
# without moving the belt itself. Light barriers are thin sphere sweeps across the belt; a sweep
# that hits one of PARTS counts as "sensor on". Belt 1 starts when S1 sees a part and stops at S2,
# belt 2 starts at S3 and stops at S4 - the same start/stop pattern the PLC uses on the cell.

import carb
import omni.usd
from omni.physx import get_physx_scene_query_interface
from pxr import Gf, PhysxSchema, Sdf, UsdGeom, UsdPhysics

# --- config ---
PARTS = ("/World/Cell/Parts/Holder",)
BELT1_PRIM = "/World/Cell/Conveyor1/Belt"
S1_PRIM = "/World/Cell/Sensors/S1/Beam"
S2_PRIM = "/World/Cell/Sensors/S2/Beam"
BELT1_VEL = (0.15, 0.0, 0.0)
BELT2_PRIM = "/World/Cell/Conveyor2/Belt"
S3_PRIM = "/World/Cell/Sensors/S3/Beam"
S4_PRIM = "/World/Cell/Sensors/S4/Beam"
BELT2_VEL = (-0.15, 0.0, 0.0)
BEAM_DIR = (0.0, -1.0, 0.0)
BEAM_DISTANCE = 2.0
BEAM_RADIUS = 0.0005
BELT_VEL_IS_WORLD = True
# --- end config ---


def step_belt(running, start_seen, stop_seen):
    # light-barrier logic of one belt: start at the first barrier, stop at the second
    if not running and start_seen:
        running = True
    if running and stop_seen:
        running = False
    return running


def _ensure_conveyor(stage, belt_path, velocity):
    prim = stage.GetPrimAtPath(Sdf.Path(belt_path))
    if not prim.IsValid():
        raise RuntimeError(f"Belt prim not found: {belt_path}")
    # surface velocity needs a rigid body; the belt itself must not move
    UsdPhysics.CollisionAPI.Apply(prim)
    UsdPhysics.RigidBodyAPI.Apply(prim).CreateKinematicEnabledAttr(True)
    surface = PhysxSchema.PhysxSurfaceVelocityAPI.Get(stage, prim.GetPath())
    if not surface:
        surface = PhysxSchema.PhysxSurfaceVelocityAPI.Apply(prim)
    surface.CreateSurfaceVelocityAttr(Gf.Vec3f(*velocity))
    surface.CreateSurfaceVelocityLocalSpaceAttr(not BELT_VEL_IS_WORLD)
    surface.CreateSurfaceVelocityEnabledAttr(False)
    return surface


def _world_pos(stage, path):
    prim = stage.GetPrimAtPath(Sdf.Path(path))
    if not prim.IsValid():
        raise RuntimeError(f"Prim not found: {path}")
    p = UsdGeom.Xformable(prim).ComputeLocalToWorldTransform(0).ExtractTranslation()
    return carb.Float3(p[0], p[1], p[2])


def _beam_sees_part(query, origin):
    found = {"hit": False}

    def report(hit):
        if str(getattr(hit, "rigid_body", "")) in PARTS:
            found["hit"] = True
            return False  # stop the sweep
        return True

    query.sweep_sphere_all(BEAM_RADIUS, origin, carb.Float3(*BEAM_DIR), BEAM_DISTANCE, report)
    return found["hit"]


def setup(db):
    state = db.per_instance_state
    state.stage = omni.usd.get_context().get_stage()
    state.query = get_physx_scene_query_interface()
    state.running = [False, False]
    state.belts = [
        _ensure_conveyor(state.stage, BELT1_PRIM, BELT1_VEL),
        _ensure_conveyor(state.stage, BELT2_PRIM, BELT2_VEL),
    ]


def compute(db):
    state = db.per_instance_state
    barriers = ((S1_PRIM, S2_PRIM), (S3_PRIM, S4_PRIM))
    for index, (start_prim, stop_prim) in enumerate(barriers):
        start_seen = _beam_sees_part(state.query, _world_pos(state.stage, start_prim))
        stop_seen = _beam_sees_part(state.query, _world_pos(state.stage, stop_prim))
        running = step_belt(state.running[index], start_seen, stop_seen)
        if running != state.running[index]:
            state.belts[index].GetSurfaceVelocityEnabledAttr().Set(running)
            print(f"[Conveyor] belt {index + 1} {'START' if running else 'STOP'}")
        state.running[index] = running
    return True
