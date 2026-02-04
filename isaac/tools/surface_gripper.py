"""Surface gripper with suction behaviour - run in the Isaac Sim Script Editor (Isaac Sim 5.x).

Alternative to the pick-as-is vacuum graph: creates an Isaac Surface Gripper on Link8 of a robot and
keeps closing it whenever it is open, which behaves like a suction cup.

1. Place a small sphere at ``MARKER_PATH`` where the gripper should grip (e.g. under the tool).
2. Run this script; it converts the marker position into the attachment joint's anchor on Link8.
3. Press Play. To drop the part later run ``gi.open_gripper(GRIPPER_PATH)``.
"""

import isaacsim.robot.surface_gripper._surface_gripper as sg
import omni.kit.app
import omni.kit.commands
import omni.usd
from pxr import Gf, PhysxSchema, Sdf, Usd, UsdGeom, UsdPhysics

# --- config ---
PHYS_KEEP = "/World/PhysicsScene"
PHYS_DISABLE = ["/World/physicsScene", "/physicsScene"]
LINK8_PATH = "/World/uarm2/swiftpro/Link8"
JOINT_PATH = LINK8_PATH + "/GripperJoint_0"
MARKER_PATH = "/World/GripMarker"
CONTAINER_PATH = LINK8_PATH + "/SurfaceGripper_NEW"
MAX_GRIP_DIST = 0.05
RETRY_INTERVAL_SEC = 9999.0  # "keep trying"
ENABLE_SUCTION_MODE = True  # re-close whenever the gripper is open
CLOSE_EVERY_N_FRAMES = 10
FIX_OFFSETS_PATH = ""  # optional rigid body whose collision offsets need valid values
CONTACT_OFFSET = 0.002
REST_OFFSET = 0.0
# --- end config ---

stage = omni.usd.get_context().get_stage()
gi = sg.acquire_surface_gripper_interface()


def prim(path):
    p = stage.GetPrimAtPath(path)
    return p if (p and p.IsValid()) else None


def must(path):
    p = prim(path)
    if not p:
        raise RuntimeError(f"Prim not found: {path}")
    return p


def set_float(p, name, value):
    attr = p.GetAttribute(name)
    if not (attr and attr.IsValid()):
        attr = p.CreateAttribute(name, Sdf.ValueTypeNames.Float)
    attr.Set(float(value))


def set_float3(p, name, xyz):
    attr = p.GetAttribute(name)
    if not (attr and attr.IsValid()):
        attr = p.CreateAttribute(name, Sdf.ValueTypeNames.Float3)
    attr.Set(Gf.Vec3f(float(xyz[0]), float(xyz[1]), float(xyz[2])))


# 1) exactly one active physics scene
if not prim(PHYS_KEEP):
    scene = UsdPhysics.Scene.Define(stage, Sdf.Path(PHYS_KEEP))
    scene.CreateGravityDirectionAttr().Set(Gf.Vec3f(0, 0, -1))
    scene.CreateGravityMagnitudeAttr().Set(9.81)
PhysxSchema.PhysxSceneAPI.Apply(stage.GetPrimAtPath(PHYS_KEEP))
for path in PHYS_DISABLE:
    if prim(path):
        prim(path).SetActive(False)
print("PhysicsScene kept:", PHYS_KEEP)

# 2) optional: valid collision offsets on a part
if FIX_OFFSETS_PATH and prim(FIX_OFFSETS_PATH):
    part = must(FIX_OFFSETS_PATH)
    set_float(part, "physxCollision:contactOffset", CONTACT_OFFSET)
    set_float(part, "physxCollision:restOffset", REST_OFFSET)
    print("Fixed collision offsets on:", FIX_OFFSETS_PATH)

# 3) surface gripper container (CreateSurfaceGripper puts the real gripper prim below it)
if not prim(CONTAINER_PATH):
    ok, created = omni.kit.commands.execute("CreateSurfaceGripper", prim_path=CONTAINER_PATH)
    if not ok or created is None:
        raise RuntimeError("CreateSurfaceGripper failed.")
gripper_prim = None
for p in stage.Traverse():
    if str(p.GetPath()).startswith(CONTAINER_PATH + "/") and p.GetTypeName() == "IsaacSurfaceGripper":
        gripper_prim = p
        break
if gripper_prim is None:
    raise RuntimeError("No IsaacSurfaceGripper below " + CONTAINER_PATH)
GRIPPER_PATH = str(gripper_prim.GetPath())
print("Gripper:", GRIPPER_PATH)

# 4) gripper properties and attachment point
set_float(gripper_prim, "isaac:maxGripDistance", MAX_GRIP_DIST)
set_float(gripper_prim, "isaac:retryInterval", RETRY_INTERVAL_SEC)
rel = gripper_prim.GetRelationship("isaac:attachmentPoints")
if not rel.IsValid():
    rel = gripper_prim.CreateRelationship("isaac:attachmentPoints")
rel.SetTargets([Sdf.Path(JOINT_PATH)])

# 5) anchor the attachment joint at the marker (marker world position -> Link8 local)
link8, marker, joint = must(LINK8_PATH), must(MARKER_PATH), must(JOINT_PATH)
t_link8 = UsdGeom.Xformable(link8).ComputeLocalToWorldTransform(Usd.TimeCode.Default())
marker_world = UsdGeom.Xformable(marker).ComputeLocalToWorldTransform(Usd.TimeCode.Default()).ExtractTranslation()
local = t_link8.GetInverse().Transform(Gf.Vec3d(*marker_world))
set_float3(joint, "physics:localPos0", (local[0], local[1], local[2]))
print("Anchor (Link8 local):", tuple(round(v, 5) for v in local))

# 6) suction mode: keep closing while the simulation runs
_previous = globals().get("_SUCTION_SUB")
if _previous is not None:
    _previous.unsubscribe()
_SUCTION_SUB = None
if ENABLE_SUCTION_MODE:
    frames = {"n": 0}

    def on_update(_event):
        frames["n"] += 1
        if frames["n"] == 10:
            print("[suction] close:", gi.close_gripper(GRIPPER_PATH), gi.get_gripper_status(GRIPPER_PATH))
        if frames["n"] % CLOSE_EVERY_N_FRAMES == 0:
            try:
                if gi.get_gripper_status(GRIPPER_PATH) == sg.GripperStatus.Open:
                    gi.close_gripper(GRIPPER_PATH)
            except Exception as exc:  # noqa: BLE001
                print("[suction] exception:", exc)

    _SUCTION_SUB = omni.kit.app.get_app().get_update_event_stream().create_subscription_to_pop(on_update)
    print("Suction mode enabled (re-closes whenever open).")

print("Ready: press Play. Drop with gi.open_gripper(GRIPPER_PATH).")
