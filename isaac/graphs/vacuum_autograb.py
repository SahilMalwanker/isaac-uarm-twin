# Vacuum auto-grab - OmniGraph Script Node (On Tick -> Script Node), one instance per tool.
# tools/build_scene.py embeds this file into the stage and fills the config block for each arm.
#
# PhysX has no suction, so the vacuum is a "pick-as-is" fixed joint: while the vacuum is on and the
# tool point (AttachPoint0 on Link8) comes within THRESH_MM of the part's grip point, a fixed joint is
# created between HOLD_LINK and the part in exactly their current relative pose, so the part neither
# snaps nor turns. Switching the vacuum off removes the joint and gives the part a small downward kick.
# The gripper of arm 2 uses the same mechanism and holds on its rotating head, so parts turn with the
# wrist. A small window with an on/off button controls the tool while the simulation plays.

import omni.ui as ui
import omni.usd
from pxr import Gf, Sdf, Usd, UsdGeom, UsdPhysics

# --- config ---
ROBOT_ID = "UARM1"
TOOL_NAME = "vacuum"
LINK8_PATH = "/World/uarm1/swiftpro/Link8"
HOLD_LINK_PATH = "/World/uarm1/swiftpro/Link8"
ATTACH_PATH = "/World/uarm1/swiftpro/Link8/GripperFrame/AttachPoint0"
OBJ_PATH = "/World/Cell/Parts/Holder"
GRIPPOINT_PATH = "/World/Cell/Parts/Holder/GripPoint"
JOINT_PATH = "/World/uarm1/swiftpro/Link8/GripperFrame/PickAsIsFixedJoint"
THRESH_MM = 10.0
RETRY_EVERY_N_TICKS = 5
DROP_VELOCITY = (0.0, 0.0, -0.005)
# --- end config ---

# omni.ui colours are 0xAABBGGRR
STYLE = {
    "Window": {"background_color": 0xFF1A1A1A},
    "Label": {"color": 0xFFD9D9D9, "font_size": 14},
    "Button": {"background_color": 0xFF333333, "border_radius": 4},
    "Button.Label": {"color": 0xFFFFFFFF, "font_size": 15},
}
BUTTON_ON = {"Button": {"background_color": 0xFF00B976, "border_radius": 4},  # NVIDIA green #76B900
             "Button.Label": {"color": 0xFF000000, "font_size": 15}}
BUTTON_OFF = {"Button": {"background_color": 0xFF333333, "border_radius": 4},
              "Button.Label": {"color": 0xFFFFFFFF, "font_size": 15}}


def _quatf(qd):
    im = qd.GetImaginary()
    return Gf.Quatf(float(qd.GetReal()), Gf.Vec3f(float(im[0]), float(im[1]), float(im[2])))


def _is_attached(stage):
    # attached = joint prim exists, is enabled and still has a body1 target
    prim = stage.GetPrimAtPath(Sdf.Path(JOINT_PATH))
    if not prim.IsValid():
        return False
    joint = UsdPhysics.Joint(prim)
    enabled = True
    if joint and joint.GetJointEnabledAttr():
        value = joint.GetJointEnabledAttr().Get()
        enabled = bool(value) if value is not None else True
    targets = joint.GetBody1Rel().GetTargets() if joint and joint.GetBody1Rel() else []
    return enabled and len(targets) > 0


def _release(stage):
    path = Sdf.Path(JOINT_PATH)
    prim = stage.GetPrimAtPath(path)
    if not prim.IsValid():
        return
    # disable and clear first (robust while playing), then remove the prim
    joint = UsdPhysics.Joint(prim)
    if joint and joint.GetJointEnabledAttr():
        joint.GetJointEnabledAttr().Set(False)
    if joint and joint.GetBody0Rel():
        joint.GetBody0Rel().ClearTargets()
    if joint and joint.GetBody1Rel():
        joint.GetBody1Rel().ClearTargets()
    stage.RemovePrim(path)
    part = stage.GetPrimAtPath(Sdf.Path(OBJ_PATH))
    if part.IsValid():
        body = UsdPhysics.RigidBodyAPI(part)
        if body:
            body.CreateVelocityAttr().Set(Gf.Vec3f(*DROP_VELOCITY))
    print("[" + ROBOT_ID + "] Released")


def _try_grab(stage):
    link8, hold, attach = Sdf.Path(LINK8_PATH), Sdf.Path(HOLD_LINK_PATH), Sdf.Path(ATTACH_PATH)
    part, grip_point, joint_path = Sdf.Path(OBJ_PATH), Sdf.Path(GRIPPOINT_PATH), Sdf.Path(JOINT_PATH)
    for path in (link8, hold, attach, part, grip_point):
        if not stage.GetPrimAtPath(path).IsValid():
            return

    t = Usd.TimeCode.Default()
    meters_per_unit = UsdGeom.GetStageMetersPerUnit(stage)
    threshold = (float(THRESH_MM) * 1e-3) / float(meters_per_unit)

    m_link8 = UsdGeom.Xformable(stage.GetPrimAtPath(link8)).ComputeLocalToWorldTransform(t)
    m_hold = UsdGeom.Xformable(stage.GetPrimAtPath(hold)).ComputeLocalToWorldTransform(t)
    m_part = UsdGeom.Xformable(stage.GetPrimAtPath(part)).ComputeLocalToWorldTransform(t)
    m_grip = UsdGeom.Xformable(stage.GetPrimAtPath(grip_point)).ComputeLocalToWorldTransform(t)

    # tool point = AttachPoint0.localPos0 on Link8
    local_tool = UsdPhysics.Joint(stage.GetPrimAtPath(attach)).GetLocalPos0Attr().Get()
    tool_world = m_link8.TransformAffine(Gf.Vec3d(local_tool[0], local_tool[1], local_tool[2]))
    distance = (m_grip.ExtractTranslation() - tool_world).GetLength()
    if distance > threshold:
        return

    # freeze the current relative pose: joint frame = GripPoint orientation, no yaw snap
    q_joint = m_grip.RemoveScaleShear().ExtractRotationQuat()
    q0 = m_hold.RemoveScaleShear().ExtractRotationQuat().GetInverse() * q_joint
    q1 = m_part.RemoveScaleShear().ExtractRotationQuat().GetInverse() * q_joint
    local_hold = m_hold.GetInverse().Transform(tool_world)
    local_part = m_part.GetInverse().Transform(tool_world)

    if stage.GetPrimAtPath(joint_path).IsValid():
        stage.RemovePrim(joint_path)
    joint = UsdPhysics.FixedJoint.Define(stage, joint_path)
    joint.CreateJointEnabledAttr().Set(True)
    joint.CreateExcludeFromArticulationAttr().Set(True)
    joint.CreateBody0Rel().SetTargets([hold])
    joint.CreateBody1Rel().SetTargets([part])
    joint.CreateLocalPos0Attr().Set(Gf.Vec3f(float(local_hold[0]), float(local_hold[1]), float(local_hold[2])))
    joint.CreateLocalRot0Attr().Set(_quatf(q0))
    joint.CreateLocalPos1Attr().Set(Gf.Vec3f(float(local_part[0]), float(local_part[1]), float(local_part[2])))
    joint.CreateLocalRot1Attr().Set(_quatf(q1))
    joint.CreateBreakForceAttr().Set(float("inf"))
    joint.CreateBreakTorqueAttr().Set(float("inf"))

    body = UsdPhysics.RigidBodyAPI(stage.GetPrimAtPath(part))
    if body and body.GetVelocityAttr():
        body.GetVelocityAttr().Set(Gf.Vec3f(0.0, 0.0, 0.0))
    print("[" + ROBOT_ID + "] Grabbed at %.3f mm" % (distance * float(meters_per_unit) * 1000.0))


def _build_window(state):
    def toggle():
        state.vacuum_on = not state.vacuum_on
        if state.btn:
            state.btn.text = ROBOT_ID + " " + TOOL_NAME + ": " + ("ON" if state.vacuum_on else "OFF")
            state.btn.set_style(BUTTON_ON if state.vacuum_on else BUTTON_OFF)
        print("[" + ROBOT_ID + "] " + TOOL_NAME + " " + ("ON" if state.vacuum_on else "OFF"))

    state.win = ui.Window(ROBOT_ID + " " + TOOL_NAME, width=300, height=130)
    state.win.frame.set_style(STYLE)
    with state.win.frame:
        with ui.VStack(spacing=8, height=0):
            ui.Label("Auto-grab when the tool point is close")
            state.btn = ui.Button(ROBOT_ID + " " + TOOL_NAME + ": OFF", height=34, clicked_fn=toggle)
            ui.Label("Threshold: %.1f mm" % float(THRESH_MM))


def setup(db):
    state = db.per_instance_state
    state.stage = omni.usd.get_context().get_stage()
    state.ui_built = False
    state.vacuum_on = False
    state.tick = 0
    state.win = None
    state.btn = None


def compute(db):
    state = db.per_instance_state
    state.tick += 1
    if not state.ui_built:
        _build_window(state)
        state.ui_built = True

    attached = _is_attached(state.stage)
    if not state.vacuum_on:
        if attached:
            _release(state.stage)
        return True
    if attached:
        return True
    if state.tick % int(RETRY_EVERY_N_TICKS) == 0:
        _try_grab(state.stage)
    return True
