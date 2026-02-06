"""Grab a part "as is" with a fixed joint - run in the Isaac Sim Script Editor while playing.

Manual version of the vacuum graph: if the suction tip of Link8 is within ``THRESH_MM`` of the part's
grip point, a fixed joint locks the part to Link8 in its current relative pose (no snap, no yaw
jump). Release with ``release_part.py``.
"""

import omni.timeline
import omni.usd
from pxr import Gf, Sdf, Usd, UsdGeom, UsdPhysics

# --- config ---
LINK8_PATH = "/World/uarm1/swiftpro/Link8"
ATTACH_PATH = "/World/uarm1/swiftpro/Link8/GripperFrame/AttachPoint0"
OBJ_PATH = "/World/Cell/Parts/Holder"
GRIPPOINT_PATH = "/World/Cell/Parts/Holder/GripPoint"
JOINT_PATH = "/World/uarm1/swiftpro/Link8/GripperFrame/PickAsIsFixedJoint"
THRESH_MM = 4.0
# --- end config ---


def quatf(qd):
    im = qd.GetImaginary()
    return Gf.Quatf(float(qd.GetReal()), Gf.Vec3f(float(im[0]), float(im[1]), float(im[2])))


def grab():
    stage = omni.usd.get_context().get_stage()
    if not omni.timeline.get_timeline_interface().is_playing():
        raise RuntimeError("Press Play first.")
    link8, attach, part = Sdf.Path(LINK8_PATH), Sdf.Path(ATTACH_PATH), Sdf.Path(OBJ_PATH)
    grip_point, joint_path = Sdf.Path(GRIPPOINT_PATH), Sdf.Path(JOINT_PATH)
    for path in (link8, attach, part, grip_point):
        if not stage.GetPrimAtPath(path).IsValid():
            raise RuntimeError(f"Missing prim: {path}")

    t = Usd.TimeCode.Default()
    meters_per_unit = float(UsdGeom.GetStageMetersPerUnit(stage))
    threshold = (THRESH_MM * 1e-3) / meters_per_unit
    m_link8 = UsdGeom.Xformable(stage.GetPrimAtPath(link8)).ComputeLocalToWorldTransform(t)
    m_part = UsdGeom.Xformable(stage.GetPrimAtPath(part)).ComputeLocalToWorldTransform(t)
    m_grip = UsdGeom.Xformable(stage.GetPrimAtPath(grip_point)).ComputeLocalToWorldTransform(t)

    local_tool = UsdPhysics.Joint(stage.GetPrimAtPath(attach)).GetLocalPos0Attr().Get()
    tool_world = m_link8.TransformAffine(Gf.Vec3d(local_tool[0], local_tool[1], local_tool[2]))
    distance = (m_grip.ExtractTranslation() - tool_world).GetLength()
    print(f"Tool to grip point: {distance * meters_per_unit * 1000.0:.3f} mm (threshold {THRESH_MM:.3f} mm)")
    if distance > threshold:
        print("Too far - not grabbing.")
        return False

    q_joint = m_grip.RemoveScaleShear().ExtractRotationQuat()
    q0 = m_link8.RemoveScaleShear().ExtractRotationQuat().GetInverse() * q_joint
    q1 = m_part.RemoveScaleShear().ExtractRotationQuat().GetInverse() * q_joint
    local_part = m_part.GetInverse().Transform(tool_world)

    if stage.GetPrimAtPath(joint_path).IsValid():
        stage.RemovePrim(joint_path)
    joint = UsdPhysics.FixedJoint.Define(stage, joint_path)
    joint.CreateJointEnabledAttr().Set(True)
    joint.CreateExcludeFromArticulationAttr().Set(True)
    joint.CreateBody0Rel().SetTargets([link8])
    joint.CreateBody1Rel().SetTargets([part])
    joint.CreateLocalPos0Attr().Set(Gf.Vec3f(float(local_tool[0]), float(local_tool[1]), float(local_tool[2])))
    joint.CreateLocalRot0Attr().Set(quatf(q0))
    joint.CreateLocalPos1Attr().Set(Gf.Vec3f(float(local_part[0]), float(local_part[1]), float(local_part[2])))
    joint.CreateLocalRot1Attr().Set(quatf(q1))
    joint.CreateBreakForceAttr().Set(float("inf"))
    joint.CreateBreakTorqueAttr().Set(float("inf"))
    print("Grabbed (pick-as-is):", JOINT_PATH)
    return True


grab()
