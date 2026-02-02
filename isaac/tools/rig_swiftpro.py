"""Physics rig for an imported uArm Swift Pro - run in the Isaac Sim Script Editor.

Import ``assets/swiftpro/swiftpro_twin.urdf`` (and ``swiftpro_twin_gripper.urdf`` for arm 2) with the
URDF importer, open the written robot USD, run this script and save. Run it on the cell stage instead
to rig every ``/World/uarm*/swiftpro`` prim in place.

It applies the values of the working twin:

* PD drives on the joints the servos move: Joint1 (base), Joint2 (shoulder), Joint6 (elbow crank) and,
  on the gripper arm, JointGripperExt (wrist);
* no drives on the passive linkage joints (Joint3, 4, 5, 7, 8, 9) - they follow the closed loops;
* three loop-closure revolute joints, excluded from the articulation tree because PhysX articulations
  must be trees: rear parallelogram (Link5-Link4), elbow drive rod (Link7-Link3) and the
  level-keeping parallelogram (Link9-Link4);
* articulation solver settings: 32 position / 1 velocity iterations, no self-collision;
* the suction tool point ``Link8/GripperFrame/AttachPoint0`` that the vacuum graph uses as reference.
"""

import omni.usd
from pxr import Gf, PhysxSchema, Usd, UsdGeom, UsdPhysics

ROBOT_ROOTS = None  # None = every /World/uarm*/swiftpro, otherwise the default prim of a robot USD

DRIVES = {
    # joint: lower/upper limit (deg), PD gains, force limit, joint friction, max joint velocity (deg/s)
    "Joint1": {"limits": (-90.0, 90.0), "stiffness": 10000.0, "damping": 1000.0, "max_force": 1.0e7,
               "friction": 0.05, "max_velocity": 1000.0},
    "Joint2": {"limits": (-29.972, 89.954), "stiffness": 10000.0, "damping": 1000.0, "max_force": 1.0e7,
               "friction": 0.05, "max_velocity": 1000.0},
    "Joint6": {"limits": (-17.0, 110.0), "stiffness": 2000.0, "damping": 200.0, "max_force": 1.0e6,
               "friction": 0.05, "max_velocity": 1000.0},
    "JointGripperExt": {"limits": (-90.0, 90.0), "stiffness": 20000.0, "damping": 5000.0, "max_force": 1.0e5,
                        "friction": 0.02, "max_velocity": 500.0},
}
PASSIVE_JOINTS = {"Joint3": (-83.079, 83.079), "Joint4": None, "Joint5": None, "Joint7": None, "Joint8": None,
                  "Joint9": None}
PASSIVE_FRICTION = 0.1

# pin positions in the frame of each connected link (metres), see isaac_uarm_twin/kinematics.py
LOOP_JOINTS = {
    "Loop_Link5_Link4": {"parent": "Link5", "body0": "Link4", "body1": "Link5",
                         "pos0": (-0.0347, 0.01619, 0.01661), "pos1": (0.0, 0.01619, 0.14197)},
    "Loop_Link7_Link3": {"parent": "Link7", "body0": "Link3", "body1": "Link7",
                         "pos0": (-0.04501, -0.00322, -0.00271), "pos1": (0.00049, -0.00322, 0.14237)},
    "Loop_Link9_Link4": {"parent": "Link9", "body0": "Link4", "body1": "Link9",
                         "pos0": (0.02765, 0.01614, 0.02708), "pos1": (-0.15828, 0.01614, 0.00005)},
}

TOOL_POINT = (0.056381, 0.000173, -0.073327)  # suction tip in Link8 coordinates
SOLVER_POSITION_ITERATIONS = 32
SOLVER_VELOCITY_ITERATIONS = 1


def find(root, name):
    for prim in Usd.PrimRange(root):
        if prim.GetName() == name:
            return prim
    return None


def robot_roots(stage):
    if ROBOT_ROOTS:
        return [stage.GetPrimAtPath(path) for path in ROBOT_ROOTS]
    roots = []
    world = stage.GetPrimAtPath("/World")
    if world and world.IsValid():
        for child in world.GetChildren():
            robot = child.GetChild("swiftpro")
            if child.GetName().startswith("uarm") and robot and robot.IsValid():
                roots.append(robot)
    if not roots:
        default = stage.GetDefaultPrim()
        if default and default.IsValid() and find(default, "Link8") is not None:
            roots.append(default)
    return roots


def rig_drives(root, report):
    for name, spec in DRIVES.items():
        prim = find(root, name)
        if prim is None:
            continue
        joint = UsdPhysics.RevoluteJoint(prim)
        joint.CreateLowerLimitAttr().Set(spec["limits"][0])
        joint.CreateUpperLimitAttr().Set(spec["limits"][1])
        drive = UsdPhysics.DriveAPI.Apply(prim, "angular")
        drive.CreateTypeAttr().Set("force")
        drive.CreateStiffnessAttr().Set(spec["stiffness"])
        drive.CreateDampingAttr().Set(spec["damping"])
        drive.CreateMaxForceAttr().Set(spec["max_force"])
        physx = PhysxSchema.PhysxJointAPI.Apply(prim)
        physx.CreateJointFrictionAttr().Set(spec["friction"])
        physx.CreateMaxJointVelocityAttr().Set(spec["max_velocity"])
        report.append(f"drive   {name:16s} k={spec['stiffness']:.0f} d={spec['damping']:.0f} limits={spec['limits']}")


def rig_passive(root, report):
    for name, limits in PASSIVE_JOINTS.items():
        prim = find(root, name)
        if prim is None:
            continue
        if prim.HasAPI(UsdPhysics.DriveAPI, "angular"):
            prim.RemoveAPI(UsdPhysics.DriveAPI, "angular")
        if limits:
            joint = UsdPhysics.RevoluteJoint(prim)
            joint.CreateLowerLimitAttr().Set(limits[0])
            joint.CreateUpperLimitAttr().Set(limits[1])
        PhysxSchema.PhysxJointAPI.Apply(prim).CreateJointFrictionAttr().Set(PASSIVE_FRICTION)
        report.append(f"passive {name:16s} free, friction {PASSIVE_FRICTION}")


def rig_loops(stage, root, report):
    identity = Gf.Quatf(1.0, 0.0, 0.0, 0.0)
    for name, spec in LOOP_JOINTS.items():
        parent, body0, body1 = (find(root, spec[key]) for key in ("parent", "body0", "body1"))
        if None in (parent, body0, body1):
            report.append(f"skip    {name} (links missing)")
            continue
        joint = UsdPhysics.RevoluteJoint.Define(stage, parent.GetPath().AppendChild(name))
        joint.CreateAxisAttr().Set("Y")
        joint.CreateBody0Rel().SetTargets([body0.GetPath()])
        joint.CreateBody1Rel().SetTargets([body1.GetPath()])
        joint.CreateLocalPos0Attr().Set(Gf.Vec3f(*spec["pos0"]))
        joint.CreateLocalPos1Attr().Set(Gf.Vec3f(*spec["pos1"]))
        joint.CreateLocalRot0Attr().Set(identity)
        joint.CreateLocalRot1Attr().Set(identity)
        joint.CreateExcludeFromArticulationAttr().Set(True)
        report.append(f"loop    {name:16s} {spec['body0']} <-> {spec['body1']}")


def rig_articulation(root, report):
    for prim in Usd.PrimRange(root):
        if prim.HasAPI(UsdPhysics.ArticulationRootAPI):
            physx = PhysxSchema.PhysxArticulationAPI.Apply(prim)
            physx.CreateEnabledSelfCollisionsAttr().Set(False)
            physx.CreateSolverPositionIterationCountAttr().Set(SOLVER_POSITION_ITERATIONS)
            physx.CreateSolverVelocityIterationCountAttr().Set(SOLVER_VELOCITY_ITERATIONS)
            report.append(f"solver  {prim.GetName()}: {SOLVER_POSITION_ITERATIONS}/{SOLVER_VELOCITY_ITERATIONS} "
                          "iterations, self-collision off")
            return


def rig_tool_point(stage, root, report):
    link8 = find(root, "Link8")
    if link8 is None:
        return
    frame = UsdGeom.Xform.Define(stage, link8.GetPath().AppendChild("GripperFrame"))
    marker = UsdPhysics.Joint.Define(stage, frame.GetPath().AppendChild("AttachPoint0"))
    marker.CreateBody0Rel().SetTargets([link8.GetPath()])
    marker.CreateLocalPos0Attr().Set(Gf.Vec3f(*TOOL_POINT))
    marker.CreateExcludeFromArticulationAttr().Set(True)
    marker.CreateJointEnabledAttr().Set(False)  # reference point only, never a constraint
    report.append("tool    Link8/GripperFrame/AttachPoint0")


def main():
    stage = omni.usd.get_context().get_stage()
    roots = [root for root in robot_roots(stage) if root and root.IsValid()]
    if not roots:
        print("[Rig] No robot found (expected /World/uarm*/swiftpro or an imported Swift Pro USD).")
        return
    for root in roots:
        report = []
        rig_drives(root, report)
        rig_passive(root, report)
        rig_loops(stage, root, report)
        rig_articulation(root, report)
        rig_tool_point(stage, root, report)
        print(f"[Rig] {root.GetPath()}")
        for line in report:
            print("      " + line)
    print("[Rig] Done - save the stage to keep the rig.")


main()
