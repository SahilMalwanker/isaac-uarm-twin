"""Release a part grabbed with ``grab_part.py`` - run in the Isaac Sim Script Editor."""

import omni.usd
from pxr import Sdf

JOINT_PATH = "/World/uarm1/swiftpro/Link8/GripperFrame/PickAsIsFixedJoint"

stage = omni.usd.get_context().get_stage()
path = Sdf.Path(JOINT_PATH)
if stage.GetPrimAtPath(path).IsValid():
    stage.RemovePrim(path)
    print("Released:", path)
else:
    print("Nothing attached at", path)
