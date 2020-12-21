import re
import traceback
import os
import numpy as np

from tesseract.tesseract_scene_graph import SimpleResourceLocator, SimpleResourceLocatorFn
from tesseract.tesseract_environment import Environment
from tesseract.tesseract_common import FilesystemPath, ManipulatorInfo
from tesseract.tesseract_command_language import JointWaypoint, CartesianWaypoint, Waypoint, \
    PlanInstructionType_FREESPACE, PlanInstruction, Instruction, isMoveInstruction, isStateWaypoint
from tesseract.tesseract_motion_planners import PlannerRequest
from tesseract.tesseract_motion_planners_simple import LVSInterpolateStateWaypoint

def _locate_resource(url):
    try:
        url_match = re.match(r"^package:\/\/tesseract_support\/(.*)$",url)
        if (url_match is None):
            return ""    
        if not "TESSERACT_SUPPORT_DIR" in os.environ:
            return ""
        tesseract_support = os.environ["TESSERACT_SUPPORT_DIR"]
        return os.path.join(tesseract_support, os.path.normpath(url_match.group(1)))
    except:
        traceback.print_exc()

def get_tesseract():
    locate_resource_fn = SimpleResourceLocatorFn(_locate_resource)
    locator = SimpleResourceLocator(locate_resource_fn)
    tesseract = Environment()
    tesseract_support = os.environ["TESSERACT_SUPPORT_DIR"]
    urdf_path = FilesystemPath(os.path.join(tesseract_support, "urdf/lbr_iiwa_14_r820.urdf"))
    srdf_path = FilesystemPath(os.path.join(tesseract_support, "urdf/lbr_iiwa_14_r820.srdf"))
    assert tesseract.init(urdf_path, srdf_path, locator)
    manip_info = ManipulatorInfo()
    manip_info.manipulator = "manipulator"
    joint_names = tesseract.getManipulatorManager().getFwdKinematicSolver("manipulator").getJointNames()

    return tesseract, manip_info, joint_names

def test_get_tesseract():
    get_tesseract()

def test_interpolatestatewaypoint_jointcart_freespace():
    tesseract, manip_info, joint_names = get_tesseract()

    request = PlannerRequest()
    request.env = tesseract
    request.env_state = tesseract.getCurrentState()
    fwd_kin = tesseract.getManipulatorManager().getFwdKinematicSolver(manip_info.manipulator)
    wp1 = JointWaypoint(joint_names, np.zeros((7,),dtype=np.float64))
    wp2 = CartesianWaypoint(fwd_kin.calcFwdKin(np.ones((7,),dtype=np.float64))[1])
    instr = PlanInstruction(Waypoint(wp1), PlanInstructionType_FREESPACE, "TEST_PROFILE", manip_info)

    composite = LVSInterpolateStateWaypoint(wp1,wp2,instr,request,ManipulatorInfo(),3.14,0.5,1.57,5)

    for c in composite:
        assert isMoveInstruction(c)
        assert isStateWaypoint(c.cast_MoveInstruction().getWaypoint())
        assert c.cast_MoveInstruction().getProfile() == instr.getProfile()

    mi = composite[-1].cast_const_MoveInstruction()
    last_position = mi.getWaypoint().cast_const_StateWaypoint().position
    _, final_pose = fwd_kin.calcFwdKin(last_position)
    assert wp2.isApprox(final_pose, 1e-3)