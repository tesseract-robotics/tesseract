import re
import traceback
import os
import numpy as np
import numpy.testing as nptest

from tesseract.tesseract_scene_graph import SimpleResourceLocator, SimpleResourceLocatorFn
from tesseract.tesseract_environment import Environment
from tesseract.tesseract_common import FilesystemPath, Isometry3d, Translation3d, Quaterniond, \
    ManipulatorInfo
from tesseract.tesseract_command_language import JointWaypoint, CartesianWaypoint, Waypoint, \
    PlanInstructionType_FREESPACE, PlanInstructionType_START, PlanInstruction, Instruction, \
    isMoveInstruction, isStateWaypoint, CompositeInstruction, flatten, isMoveInstruction, isStateWaypoint
from tesseract.tesseract_process_managers import ProcessPlanningServer, ProcessPlanningRequest, \
    FREESPACE_PLANNER_NAME

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
    
    return tesseract, manip_info

def test_planning_server_freespace():

    tesseract, manip = get_tesseract()

    joint_names = ["joint_a1", "joint_a2", "joint_a3", "joint_a4", "joint_a5", "joint_a6", "joint_a7"]

    wp1 = JointWaypoint(joint_names, np.array([0,0,0,-1.57,0,0,0],dtype=np.float64))
    wp2 = CartesianWaypoint(Isometry3d.Identity() * Translation3d(-.2,.4,0.2) * Quaterniond(0,0,1.0,0))

    start_instruction = PlanInstruction(Waypoint(wp1), PlanInstructionType_START, "DEFAULT")
    plan_f1 = PlanInstruction(Waypoint(wp2), PlanInstructionType_FREESPACE, "DEFAULT")

    program = CompositeInstruction("DEFAULT")
    program.setStartInstruction(Instruction(start_instruction))
    program.setManipulatorInfo(manip)
    program.append(Instruction(plan_f1))

    planning_server = ProcessPlanningServer(tesseract, 1)
    planning_server.loadDefaultProcessPlanners()
    request = ProcessPlanningRequest()
    request.name = FREESPACE_PLANNER_NAME
    request.instructions = Instruction(program)

    response = planning_server.run(request)
    planning_server.waitForAll()

    assert response.interface.isSuccessful()

    results = flatten(response.getResults().cast_CompositeInstruction())

    assert len(results) == 37
    for instr in results:
        assert isMoveInstruction(instr)
        wp1 = instr.cast_MoveInstruction().getWaypoint()
        assert isStateWaypoint(wp1)
        wp = wp1.cast_StateWaypoint()
        assert len(wp.joint_names) == 7
        assert isinstance(wp.position,np.ndarray)
        assert len(wp.position) == 7
