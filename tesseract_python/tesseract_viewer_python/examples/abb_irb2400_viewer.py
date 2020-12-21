from tesseract.tesseract_common import FilesystemPath, Isometry3d, Translation3d, Quaterniond, \
    ManipulatorInfo
from tesseract.tesseract_environment import Environment
from tesseract.tesseract_scene_graph import SimpleResourceLocator, SimpleResourceLocatorFn
from tesseract.tesseract_command_language import CartesianWaypoint, Waypoint, \
    PlanInstructionType_FREESPACE, PlanInstructionType_START, PlanInstruction, Instruction, \
    CompositeInstruction, flatten
from tesseract.tesseract_process_managers import ProcessPlanningServer, ProcessPlanningRequest, \
    FREESPACE_PLANNER_NAME
import os
import re
import traceback
from tesseract_viewer import TesseractViewer
import numpy as np
import time
import sys

TESSERACT_SUPPORT_DIR = os.environ["TESSERACT_SUPPORT_DIR"]

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

abb_irb2400_urdf_fname = FilesystemPath(os.path.join(TESSERACT_SUPPORT_DIR,"urdf","abb_irb2400.urdf"))
abb_irb2400_srdf_fname = FilesystemPath(os.path.join(TESSERACT_SUPPORT_DIR,"urdf","abb_irb2400.srdf"))

t_env = Environment()

# locator_fn must be kept alive by maintaining a reference
locator_fn = SimpleResourceLocatorFn(_locate_resource)
t_env.init(abb_irb2400_urdf_fname, abb_irb2400_srdf_fname, SimpleResourceLocator(locator_fn))

manip_info = ManipulatorInfo()
manip_info.manipulator = "manipulator"

viewer = TesseractViewer()

viewer.update_environment(t_env, [0,0,0])

joint_names = ["joint_%d" % (i+1) for i in range(6)]
viewer.update_joint_positions(joint_names, np.array([1,-.2,.01,.3,-.5,1]))

viewer.start_serve_background()

t_env.setState(joint_names, np.ones(6)*0.1)

wp1 = CartesianWaypoint(Isometry3d.Identity() * Translation3d(.6,-.8,0.6) * Quaterniond(0,0,1.0,0))
wp2 = CartesianWaypoint(Isometry3d.Identity() * Translation3d(.4,.8,1.5) * Quaterniond(0.7071,0,0.7071,0))

start_instruction = PlanInstruction(Waypoint(wp1), PlanInstructionType_START, "DEFAULT")
plan_f1 = PlanInstruction(Waypoint(wp2), PlanInstructionType_FREESPACE, "DEFAULT")

program = CompositeInstruction("DEFAULT")
program.setStartInstruction(Instruction(start_instruction))
program.setManipulatorInfo(manip_info)
program.append(Instruction(plan_f1))

planning_server = ProcessPlanningServer(t_env, 1)
planning_server.loadDefaultProcessPlanners()
request = ProcessPlanningRequest()
request.name = FREESPACE_PLANNER_NAME
request.instructions = Instruction(program)

response = planning_server.run(request)
planning_server.waitForAll()

assert response.interface.isSuccessful()

results = flatten(response.getResults().cast_CompositeInstruction())

viewer.update_trajectory(results)

if sys.version_info[0] < 3:
    input("press enter")
else:
    input("press enter")

