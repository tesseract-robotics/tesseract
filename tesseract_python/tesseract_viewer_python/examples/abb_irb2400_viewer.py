import tesseract
import os
import re
import traceback
from tesseract_viewer import TesseractViewer
import numpy as np
import time
import sys

TESSERACT_SUPPORT_DIR = os.environ["TESSERACT_SUPPORT_DIR"]

with open(os.path.join(TESSERACT_SUPPORT_DIR,"urdf","abb_irb2400.urdf"),'r') as f:
    abb_irb2400_urdf = f.read()

with open(os.path.join(TESSERACT_SUPPORT_DIR,"urdf","abb_irb2400.srdf"),'r') as f:
    abb_irb2400_srdf = f.read()

class TesseractSupportResourceLocator(tesseract.ResourceLocator):
    def __init__(self):
        super(TesseractSupportResourceLocator,self).__init__()

    def locateResource(self, url):
        
        url_match = re.match(r"^package:\/\/tesseract_support\/(.*)$",url)
        if (url_match is None):
            return None
        
        fname = os.path.join(TESSERACT_SUPPORT_DIR, os.path.normpath(url_match.group(1)))
        with open(fname,'rb') as f:
            resource_bytes = f.read()

        resource = tesseract.BytesResource(url, resource_bytes)

        return resource

t = tesseract.Tesseract()

t.init(abb_irb2400_urdf, abb_irb2400_srdf, TesseractSupportResourceLocator())

t_env = t.getEnvironment()

viewer = TesseractViewer()

viewer.update_environment(t_env, [0,0,0])

joint_names = ["joint_%d" % (i+1) for i in range(6)]
viewer.update_joint_positions(joint_names, np.array([1,-.2,.01,.3,-.5,1]))

viewer.start_serve_background()

t_env.setState(joint_names, np.ones(6)*0.1)

#while True:
#    time.sleep(2)
#    viewer.update_joint_positions(joint_names, np.random.rand(6)*.4)

pci = tesseract.ProblemConstructionInfo(t)

pci.init_info.type = tesseract.InitInfo.STATIONARY
#pci.init_info.data = np.array([0.0,0,0,0,0,0])

pci.basic_info.n_steps = 10
pci.basic_info.manip = "manipulator"
pci.basic_info.start_fixed = False
pci.basic_info.use_time = False
pci.basic_info.dt_upper_lim = 1
pci.basic_info.dt_lower_lim = 0.9999

pci.opt_info.max_iter = 200
pci.opt_info.min_approx_improve = 1e-3
pci.opt_info.min_trust_box_size = 1e-3

pci.kin = pci.getManipulator(pci.basic_info.manip)

start_pos_terminfo = tesseract.JointPosTermInfo()
start_pos_terminfo.name="start"
start_pos_terminfo.term_type = tesseract.TT_COST
#start_pos_terminfo.coeffs=np.ones(6)
start_pos_terminfo.first_step = 0
start_pos_terminfo.last_step = 0
#start_pos_terminfo.lower_tols=np.ones(6)*-0.01
#start_pos_terminfo.upper_tols=np.ones(6)*0.01

start_pos_terminfo.targets=np.ones(6)*0.1

end_pos_terminfo = tesseract.JointPosTermInfo()
end_pos_terminfo.name = "end"
end_pos_terminfo.term_type = tesseract.TT_COST
#end_pos_terminfo.coeffs=np.ones(6)
end_pos_terminfo.targets=np.ones(6)*0.5
end_pos_terminfo.first_step = pci.basic_info.n_steps-1
end_pos_terminfo.last_step = pci.basic_info.n_steps-1
#end_pos_terminfo.lower_tols=np.ones(6)*-0.01
#end_pos_terminfo.upper_tols=np.ones(6)*0.01

joint_vel = tesseract.JointVelTermInfo()
joint_vel.coeffs = np.ones(6)
joint_vel.targets = np.zeros(6)
joint_vel.first_step = 0
joint_vel.last_step = pci.basic_info.n_steps - 1
joint_vel.name = "Joint_vel"
joint_vel.term_type = tesseract.TT_COST

time_terminfo = tesseract.TotalTimeTermInfo()
time_terminfo.coeff = 1
time_terminfo.term_type = tesseract.TT_COST | tesseract.TT_USE_TIME
time_terminfo.name = "time"

collision = tesseract.CollisionTermInfo()
collision.name = "collision"
collision.term_type = tesseract.TT_CNT
collision.continuous = True
collision.first_step = 0
collision.last_step = pci.basic_info.n_steps-2
collision.gap = 1
collision_info = tesseract.createSafetyMarginDataVector(pci.basic_info.n_steps, .0001, 40)
for i in collision_info:
    collision.info.append(i)

pci.cost_infos.append(start_pos_terminfo)
pci.cost_infos.append(end_pos_terminfo)
#pci.cost_infos.append(time_terminfo)
pci.cnt_infos.push_back(collision)
pci.cost_infos.push_back(joint_vel)

prob = tesseract.ConstructProblem(pci)
config = tesseract.TrajOptPlannerConfig(prob)

planner = tesseract.TrajOptMotionPlanner()

planner.setConfiguration(config)
planner_status_code, planner_response = planner.solve(True)
            
assert planner_status_code.value() == 0, "Planning failed"

print(planner_response)

print(planner_response.joint_trajectory.trajectory)

viewer.update_trajectory(planner_response.joint_trajectory, False, 5)

if sys.version_info[0] < 3:
    raw_input("press enter")
else:
    input("press enter")

