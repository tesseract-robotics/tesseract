import tesseract
import os
import re
import numpy as np

def test_trajopt_motion_planner_default_config():
    t = _load_tesseract()

    config = tesseract.TrajOptPlannerDefaultConfig(t,"manipulator", "tool0", np.eye(4))

    config.params.cnt_tolerance = 1e-5

    for ind in range(10):
        tcp_pose = np.array([[0,0,1,0],[0,1,0,0],[-1,0,0, 0],[0,0,0,1]],dtype=np.float64)
        tcp_pose[0,3] = 0.5
        tcp_pose[1,3] = -0.2 + ind*.04
        tcp_pose[2,3] = 0.7
        
        waypoint = tesseract.CartesianWaypoint(tcp_pose, "base_link")
        
        config.target_waypoints.push_back(waypoint)

    config.target_waypoints[0].setIsCritical(True)
    config.target_waypoints[9].setIsCritical(True)
    
    planner = tesseract.TrajOptMotionPlanner()

    planner.setConfiguration(config)
    planner_status_code, planner_response = planner.solve(True)

    assert planner_status_code.value() == 0, "Planning failed"

    assert planner_response.joint_trajectory.trajectory.shape == (10,7)
    print(planner_response.joint_trajectory.trajectory)

    




def test_trajopt_motion_planner_pci():
    
    t = _load_tesseract()

    pci = tesseract.ProblemConstructionInfo(t)

    pci.init_info.type = tesseract.InitInfo.STATIONARY
    #pci.init_info.data = np.array([0.0,0,0,0,0,0])

    pci.basic_info.n_steps = 10
    pci.basic_info.manip = "manipulator"
    pci.basic_info.start_fixed = True
    pci.basic_info.use_time = False

    pci.opt_info.max_iter = 10000
    pci.opt_info.min_approx_improve = 1e-3
    pci.opt_info.min_trust_box_size = 1e-3

    pci.kin = pci.getManipulator(pci.basic_info.manip)

    start_pos_terminfo = tesseract.JointPosTermInfo()
    start_pos_terminfo.name="start"
    start_pos_terminfo.term_type = tesseract.TT_CNT
    #start_pos_terminfo.coeffs=np.ones(6)
    start_pos_terminfo.first_step = 0
    start_pos_terminfo.last_step = 1

    start_pos_terminfo.targets=np.random.rand(7)*0.0

    end_pos_terminfo = tesseract.JointPosTermInfo()
    end_pos_terminfo.name = "end"
    end_pos_terminfo.term_type = tesseract.TT_CNT
    #end_pos_terminfo.coeffs=np.ones(6)
    end_pos_terminfo.targets=np.random.rand(7)*0.1
    end_pos_terminfo.first_step = pci.basic_info.n_steps-1
    end_pos_terminfo.last_step = pci.basic_info.n_steps-1

    joint_vel = tesseract.JointVelTermInfo()
    joint_vel.coeffs = np.ones(7)
    joint_vel.targets = np.zeros(7)
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

    pci.cnt_infos.append(start_pos_terminfo)
    pci.cnt_infos.append(end_pos_terminfo)
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
        

    #TODO: test more stuff...
    

def _load_tesseract():
    TESSERACT_SUPPORT_DIR = os.environ["TESSERACT_SUPPORT_DIR"]

    with open(os.path.join(TESSERACT_SUPPORT_DIR,"urdf","lbr_iiwa_14_r820.urdf"),'r') as f:
        robot_urdf = f.read()

    with open(os.path.join(TESSERACT_SUPPORT_DIR,"urdf","lbr_iiwa_14_r820.srdf"),'r') as f:
        robot_srdf = f.read()

    t = tesseract.Tesseract()

    t.init(robot_urdf, robot_srdf, TesseractSupportResourceLocator())

    return t

class TesseractSupportResourceLocator(tesseract.ResourceLocator):
    def __init__(self):
        super(TesseractSupportResourceLocator,self).__init__()
        self.TESSERACT_SUPPORT_DIR = os.environ["TESSERACT_SUPPORT_DIR"]

    def locateResource(self, url):
        
        url_match = re.match(r"^package:\/\/tesseract_support\/(.*)$",url)
        if (url_match is None):
            return None
        
        fname = os.path.join(self.TESSERACT_SUPPORT_DIR, os.path.normpath(url_match.group(1)))
        with open(fname,'rb') as f:
            resource_bytes = f.read()

        resource = tesseract.BytesResource(url, resource_bytes)

        return resource

