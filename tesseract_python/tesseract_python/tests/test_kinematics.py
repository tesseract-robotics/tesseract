import tesseract
import os
import re
import numpy as np

def test_fwd_kin():
    
    TESSERACT_SUPPORT_DIR = os.environ["TESSERACT_SUPPORT_DIR"]

    with open(os.path.join(TESSERACT_SUPPORT_DIR,"urdf","abb_irb2400.urdf"),'r') as f:
        abb_irb2400_urdf = f.read()

    with open(os.path.join(TESSERACT_SUPPORT_DIR,"urdf","abb_irb2400.srdf"),'r') as f:
        abb_irb2400_srdf = f.read()

    t = tesseract.Tesseract()

    t.init(abb_irb2400_urdf, abb_irb2400_srdf, TesseractSupportResourceLocator())

    fwd_kin_manager = t.getFwdKinematicsManager()
    fwd_kin = fwd_kin_manager.getFwdKinematicSolver("manipulator")
    tcp_pose = fwd_kin.calcFwdKin(np.deg2rad([15,-30,-20,5,10,-90]))

    np.testing.assert_allclose(tcp_pose, [[ 0.32232436,  0.59589736,  0.73553609,  0.1875004 ],
                                          [-0.94497009,  0.24852968,  0.21275462,  0.05157239],
                                          [-0.05602263, -0.7636356,   0.64321235,  1.94536084],
                                          [ 0.,          0.,          0.,          1.        ]])

    

    #TODO: test more stuff...
    
def test_inv_kin():
    
    TESSERACT_SUPPORT_DIR = os.environ["TESSERACT_SUPPORT_DIR"]

    with open(os.path.join(TESSERACT_SUPPORT_DIR,"urdf","abb_irb2400.urdf"),'r') as f:
        abb_irb2400_urdf = f.read()

    with open(os.path.join(TESSERACT_SUPPORT_DIR,"urdf","abb_irb2400.srdf"),'r') as f:
        abb_irb2400_srdf = f.read()

    t = tesseract.Tesseract()

    t.init(abb_irb2400_urdf, abb_irb2400_srdf, TesseractSupportResourceLocator())

    tcp_pose = np.array([[0,0,1,0.8],[0,1,0,0],[-1,0,0, 1.2],[0,0,0,1]])

    inv_kin_manager = t.getInvKinematicsManager()
    inv_kin = inv_kin_manager.getInvKinematicSolver("manipulator")
    joint_angles = inv_kin.calcInvKin(tcp_pose, np.ones(6)*0.25)

    # Now check fwd_kin

    fwd_kin_manager = t.getFwdKinematicsManager()
    fwd_kin = fwd_kin_manager.getFwdKinematicSolver("manipulator")
    tcp_pose2 = fwd_kin.calcFwdKin(joint_angles)

    np.testing.assert_allclose(tcp_pose[0:3,3], tcp_pose2[0:3,3], atol=1e-5)

    #TODO: Handle tolerace issues with rotation

    #TODO: test more stuff...

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

