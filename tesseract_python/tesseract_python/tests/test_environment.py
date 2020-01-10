import tesseract
import os
import re
import numpy as np

def test_environment():
    
    TESSERACT_SUPPORT_DIR = os.environ["TESSERACT_SUPPORT_DIR"]

    with open(os.path.join(TESSERACT_SUPPORT_DIR,"urdf","abb_irb2400.urdf"),'r') as f:
        abb_irb2400_urdf = f.read()

    with open(os.path.join(TESSERACT_SUPPORT_DIR,"urdf","abb_irb2400.srdf"),'r') as f:
        abb_irb2400_srdf = f.read()

    t = tesseract.Tesseract()

    t.init(abb_irb2400_urdf, abb_irb2400_srdf, TesseractSupportResourceLocator())

    t_env = t.getEnvironment()
    
    assert sorted(t_env.getJointNames()) == sorted(('base_link-base', 'joint_6-tool0', 'joint_2', 'joint_1', 'joint_5', 'joint_4', 'joint_3', 'joint_6'))
    assert sorted(t_env.getLinkNames()) == sorted(('base', 'tool0', 'link_1', 'base_link', 'link_2', 'link_4', 'link_6', 'link_3', 'link_5'))
    assert t_env.getRootLinkName() == 'base_link'

    l = t_env.getLink('link_4')
    assert l.getName() == 'link_4'
    assert len(l.visual) == 1
    assert len(l.collision) == 1

    visual = l.visual[0]
    collision = l.collision[0]

    np.testing.assert_allclose(visual.origin, np.eye(4))
    assert len(visual.geometry.getVertices()) == 631
    assert len(visual.geometry.getTriangles()) == 5032
    np.testing.assert_allclose(visual.material.color.flatten(), [0.7372549, 0.3490196, 0.1607843, 1],rtol=1e-4)

    np.testing.assert_allclose(collision.origin, np.eye(4))

    j = t_env.getJoint('joint_3')
    assert j.type == 1
    assert j.getName() == 'joint_3'
    assert j.parent_link_name == 'link_2'
    assert j.child_link_name == 'link_3'
    np.testing.assert_allclose(j.axis.flatten(), [0,1,0])
    j_origin = np.eye(4)
    j_origin[2,3] = 0.705 
    np.testing.assert_allclose(j.parent_to_joint_origin_transform, j_origin)
    assert j.limits.lower == -1.0472
    assert j.limits.upper == 1.1345
    assert j.limits.velocity == 2.618
    assert j.limits.effort == 0

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

