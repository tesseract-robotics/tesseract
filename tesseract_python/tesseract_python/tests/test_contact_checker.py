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
    
    t_env.setState(["joint_3"], np.array([1.6]))

    contact_distance=0.2

    monitored_link_names = t_env.getLinkNames()
    manager = t_env.getDiscreteContactManager()
    manager.setActiveCollisionObjects(monitored_link_names)
    manager.setContactDistanceThreshold(contact_distance)

    env_state = t_env.getCurrentState()
    manager.setCollisionObjectsTransform(env_state.link_transforms)
    contacts = manager.contactTest(2)
    contact_vector = tesseract.flattenResults(contacts)

    assert len(contact_vector) == 3
    assert contact_vector[0].link_names == ('link_1', 'link_4')
    assert contact_vector[0].shape_id == (0,0)
    assert contact_vector[0].nearest_points[0].shape == (3,1)
    assert contact_vector[0].nearest_points[1].shape == (3,1)

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

