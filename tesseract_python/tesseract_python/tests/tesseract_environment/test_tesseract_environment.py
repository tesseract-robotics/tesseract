from tesseract import tesseract_scene_graph
from tesseract import tesseract_collision
from tesseract import tesseract_environment
from tesseract.tesseract_common import Isometry3d, Translation3d, AngleAxisd
from tesseract import tesseract_common
from tesseract import tesseract_collision
from tesseract import tesseract_collision_bullet
from tesseract import tesseract_urdf
import os
import re

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

def get_scene_graph():
    tesseract_support = os.environ["TESSERACT_SUPPORT_DIR"]
    path =  os.path.join(tesseract_support, "urdf/lbr_iiwa_14_r820.urdf")
    locator_fn = tesseract_scene_graph.SimpleResourceLocatorFn(_locate_resource)
    locator = tesseract_scene_graph.SimpleResourceLocator(locator_fn)    
    return tesseract_urdf.parseURDFFile(path, locator)

def get_srdf_model(scene_graph):
    tesseract_support = os.environ["TESSERACT_SUPPORT_DIR"]
    path =  os.path.join(tesseract_support, "urdf/lbr_iiwa_14_r820.srdf")
    srdf = tesseract_scene_graph.SRDFModel()    
    srdf.initFile(scene_graph, path)
    return srdf

def get_environment():
    scene_graph = get_scene_graph()
    assert scene_graph is not None

    srdf = get_srdf_model(scene_graph)
    assert srdf is not None

    env = tesseract_environment.Environment()
    assert env is not None

    assert env.getRevision() == 0

    success = env.init(scene_graph,srdf)
    assert success
    assert env.getRevision() == 2

    # env.init() now populates contact managers?

    """discrete_create_fn = tesseract_collision.DiscreteContactManagerFactoryCreateMethod(tesseract_collision_bullet.BulletDiscreteBVHManager.create)
    assert env.registerDiscreteContactManager(tesseract_collision_bullet.BulletDiscreteBVHManager.name(),
        discrete_create_fn)

    cont_create_fn = tesseract_collision.ContinuousContactManagerFactoryCreateMethod(tesseract_collision_bullet.BulletCastBVHManager.create)
    assert env.registerContinuousContactManager(tesseract_collision_bullet.BulletCastBVHManager.name(),
        cont_create_fn)

    env.setActiveDiscreteContactManager(tesseract_collision_bullet.BulletDiscreteBVHManager.name())
    env.setActiveContinuousContactManager(tesseract_collision_bullet.BulletCastBVHManager.name())"""

    return env

def test_env():
    get_environment()