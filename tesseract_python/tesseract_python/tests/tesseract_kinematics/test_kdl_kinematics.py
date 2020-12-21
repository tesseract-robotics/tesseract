from tesseract import tesseract_kinematics
from tesseract import tesseract_scene_graph
from tesseract import tesseract_kinematics_kdl
from tesseract import tesseract_urdf
from tesseract import tesseract_common

import re
import os
import traceback
import numpy as np
import numpy.testing as nptest

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


def run_active_link_names_test(kin, is_kin_tree):
    link_names = kin.getActiveLinkNames()
    assert len(link_names) == 8
    assert "base_link" not in link_names
    assert "link_1" in link_names
    assert "link_2" in link_names
    assert "link_3" in link_names
    assert "link_4" in link_names
    assert "link_5" in link_names
    assert "link_6" in link_names
    assert "link_7" in link_names
    assert "tool0" in link_names

    if not is_kin_tree:
        link_names = kin.getLinkNames()
        assert len(link_names) == 9
        assert "base_link" in link_names
        assert "link_1" in link_names
        assert "link_2" in link_names
        assert "link_3" in link_names
        assert "link_4" in link_names
        assert "link_5" in link_names
        assert "link_6" in link_names
        assert "link_7" in link_names
        assert "tool0" in link_names
    else:
        assert len(link_names) == 10
        assert "base_link" in link_names
        assert "base" in link_names
        assert "link_1" in link_names
        assert "link_2" in link_names
        assert "link_3" in link_names
        assert "link_4" in link_names
        assert "link_5" in link_names
        assert "link_6" in link_names
        assert "link_7" in link_names
        assert "tool0" in link_names



def test_kidl_kin_chain_active_link_names_unit():
    kin = tesseract_kinematics_kdl.KDLFwdKinChain()
    scene_graph = get_scene_graph()
    assert kin.init(scene_graph, "base_link", "tool0", "manip")

    run_active_link_names_test(kin, False)

def run_inv_kin_test(inv_kin, fwd_kin):

    pose = np.eye(4)
    pose[2,3] = 1.306

    seed = np.array([-0.785398, 0.785398, -0.785398, 0.785398, -0.785398, 0.785398, -0.785398])
    res, solutions = inv_kin.calcInvKin(tesseract_common.Isometry3d(pose),seed)
    assert res

    res, result = fwd_kin.calcFwdKin(solutions)
    assert res

    nptest.assert_almost_equal(pose,result.matrix(),decimal=3)

def test_kdl_kin_chain_lma_inverse_kinematic():
    inv_kin = tesseract_kinematics_kdl.KDLInvKinChainLMA()
    fwd_kin = tesseract_kinematics_kdl.KDLFwdKinChain()
    scene_graph = get_scene_graph()
    assert inv_kin.init(scene_graph, "base_link", "tool0", "manip")
    assert fwd_kin.init(scene_graph, "base_link", "tool0", "manip")

    run_inv_kin_test(inv_kin, fwd_kin)

def test_kdl_kin_chain_nr_inverse_kinematic():
    inv_kin = tesseract_kinematics_kdl.KDLInvKinChainLMA()
    fwd_kin = tesseract_kinematics_kdl.KDLFwdKinChain()
    scene_graph = get_scene_graph()
    assert inv_kin.init(scene_graph, "base_link", "tool0", "manip")
    assert fwd_kin.init(scene_graph, "base_link", "tool0", "manip")

    run_inv_kin_test(inv_kin, fwd_kin)

def test_jacobian():
    kin = tesseract_kinematics_kdl.KDLFwdKinChain()
    scene_graph = get_scene_graph()
    assert kin.init(scene_graph, "base_link", "tool0", "manip")

    jvals = np.array([-0.785398, 0.785398, -0.785398, 0.785398, -0.785398, 0.785398, -0.785398])

    link_name = "tool0"
    res, jacobian = kin.calcJacobian(jvals,link_name)
    assert res
    assert jacobian.shape == (6,7)

