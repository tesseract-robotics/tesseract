import tesseract.tesseract_scene_graph as sg
from tesseract import tesseract_common
import numpy as np
import re
import os

def _translation(p):
    H = np.eye(4)
    H[0:3,3] = p
    return tesseract_common.Isometry3d(H)

def test_tesseract_scene_graph():
    g = sg.SceneGraph()
    g.addLink(sg.Link("link_1"))
    g.addLink(sg.Link("link_2"))
    g.addLink(sg.Link("link_3"))
    g.addLink(sg.Link("link_4"))
    g.addLink(sg.Link("link_5"))

    base_joint = sg.Joint("base_joint")
    base_joint.parent_link_name = "base_link"
    base_joint.child_link_name = "link1"
    base_joint.type = sg.JointType_FIXED
    g.addJoint(base_joint)

    joint_1 = sg.Joint("joint_1")
    joint_1.parent_link_name = "link_1"
    joint_1.child_link_name = "link_2"
    joint_1.type = sg.JointType_FIXED
    g.addJoint(joint_1)

    joint_2 = sg.Joint("joint_2")
    joint_2.parent_to_joint_origin_transform = _translation([1.25,0,0])
    joint_2.parent_link_name = "link_2"
    joint_2.child_link_name = "link_3"
    joint_2.type = sg.JointType_PLANAR
    g.addJoint(joint_2)

    joint_3 = sg.Joint("joint_3")
    joint_3.parent_to_joint_origin_transform = _translation([1.25,0,0])
    joint_3.parent_link_name = "link_3"
    joint_3.child_link_name = "link_4"
    joint_3.type = sg.JointType_FLOATING
    g.addJoint(joint_3)

    joint_4 = sg.Joint("joint_4")
    joint_4.parent_to_joint_origin_transform = _translation([0,1.25,0])
    joint_4.parent_link_name = "link_2"
    joint_4.child_link_name = "link_5"
    joint_4.type = sg.JointType_REVOLUTE
    g.addJoint(joint_4)

    adjacent_links = g.getAdjacentLinkNames("link_3")
    assert len(adjacent_links) == 1
    assert adjacent_links[0] == "link_4"

    inv_adjacent_links = g.getInvAdjacentLinkNames("link_3")
    assert len(inv_adjacent_links) == 1
    assert inv_adjacent_links[0] == "link_2"

    child_link_names = g.getLinkChildrenNames("link_5")
    assert len(child_link_names) == 0

    child_link_names = g.getLinkChildrenNames("link_3")
    assert len(child_link_names) == 1
    assert child_link_names[0] == "link_4"

    child_link_names = g.getLinkChildrenNames("link_2")
    assert len(child_link_names) == 3
    assert "link_3" in child_link_names
    assert "link_4" in child_link_names
    assert "link_5" in child_link_names

    child_link_names = g.getJointChildrenNames("joint_4")
    assert len(child_link_names) == 1
    assert child_link_names[0] == "link_5"

    child_link_names = g.getJointChildrenNames("joint_3")
    assert len(child_link_names) == 1
    assert child_link_names[0] == "link_4"

    child_link_names = g.getJointChildrenNames("joint_1")
    assert len(child_link_names) == 4
    assert "link_2" in child_link_names
    assert "link_3" in child_link_names
    assert "link_4" in child_link_names
    assert "link_5" in child_link_names

    assert g.isAcyclic()
    assert g.isTree()

    g.addLink(sg.Link("link_6"))
    assert not g.isTree()

    g.removeLink("link_6")
    assert g.isTree()

    joint_5 = sg.Joint("joint_5")
    joint_5.parent_to_joint_origin_transform = _translation([0,1.5,0])
    joint_5.parent_link_name = "link_5"
    joint_5.child_link_name = "link_4"
    joint_5.type = sg.JointType_CONTINUOUS
    g.addJoint(joint_5)

    assert g.isAcyclic()
    assert not g.isTree()

    joint_6 = sg.Joint("joint_6")
    joint_6.parent_to_joint_origin_transform = _translation([0,1.25,0])
    joint_6.parent_link_name = "link_5"
    joint_6.child_link_name = "link_1"
    joint_6.type = sg.JointType_CONTINUOUS
    g.addJoint(joint_6)

    assert not g.isAcyclic()
    assert not g.isTree()

    path = g.getShortestPath("link_1", "link_4")
    
    assert len(path[0]) == 4
    assert "link_1" in path[0]
    assert "link_2" in path[0]
    assert "link_3" in path[0]
    assert "link_4" in path[0]
    assert len(path[1]) == 3
    assert "joint_1" in path[1]
    assert "joint_2" in path[1]
    assert "joint_3" in path[1]

    print(g.getName())

def _locate_resource(url):
    url_match = re.match(r"^package:\/\/tesseract_support\/(.*)$",url)
    if (url_match is None):
        return ""    
    if not "TESSERACT_SUPPORT_DIR" in os.environ:
        return ""
    tesseract_support = os.environ["TESSERACT_SUPPORT_DIR"]
    return os.path.join(tesseract_support, os.path.normpath(url_match.group(1)))

def test_load_srdf_unit():
    tesseract_support = os.environ["TESSERACT_SUPPORT_DIR"]
    srdf_file =  os.path.join(tesseract_support, "urdf/lbr_iiwa_14_r820.srdf")

    locator = sg.SimpleResourceLocator(sg.SimpleResourceLocatorFn(_locate_resource))

    g = sg.SceneGraph()

    g.setName("kuka_lbr_iiwa_14_r820")

    g.addLink(sg.Link("base_link"))
    g.addLink(sg.Link("link_1"))
    g.addLink(sg.Link("link_2"))
    g.addLink(sg.Link("link_3"))
    g.addLink(sg.Link("link_4"))
    g.addLink(sg.Link("link_5"))
    g.addLink(sg.Link("link_6"))
    g.addLink(sg.Link("link_7"))
    g.addLink(sg.Link("tool0"))
    
    base_joint = sg.Joint("base_joint")
    base_joint.parent_link_name = "base_link"
    base_joint.child_link_name = "link_1"
    base_joint.type = sg.JointType_FIXED
    g.addJoint(base_joint)

    joint_1 = sg.Joint("joint_1")
    joint_1.parent_link_name = "link_1"
    joint_1.child_link_name = "link_2"
    joint_1.type = sg.JointType_REVOLUTE
    g.addJoint(joint_1)

    joint_2 = sg.Joint("joint_2")
    joint_2.parent_to_joint_origin_transform = _translation([1.25,0,0])
    joint_2.parent_link_name = "link_2"
    joint_2.child_link_name = "link_3"
    joint_2.type = sg.JointType_REVOLUTE
    g.addJoint(joint_2)

    joint_3 = sg.Joint("joint_3")
    joint_3.parent_to_joint_origin_transform = _translation([1.25,0,0])
    joint_3.parent_link_name = "link_3"
    joint_3.child_link_name = "link_4"
    joint_3.type = sg.JointType_REVOLUTE
    g.addJoint(joint_3)

    joint_4 = sg.Joint("joint_4")
    joint_4.parent_to_joint_origin_transform = _translation([0,1.25,0])
    joint_4.parent_link_name = "link_4"
    joint_4.child_link_name = "link_5"
    joint_4.type = sg.JointType_REVOLUTE
    g.addJoint(joint_4)

    joint_5 = sg.Joint("joint_5")
    joint_5.parent_to_joint_origin_transform = _translation([0,1.25,0])
    joint_5.parent_link_name = "link_5"
    joint_5.child_link_name = "link_6"
    joint_5.type = sg.JointType_REVOLUTE
    g.addJoint(joint_5)

    joint_6 = sg.Joint("joint_6")
    joint_6.parent_to_joint_origin_transform = _translation([0,1.25,0])
    joint_6.parent_link_name = "link_6"
    joint_6.child_link_name = "link_7"
    joint_6.type = sg.JointType_REVOLUTE
    g.addJoint(joint_6)

    joint_tool0 = sg.Joint("base_joint")
    joint_tool0.parent_link_name = "link_7"
    joint_tool0.child_link_name = "tool0"
    joint_tool0.type = sg.JointType_FIXED
    g.addJoint(joint_tool0)

    srdf = sg.SRDFModel()
    assert srdf.initFile(g,srdf_file)

    sg.processSRDFAllowedCollisions(g, srdf)

    acm = g.getAllowedCollisionMatrix()

    assert acm.isCollisionAllowed("link_1", "link_2")
    assert not acm.isCollisionAllowed("base_link", "link_5")

    g.removeAllowedCollision("link_1", "link_2")

    assert not acm.isCollisionAllowed("link_1", "link_2")

    g.clearAllowedCollisions()
    assert len(acm.getAllAllowedCollisions()) == 0




    