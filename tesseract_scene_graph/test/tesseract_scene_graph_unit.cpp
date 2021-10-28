#include <tesseract_common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <gtest/gtest.h>
//#include <boost/graph/filtered_graph.hpp>
#include <iostream>
#include <fstream>
#include <kdl/treefksolverpos_recursive.hpp>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract_geometry/geometries.h>
#include <tesseract_common/utils.h>

#include <tesseract_scene_graph/graph.h>
#include <tesseract_scene_graph/utils.h>
#include <tesseract_scene_graph/kdl_parser.h>
#include <tesseract_scene_graph/scene_state.h>

// getLinks and getJoint use an internal map so need to check against graph
void checkSceneGraph(tesseract_scene_graph::SceneGraph& scene_graph)
{
  using namespace tesseract_scene_graph;

  std::vector<Link::ConstPtr> links = scene_graph.getLinks();
  std::vector<Link::ConstPtr> check_links;
  SceneGraph::vertex_iterator i, iend;
  for (boost::tie(i, iend) = boost::vertices(scene_graph); i != iend; ++i)
    check_links.push_back(boost::get(boost::vertex_link, scene_graph)[*i]);

  EXPECT_TRUE(links.size() == check_links.size());

  for (const auto& l : links)
  {
    auto it = std::find_if(
        check_links.begin(), check_links.end(), [&](const Link::ConstPtr& p) { return p.get() == l.get(); });
    EXPECT_TRUE(it != check_links.end());
  }
}

tesseract_scene_graph::SceneGraph createTestSceneGraph()
{
  using namespace tesseract_scene_graph;
  SceneGraph g;

  Link link_1("link_1");
  Link link_2("link_2");
  Link link_3("link_3");
  Link link_4("link_4");
  Link link_5("link_5");

  EXPECT_TRUE(g.addLink(link_1));
  EXPECT_TRUE(g.addLink(link_2));
  EXPECT_TRUE(g.addLink(link_3));
  EXPECT_TRUE(g.addLink(link_4));
  EXPECT_TRUE(g.addLink(link_5));

  Joint joint_1("joint_1");
  joint_1.parent_to_joint_origin_transform.translation()(0) = 1.25;
  joint_1.parent_link_name = "link_1";
  joint_1.child_link_name = "link_2";
  joint_1.type = JointType::FIXED;
  EXPECT_TRUE(g.addJoint(joint_1));

  Joint joint_2("joint_2");
  joint_2.parent_to_joint_origin_transform.translation()(0) = 1.25;
  joint_2.parent_link_name = "link_2";
  joint_2.child_link_name = "link_3";
  joint_2.type = JointType::PLANAR;
  joint_2.limits = std::make_shared<JointLimits>(-1, 1, 0, 2, 3);
  EXPECT_TRUE(g.addJoint(joint_2));

  Joint joint_3("joint_3");
  joint_3.parent_to_joint_origin_transform.translation()(0) = 1.25;
  joint_3.parent_link_name = "link_3";
  joint_3.child_link_name = "link_4";
  joint_3.type = JointType::FLOATING;
  EXPECT_TRUE(g.addJoint(joint_3));

  Joint joint_4("joint_4");
  joint_4.parent_to_joint_origin_transform.translation()(1) = 1.25;
  joint_4.parent_link_name = "link_2";
  joint_4.child_link_name = "link_5";
  joint_4.type = JointType::REVOLUTE;
  joint_4.limits = std::make_shared<JointLimits>(-1, 1, 0, 2, 3);
  EXPECT_TRUE(g.addJoint(joint_4));

  g.addAllowedCollision("link_1", "link_2", "Adjacent");
  g.addAllowedCollision("link_2", "link_3", "Adjacent");
  g.addAllowedCollision("link_3", "link_5", "Adjacent");
  g.addAllowedCollision("link_2", "link_5", "Adjacent");

  return g;
}

TEST(TesseractSceneGraphUnit, TesseractSceneGraphUnit)  // NOLINT
{
  using namespace tesseract_scene_graph;
  SceneGraph g = createTestSceneGraph();

  // Check getAdjacentLinkNames Method
  std::vector<std::string> adjacent_links = g.getAdjacentLinkNames("link_3");
  EXPECT_TRUE(adjacent_links.size() == 1);
  EXPECT_TRUE(adjacent_links[0] == "link_4");

  // Check getInvAdjacentLinkNames Method
  std::vector<std::string> inv_adjacent_links = g.getInvAdjacentLinkNames("link_3");
  EXPECT_TRUE(inv_adjacent_links.size() == 1);
  EXPECT_TRUE(inv_adjacent_links[0] == "link_2");

  // Check getLinkChildrenNames
  std::vector<std::string> child_link_names = g.getLinkChildrenNames("link_5");
  EXPECT_TRUE(child_link_names.empty());

  child_link_names = g.getLinkChildrenNames("link_3");
  EXPECT_TRUE(child_link_names.size() == 1);
  EXPECT_TRUE(child_link_names[0] == "link_4");

  child_link_names = g.getLinkChildrenNames("link_2");
  EXPECT_TRUE(child_link_names.size() == 3);
  EXPECT_TRUE(std::find(child_link_names.begin(), child_link_names.end(), "link_3") != child_link_names.end());
  EXPECT_TRUE(std::find(child_link_names.begin(), child_link_names.end(), "link_4") != child_link_names.end());
  EXPECT_TRUE(std::find(child_link_names.begin(), child_link_names.end(), "link_5") != child_link_names.end());

  // Check getAdjacencyMap
  std::unordered_map<std::string, std::string> adj_map = g.getAdjacencyMap({ "link_2", "link_3" });
  EXPECT_TRUE(adj_map.size() == 4);
  EXPECT_EQ(adj_map.at("link_3"), "link_3");
  EXPECT_EQ(adj_map.at("link_4"), "link_3");
  EXPECT_EQ(adj_map.at("link_5"), "link_2");
  EXPECT_EQ(adj_map.at("link_2"), "link_2");

  // Check getLeafLinks
  std::vector<Link::ConstPtr> leaf_links = g.getLeafLinks();
  EXPECT_TRUE(leaf_links.size() == 2);
  EXPECT_TRUE(std::find(leaf_links.begin(), leaf_links.end(), g.getLink("link_5")) != leaf_links.end());
  EXPECT_TRUE(std::find(leaf_links.begin(), leaf_links.end(), g.getLink("link_4")) != leaf_links.end());

  // Check getJointChildrenNames
  child_link_names = g.getJointChildrenNames(std::vector<std::string>({ "joint_4" }));
  EXPECT_TRUE(child_link_names.size() == 1);
  EXPECT_TRUE(child_link_names[0] == "link_5");

  child_link_names = g.getJointChildrenNames(std::vector<std::string>({ "joint_3" }));
  EXPECT_TRUE(child_link_names.size() == 1);
  EXPECT_TRUE(child_link_names[0] == "link_4");

  child_link_names = g.getJointChildrenNames(std::vector<std::string>({ "joint_1" }));
  EXPECT_TRUE(child_link_names.size() == 4);
  EXPECT_TRUE(std::find(child_link_names.begin(), child_link_names.end(), "link_2") != child_link_names.end());
  EXPECT_TRUE(std::find(child_link_names.begin(), child_link_names.end(), "link_3") != child_link_names.end());
  EXPECT_TRUE(std::find(child_link_names.begin(), child_link_names.end(), "link_4") != child_link_names.end());
  EXPECT_TRUE(std::find(child_link_names.begin(), child_link_names.end(), "link_5") != child_link_names.end());

  // check getActiveJoints
  std::vector<Joint::ConstPtr> active_joints = g.getActiveJoints();
  EXPECT_TRUE(active_joints.size() == 2);
  EXPECT_TRUE(std::find(active_joints.begin(), active_joints.end(), g.getJoint("joint_2")) != active_joints.end());
  EXPECT_TRUE(std::find(active_joints.begin(), active_joints.end(), g.getJoint("joint_4")) != active_joints.end());

  // Check Graph
  checkSceneGraph(g);

  // Save Graph
  g.saveDOT(tesseract_common::getTempPath() + "graph_acyclic_tree_example.dot");

  // Test if the graph is Acyclic
  std::cout << "Is Acyclic: " << g.isAcyclic() << std::endl;
  EXPECT_TRUE(g.isAcyclic());

  // Test if the graph is Tree
  std::cout << "Is Tree: " << g.isTree() << std::endl;
  EXPECT_TRUE(g.isTree());

  // Test for unused links
  Link link_6("link_6");
  g.addLink(link_6);
  std::cout << "Free Link, Is Tree: " << g.isTree() << std::endl;
  EXPECT_FALSE(g.isTree());

  // Check Graph
  checkSceneGraph(g);

  g.removeLink("link_6");
  std::cout << "Free Link Removed, Is Tree: " << g.isTree() << std::endl;
  EXPECT_TRUE(g.isTree());

  // Check Graph
  checkSceneGraph(g);

  Joint joint_5("joint_5");
  joint_5.parent_to_joint_origin_transform.translation()(1) = 1.5;
  joint_5.parent_link_name = "link_5";
  joint_5.child_link_name = "link_4";
  joint_5.type = JointType::CONTINUOUS;
  g.addJoint(joint_5);

  // Check Graph
  checkSceneGraph(g);

  // Save Graph
  g.saveDOT(tesseract_common::getTempPath() + "graph_acyclic_not_tree_example.dot");

  // Test if the graph is Acyclic
  std::cout << "Is Acyclic: " << g.isAcyclic() << std::endl;
  EXPECT_TRUE(g.isAcyclic());

  // Test if the graph is Tree
  std::cout << "Is Tree: " << g.isTree() << std::endl;
  EXPECT_FALSE(g.isTree());

  Joint joint_6("joint_6");
  joint_6.parent_to_joint_origin_transform.translation()(1) = 1.25;
  joint_6.parent_link_name = "link_5";
  joint_6.child_link_name = "link_1";
  joint_6.type = JointType::CONTINUOUS;
  g.addJoint(joint_6);

  // Check Graph
  checkSceneGraph(g);

  // Save Graph
  g.saveDOT(tesseract_common::getTempPath() + "graph_cyclic_not_tree_example.dot");

  // Test if the graph is Acyclic
  std::cout << "Is Acyclic: " << g.isAcyclic() << std::endl;
  EXPECT_FALSE(g.isAcyclic());

  // Test if the graph is Tree
  std::cout << "Is Tree: " << g.isTree() << std::endl;
  EXPECT_FALSE(g.isTree());

  {  // Get Shortest Path
    ShortestPath path = g.getShortestPath("link_1", "link_4");

    std::cout << path << std::endl;
    EXPECT_TRUE(path.links.size() == 3);
    EXPECT_TRUE(std::find(path.links.begin(), path.links.end(), "link_1") != path.links.end());
    EXPECT_TRUE(std::find(path.links.begin(), path.links.end(), "link_5") != path.links.end());
    EXPECT_TRUE(std::find(path.links.begin(), path.links.end(), "link_4") != path.links.end());
    EXPECT_TRUE(path.joints.size() == 2);
    EXPECT_TRUE(std::find(path.joints.begin(), path.joints.end(), "joint_5") != path.joints.end());
    EXPECT_TRUE(std::find(path.joints.begin(), path.joints.end(), "joint_6") != path.joints.end());
    EXPECT_TRUE(path.active_joints.size() == 2);
    EXPECT_TRUE(std::find(path.active_joints.begin(), path.active_joints.end(), "joint_5") != path.active_joints.end());
    EXPECT_TRUE(std::find(path.active_joints.begin(), path.active_joints.end(), "joint_6") != path.active_joints.end());

    std::cout << (g.getName().c_str()) << std::endl;
  }

  {  // Get Shortest Path wit links reversed
    ShortestPath path = g.getShortestPath("link_4", "link_1");

    std::cout << path << std::endl;
    EXPECT_TRUE(path.links.size() == 3);
    EXPECT_TRUE(std::find(path.links.begin(), path.links.end(), "link_1") != path.links.end());
    EXPECT_TRUE(std::find(path.links.begin(), path.links.end(), "link_5") != path.links.end());
    EXPECT_TRUE(std::find(path.links.begin(), path.links.end(), "link_4") != path.links.end());
    EXPECT_TRUE(path.joints.size() == 2);
    EXPECT_TRUE(std::find(path.joints.begin(), path.joints.end(), "joint_5") != path.joints.end());
    EXPECT_TRUE(std::find(path.joints.begin(), path.joints.end(), "joint_6") != path.joints.end());
    EXPECT_TRUE(path.active_joints.size() == 2);
    EXPECT_TRUE(std::find(path.active_joints.begin(), path.active_joints.end(), "joint_5") != path.active_joints.end());
    EXPECT_TRUE(std::find(path.active_joints.begin(), path.active_joints.end(), "joint_6") != path.active_joints.end());

    std::cout << (g.getName().c_str()) << std::endl;
  }

  // Should throw since this is a directory and not a file
  EXPECT_ANY_THROW(g.saveDOT(tesseract_common::getTempPath()));  // NOLINT
}

TEST(TesseractSceneGraphUnit, TesseractSceneGraphClearUnit)  // NOLINT
{
  using namespace tesseract_scene_graph;
  SceneGraph g = createTestSceneGraph();

  EXPECT_EQ(g.getLinks().size(), 5);
  EXPECT_EQ(g.getJoints().size(), 4);
  EXPECT_EQ(g.getAllowedCollisionMatrix()->getAllAllowedCollisions().size(), 4);

  g.clear();

  EXPECT_EQ(g.getLinks().size(), 0);
  EXPECT_EQ(g.getJoints().size(), 0);
  EXPECT_EQ(g.getAllowedCollisionMatrix()->getAllAllowedCollisions().size(), 0);
}

TEST(TesseractSceneGraphUnit, TesseractSceneGraphRootLinkUnit)  // NOLINT
{
  using namespace tesseract_scene_graph;
  SceneGraph g = createTestSceneGraph();

  EXPECT_EQ(g.getRoot(), "link_1");
  EXPECT_TRUE(g.setRoot("link_5"));
  EXPECT_FALSE(g.setRoot("link_10"));
}

TEST(TesseractSceneGraphUnit, TesseractSceneGraphGetLinkJointUnit)  // NOLINT
{
  using namespace tesseract_scene_graph;
  SceneGraph g = createTestSceneGraph();

  EXPECT_TRUE(g.getLink("link_1") != nullptr);
  EXPECT_TRUE(g.getJoint("joint_1") != nullptr);

  EXPECT_TRUE(g.getLink("does_not_exist") == nullptr);
  EXPECT_TRUE(g.getJoint("does_not_exist") == nullptr);
}

TEST(TesseractSceneGraphUnit, TesseractSceneGraphAddLinkJointPairUnit)  // NOLINT
{
  using namespace tesseract_scene_graph;
  SceneGraph g = createTestSceneGraph();

  Link link_6("link_6");
  Joint joint_6("joint_5");
  joint_6.parent_to_joint_origin_transform.translation()(0) = 1.25;
  joint_6.parent_link_name = "link_5";
  joint_6.child_link_name = "link_6";
  joint_6.type = JointType::FIXED;

  EXPECT_TRUE(g.addLink(link_6, joint_6));
  EXPECT_TRUE(g.getLink("link_6") != nullptr);
  EXPECT_TRUE(g.getJoint("joint_5") != nullptr);
  EXPECT_EQ(g.getLinks().size(), 6);
  EXPECT_EQ(g.getJoints().size(), 5);

  // Add link which already exists but joint does not
  Link link_6d("link_6");
  Joint joint_6d("joint_5d");
  joint_6d.parent_to_joint_origin_transform.translation()(0) = 1.25;
  joint_6d.parent_link_name = "link_5";
  joint_6d.child_link_name = "link_6";
  joint_6d.type = JointType::FIXED;
  EXPECT_FALSE(g.addLink(link_6d, joint_6d));
  EXPECT_EQ(g.getLinks().size(), 6);
  EXPECT_EQ(g.getJoints().size(), 5);

  // Check Graph
  checkSceneGraph(g);

  // Add joint which already exists but link does not
  Link link_6e("link_6e");
  Joint joint_6e("joint_5");
  joint_6d.parent_to_joint_origin_transform.translation()(0) = 1.25;
  joint_6d.parent_link_name = "link_5";
  joint_6d.child_link_name = "link_6e";
  joint_6d.type = JointType::FIXED;
  EXPECT_FALSE(g.addLink(link_6e, joint_6e));
  EXPECT_EQ(g.getLinks().size(), 6);
  EXPECT_EQ(g.getJoints().size(), 5);

  // Check Graph
  checkSceneGraph(g);
}

TEST(TesseractSceneGraphUnit, TesseractSceneGraphRemoveLinkUnit)  // NOLINT
{
  using namespace tesseract_scene_graph;
  {  // Remove Link
    SceneGraph g = createTestSceneGraph();
    EXPECT_EQ(g.getLinks().size(), 5);
    EXPECT_EQ(g.getJoints().size(), 4);
    EXPECT_TRUE(g.removeLink("link_5"));
    EXPECT_TRUE(g.getJoint("joint_4") == nullptr);
    EXPECT_TRUE(g.getLink("link_5") == nullptr);
    EXPECT_EQ(g.getLinks().size(), 4);
    EXPECT_EQ(g.getJoints().size(), 3);

    // Check Graph
    checkSceneGraph(g);
  }

  {  // Make sure all inbound and outbound edges are removed
    /** @todo Doing this causes the test to fail but assign to new object works. Need to look into why */
    //  g = createTestSceneGraph();
    SceneGraph g = createTestSceneGraph();
    EXPECT_EQ(g.getLinks().size(), 5);
    EXPECT_EQ(g.getJoints().size(), 4);
    EXPECT_TRUE(g.removeLink("link_2"));
    EXPECT_EQ(g.getLinks().size(), 4);
    EXPECT_EQ(g.getJoints().size(), 1);

    // Check Graph
    checkSceneGraph(g);
  }

  {  // Try to remove link which does not exist
    SceneGraph g = createTestSceneGraph();
    EXPECT_EQ(g.getLinks().size(), 5);
    EXPECT_EQ(g.getJoints().size(), 4);
    EXPECT_FALSE(g.removeLink("link_does_not_exist"));
    EXPECT_EQ(g.getLinks().size(), 5);
    EXPECT_EQ(g.getJoints().size(), 4);

    // Check Graph
    checkSceneGraph(g);
  }

  {  // Make sure all children are removed if enabled
    SceneGraph g = createTestSceneGraph();
    EXPECT_EQ(g.getLinks().size(), 5);
    EXPECT_EQ(g.getJoints().size(), 4);
    EXPECT_TRUE(g.removeLink("link_2", true));
    EXPECT_EQ(g.getLinks().size(), 1);
    EXPECT_EQ(g.getJoints().size(), 0);

    // Check Graph
    checkSceneGraph(g);
  }
}

TEST(TesseractSceneGraphUnit, TesseractSceneGraphMoveLinkUnit)  // NOLINT
{
  using namespace tesseract_scene_graph;
  SceneGraph g = createTestSceneGraph();

  Joint::ConstPtr j = g.getJoint("joint_4");
  EXPECT_EQ(g.getLinks().size(), 5);
  EXPECT_EQ(g.getJoints().size(), 4);
  EXPECT_NE(j->parent_link_name, "link_4");

  Joint move_joint = j->clone("move_joint_4");
  move_joint.parent_link_name = "link_4";
  EXPECT_TRUE(g.moveLink(move_joint));
  EXPECT_TRUE(g.getJoint("joint_4") == nullptr);
  EXPECT_TRUE(g.getJoint("move_joint_4") != nullptr);
  EXPECT_EQ(g.getLinks().size(), 5);
  EXPECT_EQ(g.getJoints().size(), 4);

  // Check Graph
  checkSceneGraph(g);

  // Parent link does not exist
  {
    SceneGraph ng = createTestSceneGraph();
    move_joint.parent_link_name = "link_does_not_exist";
    EXPECT_FALSE(ng.moveLink(move_joint));

    // Check Graph
    checkSceneGraph(ng);
  }

  // Child link does not exist
  {
    SceneGraph ng = createTestSceneGraph();
    move_joint.child_link_name = "link_does_not_exist";
    EXPECT_FALSE(ng.moveLink(move_joint));

    // Check Graph
    checkSceneGraph(ng);
  }
}

TEST(TesseractSceneGraphUnit, TesseractSceneGraphRemoveAllowedCollisionUnit)  // NOLINT
{
  using namespace tesseract_scene_graph;

  SceneGraph g = createTestSceneGraph();
  EXPECT_EQ(g.getAllowedCollisionMatrix()->getAllAllowedCollisions().size(), 4);
  EXPECT_TRUE(g.isCollisionAllowed("link_1", "link_2"));

  g.removeAllowedCollision("link_1", "link_2");
  EXPECT_FALSE(g.isCollisionAllowed("link_1", "link_2"));
  EXPECT_EQ(g.getAllowedCollisionMatrix()->getAllAllowedCollisions().size(), 3);

  g.clearAllowedCollisions();
  EXPECT_TRUE(g.getAllowedCollisionMatrix()->getAllAllowedCollisions().empty());
}

TEST(TesseractSceneGraphUnit, TesseractSceneGraphChangeJointOriginUnit)  // NOLINT
{
  using namespace tesseract_scene_graph;
  SceneGraph g = createTestSceneGraph();

  Joint::ConstPtr j = g.getJoint("joint_4");
  Eigen::Isometry3d origin = j->parent_to_joint_origin_transform;
  double check = origin.translation().x();
  origin.translation().x() = check + 5;

  EXPECT_EQ(g.getLinks().size(), 5);
  EXPECT_EQ(g.getJoints().size(), 4);
  EXPECT_TRUE(g.changeJointOrigin("joint_4", origin));
  EXPECT_DOUBLE_EQ(g.getJoint("joint_4")->parent_to_joint_origin_transform.translation().x(), check + 5);
  EXPECT_EQ(g.getLinks().size(), 5);
  EXPECT_EQ(g.getJoints().size(), 4);

  // Joint does not exist
  EXPECT_FALSE(g.changeJointOrigin("joint_does_not_exist", origin));
}

TEST(TesseractSceneGraphUnit, TesseractSceneGraphChangeJointLimitsUnit)  // NOLINT
{
  using namespace tesseract_scene_graph;
  SceneGraph g = createTestSceneGraph();

  JointLimits::ConstPtr jl = g.getJointLimits("joint_4");
  EXPECT_TRUE(jl != nullptr);

  JointLimits::ConstPtr jld = g.getJointLimits("joint_does_not_exist");
  EXPECT_TRUE(jld == nullptr);

  JointLimits nlimits;
  nlimits.lower = -5;
  nlimits.upper = 5;

  Joint::ConstPtr j = g.getJoint("joint_4");
  EXPECT_EQ(g.getLinks().size(), 5);
  EXPECT_EQ(g.getJoints().size(), 4);
  EXPECT_TRUE(g.changeJointLimits("joint_4", nlimits));
  EXPECT_DOUBLE_EQ(j->limits->lower, -5);
  EXPECT_DOUBLE_EQ(j->limits->upper, 5);
  EXPECT_EQ(g.getLinks().size(), 5);
  EXPECT_EQ(g.getJoints().size(), 4);

  // Joint does not exist
  EXPECT_FALSE(g.changeJointLimits("joint_does_not_exist", nlimits));

  // Cannot change limits of fixed or floating joint
  EXPECT_FALSE(g.changeJointLimits("joint_1", nlimits));
  EXPECT_FALSE(g.changeJointLimits("joint_3", nlimits));
}

TEST(TesseractSceneGraphUnit, TesseractSceneGraphChangeJointPositionLimitsUnit)  // NOLINT
{
  using namespace tesseract_scene_graph;
  SceneGraph g = createTestSceneGraph();

  Joint::ConstPtr j = g.getJoint("joint_4");
  EXPECT_EQ(g.getLinks().size(), 5);
  EXPECT_EQ(g.getJoints().size(), 4);
  EXPECT_TRUE(g.changeJointPositionLimits("joint_4", -5, 5));
  EXPECT_DOUBLE_EQ(j->limits->lower, -5);
  EXPECT_DOUBLE_EQ(j->limits->upper, 5);
  EXPECT_EQ(g.getLinks().size(), 5);
  EXPECT_EQ(g.getJoints().size(), 4);

  // Joint does not exist
  EXPECT_FALSE(g.changeJointPositionLimits("joint_does_not_exist", -5, 5));

  // Cannot change limits of fixed or floating joint
  EXPECT_FALSE(g.changeJointPositionLimits("joint_1", -5, 5));
  EXPECT_FALSE(g.changeJointPositionLimits("joint_3", -5, 5));
}

TEST(TesseractSceneGraphUnit, TesseractSceneGraphChangeJointVelocityLimitsUnit)  // NOLINT
{
  using namespace tesseract_scene_graph;
  SceneGraph g = createTestSceneGraph();

  Joint::ConstPtr j = g.getJoint("joint_4");
  EXPECT_EQ(g.getLinks().size(), 5);
  EXPECT_EQ(g.getJoints().size(), 4);
  EXPECT_TRUE(g.changeJointVelocityLimits("joint_4", 10));
  EXPECT_DOUBLE_EQ(j->limits->velocity, 10);
  EXPECT_EQ(g.getLinks().size(), 5);
  EXPECT_EQ(g.getJoints().size(), 4);

  // Joint does not exist
  EXPECT_FALSE(g.changeJointVelocityLimits("joint_does_not_exist", 10));

  // Cannot change limits of fixed or floating joint
  EXPECT_FALSE(g.changeJointVelocityLimits("joint_1", 10));
  EXPECT_FALSE(g.changeJointVelocityLimits("joint_3", 10));
}

TEST(TesseractSceneGraphUnit, TesseractSceneGraphChangeJointAccelerationLimitsUnit)  // NOLINT
{
  using namespace tesseract_scene_graph;
  SceneGraph g = createTestSceneGraph();

  Joint::ConstPtr j = g.getJoint("joint_4");
  EXPECT_EQ(g.getLinks().size(), 5);
  EXPECT_EQ(g.getJoints().size(), 4);
  EXPECT_TRUE(g.changeJointAccelerationLimits("joint_4", 20));
  EXPECT_DOUBLE_EQ(j->limits->acceleration, 20);
  EXPECT_EQ(g.getLinks().size(), 5);
  EXPECT_EQ(g.getJoints().size(), 4);

  // Joint does not exist
  EXPECT_FALSE(g.changeJointAccelerationLimits("joint_does_not_exist", 20));

  // Cannot change limits of fixed or floating joint
  EXPECT_FALSE(g.changeJointAccelerationLimits("joint_1", 20));
  EXPECT_FALSE(g.changeJointAccelerationLimits("joint_3", 20));
}

TEST(TesseractSceneGraphUnit, TesseractSceneGraphMoveJointUnit)  // NOLINT
{
  using namespace tesseract_scene_graph;
  SceneGraph g = createTestSceneGraph();

  Joint::ConstPtr j = g.getJoint("joint_4");
  EXPECT_EQ(g.getLinks().size(), 5);
  EXPECT_EQ(g.getJoints().size(), 4);
  EXPECT_NE(j->parent_link_name, "link_4");
  EXPECT_TRUE(g.moveJoint("joint_4", "link_4"));
  EXPECT_EQ(j->parent_link_name, "link_4");
  EXPECT_EQ(g.getLinks().size(), 5);
  EXPECT_EQ(g.getJoints().size(), 4);

  // Check Graph
  checkSceneGraph(g);

  // Joint does not exist
  EXPECT_FALSE(g.moveJoint("joint_does_not_exist", "link_4"));

  // Check Graph
  checkSceneGraph(g);

  // Joint does not exist
  EXPECT_FALSE(g.moveJoint("joint_4", "link_does_not_exist"));

  // Check Graph
  checkSceneGraph(g);
}

TEST(TesseractSceneGraphUnit, TesseractSceneGraphAddJointUnit)  // NOLINT
{
  using namespace tesseract_scene_graph;
  SceneGraph g = createTestSceneGraph();

  Joint joint_6("joint_5");
  joint_6.parent_to_joint_origin_transform.translation()(0) = 1.25;
  joint_6.parent_link_name = "link_5";
  joint_6.child_link_name = "link_6";
  joint_6.type = JointType::FIXED;

  // Child does not exist
  EXPECT_FALSE(g.addJoint(joint_6));
  EXPECT_EQ(g.getLinks().size(), 5);
  EXPECT_EQ(g.getJoints().size(), 4);

  Joint joint_7("joint_7");
  joint_7.parent_to_joint_origin_transform.translation()(0) = 1.25;
  joint_7.parent_link_name = "link_6";
  joint_7.child_link_name = "link_5";
  joint_7.type = JointType::FIXED;

  // Parent does not exist
  EXPECT_FALSE(g.addJoint(joint_7));
  EXPECT_EQ(g.getLinks().size(), 5);
  EXPECT_EQ(g.getJoints().size(), 4);

  Joint joint_4("joint_4");
  joint_4.parent_to_joint_origin_transform.translation()(0) = 1.25;
  joint_4.parent_link_name = "link_4";
  joint_4.child_link_name = "link_5";
  joint_4.type = JointType::FIXED;

  // Joint already exist
  EXPECT_FALSE(g.addJoint(joint_4));
  EXPECT_EQ(g.getLinks().size(), 5);
  EXPECT_EQ(g.getJoints().size(), 4);
}

TEST(TesseractSceneGraphUnit, TesseractSceneGraphRemoveJointUnit)  // NOLINT
{
  using namespace tesseract_scene_graph;
  {
    SceneGraph g = createTestSceneGraph();

    EXPECT_EQ(g.getLinks().size(), 5);
    EXPECT_EQ(g.getJoints().size(), 4);
    EXPECT_TRUE(g.removeJoint("joint_4"));
    EXPECT_TRUE(g.getJoint("joint_4") == nullptr);
    EXPECT_TRUE(g.getLink("link_5") != nullptr);  // The child link should remain
    EXPECT_EQ(g.getLinks().size(), 5);
    EXPECT_EQ(g.getJoints().size(), 3);
  }

  {  // Try to remove joint which does not exist
    SceneGraph g = createTestSceneGraph();
    EXPECT_EQ(g.getLinks().size(), 5);
    EXPECT_EQ(g.getJoints().size(), 4);
    EXPECT_FALSE(g.removeJoint("joint_does_not_exist"));
    EXPECT_EQ(g.getLinks().size(), 5);
    EXPECT_EQ(g.getJoints().size(), 4);

    // Check Graph
    checkSceneGraph(g);
  }

  {  // Make sure all children are removed if enabled
    SceneGraph g = createTestSceneGraph();
    EXPECT_EQ(g.getLinks().size(), 5);
    EXPECT_EQ(g.getJoints().size(), 4);
    EXPECT_TRUE(g.removeJoint("joint_2", true));
    EXPECT_EQ(g.getLinks().size(), 3);
    EXPECT_EQ(g.getJoints().size(), 2);

    // Check Graph
    checkSceneGraph(g);
  }
}

void printKDLTree(const KDL::SegmentMap::const_iterator& link, const std::string& prefix)
{
  std::cout << prefix << "- Segment " << GetTreeElementSegment(link->second).getName() << " has "
            << GetTreeElementChildren(link->second).size() << " children" << std::endl;

  for (auto child : GetTreeElementChildren(link->second))
    printKDLTree(child, prefix + "  ");
}

tesseract_scene_graph::SceneGraph buildTestSceneGraph()
{
  using namespace tesseract_scene_graph;
  SceneGraph g;

  Link base_link("base_link");
  Link link_1("link_1");
  Link link_2("link_2");
  Link link_3("link_3");
  Link link_4("link_4");
  Link link_5("link_5");

  EXPECT_TRUE(g.addLink(base_link));
  EXPECT_TRUE(g.addLink(link_1));
  EXPECT_TRUE(g.addLink(link_2));
  EXPECT_TRUE(g.addLink(link_3));
  EXPECT_TRUE(g.addLink(link_4));
  EXPECT_TRUE(g.addLink(link_5));

  Joint base_joint("base_joint");
  base_joint.parent_link_name = "base_link";
  base_joint.child_link_name = "link_1";
  base_joint.type = JointType::FIXED;
  EXPECT_TRUE(g.addJoint(base_joint));

  Joint joint_1("joint_1");
  joint_1.parent_link_name = "link_1";
  joint_1.child_link_name = "link_2";
  joint_1.type = JointType::REVOLUTE;
  joint_1.limits = std::make_shared<JointLimits>(-1, 1, 0, 2, 3);
  EXPECT_TRUE(g.addJoint(joint_1));

  Joint joint_2("joint_2");
  joint_2.parent_to_joint_origin_transform.translation()(0) = 1.25;
  joint_2.parent_link_name = "link_2";
  joint_2.child_link_name = "link_3";
  joint_2.type = JointType::REVOLUTE;
  joint_2.limits = std::make_shared<JointLimits>(-1, 1, 0, 2, 3);
  EXPECT_TRUE(g.addJoint(joint_2));

  Joint joint_3("joint_3");
  joint_3.parent_to_joint_origin_transform.translation()(0) = 1.25;
  joint_3.parent_link_name = "link_3";
  joint_3.child_link_name = "link_4";
  joint_3.type = JointType::REVOLUTE;
  joint_3.limits = std::make_shared<JointLimits>(-1, 1, 0, 2, 3);
  EXPECT_TRUE(g.addJoint(joint_3));

  Joint joint_4("joint_4");
  joint_4.parent_to_joint_origin_transform.translation()(1) = 1.25;
  joint_4.parent_link_name = "link_2";
  joint_4.child_link_name = "link_5";
  joint_4.type = JointType::REVOLUTE;
  joint_4.limits = std::make_shared<JointLimits>(-1, 1, 0, 2, 3);
  EXPECT_TRUE(g.addJoint(joint_4));

  return g;
}

tesseract_scene_graph::SceneGraph buildTestSceneGraphForSubTree()
{
  using namespace tesseract_scene_graph;
  std::vector<std::string> prefix{ "left_", "right_" };
  std::vector<std::string> link_names = { "base_link", "link_1", "link_2", "link_3", "link_4", "link_5" };
  SceneGraph g;

  EXPECT_TRUE(g.addLink(Link("world")));

  for (const auto& p : prefix)
  {
    for (const auto& link_name : link_names)
    {
      EXPECT_TRUE(g.addLink(Link(p + link_name)));
    }
  }

  for (const auto& p : prefix)
  {
    Joint world_joint(p + "word_joint");
    if (p == prefix[0])
      world_joint.parent_to_joint_origin_transform.translation()(0) = 1;
    else
      world_joint.parent_to_joint_origin_transform.translation()(0) = -1;

    world_joint.parent_link_name = "world";
    world_joint.child_link_name = p + "base_link";
    world_joint.type = JointType::FIXED;
    EXPECT_TRUE(g.addJoint(world_joint));

    Joint base_joint(p + "base_joint");
    base_joint.parent_link_name = p + "base_link";
    base_joint.child_link_name = p + "link_1";
    base_joint.type = JointType::FIXED;
    EXPECT_TRUE(g.addJoint(base_joint));

    Joint joint_1(p + "joint_1");
    joint_1.parent_link_name = p + "link_1";
    joint_1.child_link_name = p + "link_2";
    joint_1.type = JointType::REVOLUTE;
    joint_1.limits = std::make_shared<JointLimits>(-1, 1, 0, 2, 3);
    EXPECT_TRUE(g.addJoint(joint_1));

    Joint joint_2(p + "joint_2");
    joint_2.parent_to_joint_origin_transform.translation()(0) = 1.25;
    joint_2.parent_link_name = p + "link_2";
    joint_2.child_link_name = p + "link_3";
    joint_2.type = JointType::REVOLUTE;
    joint_2.limits = std::make_shared<JointLimits>(-1, 1, 0, 2, 3);
    EXPECT_TRUE(g.addJoint(joint_2));

    Joint joint_3(p + "joint_3");
    joint_3.parent_to_joint_origin_transform.translation()(0) = 1.25;
    joint_3.parent_link_name = p + "link_3";
    joint_3.child_link_name = p + "link_4";
    joint_3.type = JointType::REVOLUTE;
    joint_3.limits = std::make_shared<JointLimits>(-1, 1, 0, 2, 3);
    EXPECT_TRUE(g.addJoint(joint_3));

    Joint joint_4(p + "joint_4");
    joint_4.parent_to_joint_origin_transform.translation()(1) = 1.25;
    joint_4.parent_link_name = p + "link_2";
    joint_4.child_link_name = p + "link_5";
    joint_4.type = JointType::REVOLUTE;
    joint_4.limits = std::make_shared<JointLimits>(-1, 1, 0, 2, 3);
    EXPECT_TRUE(g.addJoint(joint_4));
  }

  return g;
}

void testSceneGraphKDLTree(KDL::Tree& tree)
{
  KDL::TreeFkSolverPos_recursive solver(tree);
  KDL::JntArray joints;
  joints.resize(4);
  for (unsigned int i = 0; i < 4; ++i)
    joints(i) = 0;

  {
    KDL::Frame frame;
    solver.JntToCart(joints, frame, "base_link");
    Eigen::Isometry3d e_frame = tesseract_scene_graph::convert(frame);
    EXPECT_TRUE(e_frame.isApprox(Eigen::Isometry3d::Identity(), 1e-8));
  }

  {
    KDL::Frame frame;
    solver.JntToCart(joints, frame, "link_1");
    Eigen::Isometry3d e_frame = tesseract_scene_graph::convert(frame);
    EXPECT_TRUE(e_frame.isApprox(Eigen::Isometry3d::Identity(), 1e-8));
  }

  {
    KDL::Frame frame;
    solver.JntToCart(joints, frame, "link_2");
    Eigen::Isometry3d e_frame = tesseract_scene_graph::convert(frame);
    EXPECT_TRUE(e_frame.isApprox(Eigen::Isometry3d::Identity(), 1e-8));
  }

  {
    KDL::Frame frame;
    solver.JntToCart(joints, frame, "link_3");
    Eigen::Isometry3d e_frame = tesseract_scene_graph::convert(frame);
    Eigen::Isometry3d c_frame{ Eigen::Isometry3d::Identity() };
    c_frame.translation()(0) = 1.25;
    EXPECT_TRUE(e_frame.isApprox(c_frame, 1e-8));
  }

  {
    KDL::Frame frame;
    solver.JntToCart(joints, frame, "link_4");
    Eigen::Isometry3d e_frame = tesseract_scene_graph::convert(frame);
    Eigen::Isometry3d c_frame{ Eigen::Isometry3d::Identity() };
    c_frame.translation()(0) = 2 * 1.25;
    EXPECT_TRUE(e_frame.isApprox(c_frame, 1e-8));
  }

  {
    KDL::Frame frame;
    solver.JntToCart(joints, frame, "link_5");
    Eigen::Isometry3d e_frame = tesseract_scene_graph::convert(frame);
    Eigen::Isometry3d c_frame{ Eigen::Isometry3d::Identity() };
    c_frame.translation()(1) = 1.25;
    EXPECT_TRUE(e_frame.isApprox(c_frame, 1e-8));
  }
}

void testSubSceneGraphKDLTree(tesseract_scene_graph::KDLTreeData& data,
                              tesseract_scene_graph::KDLTreeData& full_data,
                              const std::unordered_map<std::string, double>& joint_values)
{
  std::vector<std::string> prefix{ "left_", "right_" };
  KDL::TreeFkSolverPos_recursive solver(data.tree);
  KDL::TreeFkSolverPos_recursive full_solver(data.tree);

  KDL::JntArray joints;
  joints.resize(data.tree.getNrOfJoints());
  for (unsigned int i = 0; i < data.tree.getNrOfJoints(); ++i)
    joints(i) = joint_values.at(data.active_joint_names[i]);

  KDL::JntArray full_joints;
  joints.resize(full_data.tree.getNrOfJoints());
  for (unsigned int i = 0; i < full_data.tree.getNrOfJoints(); ++i)
    joints(i) = joint_values.at(full_data.active_joint_names[i]);

  EXPECT_EQ(data.tree.getRootSegment()->first, "world");
  EXPECT_EQ(full_data.tree.getRootSegment()->first, "world");

  for (const auto& p : prefix)
  {
    {
      KDL::Frame frame;
      solver.JntToCart(joints, frame, p + "link_2");

      KDL::Frame c_frame;
      full_solver.JntToCart(full_joints, c_frame, p + "link_2");

      EXPECT_TRUE(tesseract_scene_graph::convert(frame).isApprox(tesseract_scene_graph::convert(c_frame), 1e-8));
    }

    {
      KDL::Frame frame;
      solver.JntToCart(joints, frame, p + "link_3");

      KDL::Frame c_frame;
      full_solver.JntToCart(full_joints, c_frame, p + "link_3");

      EXPECT_TRUE(tesseract_scene_graph::convert(frame).isApprox(tesseract_scene_graph::convert(c_frame), 1e-8));
    }

    {
      KDL::Frame frame;
      solver.JntToCart(joints, frame, p + "link_4");

      KDL::Frame c_frame;
      full_solver.JntToCart(full_joints, c_frame, p + "link_4");

      EXPECT_TRUE(tesseract_scene_graph::convert(frame).isApprox(tesseract_scene_graph::convert(c_frame), 1e-8));
    }

    {
      KDL::Frame frame;
      solver.JntToCart(joints, frame, p + "link_5");

      KDL::Frame c_frame;
      full_solver.JntToCart(full_joints, c_frame, p + "link_5");

      EXPECT_TRUE(tesseract_scene_graph::convert(frame).isApprox(tesseract_scene_graph::convert(c_frame), 1e-8));
    }
  }
}

TEST(TesseractSceneGraphUnit, GetSourceLinkUnit)  // NOLINT
{
  using namespace tesseract_scene_graph;
  SceneGraph g = buildTestSceneGraph();

  for (int i = 1; i <= 3; ++i)
  {
    std::string link_name = "link_" + std::to_string(i);
    std::string joint_name = "joint_" + std::to_string(i);
    Link::ConstPtr l = g.getSourceLink(joint_name);
    Joint::ConstPtr j = g.getJoint(joint_name);
    EXPECT_EQ(l->getName(), link_name);
    EXPECT_EQ(j->parent_link_name, link_name);
    EXPECT_TRUE(g.getLink(link_name) == l);
  }
}

TEST(TesseractSceneGraphUnit, GetTargetLinkUnit)  // NOLINT
{
  using namespace tesseract_scene_graph;
  SceneGraph g = buildTestSceneGraph();

  for (int i = 1; i <= 3; ++i)
  {
    std::string link_name = "link_" + std::to_string(i + 1);
    std::string joint_name = "joint_" + std::to_string(i);
    Link::ConstPtr l = g.getTargetLink(joint_name);
    Joint::ConstPtr j = g.getJoint(joint_name);
    EXPECT_EQ(l->getName(), link_name);
    EXPECT_EQ(j->child_link_name, link_name);
    EXPECT_TRUE(g.getLink(link_name) == l);
  }
}

TEST(TesseractSceneGraphUnit, GetInboundJointsUnit)  // NOLINT
{
  using namespace tesseract_scene_graph;
  SceneGraph g = buildTestSceneGraph();

  for (int i = 2; i <= 4; ++i)
  {
    std::string link_name = "link_" + std::to_string(i);
    std::string joint_name = "joint_" + std::to_string(i - 1);
    std::vector<Joint::ConstPtr> j = g.getInboundJoints(link_name);
    Link::ConstPtr l = g.getLink(link_name);
    EXPECT_EQ(j.size(), 1);
    EXPECT_EQ(j[0]->getName(), joint_name);
    EXPECT_EQ(j[0]->child_link_name, link_name);
    EXPECT_TRUE(g.getJoint(joint_name) == j[0]);
  }
}

TEST(TesseractSceneGraphUnit, GetOutboundJointsUnit)  // NOLINT
{
  using namespace tesseract_scene_graph;
  SceneGraph g = buildTestSceneGraph();

  for (int i = 1; i <= 3; ++i)
  {
    std::string link_name = "link_" + std::to_string(i);
    std::string joint_name = "joint_" + std::to_string(i);
    std::vector<Joint::ConstPtr> j = g.getOutboundJoints(link_name);
    Link::ConstPtr l = g.getLink(link_name);
    if (i == 2)
    {
      EXPECT_EQ(j.size(), 2);
      EXPECT_EQ(j[0]->getName(), joint_name);
      EXPECT_EQ(j[0]->parent_link_name, link_name);
      EXPECT_TRUE(g.getJoint(joint_name) == j[0]);

      EXPECT_EQ(j[1]->getName(), "joint_4");
      EXPECT_EQ(j[1]->parent_link_name, link_name);
      EXPECT_TRUE(g.getJoint("joint_4") == j[1]);
    }
    else
    {
      EXPECT_EQ(j.size(), 1);
      EXPECT_EQ(j[0]->getName(), joint_name);
      EXPECT_EQ(j[0]->parent_link_name, link_name);
      EXPECT_TRUE(g.getJoint(joint_name) == j[0]);
    }
  }
}

TEST(TesseractSceneGraphUnit, LoadKDLUnit)  // NOLINT
{
  using namespace tesseract_scene_graph;
  SceneGraph g = buildTestSceneGraph();

  // Check to make sure all links are enabled
  for (const auto& link : g.getLinks())
  {
    EXPECT_TRUE(g.getLinkCollisionEnabled(link->getName()));
    EXPECT_TRUE(g.getLinkVisibility(link->getName()));
  }

  std::vector<std::string> joint_names{ "joint_1", "joint_2", "joint_3", "joint_4" };
  std::vector<std::string> link_names{ "base_link", "link_1", "link_2", "link_3", "link_4", "link_5" };
  std::vector<std::string> static_link_names{ "base_link", "link_1" };
  std::vector<std::string> active_link_names{ "link_2", "link_3", "link_4", "link_5" };
  {
    KDLTreeData data = parseSceneGraph(g);

    EXPECT_TRUE(tesseract_common::isIdentical(data.link_names, link_names, false));
    EXPECT_TRUE(tesseract_common::isIdentical(data.static_link_names, static_link_names, false));
    EXPECT_TRUE(tesseract_common::isIdentical(data.active_joint_names, joint_names, false));
    EXPECT_TRUE(tesseract_common::isIdentical(data.active_link_names, active_link_names, false));

    testSceneGraphKDLTree(data.tree);

    // walk through tree
    std::cout << " ======================================" << std::endl;
    std::cout << " Tree has " << data.tree.getNrOfSegments() << " link(s) and a root link" << std::endl;
    std::cout << " ======================================" << std::endl;
    auto root = data.tree.getRootSegment();
    printKDLTree(root, "");

    EXPECT_EQ(data.tree.getNrOfJoints(), 4);
    EXPECT_EQ(data.tree.getNrOfSegments(), 5);
  }

  SceneGraph::Ptr g_clone = g.clone();

  // Check to make sure all links are enabled
  for (const auto& link : g_clone->getLinks())
  {
    EXPECT_TRUE(g_clone->getLinkCollisionEnabled(link->getName()));
    EXPECT_TRUE(g_clone->getLinkVisibility(link->getName()));
  }

  {
    KDLTreeData data = parseSceneGraph(*g_clone);

    EXPECT_TRUE(tesseract_common::isIdentical(data.link_names, link_names, false));
    EXPECT_TRUE(tesseract_common::isIdentical(data.static_link_names, static_link_names, false));
    EXPECT_TRUE(tesseract_common::isIdentical(data.active_joint_names, joint_names, false));
    EXPECT_TRUE(tesseract_common::isIdentical(data.active_link_names, active_link_names, false));

    testSceneGraphKDLTree(data.tree);

    // walk through tree
    std::cout << " ======================================" << std::endl;
    std::cout << " Tree has " << data.tree.getNrOfSegments() << " link(s) and a root link" << std::endl;
    std::cout << " ======================================" << std::endl;
    auto root = data.tree.getRootSegment();
    printKDLTree(root, "");

    EXPECT_EQ(data.tree.getNrOfJoints(), 4);
    EXPECT_EQ(data.tree.getNrOfSegments(), 5);
  }
}

TEST(TesseractSceneGraphUnit, LoadSubKDLUnit)  // NOLINT
{
  using namespace tesseract_scene_graph;
  SceneGraph g = buildTestSceneGraphForSubTree();
  std::vector<std::string> joint_names{ "left_joint_1",  "left_joint_2",  "left_joint_3",  "left_joint_4",
                                        "right_joint_1", "right_joint_2", "right_joint_3", "right_joint_4" };
  std::vector<std::string> sub_joint_names{ "left_joint_2",  "left_joint_3",  "left_joint_4",
                                            "right_joint_2", "right_joint_3", "right_joint_4" };
  std::vector<std::string> link_names{ "world",        "left_link_2",  "left_link_3",  "left_link_4", "left_link_5",
                                       "right_link_2", "right_link_3", "right_link_4", "right_link_5" };

  std::vector<std::string> static_link_names{ "world", "left_link_2", "right_link_2" };

  std::vector<std::string> active_link_names{ "left_link_3",  "left_link_4",  "left_link_5",
                                              "right_link_3", "right_link_4", "right_link_5" };

  std::unordered_map<std::string, double> joint_values;
  for (const auto& joint_name : joint_names)
  {
    auto jnt = g.getJoint(joint_name);
    std::uniform_real_distribution<double> sample(jnt->limits->lower, jnt->limits->upper);
    joint_values[joint_name] = sample(tesseract_common::mersenne);
  }

  {
    KDLTreeData full_data = parseSceneGraph(g);
    KDLTreeData data = parseSceneGraph(g, sub_joint_names, joint_values);

    EXPECT_TRUE(tesseract_common::isIdentical(data.link_names, link_names, false));
    EXPECT_TRUE(tesseract_common::isIdentical(data.static_link_names, static_link_names, false));
    EXPECT_TRUE(tesseract_common::isIdentical(data.active_joint_names, sub_joint_names, false));
    EXPECT_TRUE(tesseract_common::isIdentical(data.active_link_names, active_link_names, false));

    testSubSceneGraphKDLTree(data, full_data, joint_values);

    // walk through tree
    std::cout << " ======================================" << std::endl;
    std::cout << " Tree has " << data.tree.getNrOfSegments() << " link(s) and a root link" << std::endl;
    std::cout << " ======================================" << std::endl;
    auto root = data.tree.getRootSegment();
    printKDLTree(root, "");

    EXPECT_EQ(data.tree.getNrOfJoints(), 6);
    EXPECT_EQ(data.tree.getNrOfSegments(), 8);
  }

  SceneGraph::Ptr g_clone = g.clone();

  // Check to make sure all links are enabled
  for (const auto& link : g_clone->getLinks())
  {
    EXPECT_TRUE(g_clone->getLinkCollisionEnabled(link->getName()));
    EXPECT_TRUE(g_clone->getLinkVisibility(link->getName()));
  }

  {
    KDLTreeData full_data = parseSceneGraph(g);
    KDLTreeData data = parseSceneGraph(*g_clone, sub_joint_names, joint_values);

    EXPECT_TRUE(tesseract_common::isIdentical(data.link_names, link_names, false));
    EXPECT_TRUE(tesseract_common::isIdentical(data.static_link_names, static_link_names, false));
    EXPECT_TRUE(tesseract_common::isIdentical(data.active_joint_names, sub_joint_names, false));
    EXPECT_TRUE(tesseract_common::isIdentical(data.active_link_names, active_link_names, false));

    testSubSceneGraphKDLTree(data, full_data, joint_values);

    // walk through tree
    std::cout << " ======================================" << std::endl;
    std::cout << " Tree has " << data.tree.getNrOfSegments() << " link(s) and a root link" << std::endl;
    std::cout << " ======================================" << std::endl;
    auto root = data.tree.getRootSegment();
    printKDLTree(root, "");

    EXPECT_EQ(data.tree.getNrOfJoints(), 6);
    EXPECT_EQ(data.tree.getNrOfSegments(), 8);
  }
}

/// Testing AllowedCollisionMatrix
TEST(TesseractSceneGraphUnit, TestAllowedCollisionMatrix)  // NOLINT
{
  tesseract_scene_graph::AllowedCollisionMatrix acm;

  acm.addAllowedCollision("link1", "link2", "test");
  // collision between link1 and link2 should be allowed
  EXPECT_TRUE(acm.isCollisionAllowed("link1", "link2"));
  // but now between link2 and link3
  EXPECT_FALSE(acm.isCollisionAllowed("link2", "link3"));

  acm.removeAllowedCollision("link1", "link2");
  // now collision link1 and link2 is not allowed anymore
  EXPECT_FALSE(acm.isCollisionAllowed("link1", "link2"));

  acm.addAllowedCollision("link3", "link3", "test");
  EXPECT_EQ(acm.getAllAllowedCollisions().size(), 1);
  acm.clearAllowedCollisions();
  EXPECT_EQ(acm.getAllAllowedCollisions().size(), 0);

  tesseract_scene_graph::AllowedCollisionMatrix acm2;
  acm.addAllowedCollision("link1", "link2", "test");
  acm2.addAllowedCollision("link1", "link2", "test");
  acm2.addAllowedCollision("link1", "link3", "test");
  acm.insertAllowedCollisionMatrix(acm2);

  EXPECT_EQ(acm.getAllAllowedCollisions().size(), 2);
  EXPECT_TRUE(acm.isCollisionAllowed("link1", "link2"));
  EXPECT_TRUE(acm.isCollisionAllowed("link1", "link3"));
  EXPECT_FALSE(acm.isCollisionAllowed("link2", "link3"));
  EXPECT_EQ(acm.getAllAllowedCollisions().size(), 2);
}

TEST(TesseractSceneGraphUnit, TestChangeJointOrigin)  // NOLINT
{
  using namespace tesseract_scene_graph;
  SceneGraph g;

  Link link_1("link_n1");
  Link link_2("link_n2");

  Joint joint_1("joint_n1");
  joint_1.parent_link_name = "link_n1";
  joint_1.child_link_name = "link_n2";
  joint_1.type = JointType::FIXED;

  g.addLink(link_1);
  g.addLink(link_2);
  g.addJoint(joint_1);

  Eigen::Isometry3d new_origin = Eigen::Isometry3d::Identity();
  new_origin.translation()(0) += 1.234;
  g.changeJointOrigin("joint_n1", new_origin);

  // Check that the transform got updated and the edge was recalculated.
  EXPECT_TRUE(g.getJoint("joint_n1")->parent_to_joint_origin_transform.isApprox(new_origin));
  tesseract_scene_graph::SceneGraph::edge_descriptor e = g.getEdge("joint_n1");
  double d = boost::get(boost::edge_weight_t(), g)[e];
  EXPECT_EQ(d, g.getJoint("joint_n1")->parent_to_joint_origin_transform.translation().norm());
}

TEST(TesseractSceneGraphUnit, TesseractSceneGraphInsertEmptyUnit)  // NOLINT
{
  // Test inserting graph into empty graph
  tesseract_scene_graph::SceneGraph g = buildTestSceneGraph();
  tesseract_scene_graph::AllowedCollisionMatrix acm;
  acm.addAllowedCollision("link1", "link2", "test");
  acm.addAllowedCollision("link1", "link3", "test");
  g.getAllowedCollisionMatrix()->insertAllowedCollisionMatrix(acm);
  EXPECT_EQ(g.getAllowedCollisionMatrix()->getAllAllowedCollisions().size(), 2);

  tesseract_scene_graph::SceneGraph ng;
  EXPECT_TRUE(ng.insertSceneGraph(g));

  // Check Graph
  checkSceneGraph(ng);

  EXPECT_EQ(g.getLinks().size(), ng.getLinks().size());
  EXPECT_EQ(g.getJoints().size(), ng.getJoints().size());
  EXPECT_EQ(g.getRoot(), ng.getRoot());
  EXPECT_EQ(ng.getAllowedCollisionMatrix()->getAllAllowedCollisions().size(), 2);

  for (const auto& l : g.getLinks())
  {
    EXPECT_TRUE(ng.getLink(l->getName()) != nullptr);
  }

  for (const auto& j : g.getJoints())
  {
    EXPECT_TRUE(ng.getJoint(j->getName()) != nullptr);
  }

  for (const auto& entry : g.getAllowedCollisionMatrix()->getAllAllowedCollisions())
  {
    EXPECT_TRUE(ng.getAllowedCollisionMatrix()->isCollisionAllowed(entry.first.first, entry.first.second));
  }

  // Save Graph
  ng.saveDOT(tesseract_common::getTempPath() + "graph_insert_empty_example.dot");
}

TEST(TesseractSceneGraphUnit, TesseractSceneGraphInsertWithoutJointNoPrefixUnit)  // NOLINT
{
  // Test inserting graph into empty graph
  tesseract_scene_graph::SceneGraph g = buildTestSceneGraph();
  tesseract_scene_graph::AllowedCollisionMatrix acm;
  acm.addAllowedCollision("link1", "link2", "test");
  acm.addAllowedCollision("link1", "link3", "test");
  g.getAllowedCollisionMatrix()->insertAllowedCollisionMatrix(acm);
  EXPECT_EQ(g.getAllowedCollisionMatrix()->getAllAllowedCollisions().size(), 2);

  tesseract_scene_graph::SceneGraph ng = buildTestSceneGraph();
  ng.getAllowedCollisionMatrix()->insertAllowedCollisionMatrix(acm);
  EXPECT_EQ(ng.getAllowedCollisionMatrix()->getAllAllowedCollisions().size(), 2);

  // Insert without prefix which should fail leaving the original graph
  EXPECT_FALSE(ng.insertSceneGraph(g));
  EXPECT_EQ(g.getLinks().size(), ng.getLinks().size());
  EXPECT_EQ(g.getJoints().size(), ng.getJoints().size());
  EXPECT_EQ(g.getRoot(), ng.getRoot());
  EXPECT_EQ(ng.getAllowedCollisionMatrix()->getAllAllowedCollisions().size(), 2);

  for (const auto& l : g.getLinks())
  {
    EXPECT_TRUE(ng.getLink(l->getName()) != nullptr);
  }

  for (const auto& j : g.getJoints())
  {
    EXPECT_TRUE(ng.getJoint(j->getName()) != nullptr);
  }

  for (const auto& entry : g.getAllowedCollisionMatrix()->getAllAllowedCollisions())
  {
    EXPECT_TRUE(ng.getAllowedCollisionMatrix()->isCollisionAllowed(entry.first.first, entry.first.second));
  }
}

TEST(TesseractSceneGraphUnit, TesseractSceneGraphInsertWithoutJointWithPrefixUnit)  // NOLINT
{
  // Test inserting graph into empty graph
  tesseract_scene_graph::SceneGraph g = buildTestSceneGraph();
  tesseract_scene_graph::AllowedCollisionMatrix acm;
  acm.addAllowedCollision("link1", "link2", "test");
  acm.addAllowedCollision("link1", "link3", "test");
  g.getAllowedCollisionMatrix()->insertAllowedCollisionMatrix(acm);
  EXPECT_EQ(g.getAllowedCollisionMatrix()->getAllAllowedCollisions().size(), 2);

  tesseract_scene_graph::SceneGraph ng = buildTestSceneGraph();
  ng.getAllowedCollisionMatrix()->insertAllowedCollisionMatrix(acm);
  EXPECT_EQ(ng.getAllowedCollisionMatrix()->getAllAllowedCollisions().size(), 2);

  std::string prefix = "r1::";
  EXPECT_TRUE(ng.insertSceneGraph(g, prefix));
  EXPECT_FALSE(ng.isTree());

  // Check Graph
  checkSceneGraph(ng);

  EXPECT_EQ(2 * g.getLinks().size(), ng.getLinks().size());
  EXPECT_EQ(2 * g.getJoints().size(), ng.getJoints().size());
  EXPECT_EQ(g.getRoot(), ng.getRoot());
  EXPECT_EQ(ng.getAllowedCollisionMatrix()->getAllAllowedCollisions().size(), 4);

  for (const auto& l : g.getLinks())
  {
    EXPECT_TRUE(ng.getLink(l->getName()) != nullptr);
    EXPECT_TRUE(ng.getLink(prefix + l->getName()) != nullptr);
  }

  for (const auto& j : g.getJoints())
  {
    EXPECT_TRUE(ng.getJoint(j->getName()) != nullptr);
    EXPECT_TRUE(ng.getJoint(prefix + j->getName()) != nullptr);
  }

  for (const auto& entry : g.getAllowedCollisionMatrix()->getAllAllowedCollisions())
  {
    EXPECT_TRUE(ng.getAllowedCollisionMatrix()->isCollisionAllowed(entry.first.first, entry.first.second));
    EXPECT_TRUE(
        ng.getAllowedCollisionMatrix()->isCollisionAllowed(prefix + entry.first.first, prefix + entry.first.second));
  }

  // Save Graph
  ng.saveDOT(tesseract_common::getTempPath() + "graph_insert_example.dot");
}

TEST(TesseractSceneGraphUnit, TesseractSceneGraphInsertWithJointWithPrefixUnit)  // NOLINT
{
  // Test inserting graph into empty graph
  tesseract_scene_graph::SceneGraph g = buildTestSceneGraph();
  tesseract_scene_graph::AllowedCollisionMatrix acm;
  acm.addAllowedCollision("link1", "link2", "test");
  acm.addAllowedCollision("link1", "link3", "test");
  g.getAllowedCollisionMatrix()->insertAllowedCollisionMatrix(acm);
  EXPECT_EQ(g.getAllowedCollisionMatrix()->getAllAllowedCollisions().size(), 2);
  EXPECT_TRUE(g.isCollisionAllowed("link1", "link2"));
  EXPECT_TRUE(g.isCollisionAllowed("link1", "link3"));

  tesseract_scene_graph::SceneGraph ng = buildTestSceneGraph();
  ng.getAllowedCollisionMatrix()->insertAllowedCollisionMatrix(acm);
  EXPECT_EQ(ng.getAllowedCollisionMatrix()->getAllAllowedCollisions().size(), 2);
  EXPECT_TRUE(ng.isCollisionAllowed("link1", "link2"));
  EXPECT_TRUE(ng.isCollisionAllowed("link1", "link3"));

  std::string prefix = "r1::";

  tesseract_scene_graph::Joint new_joint("insert_graph_joint");
  new_joint.parent_link_name = "base_link";
  new_joint.child_link_name = prefix + new_joint.parent_link_name;
  new_joint.type = tesseract_scene_graph::JointType::FIXED;
  new_joint.parent_to_joint_origin_transform = Eigen::Translation3d(1, 0, 0) * Eigen::Isometry3d::Identity();
  EXPECT_TRUE(ng.insertSceneGraph(g, new_joint, prefix));
  EXPECT_TRUE(ng.isTree());

  // Check Graph
  checkSceneGraph(ng);

  EXPECT_EQ(2 * g.getLinks().size(), ng.getLinks().size());
  EXPECT_EQ((2 * g.getJoints().size()) + 1, ng.getJoints().size());
  EXPECT_EQ(g.getRoot(), ng.getRoot());
  EXPECT_EQ(ng.getAllowedCollisionMatrix()->getAllAllowedCollisions().size(), 4);

  for (const auto& l : g.getLinks())
  {
    EXPECT_TRUE(ng.getLink(l->getName()) != nullptr);
    EXPECT_TRUE(ng.getLink(prefix + l->getName()) != nullptr);
  }

  for (const auto& j : g.getJoints())
  {
    EXPECT_TRUE(ng.getJoint(j->getName()) != nullptr);
    EXPECT_TRUE(ng.getJoint(prefix + j->getName()) != nullptr);
  }

  for (const auto& entry : g.getAllowedCollisionMatrix()->getAllAllowedCollisions())
  {
    EXPECT_TRUE(ng.getAllowedCollisionMatrix()->isCollisionAllowed(entry.first.first, entry.first.second));
    EXPECT_TRUE(
        ng.getAllowedCollisionMatrix()->isCollisionAllowed(prefix + entry.first.first, prefix + entry.first.second));
  }

  // Save Graph
  ng.saveDOT(tesseract_common::getTempPath() + "graph_insert_with_joint_example.dot");
}

TEST(TesseractSceneGraphUnit, TesseractSceneState)  // NOLINT
{
  tesseract_scene_graph::SceneState state;
  state.joints["j1"] = 1;
  state.joints["j2"] = 2;
  state.joints["j3"] = 3;
  state.joints["j4"] = 4;
  state.joints["j5"] = 5;
  state.joints["j6"] = 6;

  Eigen::VectorXd jv = state.getJointValues({ "j2", "j3", "j6" });

  EXPECT_NEAR(jv(0), state.joints["j2"], 1e-6);
  EXPECT_NEAR(jv(1), state.joints["j3"], 1e-6);
  EXPECT_NEAR(jv(2), state.joints["j6"], 1e-6);
}

TEST(TesseractSceneGraphUnit, TesseractSceneGraphKDLConversions)  // NOLINT
{
  {  // Eigen to KDL
    Eigen::Isometry3d t = Eigen::Isometry3d::Identity();
    t.translation() = Eigen::Vector3d(1, 2, 3);
    KDL::Frame kdl_t = tesseract_scene_graph::convert(t);
    for (int i = 0; i < 4; ++i)
    {
      for (int j = 0; j < 4; ++j)
      {
        EXPECT_DOUBLE_EQ(kdl_t(i, j), t(i, j));  // NOLINT
      }
    }
  }

  {  // KDL to Eigen
    KDL::Frame kdl_t = KDL::Frame::Identity();
    kdl_t.p = KDL::Vector(1, 2, 3);
    Eigen::Isometry3d t = tesseract_scene_graph::convert(kdl_t);
    for (int i = 0; i < 4; ++i)
    {
      for (int j = 0; j < 4; ++j)
      {
        EXPECT_DOUBLE_EQ(kdl_t(i, j), t(i, j));
      }
    }
  }

  {  // Eigen to KDL
    Eigen::Vector3d v(1, 2, 3);
    KDL::Vector kdl_v = tesseract_scene_graph::convert(v);
    for (int i = 0; i < 3; ++i)
    {
      EXPECT_DOUBLE_EQ(kdl_v(i), v(i));
    }
  }

  {  // KDL to Eigen
    KDL::Vector kdl_v(1, 2, 3);
    Eigen::Vector3d v = tesseract_scene_graph::convert(kdl_v);
    for (int i = 0; i < 3; ++i)
    {
      EXPECT_DOUBLE_EQ(kdl_v(i), v(i));
    }
  }

  {  // Eigen to KDL
    Eigen::MatrixXd t = Eigen::MatrixXd::Random(6, 6);
    KDL::Jacobian kdl_t = tesseract_scene_graph::convert(t);
    for (int i = 0; i < 6; ++i)
    {
      for (int j = 0; j < 6; ++j)
      {
        EXPECT_DOUBLE_EQ(kdl_t(static_cast<unsigned>(i), static_cast<unsigned>(j)), t(i, j));
      }
    }
  }

  {  // KDL to Eigen
    KDL::Jacobian kdl_t;
    kdl_t.resize(6);
    kdl_t.data = Eigen::MatrixXd::Random(6, 6);
    Eigen::MatrixXd t = tesseract_scene_graph::convert(kdl_t);
    EXPECT_EQ(t.cols(), 6);
    EXPECT_EQ(t.rows(), 6);
    for (int i = 0; i < 6; ++i)
    {
      for (int j = 0; j < 6; ++j)
      {
        EXPECT_DOUBLE_EQ(kdl_t(static_cast<unsigned>(i), static_cast<unsigned>(j)), t(i, j));
      }
    }
  }

  {  // KDL to Eigen
    KDL::Jacobian kdl_t;
    kdl_t.resize(6);
    kdl_t.data = Eigen::MatrixXd::Random(6, 6);

    std::vector<int> q_nrs{ 0, 2, 5 };
    Eigen::MatrixXd t = tesseract_scene_graph::convert(kdl_t, q_nrs);
    EXPECT_EQ(t.cols(), 3);
    EXPECT_EQ(t.rows(), 6);
    for (int i = 0; i < 6; ++i)
    {
      EXPECT_DOUBLE_EQ(kdl_t(static_cast<unsigned>(i), 0), t(i, 0));
      EXPECT_DOUBLE_EQ(kdl_t(static_cast<unsigned>(i), 2), t(i, 1));
      EXPECT_DOUBLE_EQ(kdl_t(static_cast<unsigned>(i), 5), t(i, 2));
    }
  }
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);

  return RUN_ALL_TESTS();
}
