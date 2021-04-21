#include <tesseract_common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <gtest/gtest.h>
//#include <boost/graph/filtered_graph.hpp>
#include <iostream>
#include <fstream>
#include <tesseract_geometry/geometries.h>
#include <tesseract_common/utils.h>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract_scene_graph/graph.h>
#include <tesseract_scene_graph/utils.h>
#include <tesseract_scene_graph/resource_locator.h>
#include <tesseract_scene_graph/parser/kdl_parser.h>

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

  g.addLink(link_1);
  g.addLink(link_2);
  g.addLink(link_3);
  g.addLink(link_4);
  g.addLink(link_5);

  Joint joint_1("joint_1");
  joint_1.parent_to_joint_origin_transform.translation()(0) = 1.25;
  joint_1.parent_link_name = "link_1";
  joint_1.child_link_name = "link_2";
  joint_1.type = JointType::FIXED;
  g.addJoint(joint_1);

  Joint joint_2("joint_2");
  joint_2.parent_to_joint_origin_transform.translation()(0) = 1.25;
  joint_2.parent_link_name = "link_2";
  joint_2.child_link_name = "link_3";
  joint_2.type = JointType::PLANAR;
  g.addJoint(joint_2);

  Joint joint_3("joint_3");
  joint_3.parent_to_joint_origin_transform.translation()(0) = 1.25;
  joint_3.parent_link_name = "link_3";
  joint_3.child_link_name = "link_4";
  joint_3.type = JointType::FLOATING;
  g.addJoint(joint_3);

  Joint joint_4("joint_4");
  joint_4.parent_to_joint_origin_transform.translation()(1) = 1.25;
  joint_4.parent_link_name = "link_2";
  joint_4.child_link_name = "link_5";
  joint_4.type = JointType::REVOLUTE;
  g.addJoint(joint_4);

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

  // Check getJointChildrenNames
  child_link_names = g.getJointChildrenNames("joint_4");
  EXPECT_TRUE(child_link_names.size() == 1);
  EXPECT_TRUE(child_link_names[0] == "link_5");

  child_link_names = g.getJointChildrenNames("joint_3");
  EXPECT_TRUE(child_link_names.size() == 1);
  EXPECT_TRUE(child_link_names[0] == "link_4");

  child_link_names = g.getJointChildrenNames("joint_1");
  EXPECT_TRUE(child_link_names.size() == 4);
  EXPECT_TRUE(std::find(child_link_names.begin(), child_link_names.end(), "link_2") != child_link_names.end());
  EXPECT_TRUE(std::find(child_link_names.begin(), child_link_names.end(), "link_3") != child_link_names.end());
  EXPECT_TRUE(std::find(child_link_names.begin(), child_link_names.end(), "link_4") != child_link_names.end());
  EXPECT_TRUE(std::find(child_link_names.begin(), child_link_names.end(), "link_5") != child_link_names.end());

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

  // Get Shortest Path
  SceneGraph::Path path = g.getShortestPath("link_1", "link_4");

  // Todo:: Look at using filtered graph for chains and subgraphs
  // boost::filtered_graph<

  std::cout << path << std::endl;
  EXPECT_TRUE(path.first.size() == 4);
  EXPECT_TRUE(std::find(path.first.begin(), path.first.end(), "link_1") != path.first.end());
  EXPECT_TRUE(std::find(path.first.begin(), path.first.end(), "link_2") != path.first.end());
  EXPECT_TRUE(std::find(path.first.begin(), path.first.end(), "link_3") != path.first.end());
  EXPECT_TRUE(std::find(path.first.begin(), path.first.end(), "link_4") != path.first.end());
  EXPECT_TRUE(path.second.size() == 3);
  EXPECT_TRUE(std::find(path.second.begin(), path.second.end(), "joint_1") != path.second.end());
  EXPECT_TRUE(std::find(path.second.begin(), path.second.end(), "joint_2") != path.second.end());
  EXPECT_TRUE(std::find(path.second.begin(), path.second.end(), "joint_3") != path.second.end());

  std::cout << (g.getName().c_str()) << std::endl;

  // Should throw since this is a directory and not a file
  EXPECT_ANY_THROW(g.saveDOT(tesseract_common::getTempPath()));
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

  Link link_6d("link_6");
  Joint joint_6d("joint_5");
  joint_6d.parent_to_joint_origin_transform.translation()(0) = 1.25;
  joint_6d.parent_link_name = "link_5";
  joint_6d.child_link_name = "link_6";
  joint_6d.type = JointType::FIXED;
  EXPECT_FALSE(g.addLink(link_6d, joint_6d));
  EXPECT_EQ(g.getLinks().size(), 6);
  EXPECT_EQ(g.getJoints().size(), 5);
}

TEST(TesseractSceneGraphUnit, TesseractSceneGraphRemoveLinkUnit)  // NOLINT
{
  using namespace tesseract_scene_graph;
  SceneGraph g = createTestSceneGraph();

  EXPECT_EQ(g.getLinks().size(), 5);
  EXPECT_EQ(g.getJoints().size(), 4);
  EXPECT_TRUE(g.removeLink("link_5"));
  EXPECT_TRUE(g.getJoint("joint_4") == nullptr);
  EXPECT_TRUE(g.getLink("link_5") == nullptr);
  EXPECT_EQ(g.getLinks().size(), 4);
  EXPECT_EQ(g.getJoints().size(), 3);

  // Make sure all children are removed
  /** @todo Doing this causes the test to fail but assign to new object works. Need to look into why */
  //  g = createTestSceneGraph();
  SceneGraph ng = createTestSceneGraph();
  EXPECT_EQ(ng.getLinks().size(), 5);
  EXPECT_EQ(ng.getJoints().size(), 4);
  EXPECT_TRUE(ng.removeLink("link_2"));
  EXPECT_EQ(ng.getLinks().size(), 4);
  EXPECT_EQ(ng.getJoints().size(), 1);
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
}

TEST(TesseractSceneGraphUnit, TesseractSceneGraphChangeJointLimitsUnit)  // NOLINT
{
  using namespace tesseract_scene_graph;
  SceneGraph g = createTestSceneGraph();

  JointLimits::ConstPtr jl = g.getJointLimits("joint_4");
  EXPECT_TRUE(jl == nullptr);

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
  SceneGraph g = createTestSceneGraph();

  EXPECT_EQ(g.getLinks().size(), 5);
  EXPECT_EQ(g.getJoints().size(), 4);
  EXPECT_TRUE(g.removeJoint("joint_4"));
  EXPECT_TRUE(g.getJoint("joint_4") == nullptr);
  EXPECT_TRUE(g.getLink("link_5") != nullptr);  // The child link should remain
  EXPECT_EQ(g.getLinks().size(), 5);
  EXPECT_EQ(g.getJoints().size(), 3);
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

  g.addLink(base_link);
  g.addLink(link_1);
  g.addLink(link_2);
  g.addLink(link_3);
  g.addLink(link_4);
  g.addLink(link_5);

  Joint base_joint("base_joint");
  base_joint.parent_link_name = "base_link";
  base_joint.child_link_name = "link_1";
  base_joint.type = JointType::FIXED;
  g.addJoint(base_joint);

  Joint joint_1("joint_1");
  joint_1.parent_link_name = "link_1";
  joint_1.child_link_name = "link_2";
  joint_1.type = JointType::REVOLUTE;
  g.addJoint(joint_1);

  Joint joint_2("joint_2");
  joint_2.parent_to_joint_origin_transform.translation()(0) = 1.25;
  joint_2.parent_link_name = "link_2";
  joint_2.child_link_name = "link_3";
  joint_2.type = JointType::REVOLUTE;
  g.addJoint(joint_2);

  Joint joint_3("joint_3");
  joint_3.parent_to_joint_origin_transform.translation()(0) = 1.25;
  joint_3.parent_link_name = "link_3";
  joint_3.child_link_name = "link_4";
  joint_3.type = JointType::REVOLUTE;
  g.addJoint(joint_3);

  Joint joint_4("joint_4");
  joint_4.parent_to_joint_origin_transform.translation()(1) = 1.25;
  joint_4.parent_link_name = "link_2";
  joint_4.child_link_name = "link_5";
  joint_4.type = JointType::REVOLUTE;
  g.addJoint(joint_4);

  return g;
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

  {
    KDL::Tree tree;
    EXPECT_TRUE(parseSceneGraph(g, tree));

    // walk through tree
    std::cout << " ======================================" << std::endl;
    std::cout << " Tree has " << tree.getNrOfSegments() << " link(s) and a root link" << std::endl;
    std::cout << " ======================================" << std::endl;
    auto root = tree.getRootSegment();
    printKDLTree(root, "");

    EXPECT_EQ(tree.getNrOfJoints(), 4);
    EXPECT_EQ(tree.getNrOfSegments(), 5);
  }

  SceneGraph::Ptr g_clone = g.clone();

  // Check to make sure all links are enabled
  for (const auto& link : g_clone->getLinks())
  {
    EXPECT_TRUE(g_clone->getLinkCollisionEnabled(link->getName()));
    EXPECT_TRUE(g_clone->getLinkVisibility(link->getName()));
  }

  {
    KDL::Tree tree;
    EXPECT_TRUE(parseSceneGraph(*g_clone, tree));

    // walk through tree
    std::cout << " ======================================" << std::endl;
    std::cout << " Tree has " << tree.getNrOfSegments() << " link(s) and a root link" << std::endl;
    std::cout << " ======================================" << std::endl;
    auto root = tree.getRootSegment();
    printKDLTree(root, "");

    EXPECT_EQ(tree.getNrOfJoints(), 4);
    EXPECT_EQ(tree.getNrOfSegments(), 5);
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
  EXPECT_TRUE(ng.insertSceneGraph(g, std::move(new_joint), prefix));
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

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);

  return RUN_ALL_TESTS();
}
