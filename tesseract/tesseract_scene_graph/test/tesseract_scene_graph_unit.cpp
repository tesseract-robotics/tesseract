#include <tesseract_common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <gtest/gtest.h>
//#include <boost/graph/filtered_graph.hpp>
#include <iostream>
#include <fstream>
#include <tesseract_geometry/geometries.h>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract_scene_graph/graph.h>
#include <tesseract_scene_graph/utils.h>
#include <tesseract_scene_graph/resource_locator.h>
#include <tesseract_scene_graph/parser/srdf_parser.h>
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

TEST(TesseractSceneGraphUnit, TesseractSceneGraphUnit)  // NOLINT
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
  g.saveDOT("/tmp/graph_acyclic_tree_example.dot");

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

  g.removeLink(link_6.getName());
  std::cout << "Free Link Removed, Is Tree: " << g.isTree() << std::endl;
  EXPECT_TRUE(g.isTree());

  // Check Graph
  checkSceneGraph(g);

  Joint joint_5("joint_5");
  joint_5.parent_to_joint_origin_transform.translation()(1) = 1.25;
  joint_5.parent_link_name = "link_5";
  joint_5.child_link_name = "link_4";
  joint_5.type = JointType::CONTINUOUS;
  g.addJoint(joint_5);

  // Check Graph
  checkSceneGraph(g);

  // Save Graph
  g.saveDOT("/tmp/graph_acyclic_not_tree_example.dot");

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
  g.saveDOT("/tmp/graph_cyclic_not_tree_example.dot");

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
}

std::string locateResource(const std::string& url)
{
  std::string mod_url = url;
  if (url.find("package://tesseract_support") == 0)
  {
    mod_url.erase(0, strlen("package://tesseract_support"));
    size_t pos = mod_url.find('/');
    if (pos == std::string::npos)
    {
      return std::string();
    }

    std::string package = mod_url.substr(0, pos);
    mod_url.erase(0, pos);
    std::string package_path = std::string(TESSERACT_SUPPORT_DIR);

    if (package_path.empty())
    {
      return std::string();
    }

    mod_url = package_path + mod_url;
  }

  return mod_url;
}

TEST(TesseractSceneGraphUnit, LoadSRDFUnit)  // NOLINT
{
  using namespace tesseract_scene_graph;

  std::string srdf_file = std::string(TESSERACT_SUPPORT_DIR) + "/urdf/lbr_iiwa_14_r820.srdf";

  ResourceLocator::Ptr locator = std::make_shared<SimpleResourceLocator>(locateResource);
  SceneGraph g;

  g.setName("kuka_lbr_iiwa_14_r820");

  Link base_link("base_link");
  Link link_1("link_1");
  Link link_2("link_2");
  Link link_3("link_3");
  Link link_4("link_4");
  Link link_5("link_5");
  Link link_6("link_6");
  Link link_7("link_7");
  Link tool0("tool0");

  g.addLink(base_link);
  g.addLink(link_1);
  g.addLink(link_2);
  g.addLink(link_3);
  g.addLink(link_4);
  g.addLink(link_5);
  g.addLink(link_6);
  g.addLink(link_7);
  g.addLink(tool0);

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
  joint_4.parent_link_name = "link_4";
  joint_4.child_link_name = "link_5";
  joint_4.type = JointType::REVOLUTE;
  g.addJoint(joint_4);

  Joint joint_5("joint_5");
  joint_5.parent_to_joint_origin_transform.translation()(1) = 1.25;
  joint_5.parent_link_name = "link_5";
  joint_5.child_link_name = "link_6";
  joint_5.type = JointType::REVOLUTE;
  g.addJoint(joint_5);

  Joint joint_6("joint_6");
  joint_6.parent_to_joint_origin_transform.translation()(1) = 1.25;
  joint_6.parent_link_name = "link_6";
  joint_6.child_link_name = "link_7";
  joint_6.type = JointType::REVOLUTE;
  g.addJoint(joint_6);

  Joint joint_tool0("joint_tool0");
  joint_tool0.parent_link_name = "link_7";
  joint_tool0.child_link_name = "tool0";
  joint_tool0.type = JointType::FIXED;
  g.addJoint(joint_tool0);

  SRDFModel srdf;
  EXPECT_TRUE(srdf.initFile(g, srdf_file));

  processSRDFAllowedCollisions(g, srdf);

  AllowedCollisionMatrix::ConstPtr acm = g.getAllowedCollisionMatrix();

  // collision between link1 and link2 should be allowed
  EXPECT_TRUE(acm->isCollisionAllowed("link_1", "link_2"));

  // collision between link1 and link2 should be allowed
  EXPECT_FALSE(acm->isCollisionAllowed("base_link", "link_5"));

  g.removeAllowedCollision("link_1", "link_2");
  // now collision link1 and link2 is not allowed anymore
  EXPECT_FALSE(acm->isCollisionAllowed("link_1", "link_2"));

  g.clearAllowedCollisions();
  EXPECT_EQ(acm->getAllAllowedCollisions().size(), 0);
}

void printKDLTree(const KDL::SegmentMap::const_iterator& link, const std::string& prefix)
{
  std::cout << prefix << "- Segment " << GetTreeElementSegment(link->second).getName() << " has "
            << GetTreeElementChildren(link->second).size() << " children" << std::endl;

  for (auto child : GetTreeElementChildren(link->second))
    printKDLTree(child, prefix + "  ");
}

TEST(TesseractSceneGraphUnit, LoadKDLUnit)  // NOLINT
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

  KDL::Tree tree;
  EXPECT_TRUE(parseSceneGraph(g, tree));

  // walk through tree
  std::cout << " ======================================" << std::endl;
  std::cout << " Tree has " << tree.getNrOfSegments() << " link(s) and a root link" << std::endl;
  std::cout << " ======================================" << std::endl;
  auto root = tree.getRootSegment();
  printKDLTree(root, "");

  EXPECT_TRUE(tree.getNrOfJoints() == 4);
  EXPECT_TRUE(tree.getNrOfSegments() == 5);
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

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);

  return RUN_ALL_TESTS();
}
