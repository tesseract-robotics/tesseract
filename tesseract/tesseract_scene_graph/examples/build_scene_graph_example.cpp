#include <console_bridge/console.h>
#include <tesseract_scene_graph/graph.h>
#include <iostream>

using namespace tesseract_scene_graph;

std::string toString(const SceneGraph::Path& path)
{
  std::stringstream ss;
  ss << path;
  return ss.str();
}

std::string toString(bool b) { return b ? "true" : "false"; }

int main(int /*argc*/, char** /*argv*/)
{
  console_bridge::setLogLevel(console_bridge::LogLevel::CONSOLE_BRIDGE_LOG_INFO);

  // Create scene graph
  SceneGraph g;

  // Create links
  Link link_1("link_1");
  Link link_2("link_2");
  Link link_3("link_3");
  Link link_4("link_4");
  Link link_5("link_5");

  // Add links
  g.addLink(link_1);
  g.addLink(link_2);
  g.addLink(link_3);
  g.addLink(link_4);
  g.addLink(link_5);

  // Create joints
  Joint joint_1("joint_1");
  joint_1.parent_to_joint_origin_transform.translation()(0) = 1.25;
  joint_1.parent_link_name = "link_1";
  joint_1.child_link_name = "link_2";
  joint_1.type = JointType::FIXED;

  Joint joint_2("joint_2");
  joint_2.parent_to_joint_origin_transform.translation()(0) = 1.25;
  joint_2.parent_link_name = "link_2";
  joint_2.child_link_name = "link_3";
  joint_2.type = JointType::PLANAR;

  Joint joint_3("joint_3");
  joint_3.parent_to_joint_origin_transform.translation()(0) = 1.25;
  joint_3.parent_link_name = "link_3";
  joint_3.child_link_name = "link_4";
  joint_3.type = JointType::FLOATING;

  Joint joint_4("joint_4");
  joint_4.parent_to_joint_origin_transform.translation()(1) = 1.25;
  joint_4.parent_link_name = "link_2";
  joint_4.child_link_name = "link_5";
  joint_4.type = JointType::REVOLUTE;

  // Add joints
  g.addJoint(joint_1);
  g.addJoint(joint_2);
  g.addJoint(joint_3);
  g.addJoint(joint_4);

  // Check getAdjacentLinkNames Method
  std::vector<std::string> adjacent_links = g.getAdjacentLinkNames("link_3");
  for (const auto& adj : adjacent_links)
    CONSOLE_BRIDGE_logInform(adj.c_str());

  // Check getInvAdjacentLinkNames Method
  std::vector<std::string> inv_adjacent_links = g.getInvAdjacentLinkNames("link_3");
  for (const auto& inv_adj : inv_adjacent_links)
    CONSOLE_BRIDGE_logInform(inv_adj.c_str());

  // Check getLinkChildrenNames
  std::vector<std::string> child_link_names = g.getLinkChildrenNames("link_2");
  for (const auto& child_link : child_link_names)
    CONSOLE_BRIDGE_logInform(child_link.c_str());

  // Check getJointChildrenNames
  child_link_names = g.getJointChildrenNames("joint_1");
  for (const auto& child_link : child_link_names)
    CONSOLE_BRIDGE_logInform(child_link.c_str());

  // Save Graph
  g.saveDOT("/tmp/graph_acyclic_tree_example.dot");

  // Test if the graph is Acyclic
  bool is_acyclic = g.isAcyclic();
  CONSOLE_BRIDGE_logInform(toString(is_acyclic).c_str());

  // Test if the graph is Tree
  bool is_tree = g.isTree();
  CONSOLE_BRIDGE_logInform(toString(is_tree).c_str());

  // Test for unused links
  Link link_6("link_6");
  g.addLink(link_6);
  is_tree = g.isTree();
  CONSOLE_BRIDGE_logInform(toString(is_tree).c_str());

  // Remove unused link
  g.removeLink(link_6.getName());
  is_tree = g.isTree();
  CONSOLE_BRIDGE_logInform(toString(is_tree).c_str());

  // Add new joint
  Joint joint_5("joint_5");
  joint_5.parent_to_joint_origin_transform.translation()(1) = 1.25;
  joint_5.parent_link_name = "link_5";
  joint_5.child_link_name = "link_4";
  joint_5.type = JointType::CONTINUOUS;
  g.addJoint(joint_5);

  // Save new graph
  g.saveDOT("/tmp/graph_acyclic_not_tree_example.dot");

  // Test again if the graph is Acyclic
  is_acyclic = g.isAcyclic();
  std::cout << toString(is_acyclic).c_str();

  // Test again if the graph is Tree
  is_tree = g.isTree();
  CONSOLE_BRIDGE_logInform(toString(is_tree).c_str());

  // Get Shortest Path
  SceneGraph::Path path = g.getShortestPath("link_1", "link_4");
  CONSOLE_BRIDGE_logInform(toString(path).c_str());
}
