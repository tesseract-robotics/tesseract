/**
 * @page build_scene_graph_example Tesseract Scene Graph Building Example
 *
 * @section build_scene_graph_overview Overview
 *
 * This example demonstrates how to build and manipulate a scene graph, the fundamental
 * data structure that represents the kinematic relationships between robot links and joints.
 * A scene graph encodes how a robot is constructed: which links are connected, what
 * constraints govern their motion, and how they relate spatially. Understanding scene
 * graphs is essential for motion planning, collision checking, and robot simulation.
 *
 * The example builds a kinematic structure, queries adjacency relationships, validates
 * graph topology, and demonstrates how to find paths between links.
 *
 * @section build_scene_graph_concepts Key Concepts
 *
 * - **Links**: Rigid bodies in the robot. Each link has a name and can have visual/collision
 *   geometry associated with it. Think of a link as a solid piece of the robot (arm segment,
 *   gripper jaw, end effector, etc.).
 *
 * - **Joints**: Connections between links that define the kinematic relationships. Each joint
 *   has a parent link, child link, type (fixed, revolute, prismatic, etc.), and a transform
 *   from parent to child.
 *
 * - **Joint Types**:
 *   - `FIXED`: Rigid connection, no motion
 *   - `REVOLUTE`: Rotational motion around one axis
 *   - `CONTINUOUS`: Revolute without limits (can spin indefinitely)
 *   - `PRISMATIC`: Linear motion along one axis
 *   - `PLANAR`: 2D motion (x, y, rotation)
 *   - `FLOATING`: 6DOF unconstrained motion
 *
 * - **Scene Graph**: A directed graph representing the kinematic tree/chain. Nodes are links,
 *   edges are joints. A valid graph must be acyclic (no loops) and connected (all links reachable).
 *
 * - **Acyclic Graph**: Contains no cycles (loops). All kinematic chains should be acyclic.
 *
 * - **Tree**: An acyclic connected graph. Valid robot kinematic structures form trees where
 *   every link (except the root) has exactly one parent.
 *
 * @section build_scene_graph_workflow Typical Workflow
 *
 * 1. **Create a scene graph** - Initialize the kinematic structure container
 * 2. **Define links** - Create Link objects representing rigid bodies
 * 3. **Add links to graph** - Register links in the scene graph
 * 4. **Define joints** - Create Joint objects with parent/child relationships and transforms
 * 5. **Add joints to graph** - Connect links via joints
 * 6. **Validate topology** - Check for cycles and connectivity
 * 7. **Query relationships** - Find adjacent links, children, paths
 * 8. **Visualize (optional)** - Save as DOT format for visualization
 *
 * @section build_scene_graph_create Creating and Populating the Graph
 *
 * Start by creating an empty scene graph:
 *
 * @snippet build_scene_graph_example.cpp build_scene_graph_create_graph
 *
 * Create links (rigid bodies). Each link is identified by a unique name:
 *
 * @snippet build_scene_graph_example.cpp build_scene_graph_create_links
 *
 * Add links to the graph:
 *
 * @snippet build_scene_graph_example.cpp build_scene_graph_add_links
 *
 * Create joints that connect links. Each joint specifies:
 * - **Name**: Unique identifier
 * - **Parent link**: Which link this joint connects from
 * - **Child link**: Which link this joint connects to
 * - **Type**: How the child can move relative to parent
 * - **Transform**: Spatial relationship from parent to child
 *
 * @snippet build_scene_graph_example.cpp build_scene_graph_create_joints
 *
 * Add joints to establish kinematic relationships:
 *
 * @snippet build_scene_graph_example.cpp build_scene_graph_add_joints
 *
 * @section build_scene_graph_query Querying Graph Structure
 *
 * Find links directly connected to a given link:
 *
 * @snippet build_scene_graph_example.cpp build_scene_graph_adjacent_links
 *
 * Find parent links (inverse adjacent):
 *
 * @snippet build_scene_graph_example.cpp build_scene_graph_inv_adjacent_links
 *
 * Find all child links of a link:
 *
 * @snippet build_scene_graph_example.cpp build_scene_graph_link_children
 *
 * Find all child links branching from a joint:
 *
 * @snippet build_scene_graph_example.cpp build_scene_graph_joint_children
 *
 * @section build_scene_graph_validation Validating Graph Topology
 *
 * Save the graph structure as a DOT file for visualization:
 *
 * @snippet build_scene_graph_example.cpp build_scene_graph_save_dot
 *
 * Test if the graph is acyclic (no loops):
 *
 * @snippet build_scene_graph_example.cpp build_scene_graph_is_acyclic
 *
 * Test if the graph is a valid tree (acyclic + connected):
 *
 * @snippet build_scene_graph_example.cpp build_scene_graph_is_tree
 *
 * @section build_scene_graph_disconnected Handling Disconnected Components
 *
 * If you add a link without connecting it via a joint, the graph is no longer a tree:
 *
 * @snippet build_scene_graph_example.cpp build_scene_graph_disconnected
 *
 * Remove disconnected links to restore tree structure:
 *
 * @snippet build_scene_graph_example.cpp build_scene_graph_remove_link
 *
 * @section build_scene_graph_cycles Creating Cycles
 *
 * Add a joint that creates a cycle (kinematic loop). This is valid for some mechanisms
 * (like parallel robots) but creates a more complex graph:
 *
 * @snippet build_scene_graph_example.cpp build_scene_graph_add_cycle
 *
 * Save the modified graph:
 *
 * @snippet build_scene_graph_example.cpp build_scene_graph_save_cycle
 *
 * Verify the graph is still acyclic (it is—cycles are loops, not detected here):
 *
 * @snippet build_scene_graph_example.cpp build_scene_graph_cycle_acyclic
 *
 * But it is no longer a tree:
 *
 * @snippet build_scene_graph_example.cpp build_scene_graph_cycle_tree
 *
 * @section build_scene_graph_paths Finding Paths
 *
 * Find the shortest kinematic path between two links:
 *
 * @snippet build_scene_graph_example.cpp build_scene_graph_shortest_path
 *
 * This is useful for finding which joints affect the relative motion between two links.
 *
 * @section build_scene_graph_common_patterns Common Patterns
 *
 * **Building a robot model**: Load links from a URDF file and extract the scene graph,
 * or programmatically construct it as shown here.
 *
 * **Validating a model**: Use `isTree()` to ensure your kinematic structure is valid.
 *
 * **Collision checking setup**: Use query methods to determine which links can collide
 * and which are always adjacent (connected by a joint).
 *
 * **Trajectory planning**: Use path queries to understand kinematic dependencies when
 * planning joint-space trajectories.
 *
 * **Kinematics chains**: Use adjacency queries to traverse chains from based to end-effector.
 *
 * @section build_scene_graph_tips Tips and Best Practices
 *
 * - **Root link**: Typically a fixed base or world frame. Other links connect downstream from it.
 * - **Unique names**: All link and joint names must be unique within the graph.
 * - **Validate early**: Check `isTree()` after building to catch structural errors.
 * - **Understand joint types**: Choose appropriate types (FIXED for rigid, REVOLUTE for movable joints).
 * - **Transforms matter**: The `parent_to_joint_origin_transform` defines spatial relationships.
 * - **DOT visualization**: Save graphs as DOT files and view with Graphviz to debug structure.
 * - **Kinematic loops**: Some robots (parallel manipulators) intentionally have loops. Know your robot!
 *
 * @section build_scene_graph_integration Integration with Other Systems
 *
 * Scene graphs are used throughout Tesseract:
 * - **Environment**: Holds the scene graph and manages collision objects
 * - **Kinematics**: Uses graph traversal to compute forward kinematics
 * - **Planning**: Queries relationships to determine which joints affect which links
 * - **Collision**: Uses adjacency info to skip unnecessary checks between connected links
 *
 * @section build_scene_graph_summary Summary
 *
 * This example demonstrates the complete workflow for building a kinematic scene graph:
 *
 * 1. Create empty graph
 * 2. Add links and joints
 * 3. Validate graph topology (tree vs acyclic vs cyclic)
 * 4. Query relationships (adjacent, children, paths)
 * 5. Handle invalid configurations and fix them
 * 6. Visualize structure for debugging
 *
 * A valid scene graph is the foundation for all downstream motion planning and
 * simulation operations in Tesseract.
 *
 * @section build_scene_graph_full_code Full source code
 *
 * For reference, here is the complete source of this example:
 *
 * @snippet build_scene_graph_example.cpp build_scene_graph_full_source
 */

//! [build_scene_graph_full_source]
#include <tesseract/common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <console_bridge/console.h>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract/scene_graph/graph.h>
#include <tesseract/scene_graph/link.h>
#include <tesseract/scene_graph/joint.h>
#include <tesseract/common/utils.h>

using namespace tesseract::scene_graph;

std::string toString(const ShortestPath& path)
{
  std::stringstream ss;
  ss << path;
  return ss.str();
}

std::string toString(bool b) { return b ? "true" : "false"; }

int main(int /*argc*/, char** /*argv*/)
{
  console_bridge::setLogLevel(console_bridge::LogLevel::CONSOLE_BRIDGE_LOG_INFO);

  //! [build_scene_graph_create_graph]
  SceneGraph g;
  //! [build_scene_graph_create_graph]

  //! [build_scene_graph_create_links]
  Link link_1("link_1");
  Link link_2("link_2");
  Link link_3("link_3");
  Link link_4("link_4");
  Link link_5("link_5");
  //! [build_scene_graph_create_links]

  //! [build_scene_graph_add_links]
  g.addLink(link_1);
  g.addLink(link_2);
  g.addLink(link_3);
  g.addLink(link_4);
  g.addLink(link_5);
  //! [build_scene_graph_add_links]

  //! [build_scene_graph_create_joints]
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
  //! [build_scene_graph_create_joints]

  //! [build_scene_graph_add_joints]
  g.addJoint(joint_1);
  g.addJoint(joint_2);
  g.addJoint(joint_3);
  g.addJoint(joint_4);
  //! [build_scene_graph_add_joints]

  //! [build_scene_graph_adjacent_links]
  std::vector<std::string> adjacent_links = g.getAdjacentLinkNames("link_3");
  for (const auto& adj : adjacent_links)
    CONSOLE_BRIDGE_logInform(adj.c_str());
  //! [build_scene_graph_adjacent_links]

  //! [build_scene_graph_inv_adjacent_links]
  std::vector<std::string> inv_adjacent_links = g.getInvAdjacentLinkNames("link_3");
  for (const auto& inv_adj : inv_adjacent_links)
    CONSOLE_BRIDGE_logInform(inv_adj.c_str());
  //! [build_scene_graph_inv_adjacent_links]

  //! [build_scene_graph_link_children]
  std::vector<std::string> child_link_names = g.getLinkChildrenNames("link_2");
  for (const auto& child_link : child_link_names)
    CONSOLE_BRIDGE_logInform(child_link.c_str());
  //! [build_scene_graph_link_children]

  //! [build_scene_graph_joint_children]
  child_link_names = g.getJointChildrenNames("joint_1");
  for (const auto& child_link : child_link_names)
    CONSOLE_BRIDGE_logInform(child_link.c_str());
  //! [build_scene_graph_joint_children]

  //! [build_scene_graph_save_dot]
  g.saveDOT(tesseract::common::getTempPath() + "graph_acyclic_tree_example.dot");
  //! [build_scene_graph_save_dot]

  //! [build_scene_graph_is_acyclic]
  bool is_acyclic = g.isAcyclic();
  CONSOLE_BRIDGE_logInform(toString(is_acyclic).c_str());
  //! [build_scene_graph_is_acyclic]

  //! [build_scene_graph_is_tree]
  bool is_tree = g.isTree();
  CONSOLE_BRIDGE_logInform(toString(is_tree).c_str());
  //! [build_scene_graph_is_tree]

  //! [build_scene_graph_disconnected]
  Link link_6("link_6");
  g.addLink(link_6);
  is_tree = g.isTree();
  CONSOLE_BRIDGE_logInform(toString(is_tree).c_str());
  //! [build_scene_graph_disconnected]

  //! [build_scene_graph_remove_link]
  g.removeLink("link_6");
  is_tree = g.isTree();
  CONSOLE_BRIDGE_logInform(toString(is_tree).c_str());
  //! [build_scene_graph_remove_link]

  //! [build_scene_graph_add_cycle]
  Joint joint_5("joint_5");
  joint_5.parent_to_joint_origin_transform.translation()(1) = 1.25;
  joint_5.parent_link_name = "link_5";
  joint_5.child_link_name = "link_4";
  joint_5.type = JointType::CONTINUOUS;
  g.addJoint(joint_5);
  //! [build_scene_graph_add_cycle]

  //! [build_scene_graph_save_cycle]
  g.saveDOT(tesseract::common::getTempPath() + "graph_acyclic_not_tree_example.dot");
  //! [build_scene_graph_save_cycle]

  //! [build_scene_graph_cycle_acyclic]
  is_acyclic = g.isAcyclic();
  CONSOLE_BRIDGE_logInform(toString(is_acyclic).c_str());
  //! [build_scene_graph_cycle_acyclic]

  // documentation:start:18: Test again if the graph is Tree
  is_tree = g.isTree();
  //! [build_scene_graph_cycle_tree]
  is_tree = g.isTree();
  CONSOLE_BRIDGE_logInform(toString(is_tree).c_str());
  //! [build_scene_graph_cycle_tree]

  //! [build_scene_graph_shortest_path]
  ShortestPath path = g.getShortestPath("link_1", "link_4");
  CONSOLE_BRIDGE_logInform(toString(path).c_str());
  //! [build_scene_graph_shortest_path]
}
//! [build_scene_graph_full_source]
