/**
 * @page load_urdf_example Tesseract URDF Loading Example
 *
 * @section load_urdf_overview Overview
 *
 * This example demonstrates how to load and parse URDF (Unified Robot Description Format)
 * files in Tesseract. URDF is a standard XML format used across the ROS ecosystem to
 * describe robot structure, kinematics, dynamics, and geometry. Loading URDF files is
 * typically the first step when setting up a robot for collision checking, motion planning,
 * or simulation. This example shows the minimal workflow to load a URDF and access its
 * resulting kinematic structure.
 *
 * The example loads the KUKA LBR iiwa 14 R820 robot from a URDF file, parses it into
 * Tesseract's scene graph representation, validates the structure, and visualizes it.
 *
 * @section load_urdf_concepts Key Concepts
 *
 * - **URDF (Unified Robot Description Format)**: XML format that defines a robot's kinematic
 *   chain (links and joints), collision geometry, visual geometry, and dynamics (mass, inertia).
 *   The de facto standard for robot description in ROS.
 *
 * - **URDF Links**: Rigid bodies in the robot model. Each link can optionally have visual mesh,
 *   collision geometry, inertia properties, and materials.
 *
 * - **URDF Joints**: Connections between links defining kinematic relationships. Specify
 *   parent/child links, joint type (revolute, prismatic, fixed, etc.), axis of motion, and limits.
 *
 * - **Kinematic Tree**: The hierarchical structure formed by links and joints. The root link
 *   is typically a fixed base, and other links branch downward.
 *
 * - **Scene Graph**: Tesseract's internal representation of a robot's kinematic structure,
 *   parsed from URDF. Enables efficient queries about relationships between links.
 *
 * - **Resource Locator**: Resolves package:// URIs (e.g., `package://robot_description/...`)
 *   to actual file paths on disk.
 *
 * @section load_urdf_urdf_structure URDF File Structure
 *
 * A typical URDF contains:
 *
 * ```xml
 * <?xml version="1.0"?>
 * <robot name=\"robot_name\">
 *   <!-- Links: rigid bodies -->
 *   <link name=\"base_link\">
 *     <inertial>
 *       <mass value=\"10.0\"/>
 *       <origin xyz=\"0 0 0.5\" rpy=\"0 0 0\"/>
 *       <inertia ixx=\"0.1\" ixy=\"0\" ixz=\"0\" iyy=\"0.1\" iyz=\"0\" izz=\"0.05\"/>
 *     </inertial>
 *     <visual>
 *       <origin xyz=\"0 0 0\" rpy=\"0 0 0\"/>
 *       <geometry>
 *         <mesh filename=\"package://robot_description/meshes/base.dae\"/>
 *       </geometry>
 *     </visual>
 *     <collision>
 *       <origin xyz=\"0 0 0\" rpy=\"0 0 0\"/>
 *       <geometry>
 *         <box size=\"1 1 1\"/>
 *       </geometry>
 *     </collision>
 *   </link>
 *
 *   <!-- Joints: connect links -->
 *   <joint name=\"joint_1\" type=\"revolute\">
 *     <parent link=\"base_link\"/>
 *     <child link=\"link_1\"/>
 *     <origin xyz=\"0 0 1\" rpy=\"0 0 0\"/>
 *     <axis xyz=\"0 0 1\"/>
 *     <limit lower=\"0\" upper=\"3.14\" effort=\"50\" velocity=\"2.0\"/>
 *   </joint>
 * </robot>
 * ```
 *
 * @section load_urdf_types URDF Joint Types
 *
 * | Type | Description | DOF | Use Case |
 * |------|-------------|-----|----------|
 * | `revolute` | Rotational, limited range | 1 | Robot joints with angle limits |
 * | `continuous` | Rotational, unlimited | 1 | Wheels, continuous rotations |
 * | `prismatic` | Linear motion | 1 | Telescoping arms, linear actuators |
 * | `planar` | 2D motion | 2 | Mobile bases |
 * | `floating` | 6 DOF unconstrained | 6 | Flying robots, free bodies |
 * | `fixed` | No motion | 0 | Rigid connections, end effectors |
 *
 * @section load_urdf_workflow Typical Workflow
 *
 * 1. **Locate URDF file** - Find the robot's URDF file in the package directory
 * 2. **Create resource locator** - Set up path resolution for package:// URIs
 * 3. **Parse URDF** - Call parseURDFFile() to convert XML to scene graph
 * 4. **Validate structure** - Check that the graph is a valid tree
 * 5. **Access robot data** - Query links, joints, and kinematic relationships
 * 6. **Use in downstream** - Pass to collision checkers, planners, kinematics solvers
 *
 * @section load_urdf_loading Locating and Loading URDF Files
 *
 * Use the resource locator to find URDF files using package:// paths (standard in ROS):
 *
 * @snippet load_urdf_example.cpp load_urdf_get_urdf_file
 *
 * Parse the URDF file into a scene graph using parseURDFFile():
 *
 * @snippet load_urdf_example.cpp load_urdf_parse_urdf
 *
 * @section load_urdf_accessing Accessing Loaded Robot Data
 *
 * Once parsed, query the scene graph to access robot structure information:
 *
 * @snippet load_urdf_example.cpp load_urdf_print_info
 *
 * The quantities you can access include:
 * - `getJoints()`: All joints in the robot
 * - `getLinks()`: All rigid bodies
 * - `isTree()`: Whether structure is a valid kinematic tree
 * - `isAcyclic()`: Whether structure contains no cycles
 * - `getAdjacentLinkNames()`: Links directly connected via a joint
 * - `getLinkChildrenNames()`: All descendant links
 * - `getShortestPath()`: Kinematic path between links
 *
 * @section load_urdf_visualization Visualizing the Structure
 *
 * Save the kinematic structure as a DOT graph file for visualization with Graphviz:
 *
 * @snippet load_urdf_example.cpp load_urdf_save_graph
 *
 * This creates a visual representation of how links and joints are connected, useful for
 * debugging and understanding complex robot structures.
 *
 * @section load_urdf_common_patterns Common Patterns
 *
 * **Loading a robot at startup**: Find URDF via package path, parse it, validate the structure,
 * then use the scene graph throughout your application.
 *
 * **Multi-robot scenarios**: Load multiple URDF files separately, creating independent scene
 * graphs, then combine them for collision checking.
 *
 * **Custom modifications**: After parsing, you can programmatically add/remove links and joints
 * (e.g., adding a gripper not defined in the base URDF).
 *
 * **Error handling**: URDF files can have syntax errors or broken package paths. Always wrap
 * parsing in try-catch to handle failures gracefully.
 *
 * @section load_urdf_tips Tips and Best Practices
 *
 * - **Use package:// URIs**: Always reference URDF files with `package://` paths for portability
 *   across different installations
 * - **Understand your robot**: Know the URDF format before debugging parsing issues
 * - **Validate structure**: Call `isTree()` and `isAcyclic()` after parsing to catch malformed URDFs
 * - **Check geometry paths**: URDF may reference mesh files with relative paths; ensure the
 *   resource locator can resolve them
 * - **Visualize the graph**: Use saveDOT() to visualize kinematic structure when debugging
 * - **URDF versions**: Ensure your URDF is compatible with your Tesseract version
 * - **Performance**: Large URDFs with complex meshes take longer to parse; consider preprocessing
 * - **Keep URDFs current**: Robot parameters (link masses, inertias) affect planning and simulation
 *
 * @section load_urdf_troubleshooting Common Issues
 *
 * **File not found**: Ensure `package://` paths are correctly registered with the resource locator.
 * Check that the package is installed and the path is correct.
 *
 * **Parsing errors**: URDF syntax errors or malformed XML will cause parsing to fail. Validate
 * your URDF with standard tools (e.g., `urdf_parser` from ROS).
 *
 * **Missing meshes**: If the URDF references mesh files (STL, DAE, OBJ) and the resource locator
 * can't find them, collision geometry may be missing. Verify mesh file paths.
 *
 * **Invalid kinematic structure**: If `isTree()` returns false, check for disconnected links
 * (links not reachable from the root) or multiple parent links.
 *
 * @section load_urdf_integration Integration with Tesseract
 *
 * - **Environment**: The scene graph from URDF is the foundation for Tesseract environments
 * - **Collision Checking**: Uses the parsed geometry and link relationships
 * - **Kinematics**: Scene graph enables forward/inverse kinematics computation
 * - **Motion Planning**: Knows which joints affect which links based on parsed structure
 * - **Visualization**: URDF meshes are rendered in 3D displays
 * - **SRDF Integration**: SRDF files extend URDF with semantic information (see parse_srdf_example)
 *
 * @section load_urdf_urdf_vs_srdf URDF vs SRDF
 *
 * | Aspect | URDF | SRDF |
 * |--------|------|------|
 * | Purpose | Describe robot structure | Add semantic information |
 * | File extension | .urdf | .srdf |
 * | Format | XML | XML |
 * | Defines | Links, joints, geometry, mass | Groups, collision pairs, IK solvers |
 * | Required | Yes | Optional but recommended |
 *
 * For complete robot setup, load both URDF (structure) and SRDF (semantics). See
 * @ref parse_srdf_example for SRDF parsing.
 *
 * @section load_urdf_summary Summary
 *
 * This example demonstrates the fundamental URDF loading workflow in Tesseract:
 *
 * 1. Locate URDF file using package paths
 * 2. Parse URDF into scene graph representation
 * 3. Validate kinematic structure
 * 4. Access and query robot data
 * 5. Visualize structure for debugging
 *
 * URDF loading is the starting point for all robot manipulation, planning, and simulation
 * tasks in Tesseract. A valid, well-formed URDF ensures reliable operation of downstream systems.
 *
 * @section load_urdf_full_code Full source code
 *
 * For reference, here is the complete source of this example:
 *
 * @snippet load_urdf_example.cpp load_urdf_full_source
 */

//! [load_urdf_full_source]
#include <tesseract/common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <console_bridge/console.h>
#include <tesseract/scene_graph/graph.h>
#include <tesseract/common/utils.h>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract/urdf/urdf_parser.h>
#include <tesseract/common/resource_locator.h>

using namespace tesseract::scene_graph;
using namespace tesseract::urdf;

std::string toString(const ShortestPath& path)
{
  std::stringstream ss;
  ss << path;
  return ss.str();
}

std::string toString(bool b) { return b ? "true" : "false"; }

int main(int /*argc*/, char** /*argv*/)
{
  //! [load_urdf_get_urdf_file]
  tesseract::common::GeneralResourceLocator locator;
  std::string urdf_file =
      locator.locateResource("package://tesseract/support/urdf/lbr_iiwa_14_r820.urdf")->getFilePath();
  //! [load_urdf_get_urdf_file]

  //! [load_urdf_parse_urdf]
  SceneGraph::Ptr g = parseURDFFile(urdf_file, locator);
  //! [load_urdf_parse_urdf]

  //! [load_urdf_print_info]
  CONSOLE_BRIDGE_logInform(std::to_string(g->getJoints().size()).c_str());
  CONSOLE_BRIDGE_logInform(std::to_string(g->getLinks().size()).c_str());
  CONSOLE_BRIDGE_logInform(toString(g->isTree()).c_str());
  CONSOLE_BRIDGE_logInform(toString(g->isAcyclic()).c_str());
  //! [load_urdf_print_info]

  //! [load_urdf_save_graph]
  g->saveDOT(tesseract::common::getTempPath() + "tesseract_urdf_import.dot");
  //! [load_urdf_save_graph]
}
//! [load_urdf_full_source]
