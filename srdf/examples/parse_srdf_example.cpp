/**
 * @page parse_srdf_example Tesseract SRDF Parsing Example
 *
 * @section parse_srdf_overview Overview
 *
 * This example demonstrates how to parse and utilize SRDF (Semantic Robot Description Format)
 * files in Tesseract. While URDF (Unified Robot Description Format) defines the kinematic
 * and dynamic structure of a robot, SRDF adds semantic information such as collision pairs
 * that should be ignored, end-effector definitions, and planning groups. Understanding SRDF
 * parsing is essential for setting up realistic collision checking and motion planning.
 *
 * The example loads an SRDF file corresponding to a KUKA LBR iiwa 14 R820 robot, parses it,
 * and uses it to configure the allowed collision matrix for efficient collision checking.
 *
 * @section parse_srdf_concepts Key Concepts
 *
 * - **URDF (Unified Robot Description Format)**: XML format defining the kinematic tree,
 *   geometry, mass, and inertia of a robot. Describes *what* the robot looks like physically.
 *
 * - **SRDF (Semantic Robot Description Format)**: XML format extending URDF with semantic
 *   information about robot structure. Defines *how* to use the robot (which collisions to
 *   ignore, which links form groups, etc.).
 *
 * - **Allowed Collision Matrix (ACM)**: A data structure that specifies which pairs of links
 *   are allowed to be in contact without triggering collision warnings. Includes adjacent links
 *   (connected by a joint) and deliberately colliding pairs.
 *
 * - **Adjacent Links**: Links directly connected by a joint. Self-collision checking can skip
 *   these pairs since they naturally touch.
 *
 * - **Collision Groups**: Named subsets of links used for collision checking (e.g., "arm_collision",
 *   "gripper_collision").
 *
 * - **Disabled Collisions**: Explicitly marked pairs that can collide without issue (e.g., fingers
 *   intentionally closing on an object).
 *
 * @section parse_srdf_urdf_srdf_relationship URDF vs SRDF
 *
 * | Aspect | URDF | SRDF |
 * |--------|------|------|
 * | Defines | Kinematic structure | Semantic/intent |
 * | Format | XML | XML |
 * | Contains | Links, joints, geometry | Groups, disabled collisions, IK solvers |
 * | Requirement | Always needed | Optional but recommended |
 * | File extension | .urdf | .srdf |
 *
 * @section parse_srdf_workflow Typical Workflow
 *
 * 1. **Define robot structure** - Create/load URDF with kinematic tree
 * 2. **Create scene graph** - Build from URDF or programmatically (as in this example)
 * 3. **Load SRDF file** - Parse semantic information
 * 4. **Configure allowed collisions** - Use SRDF data to set up ACM
 * 5. **Apply manually** - Add any additional collision rules
 * 6. **Use for collision checking** - All collision queries now use configured ACM
 *
 * @section parse_srdf_building_graph Building the Robot Structure
 *
 * First, create a scene graph representing the robot. In practice, you'd load this from URDF,
 * but this example builds it manually to keep the example self-contained:
 *
 * @snippet parse_srdf_example.cpp parse_srdf_create_graph
 *
 * @section parse_srdf_loading_file Loading the SRDF File
 *
 * Locate and load the robot's SRDF file using the resource locator:
 *
 * @snippet parse_srdf_example.cpp parse_srdf_get_srdf_file
 *
 * Parse the SRDF file with proper error handling:
 *
 * @snippet parse_srdf_example.cpp parse_srdf_parse_srdf
 *
 * @section parse_srdf_allowed_collisions Configuring Allowed Collisions
 *
 * You can manually add allowed collision pairs before or after parsing SRDF.
 * For example, to allow collisions between adjacent links already connected by a joint:
 *
 * @snippet parse_srdf_example.cpp parse_srdf_add_allowed_collisions
 *
 * @section parse_srdf_acm Inspecting the Allowed Collision Matrix
 *
 * After configuring collisions, you can inspect and query the ACM:
 *
 * @snippet parse_srdf_example.cpp parse_srdf_get_acm_info
 *
 * @section parse_srdf_srdf_contents What's in an SRDF File?
 *
 * A typical SRDF contains:
 *
 * ```xml
 * <robot name="robot_name">
 *   <!-- Group definitions -->
 *   <group name="arm">
 *     <joint name="joint_1"/>
 *     <joint name="joint_2"/>
 *   </group>
 *
 *   <!-- Link pairs that can collide without issue -->
 *   <disable_collisions link1="link_1" link2="link_2" reason="adjacent"/>
 *   <disable_collisions link1="finger_left" link2="finger_right" reason="user_defined"/>
 *
 *   <!-- End effector definitions -->
 *   <end_effector name="tool0" parent_link="link_7" group="tool"/>
 * </robot>
 * ```
 *
 * @section parse_srdf_common_patterns Common Patterns
 *
 * **Setting up a robot for planning**: Load URDF → build scene graph → parse SRDF →
 * configure ACM → pass to motion planner
 *
 * **Custom collision rules**: After parsing, call `g.addAllowedCollision()` to manually
 * disable collisions for pairs not in SRDF
 *
 * **Collision checking with ACM**: When performing collision checks, the ACM automatically
 * filters skipped pairs, making checks faster
 *
 * **Multi-robot scenarios**: Parse SRDF for each robot, then add custom rules for inter-robot
 * collisions
 *
 * @section parse_srdf_tips Tips and Best Practices
 *
 * - **Always parse SRDF after building the scene graph**: The scene graph must exist before SRDF
 *   references its links and joints
 * - **Check for parse errors**: SRDF parsing can fail if links/joints don't exist. Always wrap
 *   in try-catch and log errors
 * - **Adjacent collisions matter**: The SRDF typically disables collisions between adjacent links
 *   to avoid false positives from numerical precision
 * - **Test your ACM**: After configuration, query the ACM to verify expected pairs are allowed
 * - **Performance**: Larger ACM matrices are slower to query, so be selective about what you disable
 * - **Version compatibility**: Ensure URDF and SRDF versions match and are compatible with your
 *   Tesseract version
 * - **Resource locator**: Use proper package paths (e.g., `package://robot_description/...`) so
 *   files work across different installation directories
 *
 * @section parse_srdf_acm_queries Using ACM for Collision Checking
 *
 * Once configured, the ACM is used automatically during collision queries:
 *
 * ```cpp
 * ContactResultMap result;
 * ContactRequest request(ContactTestType::CLOSEST);
 * // The ACM is consulted automatically to skip allowed pairs
 * checker.contactTest(result, request);
 * ```
 *
 * @section parse_srdf_integration Integration with Tesseract
 *
 * - **Environment**: Holds both the scene graph and ACM. SRDF parsing updates the Environment's ACM
 * - **Collision Checking**: Uses ACM to filter which pairs to check
 * - **Motion Planning**: Uses ACM during trajectory validation
 * - **Kinematics**: Scene graph from SRDF enables forward/inverse kinematics
 * - **Visualization**: SRDF group definitions determine what to display
 *
 * @section parse_srdf_summary Summary
 *
 * This example demonstrates the complete SRDF parsing workflow:
 *
 * 1. Build or load a scene graph (kinematic structure)
 * 2. Load and parse an SRDF file with semantic information
 * 3. Configure the allowed collision matrix from SRDF data
 * 4. Optionally add custom collision rules
 * 5. Use the ACM for efficient collision checking
 *
 * SRDF parsing bridges the gap between robot description formats and Tesseract's
 * collision/planning systems, enabling realistic and efficient robot simulation.
 *
 * @section parse_srdf_full_code Full source code
 *
 * For reference, here is the complete source of this example:
 *
 * @snippet parse_srdf_example.cpp parse_srdf_full_source
 */

//! [parse_srdf_full_source]
#include <console_bridge/console.h>
#include <tesseract/scene_graph/graph.h>
#include <tesseract/scene_graph/link.h>
#include <tesseract/scene_graph/joint.h>
#include <tesseract/common/allowed_collision_matrix.h>
#include <tesseract/common/resource_locator.h>
#include <tesseract/common/utils.h>
#include <tesseract/srdf/srdf_model.h>
#include <tesseract/srdf/utils.h>

using namespace tesseract::scene_graph;
using namespace tesseract::srdf;

std::string toString(const ShortestPath& path)
{
  std::stringstream ss;
  ss << path;
  return ss.str();
}

std::string toString(bool b) { return b ? "true" : "false"; }

int main(int /*argc*/, char** /*argv*/)
{
  //! [parse_srdf_create_graph]
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
  //! [parse_srdf_create_graph]

  //! [parse_srdf_get_srdf_file]
  tesseract::common::GeneralResourceLocator locator;
  std::string srdf_file =
      locator.locateResource("package://tesseract/support/urdf/lbr_iiwa_14_r820.srdf")->getFilePath();
  //! [parse_srdf_get_srdf_file]

  //! [parse_srdf_parse_srdf]
  SRDFModel srdf;
  try
  {
    srdf.initFile(g, srdf_file, locator);
  }
  catch (const std::exception& e)
  {
    CONSOLE_BRIDGE_logError("Failed to parse SRDF.");
    tesseract::common::printNestedException(e);
    return 1;
  }
  //! [parse_srdf_parse_srdf]

  //! [parse_srdf_add_allowed_collisions]
  g.addAllowedCollision("link_1", "link_2", "adjacent");

  processSRDFAllowedCollisions(g, srdf);
  //! [parse_srdf_add_allowed_collisions]

  //! [parse_srdf_get_acm_info]
  tesseract::common::AllowedCollisionMatrix::ConstPtr acm = g.getAllowedCollisionMatrix();
  const tesseract::common::AllowedCollisionEntries& acm_entries = acm->getAllAllowedCollisions();
  CONSOLE_BRIDGE_logInform("ACM Number of entries: %d", acm_entries.size());
  //! [parse_srdf_get_acm_info]
}
//! [parse_srdf_full_source]
