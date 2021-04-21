#include <tesseract_common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <gtest/gtest.h>
#include <tesseract_urdf/urdf_parser.h>
#include <tesseract_scene_graph/resource_locator.h>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract_environment/core/manipulator_manager.h>

using namespace tesseract_environment;
using namespace tesseract_scene_graph;
using namespace tesseract_srdf;

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

enum class ABBConfig
{
  ROBOT_ONLY,
  ROBOT_ON_RAIL,
  ROBOT_WITH_POSITIONER
};

tesseract_scene_graph::SceneGraph::Ptr getSceneGraph(ABBConfig config = ABBConfig::ROBOT_ONLY)
{
  tesseract_scene_graph::ResourceLocator::Ptr locator =
      std::make_shared<tesseract_scene_graph::SimpleResourceLocator>(locateResource);

  if (config == ABBConfig::ROBOT_ON_RAIL)
  {
    std::string path = std::string(TESSERACT_SUPPORT_DIR) + "/urdf/abb_irb2400_on_positioner.urdf";
    return tesseract_urdf::parseURDFFile(path, locator);
  }

  if (config == ABBConfig::ROBOT_WITH_POSITIONER)
  {
    std::string path = std::string(TESSERACT_SUPPORT_DIR) + "/urdf/abb_irb2400_external_positioner.urdf";
    return tesseract_urdf::parseURDFFile(path, locator);
  }

  if (config == ABBConfig::ROBOT_ONLY)
  {
    std::string path = std::string(TESSERACT_SUPPORT_DIR) + "/urdf/abb_irb2400.urdf";
    return tesseract_urdf::parseURDFFile(path, locator);
  }

  return nullptr;
}

SRDFModel::Ptr getSRDFModel(const SceneGraph::Ptr& scene_graph, ABBConfig config = ABBConfig::ROBOT_ONLY)
{
  if (config == ABBConfig::ROBOT_ON_RAIL)
  {
    auto srdf = std::make_shared<SRDFModel>();
    std::string path = std::string(TESSERACT_SUPPORT_DIR) + "/urdf/abb_irb2400_on_positioner.srdf";
    srdf->initFile(*scene_graph, path);
    return srdf;
  }

  if (config == ABBConfig::ROBOT_WITH_POSITIONER)
  {
    auto srdf = std::make_shared<SRDFModel>();
    std::string path = std::string(TESSERACT_SUPPORT_DIR) + "/urdf/abb_irb2400_external_positioner.srdf";
    srdf->initFile(*scene_graph, path);
    return srdf;
  }

  if (config == ABBConfig::ROBOT_ONLY)
  {
    auto srdf = std::make_shared<SRDFModel>();
    std::string path = std::string(TESSERACT_SUPPORT_DIR) + "/urdf/abb_irb2400.srdf";
    srdf->initFile(*scene_graph, path);
    return srdf;
  }

  return nullptr;
}

void runCheckAvailableSolvers(ManipulatorManager& manager)
{
  using namespace tesseract_kinematics;

  /////////////////////////
  ////// Forward Kin //////
  /////////////////////////

  {  // Check available fwd kinematics solvers
    std::vector<std::string> check_available_fwd_solvers{ "KDLFwdKinTree", "KDLFwdKinChain" };
    std::vector<std::string> available_fwd_solvers = manager.getAvailableFwdKinematicsSolvers();
    EXPECT_EQ(available_fwd_solvers, check_available_fwd_solvers);
  }

  {  // Check available fwd kinematics chain solvers
    std::vector<std::string> check_chain_fwd_solvers{ "KDLFwdKinChain" };
    std::vector<std::string> chain_fwd_solvers =
        manager.getAvailableFwdKinematicsSolvers(ForwardKinematicsFactoryType::CHAIN);
    EXPECT_EQ(chain_fwd_solvers, check_chain_fwd_solvers);
  }

  {  // Check available fwd kinematics tree solvers
    std::vector<std::string> check_tree_fwd_solvers{ "KDLFwdKinTree" };
    std::vector<std::string> tree_fwd_solvers =
        manager.getAvailableFwdKinematicsSolvers(ForwardKinematicsFactoryType::TREE);
    EXPECT_EQ(tree_fwd_solvers, check_tree_fwd_solvers);
  }

  {  // Check available fwd kinematics graph solvers
    std::vector<std::string> check_graph_fwd_solvers;
    std::vector<std::string> graph_fwd_solvers =
        manager.getAvailableFwdKinematicsSolvers(ForwardKinematicsFactoryType::GRAPH);
    EXPECT_EQ(graph_fwd_solvers, check_graph_fwd_solvers);
  }

  {  // Check available fwd kinematics chain factory
    ForwardKinematicsFactory::ConstPtr chain_fwd_factory = manager.getFwdKinematicFactory("KDLFwdKinChain");
    EXPECT_TRUE(chain_fwd_factory != nullptr);
  }

  {  // Check available fwd kinematics tree factory
    ForwardKinematicsFactory::ConstPtr chain_fwd_factory = manager.getFwdKinematicFactory("KDLFwdKinTree");
    EXPECT_TRUE(chain_fwd_factory != nullptr);
  }

  {  // Check available fwd kinematics graph factory (Currently not supported)
    ForwardKinematicsFactory::ConstPtr chain_fwd_factory = manager.getFwdKinematicFactory("KDLFwdKinGraph");
    EXPECT_TRUE(chain_fwd_factory == nullptr);
  }

  /////////////////////////
  ////// Inverse Kin //////
  /////////////////////////

  {  // Check available inverse kinematics solvers
    std::vector<std::string> check_available_inv_solvers{ "KDLInvKinChainLMA" };
    std::vector<std::string> available_inv_solvers = manager.getAvailableInvKinematicsSolvers();
    EXPECT_EQ(available_inv_solvers, check_available_inv_solvers);
  }

  {  // Check available inverse kinematics chain solvers
    std::vector<std::string> check_chain_inv_solvers{ "KDLInvKinChainLMA" };
    std::vector<std::string> chain_inv_solvers =
        manager.getAvailableInvKinematicsSolvers(InverseKinematicsFactoryType::CHAIN);
    EXPECT_EQ(chain_inv_solvers, check_chain_inv_solvers);
  }

  {  // Check available inverse kinematics tree solvers
    std::vector<std::string> check_tree_inv_solvers;
    std::vector<std::string> tree_inv_solvers =
        manager.getAvailableInvKinematicsSolvers(InverseKinematicsFactoryType::TREE);
    EXPECT_EQ(tree_inv_solvers, check_tree_inv_solvers);
  }

  {  // Check available inverse kinematics graph solvers
    std::vector<std::string> check_graph_inv_solvers;
    std::vector<std::string> graph_inv_solvers =
        manager.getAvailableInvKinematicsSolvers(InverseKinematicsFactoryType::GRAPH);
    EXPECT_EQ(graph_inv_solvers, check_graph_inv_solvers);
  }

  {  // Check available inverse kinematics chain factory
    InverseKinematicsFactory::ConstPtr chain_inv_factory = manager.getInvKinematicFactory("KDLInvKinChainLMA");
    EXPECT_TRUE(chain_inv_factory != nullptr);
  }

  {  // Check available inverse kinematics tree factory
    InverseKinematicsFactory::ConstPtr chain_inv_factory = manager.getInvKinematicFactory("KDLFwdKinTree");
    EXPECT_TRUE(chain_inv_factory == nullptr);
  }

  {  // Check available inverse kinematics graph factory (Currently not supported)
    InverseKinematicsFactory::ConstPtr chain_inv_factory = manager.getInvKinematicFactory("KDLFwdKinGraph");
    EXPECT_TRUE(chain_inv_factory == nullptr);
  }
}

TEST(TesseractEnvironmentManipulatorManagerUnit, RobotOnPositionerUnit)  // NOLINT
{
  using namespace tesseract_scene_graph;
  SceneGraph::Ptr g = getSceneGraph(ABBConfig::ROBOT_ON_RAIL);
  SRDFModel::Ptr srdf = getSRDFModel(g, ABBConfig::ROBOT_ON_RAIL);
  g->saveDOT(tesseract_common::getTempPath() + "abb_robot_on_positioner.dot");

  KinematicsInformation& kin_info = srdf->kinematics_information;

  ManipulatorManager manager;
  EXPECT_FALSE(manager.isInitialized());
  EXPECT_FALSE(manager.init(nullptr, kin_info));
  EXPECT_TRUE(manager.init(g, kin_info));
  runCheckAvailableSolvers(manager);

  // Check group names
  std::vector<std::string> group_names{ "manipulator", "positioner", "full_manipulator" };
  EXPECT_EQ(manager.getGroupNames().size(), group_names.size());
  EXPECT_EQ(manager.getGroupNames(), group_names);
  for (const auto& group_name : group_names)
  {
    EXPECT_TRUE(manager.hasGroup(group_name));
  }

  // Check for chain group information
  const ChainGroups& chain_groups = manager.getChainGroups();
  EXPECT_EQ(chain_groups.size(), 3);
  auto chain_gantry_it = chain_groups.find("full_manipulator");
  auto chain_manipulator_it = chain_groups.find("manipulator");
  auto chain_positioner_it = chain_groups.find("positioner");
  EXPECT_TRUE(chain_gantry_it != chain_groups.end());
  EXPECT_TRUE(chain_manipulator_it != chain_groups.end());
  EXPECT_TRUE(chain_positioner_it != chain_groups.end());
  /** @todo check for solver */

  // Check for rop group information
  const GroupROPKinematics& group_rop_kinematics = manager.getROPKinematicsSolvers();
  EXPECT_EQ(group_rop_kinematics.size(), 1);
  auto rop_solver_it = group_rop_kinematics.find("full_manipulator");
  EXPECT_TRUE(rop_solver_it != group_rop_kinematics.end());
  /** @todo check for solver */

  // Check for opw group information
  const GroupOPWKinematics& group_opw_kinematics = manager.getOPWKinematicsSolvers();
  EXPECT_EQ(group_opw_kinematics.size(), 1);
  auto opw_solver_it = group_opw_kinematics.find("manipulator");
  EXPECT_TRUE(opw_solver_it != group_opw_kinematics.end());
  /** @todo check for solver */

  // Check for group states information
  const GroupJointStates& group_states = manager.getGroupJointStates();
  EXPECT_EQ(group_states.size(), 1);
  auto group_state_it = group_states.find("manipulator");
  EXPECT_TRUE(group_state_it != group_states.end());
  EXPECT_EQ(group_state_it->second.size(), 1);
  EXPECT_TRUE(group_state_it->second.find("all-zeros") != group_state_it->second.end());

  // Check for tcp information
  const GroupTCPs& group_tcps = manager.getGroupTCPs();
  EXPECT_EQ(group_tcps.size(), 1);
  auto tcp_it = group_tcps.find("full_manipulator");
  EXPECT_TRUE(tcp_it != group_tcps.end());
  EXPECT_EQ(tcp_it->second.size(), 2);
  EXPECT_TRUE(tcp_it->second.find("laser") != tcp_it->second.end());
  EXPECT_TRUE(tcp_it->second.find("welder") != tcp_it->second.end());
}

TEST(TesseractEnvironmentManipulatorManagerUnit, RobotWithExternalPositionerUnit)  // NOLINT
{
  using namespace tesseract_scene_graph;
  SceneGraph::Ptr g = getSceneGraph(ABBConfig::ROBOT_WITH_POSITIONER);
  SRDFModel::Ptr srdf = getSRDFModel(g, ABBConfig::ROBOT_WITH_POSITIONER);
  g->saveDOT(tesseract_common::getTempPath() + "abb_robot_with_external_positioner.dot");

  KinematicsInformation& kin_info = srdf->kinematics_information;

  ManipulatorManager manager;
  EXPECT_FALSE(manager.isInitialized());
  EXPECT_FALSE(manager.init(nullptr, kin_info));
  EXPECT_TRUE(manager.init(g, kin_info));
  runCheckAvailableSolvers(manager);

  // Check group names
  std::vector<std::string> group_names{ "manipulator", "positioner", "full_manipulator" };
  EXPECT_EQ(manager.getGroupNames().size(), group_names.size());
  EXPECT_EQ(manager.getGroupNames(), group_names);
  for (const auto& group_name : group_names)
  {
    EXPECT_TRUE(manager.hasGroup(group_name));
  }

  // Check for chain group information
  const ChainGroups& chain_groups = manager.getChainGroups();
  EXPECT_EQ(chain_groups.size(), 3);
  auto chain_gantry_it = chain_groups.find("full_manipulator");
  auto chain_manipulator_it = chain_groups.find("manipulator");
  auto chain_positioner_it = chain_groups.find("positioner");
  EXPECT_TRUE(chain_gantry_it != chain_groups.end());
  EXPECT_TRUE(chain_manipulator_it != chain_groups.end());
  EXPECT_TRUE(chain_positioner_it != chain_groups.end());
  /** @todo check for solver */

  // Check for rop group information
  const GroupREPKinematics& group_rep_kinematics = manager.getREPKinematicsSolvers();
  EXPECT_EQ(group_rep_kinematics.size(), 1);
  auto rep_solver_it = group_rep_kinematics.find("full_manipulator");
  EXPECT_TRUE(rep_solver_it != group_rep_kinematics.end());
  /** @todo check for solver */

  // Check for opw group information
  const GroupOPWKinematics& group_opw_kinematics = manager.getOPWKinematicsSolvers();
  EXPECT_EQ(group_opw_kinematics.size(), 1);
  auto opw_solver_it = group_opw_kinematics.find("manipulator");
  EXPECT_TRUE(opw_solver_it != group_opw_kinematics.end());
  /** @todo check for solver */

  // Check for group states information
  const GroupJointStates& group_states = manager.getGroupJointStates();
  EXPECT_EQ(group_states.size(), 1);
  auto group_state_it = group_states.find("manipulator");
  EXPECT_TRUE(group_state_it != group_states.end());
  EXPECT_EQ(group_state_it->second.size(), 1);
  EXPECT_TRUE(group_state_it->second.find("all-zeros") != group_state_it->second.end());

  // Check for tcp information
  const GroupTCPs& group_tcps = manager.getGroupTCPs();
  EXPECT_EQ(group_tcps.size(), 1);
  auto tcp_it = group_tcps.find("full_manipulator");
  EXPECT_TRUE(tcp_it != group_tcps.end());
  EXPECT_EQ(tcp_it->second.size(), 2);
  EXPECT_TRUE(tcp_it->second.find("laser") != tcp_it->second.end());
  EXPECT_TRUE(tcp_it->second.find("welder") != tcp_it->second.end());
}

TEST(TesseractEnvironmentManipulatorManagerUnit, AddRemoveChainGroupUnit)  // NOLINT
{
  using namespace tesseract_scene_graph;
  SceneGraph::Ptr g = getSceneGraph(ABBConfig::ROBOT_WITH_POSITIONER);

  ManipulatorManager manager;
  EXPECT_FALSE(manager.isInitialized());
  EXPECT_TRUE(manager.init(g, KinematicsInformation()));
  EXPECT_TRUE(manager.isInitialized());
  runCheckAvailableSolvers(manager);

  // ADD
  ChainGroup chain_group;
  chain_group.push_back(std::make_pair("base_link", "tool0"));
  EXPECT_TRUE(manager.addChainGroup("manipulator", chain_group));
  EXPECT_TRUE(manager.getChainGroup("manipulator") == chain_group);
  EXPECT_TRUE(manager.hasChainGroup("manipulator"));
  EXPECT_EQ(manager.getChainGroups().size(), 1);
  EXPECT_EQ(manager.getGroupNames().size(), 1);
  EXPECT_TRUE(manager.hasGroup("manipulator"));
  /** @todo check for solver */

  // Remove
  manager.removeChainGroup("manipulator");
  EXPECT_FALSE(manager.hasChainGroup("manipulator"));
  EXPECT_EQ(manager.getChainGroups().size(), 0);
  EXPECT_EQ(manager.getGroupNames().size(), 0);
  EXPECT_FALSE(manager.hasGroup("manipulator"));
  /** @todo check that solver was removed */
}

TEST(TesseractEnvironmentManipulatorManagerUnit, AddRemoveJointGroupUnit)  // NOLINT
{
  using namespace tesseract_scene_graph;
  SceneGraph::Ptr g = getSceneGraph(ABBConfig::ROBOT_WITH_POSITIONER);

  ManipulatorManager manager;
  EXPECT_FALSE(manager.isInitialized());
  EXPECT_TRUE(manager.init(g, KinematicsInformation()));
  EXPECT_TRUE(manager.isInitialized());
  runCheckAvailableSolvers(manager);

  // ADD
  JointGroup joint_group = { "joint_1", "joint_2", "joint_3", "joint_4", "joint_5", "joint_6" };
  EXPECT_TRUE(manager.addJointGroup("manipulator", joint_group));
  EXPECT_TRUE(manager.getJointGroup("manipulator") == joint_group);
  EXPECT_TRUE(manager.hasJointGroup("manipulator"));
  EXPECT_EQ(manager.getJointGroups().size(), 1);
  EXPECT_EQ(manager.getGroupNames().size(), 1);
  EXPECT_TRUE(manager.hasGroup("manipulator"));
  /** @todo check for solver */

  // Remove
  manager.removeJointGroup("manipulator");
  EXPECT_FALSE(manager.hasJointGroup("manipulator"));
  EXPECT_EQ(manager.getJointGroups().size(), 0);
  EXPECT_EQ(manager.getGroupNames().size(), 0);
  EXPECT_FALSE(manager.hasGroup("manipulator"));
  /** @todo check that solver was removed */
}

TEST(TesseractEnvironmentManipulatorManagerUnit, AddRemoveLinkGroupUnit)  // NOLINT
{
  using namespace tesseract_scene_graph;
  SceneGraph::Ptr g = getSceneGraph(ABBConfig::ROBOT_WITH_POSITIONER);

  ManipulatorManager manager;
  EXPECT_FALSE(manager.isInitialized());
  EXPECT_TRUE(manager.init(g, KinematicsInformation()));
  EXPECT_TRUE(manager.isInitialized());
  runCheckAvailableSolvers(manager);

  // ADD this should fail because link groups are currently not supported.
  LinkGroup link_group = { "link_1", "link_2", "link_3", "link_4", "link_5", "link_6" };
  EXPECT_FALSE(manager.addLinkGroup("manipulator", link_group));
  EXPECT_FALSE(manager.hasLinkGroup("manipulator"));
  EXPECT_EQ(manager.getLinkGroups().size(), 0);
  EXPECT_EQ(manager.getGroupNames().size(), 0);
  EXPECT_FALSE(manager.hasGroup("manipulator"));
  /** @todo check for solver */

  // Remove
  manager.removeLinkGroup("manipulator");
  EXPECT_FALSE(manager.hasLinkGroup("manipulator"));
  EXPECT_EQ(manager.getLinkGroups().size(), 0);
  EXPECT_EQ(manager.getGroupNames().size(), 0);
  EXPECT_FALSE(manager.hasGroup("manipulator"));
  /** @todo check that solver was removed */
}

TEST(TesseractEnvironmentManipulatorManagerUnit, AddRemoveROPKinematicsSolverUnit)  // NOLINT
{
  using namespace tesseract_scene_graph;
  SceneGraph::Ptr g = getSceneGraph(ABBConfig::ROBOT_ON_RAIL);

  ManipulatorManager manager;
  EXPECT_FALSE(manager.isInitialized());
  EXPECT_TRUE(manager.init(g, KinematicsInformation()));
  EXPECT_TRUE(manager.isInitialized());
  runCheckAvailableSolvers(manager);

  ROPKinematicParameters rop_group;
  rop_group.manipulator_group = "manipulator";
  rop_group.manipulator_ik_solver = "KDLInvKinChainLMA";
  rop_group.manipulator_reach = 2.3;
  rop_group.positioner_group = "positioner";
  rop_group.positioner_sample_resolution["positioner_joint_1"] = 0.1;

  // ADD full manipulator, manipulator and positioner does not exist
  EXPECT_FALSE(manager.addROPKinematicsSolver("full_manipulator", rop_group));
  EXPECT_FALSE(manager.hasROPKinematicsSolver("full_manipulator"));
  EXPECT_EQ(manager.getROPKinematicsSolvers().size(), 0);
  EXPECT_EQ(manager.getGroupNames().size(), 0);

  // ADD full manipulator but manipulator and positioner does not exist
  ChainGroup full_group;
  full_group.push_back(std::make_pair("positioner_base_link", "tool0"));
  EXPECT_TRUE(manager.addChainGroup("full_manipulator", full_group));
  EXPECT_TRUE(manager.hasGroup("full_manipulator"));

  EXPECT_FALSE(manager.addROPKinematicsSolver("full_manipulator", rop_group));
  EXPECT_FALSE(manager.hasROPKinematicsSolver("full_manipulator"));
  EXPECT_EQ(manager.getROPKinematicsSolvers().size(), 0);
  EXPECT_EQ(manager.getGroupNames().size(), 1);

  // ADD manipulator exist and positioner do not exist
  ChainGroup chain_group;
  chain_group.push_back(std::make_pair("base_link", "tool0"));
  EXPECT_TRUE(manager.addChainGroup("manipulator", chain_group));

  EXPECT_FALSE(manager.addROPKinematicsSolver("full_manipulator", rop_group));
  EXPECT_FALSE(manager.hasROPKinematicsSolver("full_manipulator"));
  EXPECT_EQ(manager.getROPKinematicsSolvers().size(), 0);
  EXPECT_EQ(manager.getGroupNames().size(), 2);

  // ADD manipulator and positioner exists but full manipulator does not exist
  ChainGroup positioner_group;
  positioner_group.push_back(std::make_pair("positioner_base_link", "positioner_tool0"));
  EXPECT_TRUE(manager.addChainGroup("positioner", positioner_group));

  EXPECT_TRUE(manager.addROPKinematicsSolver("full_manipulator", rop_group));
  EXPECT_TRUE(manager.getROPKinematicsSolver("full_manipulator") == rop_group);
  EXPECT_TRUE(manager.hasROPKinematicsSolver("full_manipulator"));
  EXPECT_EQ(manager.getROPKinematicsSolvers().size(), 1);
  EXPECT_EQ(manager.getGroupNames().size(), 3);
  /** @todo check for solver */

  // Remove
  manager.removeROPKinematicsSolver("full_manipulator");
  EXPECT_FALSE(manager.hasROPKinematicsSolver("full_manipulator"));
  EXPECT_EQ(manager.getROPKinematicsSolvers().size(), 0);
  /** @todo check that solver was removed */
}

TEST(TesseractEnvironmentManipulatorManagerUnit, AddRemoveREPKinematicsSolverUnit)  // NOLINT
{
  using namespace tesseract_scene_graph;
  SceneGraph::Ptr g = getSceneGraph(ABBConfig::ROBOT_WITH_POSITIONER);

  ManipulatorManager manager;
  EXPECT_FALSE(manager.isInitialized());
  EXPECT_TRUE(manager.init(g, KinematicsInformation()));
  EXPECT_TRUE(manager.isInitialized());
  runCheckAvailableSolvers(manager);

  REPKinematicParameters rep_group;
  rep_group.manipulator_group = "manipulator";
  rep_group.manipulator_ik_solver = "KDLInvKinChainLMA";
  rep_group.manipulator_reach = 2.3;
  rep_group.positioner_group = "positioner";
  rep_group.positioner_sample_resolution["positioner_joint_1"] = 0.1;

  // ADD full manipulator, manipulator and positioner does not exist
  EXPECT_FALSE(manager.addREPKinematicsSolver("full_manipulator", rep_group));
  EXPECT_FALSE(manager.hasREPKinematicsSolver("full_manipulator"));
  EXPECT_EQ(manager.getREPKinematicsSolvers().size(), 0);
  EXPECT_EQ(manager.getGroupNames().size(), 0);

  // ADD full manipulator but manipulator and positioner does not exist
  ChainGroup full_group;
  full_group.push_back(std::make_pair("world", "tool0"));
  EXPECT_TRUE(manager.addChainGroup("full_manipulator", full_group));
  EXPECT_TRUE(manager.hasGroup("full_manipulator"));

  EXPECT_FALSE(manager.addREPKinematicsSolver("full_manipulator", rep_group));
  EXPECT_FALSE(manager.hasREPKinematicsSolver("full_manipulator"));
  EXPECT_EQ(manager.getREPKinematicsSolvers().size(), 0);
  EXPECT_EQ(manager.getGroupNames().size(), 1);

  // ADD manipulator exist and positioner do not exist
  ChainGroup chain_group;
  chain_group.push_back(std::make_pair("world", "tool0"));
  EXPECT_TRUE(manager.addChainGroup("manipulator", chain_group));

  EXPECT_FALSE(manager.addREPKinematicsSolver("full_manipulator", rep_group));
  EXPECT_FALSE(manager.hasREPKinematicsSolver("full_manipulator"));
  EXPECT_EQ(manager.getREPKinematicsSolvers().size(), 0);
  EXPECT_EQ(manager.getGroupNames().size(), 2);

  // ADD manipulator and positioner exists but full manipulator does not exist
  ChainGroup positioner_group;
  positioner_group.push_back(std::make_pair("world", "positioner_tool0"));
  EXPECT_TRUE(manager.addChainGroup("positioner", positioner_group));

  EXPECT_TRUE(manager.addREPKinematicsSolver("full_manipulator", rep_group));
  EXPECT_TRUE(manager.getREPKinematicsSolver("full_manipulator") == rep_group);
  EXPECT_TRUE(manager.hasREPKinematicsSolver("full_manipulator"));
  EXPECT_EQ(manager.getREPKinematicsSolvers().size(), 1);
  EXPECT_EQ(manager.getGroupNames().size(), 3);
  /** @todo check for solver */

  // Remove
  manager.removeREPKinematicsSolver("full_manipulator");
  EXPECT_FALSE(manager.hasREPKinematicsSolver("full_manipulator"));
  EXPECT_EQ(manager.getREPKinematicsSolvers().size(), 0);
  /** @todo check that solver was removed */
}

TEST(TesseractEnvironmentManipulatorManagerUnit, AddRemoveOPWKinematicsSolverUnit)  // NOLINT
{
  using namespace tesseract_scene_graph;
  SceneGraph::Ptr g = getSceneGraph(ABBConfig::ROBOT_ONLY);

  ManipulatorManager manager;
  EXPECT_FALSE(manager.isInitialized());
  EXPECT_TRUE(manager.init(g, KinematicsInformation()));
  EXPECT_TRUE(manager.isInitialized());
  runCheckAvailableSolvers(manager);

  OPWKinematicParameters opw_group;
  opw_group.a1 = 0.1;
  opw_group.a2 = -0.135;
  opw_group.b = 0;
  opw_group.c1 = 0.615;
  opw_group.c2 = 0.705;
  opw_group.c3 = 0.755;
  opw_group.c4 = 0.085;
  opw_group.offsets = { 0.0, 0.0, -1.570796, 0.0, 0.0, 0.0 };
  opw_group.sign_corrections = { 1, 1, 1, 1, 1, 1 };

  // ADD manipulator does not exist
  EXPECT_FALSE(manager.addOPWKinematicsSolver("manipulator", opw_group));
  EXPECT_FALSE(manager.hasOPWKinematicsSolver("manipulator"));
  EXPECT_EQ(manager.getOPWKinematicsSolvers().size(), 0);
  EXPECT_EQ(manager.getGroupNames().size(), 0);

  // ADD manipulator exist
  ChainGroup chain_group;
  chain_group.push_back(std::make_pair("base_link", "tool0"));
  EXPECT_TRUE(manager.addChainGroup("manipulator", chain_group));

  EXPECT_TRUE(manager.addOPWKinematicsSolver("manipulator", opw_group));
  EXPECT_TRUE(manager.getOPWKinematicsSolver("manipulator") == opw_group);
  EXPECT_TRUE(manager.hasOPWKinematicsSolver("manipulator"));
  EXPECT_EQ(manager.getOPWKinematicsSolvers().size(), 1);
  EXPECT_EQ(manager.getGroupNames().size(), 1);
  EXPECT_TRUE(manager.hasGroup("manipulator"));
  /** @todo check for solver */

  // Remove
  manager.removeOPWKinematicsSovler("manipulator");
  EXPECT_FALSE(manager.hasOPWKinematicsSolver("manipulator"));
  EXPECT_EQ(manager.getOPWKinematicsSolvers().size(), 0);
  /** @todo check that solver was removed */
}

TEST(TesseractEnvironmentManipulatorManagerUnit, AddRemoveGroupJointStateUnit)  // NOLINT
{
  using namespace tesseract_scene_graph;
  SceneGraph::Ptr g = getSceneGraph(ABBConfig::ROBOT_ONLY);

  ManipulatorManager manager;
  EXPECT_FALSE(manager.isInitialized());
  EXPECT_TRUE(manager.init(g, KinematicsInformation()));
  EXPECT_TRUE(manager.isInitialized());
  runCheckAvailableSolvers(manager);

  // ADD
  GroupsJointState group_states;
  group_states["joint_1"] = 0;
  group_states["joint_2"] = 0;
  group_states["joint_3"] = 0;
  group_states["joint_4"] = 0;
  group_states["joint_5"] = 0;
  group_states["joint_6"] = 0;

  EXPECT_FALSE(manager.addGroupJointState("manipulator", "all-zeros", group_states));
  EXPECT_FALSE(manager.hasGroupJointState("manipulator", "all-zeros"));
  EXPECT_EQ(manager.getGroupJointStates().size(), 0);

  // ADD manipulator
  ChainGroup chain_group;
  chain_group.push_back(std::make_pair("base_link", "tool0"));
  EXPECT_TRUE(manager.addChainGroup("manipulator", chain_group));

  EXPECT_TRUE(manager.addGroupJointState("manipulator", "all-zeros", group_states));
  EXPECT_TRUE(manager.getGroupsJointState("manipulator", "all-zeros") == group_states);
  EXPECT_TRUE(manager.hasGroupJointState("manipulator", "all-zeros"));
  EXPECT_EQ(manager.getGroupsJointStates("manipulator").size(), 1);
  EXPECT_EQ(manager.getGroupJointStates().size(), 1);

  // Remove
  manager.removeGroupJointState("manipulator", "all-zeros");
  EXPECT_FALSE(manager.hasGroupJointState("manipulator", "all-zeros"));
  EXPECT_EQ(manager.getGroupJointStates().size(), 0);
}

TEST(TesseractEnvironmentManipulatorManagerUnit, AddRemoveGroupTCPUnit)  // NOLINT
{
  using namespace tesseract_scene_graph;
  SceneGraph::Ptr g = getSceneGraph(ABBConfig::ROBOT_ONLY);

  ManipulatorManager manager;
  EXPECT_FALSE(manager.isInitialized());
  EXPECT_TRUE(manager.init(g, KinematicsInformation()));
  EXPECT_TRUE(manager.isInitialized());
  runCheckAvailableSolvers(manager);

  // ADD failure
  GroupsTCPs group_tcps;
  Eigen::Isometry3d tcp_laser = Eigen::Isometry3d::Identity();
  tcp_laser.translation() = Eigen::Vector3d(1, 0.1, 1);

  Eigen::Isometry3d tcp_welder = Eigen::Isometry3d::Identity();
  tcp_welder.translation() = Eigen::Vector3d(0.1, 1, 0.2);

  EXPECT_FALSE(manager.addGroupTCP("manipulator", "laser", tcp_laser));
  EXPECT_FALSE(manager.addGroupTCP("manipulator", "welder", tcp_welder));
  EXPECT_FALSE(manager.hasGroupTCP("manipulator", "laser"));
  EXPECT_FALSE(manager.hasGroupTCP("manipulator", "welder"));
  EXPECT_EQ(manager.getGroupTCPs().size(), 0);

  // ADD manipulator exist
  ChainGroup chain_group;
  chain_group.push_back(std::make_pair("base_link", "tool0"));
  EXPECT_TRUE(manager.addChainGroup("manipulator", chain_group));

  EXPECT_TRUE(manager.addGroupTCP("manipulator", "laser", tcp_laser));
  EXPECT_TRUE(manager.addGroupTCP("manipulator", "welder", tcp_welder));
  EXPECT_TRUE(manager.getGroupsTCP("manipulator", "laser").isApprox(tcp_laser, 1e-6));
  EXPECT_TRUE(manager.getGroupsTCP("manipulator", "welder").isApprox(tcp_welder, 1e-6));
  EXPECT_TRUE(manager.hasGroupTCP("manipulator", "laser"));
  EXPECT_TRUE(manager.hasGroupTCP("manipulator", "welder"));
  EXPECT_EQ(manager.getGroupsTCPs("manipulator").size(), 2);
  EXPECT_EQ(manager.getGroupTCPs().size(), 1);

  // Remove
  manager.removeGroupTCP("manipulator", "laser");
  manager.removeGroupTCP("manipulator", "welder");
  EXPECT_FALSE(manager.hasGroupTCP("manipulator", "laser"));
  EXPECT_FALSE(manager.hasGroupTCP("manipulator", "welder"));
  EXPECT_EQ(manager.getGroupTCPs().size(), 0);
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);

  return RUN_ALL_TESTS();
}
