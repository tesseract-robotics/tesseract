#ifndef TESSERACT_STATE_SOLVER_STATE_SOLVER_TEST_SUITE_H
#define TESSERACT_STATE_SOLVER_STATE_SOLVER_TEST_SUITE_H

#include <tesseract_common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <gtest/gtest.h>
#include <algorithm>
#include <vector>
#include <tesseract_urdf/urdf_parser.h>
#include <tesseract_geometry/impl/box.h>
#include <tesseract_common/resource_locator.h>
#include <tesseract_common/utils.h>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract_state_solver/kdl/kdl_state_solver.h>

namespace tesseract_scene_graph::test_suite
{
inline std::string locateResource(const std::string& url)
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

inline SceneGraph::UPtr getSceneGraph()
{
  std::string path = std::string(TESSERACT_SUPPORT_DIR) + "/urdf/lbr_iiwa_14_r820.urdf";

  tesseract_common::SimpleResourceLocator locator(locateResource);
  return tesseract_urdf::parseURDFFile(path, locator);
}

inline SceneGraph::UPtr getSubSceneGraph()
{
  auto subgraph = std::make_unique<SceneGraph>();
  subgraph->setName("subgraph");

  // Now add a link to empty environment
  auto visual = std::make_shared<Visual>();
  visual->geometry = std::make_shared<tesseract_geometry::Box>(1, 1, 1);
  auto collision = std::make_shared<Collision>();
  collision->geometry = std::make_shared<tesseract_geometry::Box>(1, 1, 1);

  const std::string link_name1 = "subgraph_base_link";
  const std::string link_name2 = "subgraph_link_1";
  const std::string joint_name1 = "subgraph_joint1";
  Link link_1(link_name1);
  link_1.visual.push_back(visual);
  link_1.collision.push_back(collision);
  Link link_2(link_name2);

  Joint joint_1(joint_name1);
  joint_1.parent_to_joint_origin_transform.translation()(0) = 1.25;
  joint_1.parent_link_name = link_name1;
  joint_1.child_link_name = link_name2;
  joint_1.type = JointType::FIXED;

  subgraph->addLink(link_1);
  subgraph->addLink(link_2);
  subgraph->addJoint(joint_1);

  return subgraph;
}

inline void runCompareSceneStates(const SceneState& base_state, const SceneState& compare_state)
{
  EXPECT_EQ(base_state.joints.size(), compare_state.joints.size());
  EXPECT_EQ(base_state.joint_transforms.size(), compare_state.joint_transforms.size());
  EXPECT_EQ(base_state.link_transforms.size(), compare_state.link_transforms.size());

  for (const auto& pair : base_state.joints)
  {
    EXPECT_NEAR(pair.second, compare_state.joints.at(pair.first), 1e-6);
  }

  for (const auto& pair : base_state.joint_transforms)
  {
    EXPECT_TRUE(pair.second.isApprox(compare_state.joint_transforms.at(pair.first), 1e-6));
  }

  for (const auto& link_pair : base_state.link_transforms)
  {
    EXPECT_TRUE(link_pair.second.isApprox(compare_state.link_transforms.at(link_pair.first), 1e-6));
  }
}

inline void runCompareStateSolver(const StateSolver& base_solver, StateSolver& comp_solver)
{
  EXPECT_EQ(base_solver.getBaseLinkName(), comp_solver.getBaseLinkName());
  EXPECT_TRUE(tesseract_common::isIdentical(base_solver.getJointNames(), comp_solver.getJointNames(), false));
  EXPECT_TRUE(
      tesseract_common::isIdentical(base_solver.getActiveJointNames(), comp_solver.getActiveJointNames(), false));
  EXPECT_TRUE(tesseract_common::isIdentical(base_solver.getLinkNames(), comp_solver.getLinkNames(), false));
  EXPECT_TRUE(tesseract_common::isIdentical(base_solver.getActiveLinkNames(), comp_solver.getActiveLinkNames(), false));
  EXPECT_TRUE(tesseract_common::isIdentical(base_solver.getStaticLinkNames(), comp_solver.getStaticLinkNames(), false));

  for (int i = 0; i < 10; ++i)
  {
    SceneState base_random_state = base_solver.getRandomState();
    SceneState comp_state_const = comp_solver.getState(base_random_state.joints);
    comp_solver.setState(base_random_state.joints);
    const SceneState& comp_state = comp_solver.getState();

    runCompareSceneStates(base_random_state, comp_state_const);
    runCompareSceneStates(base_random_state, comp_state);
  }
}

inline void runCompareStateSolverLimits(const SceneGraph& scene_graph, const StateSolver& comp_solver)
{
  std::vector<std::string> comp_joint_names = comp_solver.getActiveJointNames();
  tesseract_common::KinematicLimits limits = comp_solver.getLimits();

  for (Eigen::Index i = 0; i < static_cast<Eigen::Index>(comp_joint_names.size()); ++i)
  {
    const auto& scene_joint = scene_graph.getJoint(comp_joint_names[static_cast<std::size_t>(i)]);
    EXPECT_NEAR(limits.joint_limits(i, 0), scene_joint->limits->lower, 1e-5);
    EXPECT_NEAR(limits.joint_limits(i, 1), scene_joint->limits->upper, 1e-5);
    EXPECT_NEAR(limits.velocity_limits(i), scene_joint->limits->velocity, 1e-5);
    EXPECT_NEAR(limits.acceleration_limits(i), scene_joint->limits->acceleration, 1e-5);
  }
}

/**
 * @brief Numerically calculate a jacobian. This is mainly used for testing
 * @param jacobian (Return) The jacobian which gets filled out.
 * @param state_solver          The state solver object
 * @param joint_values The joint values for which to calculate the jacobian
 * @param link_name    The link_name for which the jacobian should be calculated
 * @param link_point   The point on the link for which to calculate the jacobian
 */
inline static void numericalJacobian(Eigen::Ref<Eigen::MatrixXd> jacobian,
                                     const Eigen::Isometry3d& change_base,
                                     const StateSolver& state_solver,
                                     const std::vector<std::string>& joint_names,
                                     const Eigen::Ref<const Eigen::VectorXd>& joint_values,
                                     const std::string& link_name,
                                     const Eigen::Ref<const Eigen::Vector3d>& link_point)
{
  Eigen::VectorXd njvals;
  double delta = 0.001;
  tesseract_common::TransformMap poses;
  if (joint_names.empty())
    poses = state_solver.getState(joint_values).link_transforms;
  else
    poses = state_solver.getState(joint_names, joint_values).link_transforms;

  Eigen::Isometry3d pose = poses[link_name];
  pose = change_base * pose;

  for (int i = 0; i < static_cast<int>(joint_values.size()); ++i)
  {
    njvals = joint_values;
    njvals[i] += delta;
    tesseract_common::TransformMap updated_poses;
    if (joint_names.empty())
      updated_poses = state_solver.getState(njvals).link_transforms;
    else
      updated_poses = state_solver.getState(joint_names, njvals).link_transforms;

    Eigen::Isometry3d updated_pose = updated_poses[link_name];
    updated_pose = change_base * updated_pose;

    Eigen::Vector3d temp = pose * link_point;
    Eigen::Vector3d temp2 = updated_pose * link_point;
    jacobian(0, i) = (temp2.x() - temp.x()) / delta;
    jacobian(1, i) = (temp2.y() - temp.y()) / delta;
    jacobian(2, i) = (temp2.z() - temp.z()) / delta;

    Eigen::AngleAxisd r12(pose.rotation().transpose() * updated_pose.rotation());  // rotation from p1 -> p2
    double theta = r12.angle();
    theta = copysign(fmod(fabs(theta), 2.0 * M_PI), theta);
    if (theta < -M_PI)
      theta = theta + 2. * M_PI;
    if (theta > M_PI)
      theta = theta - 2. * M_PI;
    Eigen::VectorXd omega = (pose.rotation() * r12.axis() * theta) / delta;
    jacobian(3, i) = omega(0);
    jacobian(4, i) = omega(1);
    jacobian(5, i) = omega(2);
  }
}

/**
 * @brief Run a kinematic jacobian test
 * @param state_solver The state solver object
 * @param jvals The joint values to calculate the jacobian about
 * @param link_name Name of link to calculate jacobian. If empty it will use the function that does not require link
 * name
 * @param link_point Is expressed in the same base frame of the jacobian and is a vector from the old point to the new
 * point.
 * @param change_base The transform from the desired frame to the current base frame of the jacobian
 */
inline void runCompareJacobian(StateSolver& state_solver,
                               const std::vector<std::string>& joint_names,
                               const Eigen::VectorXd& jvals,
                               const std::string& link_name,
                               const Eigen::Vector3d& link_point,
                               const Eigen::Isometry3d& change_base)
{
  Eigen::MatrixXd jacobian, numerical_jacobian;
  jacobian.resize(6, jvals.size());

  tesseract_common::TransformMap poses;

  // The numerical jacobian orders things base on the provided joint list
  // The order needs to be calculated to compare
  std::vector<std::string> solver_jn = state_solver.getActiveJointNames();
  std::vector<long> order;
  if (joint_names.empty())
  {
    for (int i = 0; i < static_cast<int>(solver_jn.size()); ++i)
      order.push_back(i);

    poses = state_solver.getState(jvals).link_transforms;
    jacobian = state_solver.getJacobian(jvals, link_name);
  }
  else
  {
    for (const auto& joint_name : solver_jn)
      order.push_back(
          std::distance(joint_names.begin(), std::find(joint_names.begin(), joint_names.end(), joint_name)));

    poses = state_solver.getState(joint_names, jvals).link_transforms;
    jacobian = state_solver.getJacobian(joint_names, jvals, link_name);
  }

  tesseract_common::jacobianChangeBase(jacobian, change_base);
  tesseract_common::jacobianChangeRefPoint(jacobian, (change_base * poses[link_name]).linear() * link_point);

  numerical_jacobian.resize(6, jvals.size());
  numericalJacobian(numerical_jacobian, change_base, state_solver, joint_names, jvals, link_name, link_point);

  for (int i = 0; i < 6; ++i)
  {
    for (int j = 0; j < static_cast<int>(jvals.size()); ++j)
    {
      EXPECT_NEAR(numerical_jacobian(i, order[static_cast<std::size_t>(j)]), jacobian(i, j), 1e-3);
    }
  }
}

template <typename S>
inline void runJacobianTest()
{
  // Get the scene graph
  auto scene_graph = getSceneGraph();
  auto state_solver = S(*scene_graph);

  std::vector<std::string> joint_names_empty;
  std::vector<std::string> link_names = { "base_link", "link_1", "link_2", "link_3", "link_4",
                                          "link_5",    "link_6", "link_7", "tool0" };

  //////////////////////////////////////////////////////////////////
  // Test forward kinematics when tip link is the base of the chain
  //////////////////////////////////////////////////////////////////
  Eigen::MatrixXd jacobian, numerical_jacobian;
  Eigen::VectorXd jvals;
  jvals.resize(7);

  //  jvals(0) = -0.785398;
  //  jvals(1) = 0.785398;
  //  jvals(2) = -0.785398;
  //  jvals(3) = 0.785398;
  //  jvals(4) = -0.785398;
  //  jvals(5) = 0.785398;
  //  jvals(6) = -0.785398;
  jvals(0) = -0.1;
  jvals(1) = 0.2;
  jvals(2) = -0.3;
  jvals(3) = 0.4;
  jvals(4) = -0.5;
  jvals(5) = 0.6;
  jvals(6) = -0.7;

  ///////////////////////////
  // Test Jacobian
  ///////////////////////////
  {
    Eigen::Vector3d link_point(0, 0, 0);
    for (const auto& link_name : link_names)
      runCompareJacobian(state_solver, joint_names_empty, jvals, link_name, link_point, Eigen::Isometry3d::Identity());

    // NOLINTNEXTLINE
    EXPECT_ANY_THROW(runCompareJacobian(
        state_solver, joint_names_empty, jvals, "", link_point, Eigen::Isometry3d::Identity()));  // NOLINT
  }

  ///////////////////////////
  // Test Jacobian at Point
  ///////////////////////////
  for (int k = 0; k < 3; ++k)
  {
    Eigen::Vector3d link_point(0, 0, 0);
    link_point[k] = 1;

    for (const auto& link_name : link_names)
      runCompareJacobian(state_solver, joint_names_empty, jvals, link_name, link_point, Eigen::Isometry3d::Identity());

    // NOLINTNEXTLINE
    EXPECT_ANY_THROW(runCompareJacobian(
        state_solver, joint_names_empty, jvals, "", link_point, Eigen::Isometry3d::Identity()));  // NOLINT
  }

  ///////////////////////////////////////////
  // Test Jacobian with change base
  ///////////////////////////////////////////
  for (int k = 0; k < 3; ++k)
  {
    Eigen::Vector3d link_point(0, 0, 0);
    Eigen::Isometry3d change_base;
    change_base.setIdentity();
    change_base(0, 0) = 0;
    change_base(1, 0) = 1;
    change_base(0, 1) = -1;
    change_base(1, 1) = 0;
    change_base.translation() = Eigen::Vector3d(0, 0, 0);
    change_base.translation()[k] = 1;

    for (const auto& link_name : link_names)
      runCompareJacobian(state_solver, joint_names_empty, jvals, link_name, link_point, change_base);

    // NOLINTNEXTLINE
    EXPECT_ANY_THROW(
        runCompareJacobian(state_solver, joint_names_empty, jvals, "", link_point, change_base));  // NOLINT
  }

  ///////////////////////////////////////////
  // Test Jacobian at point with change base
  ///////////////////////////////////////////
  for (int k = 0; k < 3; ++k)
  {
    Eigen::Vector3d link_point(0, 0, 0);
    link_point[k] = 1;

    Eigen::Isometry3d change_base;
    change_base.setIdentity();
    change_base(0, 0) = 0;
    change_base(1, 0) = 1;
    change_base(0, 1) = -1;
    change_base(1, 1) = 0;
    change_base.translation() = link_point;

    for (const auto& link_name : link_names)
      runCompareJacobian(state_solver, joint_names_empty, jvals, link_name, link_point, change_base);

    // NOLINTNEXTLINE
    EXPECT_ANY_THROW(
        runCompareJacobian(state_solver, joint_names_empty, jvals, "", link_point, change_base));  // NOLINT
  }

  /////////////////////////////////
  // Test Jacobian with joint names
  /////////////////////////////////
  std::vector<std::string> joint_names = state_solver.getActiveJointNames();
  {
    Eigen::Vector3d link_point(0, 0, 0);
    for (const auto& link_name : link_names)
      runCompareJacobian(state_solver, joint_names, jvals, link_name, link_point, Eigen::Isometry3d::Identity());

    // NOLINTNEXTLINE
    EXPECT_ANY_THROW(
        runCompareJacobian(state_solver, joint_names, jvals, "", link_point, Eigen::Isometry3d::Identity()));  // NOLINT
  }

  ///////////////////////////
  // Test Jacobian at Point
  ///////////////////////////
  for (int k = 0; k < 3; ++k)
  {
    Eigen::Vector3d link_point(0, 0, 0);
    link_point[k] = 1;

    for (const auto& link_name : link_names)
      runCompareJacobian(state_solver, joint_names, jvals, link_name, link_point, Eigen::Isometry3d::Identity());

    // NOLINTNEXTLINE
    EXPECT_ANY_THROW(
        runCompareJacobian(state_solver, joint_names, jvals, "", link_point, Eigen::Isometry3d::Identity()));  // NOLINT
  }

  ///////////////////////////////////////////
  // Test Jacobian with change base
  ///////////////////////////////////////////
  for (int k = 0; k < 3; ++k)
  {
    Eigen::Vector3d link_point(0, 0, 0);
    Eigen::Isometry3d change_base;
    change_base.setIdentity();
    change_base(0, 0) = 0;
    change_base(1, 0) = 1;
    change_base(0, 1) = -1;
    change_base(1, 1) = 0;
    change_base.translation() = Eigen::Vector3d(0, 0, 0);
    change_base.translation()[k] = 1;

    for (const auto& link_name : link_names)
      runCompareJacobian(state_solver, joint_names, jvals, link_name, link_point, change_base);

    // NOLINTNEXTLINE
    EXPECT_ANY_THROW(runCompareJacobian(state_solver, joint_names, jvals, "", link_point, change_base));  // NOLINT
  }

  ///////////////////////////////////////////
  // Test Jacobian at point with change base
  ///////////////////////////////////////////
  for (int k = 0; k < 3; ++k)
  {
    Eigen::Vector3d link_point(0, 0, 0);
    link_point[k] = 1;

    Eigen::Isometry3d change_base;
    change_base.setIdentity();
    change_base(0, 0) = 0;
    change_base(1, 0) = 1;
    change_base(0, 1) = -1;
    change_base(1, 1) = 0;
    change_base.translation() = link_point;

    for (const auto& link_name : link_names)
      runCompareJacobian(state_solver, joint_names, jvals, link_name, link_point, change_base);

    // NOLINTNEXTLINE
    EXPECT_ANY_THROW(runCompareJacobian(state_solver, joint_names, jvals, "", link_point, change_base));  // NOLINT
  }

  ////////////////////////////////////////////////////
  // Test Jacobian with joint names in different order
  ///////////////////////////////////////////////////
  std::reverse(joint_names.begin(), joint_names.end());
  jvals(0) = -0.7;
  jvals(1) = 0.6;
  jvals(2) = -0.5;
  jvals(3) = 0.4;
  jvals(4) = -0.3;
  jvals(5) = 0.2;
  jvals(6) = -0.1;

  {
    Eigen::Vector3d link_point(0, 0, 0);
    for (const auto& link_name : link_names)
      runCompareJacobian(state_solver, joint_names, jvals, link_name, link_point, Eigen::Isometry3d::Identity());

    // NOLINTNEXTLINE
    EXPECT_ANY_THROW(
        runCompareJacobian(state_solver, joint_names, jvals, "", link_point, Eigen::Isometry3d::Identity()));  // NOLINT
  }

  ///////////////////////////
  // Test Jacobian at Point
  ///////////////////////////
  for (int k = 0; k < 3; ++k)
  {
    Eigen::Vector3d link_point(0, 0, 0);
    link_point[k] = 1;

    for (const auto& link_name : link_names)
      runCompareJacobian(state_solver, joint_names, jvals, link_name, link_point, Eigen::Isometry3d::Identity());

    // NOLINTNEXTLINE
    EXPECT_ANY_THROW(
        runCompareJacobian(state_solver, joint_names, jvals, "", link_point, Eigen::Isometry3d::Identity()));  // NOLINT
  }

  ///////////////////////////////////////////
  // Test Jacobian with change base
  ///////////////////////////////////////////
  for (int k = 0; k < 3; ++k)
  {
    Eigen::Vector3d link_point(0, 0, 0);
    Eigen::Isometry3d change_base;
    change_base.setIdentity();
    change_base(0, 0) = 0;
    change_base(1, 0) = 1;
    change_base(0, 1) = -1;
    change_base(1, 1) = 0;
    change_base.translation() = Eigen::Vector3d(0, 0, 0);
    change_base.translation()[k] = 1;

    for (const auto& link_name : link_names)
      runCompareJacobian(state_solver, joint_names, jvals, link_name, link_point, change_base);

    EXPECT_ANY_THROW(runCompareJacobian(state_solver, joint_names, jvals, "", link_point, change_base));  // NOLINT
  }

  ///////////////////////////////////////////
  // Test Jacobian at point with change base
  ///////////////////////////////////////////
  for (int k = 0; k < 3; ++k)
  {
    Eigen::Vector3d link_point(0, 0, 0);
    link_point[k] = 1;

    Eigen::Isometry3d change_base;
    change_base.setIdentity();
    change_base(0, 0) = 0;
    change_base(1, 0) = 1;
    change_base(0, 1) = -1;
    change_base(1, 1) = 0;
    change_base.translation() = link_point;

    for (const auto& link_name : link_names)
      runCompareJacobian(state_solver, joint_names, jvals, link_name, link_point, change_base);

    EXPECT_ANY_THROW(runCompareJacobian(state_solver, joint_names, jvals, "", link_point, change_base));  // NOLINT
  }
}

template <typename S>
void runAddandRemoveLinkTest()
{
  // Get the scene graph
  auto scene_graph = getSceneGraph();
  auto state_solver = S(*scene_graph);

  auto visual = std::make_shared<Visual>();
  visual->geometry = std::make_shared<tesseract_geometry::Box>(1, 1, 1);
  auto collision = std::make_shared<Collision>();
  collision->geometry = std::make_shared<tesseract_geometry::Box>(1, 1, 1);

  const std::string link_name1 = "link_n1";
  const std::string link_name2 = "link_n2";
  const std::string joint_name1 = "joint_n1";
  const std::string joint_name2 = "joint_n2";

  Link link_1(link_name1);
  link_1.visual.push_back(visual);
  link_1.collision.push_back(collision);

  Joint joint_1(joint_name1);
  joint_1.parent_link_name = scene_graph->getRoot();
  joint_1.child_link_name = link_name1;
  joint_1.type = JointType::FIXED;

  Link link_2(link_name2);

  Joint joint_2(joint_name2);
  joint_2.parent_to_joint_origin_transform.translation()(0) = 1.25;
  joint_2.parent_link_name = link_name1;
  joint_2.child_link_name = link_name2;
  joint_2.type = JointType::FIXED;

  // Test adding link

  EXPECT_TRUE(scene_graph->addLink(link_1, joint_1));
  EXPECT_TRUE(state_solver.addLink(link_1, joint_1));

  KDLStateSolver base_state_solver(*scene_graph);

  runCompareStateSolver(base_state_solver, state_solver);
  runCompareStateSolverLimits(*scene_graph, base_state_solver);

  std::vector<std::string> joint_names = state_solver.getActiveJointNames();
  SceneState state = state_solver.getState();
  // Fixed joints are not listed
  EXPECT_TRUE(std::find(joint_names.begin(), joint_names.end(), joint_name1) == joint_names.end());
  EXPECT_TRUE(state.link_transforms.find(link_name1) != state.link_transforms.end());
  EXPECT_TRUE(state.joint_transforms.find(joint_name1) != state.joint_transforms.end());
  // Fixed joints are not listed
  EXPECT_TRUE(state.joints.find(joint_name1) == state.joints.end());

  EXPECT_TRUE(scene_graph->addLink(link_2, joint_2));
  EXPECT_TRUE(state_solver.addLink(link_2, joint_2));

  base_state_solver = KDLStateSolver(*scene_graph);

  runCompareStateSolver(base_state_solver, state_solver);
  runCompareStateSolverLimits(*scene_graph, base_state_solver);

  joint_names = state_solver.getActiveJointNames();
  state = state_solver.getState();
  // Fixed joints are not listed
  EXPECT_TRUE(std::find(joint_names.begin(), joint_names.end(), joint_name2) == joint_names.end());
  EXPECT_TRUE(state.link_transforms.find(link_name2) != state.link_transforms.end());
  EXPECT_TRUE(state.joint_transforms.find(joint_name2) != state.joint_transforms.end());
  // Fixed joints are not listed
  EXPECT_TRUE(state.joints.find(joint_name2) == state.joints.end());

  scene_graph->saveDOT(tesseract_common::getTempPath() + "state_solver_before_remove_link_unit.dot");

  // Test removing link
  EXPECT_TRUE(scene_graph->removeLink(link_name1, true));
  EXPECT_TRUE(state_solver.removeLink(link_name1));

  base_state_solver = KDLStateSolver(*scene_graph);

  runCompareStateSolver(base_state_solver, state_solver);
  runCompareStateSolverLimits(*scene_graph, base_state_solver);

  joint_names = state_solver.getActiveJointNames();
  state = state_solver.getState();
  EXPECT_TRUE(std::find(joint_names.begin(), joint_names.end(), joint_name1) == joint_names.end());
  EXPECT_TRUE(state.link_transforms.find(link_name1) == state.link_transforms.end());
  EXPECT_TRUE(state.joint_transforms.find(joint_name1) == state.joint_transforms.end());
  EXPECT_TRUE(state.joints.find(joint_name1) == state.joints.end());
  EXPECT_TRUE(state.link_transforms.find(link_name2) == state.link_transforms.end());
  EXPECT_TRUE(state.joint_transforms.find(joint_name2) == state.joint_transforms.end());
  EXPECT_TRUE(state.joints.find(joint_name2) == state.joints.end());

  scene_graph->saveDOT(tesseract_common::getTempPath() + "state_solver_after_remove_link_unit.dot");

  // Test against double removing

  EXPECT_FALSE(state_solver.removeLink(link_name1));
  runCompareStateSolver(base_state_solver, state_solver);
  runCompareStateSolverLimits(*scene_graph, base_state_solver);

  EXPECT_FALSE(state_solver.removeLink(link_name2));
  runCompareStateSolver(base_state_solver, state_solver);
  runCompareStateSolverLimits(*scene_graph, base_state_solver);

  EXPECT_FALSE(state_solver.removeJoint(joint_name1));
  runCompareStateSolver(base_state_solver, state_solver);
  runCompareStateSolverLimits(*scene_graph, base_state_solver);

  EXPECT_FALSE(state_solver.removeJoint(joint_name2));
  runCompareStateSolver(base_state_solver, state_solver);
  runCompareStateSolverLimits(*scene_graph, base_state_solver);

  /////////////////////////////////////////////////////////////////////////////////////

  // Test adding link

  EXPECT_TRUE(scene_graph->addLink(link_1, joint_1));
  EXPECT_TRUE(state_solver.addLink(link_1, joint_1));

  base_state_solver = KDLStateSolver(*scene_graph);

  runCompareStateSolver(base_state_solver, state_solver);
  runCompareStateSolverLimits(*scene_graph, base_state_solver);

  joint_names = state_solver.getActiveJointNames();
  state = state_solver.getState();
  // Fixed joints are not listed
  EXPECT_TRUE(std::find(joint_names.begin(), joint_names.end(), joint_name1) == joint_names.end());
  EXPECT_TRUE(state.link_transforms.find(link_name1) != state.link_transforms.end());
  EXPECT_TRUE(state.joint_transforms.find(joint_name1) != state.joint_transforms.end());
  // Fixed joints are not listed
  EXPECT_TRUE(state.joints.find(joint_name1) == state.joints.end());

  EXPECT_TRUE(scene_graph->addLink(link_2, joint_2));
  EXPECT_TRUE(state_solver.addLink(link_2, joint_2));

  base_state_solver = KDLStateSolver(*scene_graph);

  runCompareStateSolver(base_state_solver, state_solver);
  runCompareStateSolverLimits(*scene_graph, base_state_solver);

  joint_names = state_solver.getActiveJointNames();
  state = state_solver.getState();
  // Fixed joints are not listed
  EXPECT_TRUE(std::find(joint_names.begin(), joint_names.end(), joint_name2) == joint_names.end());
  EXPECT_TRUE(state.link_transforms.find(link_name2) != state.link_transforms.end());
  EXPECT_TRUE(state.joint_transforms.find(joint_name2) != state.joint_transforms.end());
  // Fixed joints are not listed
  EXPECT_TRUE(state.joints.find(joint_name2) == state.joints.end());

  scene_graph->saveDOT(tesseract_common::getTempPath() + "state_solver_before_remove_link_unit2.dot");

  // Test removing link
  EXPECT_TRUE(scene_graph->removeJoint(joint_name1, true));
  EXPECT_TRUE(state_solver.removeJoint(joint_name1));

  base_state_solver = KDLStateSolver(*scene_graph);

  runCompareStateSolver(base_state_solver, state_solver);
  runCompareStateSolverLimits(*scene_graph, base_state_solver);

  joint_names = state_solver.getActiveJointNames();
  state = state_solver.getState();
  EXPECT_TRUE(std::find(joint_names.begin(), joint_names.end(), joint_name1) == joint_names.end());
  EXPECT_TRUE(state.link_transforms.find(link_name1) == state.link_transforms.end());
  EXPECT_TRUE(state.joint_transforms.find(joint_name1) == state.joint_transforms.end());
  EXPECT_TRUE(state.joints.find(joint_name1) == state.joints.end());
  EXPECT_TRUE(state.link_transforms.find(link_name2) == state.link_transforms.end());
  EXPECT_TRUE(state.joint_transforms.find(joint_name2) == state.joint_transforms.end());
  EXPECT_TRUE(state.joints.find(joint_name2) == state.joints.end());

  scene_graph->saveDOT(tesseract_common::getTempPath() + "state_solver_after_remove_link_unit2.dot");

  EXPECT_FALSE(state_solver.removeLink(link_name1));
  runCompareStateSolver(base_state_solver, state_solver);
  runCompareStateSolverLimits(*scene_graph, base_state_solver);

  EXPECT_FALSE(state_solver.removeLink(link_name2));
  runCompareStateSolver(base_state_solver, state_solver);
  runCompareStateSolverLimits(*scene_graph, base_state_solver);

  EXPECT_FALSE(state_solver.removeJoint(joint_name1));
  runCompareStateSolver(base_state_solver, state_solver);
  runCompareStateSolverLimits(*scene_graph, base_state_solver);

  EXPECT_FALSE(state_solver.removeJoint(joint_name2));
  runCompareStateSolver(base_state_solver, state_solver);
  runCompareStateSolverLimits(*scene_graph, base_state_solver);
}

template <typename S>
void runAddSceneGraphTest()
{
  // Get the scene graph
  auto scene_graph = getSceneGraph();
  auto state_solver = S(*scene_graph);

  auto subgraph = std::make_unique<SceneGraph>();
  subgraph->setName("subgraph");

  Joint joint_1_empty("provided_subgraph_joint");
  joint_1_empty.parent_link_name = "base_link";
  joint_1_empty.child_link_name = "prefix_subgraph_base_link";
  joint_1_empty.type = JointType::FIXED;

  EXPECT_FALSE(scene_graph->insertSceneGraph(*subgraph, joint_1_empty));
  EXPECT_FALSE(state_solver.insertSceneGraph(*subgraph, joint_1_empty));

  KDLStateSolver base_state_solver(*scene_graph);

  runCompareStateSolver(base_state_solver, state_solver);
  runCompareStateSolverLimits(*scene_graph, base_state_solver);

  subgraph = getSubSceneGraph();

  const std::string subgraph_joint_name = "attach_subgraph_joint";

  Joint joint(subgraph_joint_name);
  joint.parent_link_name = scene_graph->getRoot();
  joint.child_link_name = subgraph->getRoot();
  joint.type = JointType::FIXED;

  EXPECT_TRUE(scene_graph->insertSceneGraph(*subgraph, joint));
  EXPECT_TRUE(state_solver.insertSceneGraph(*subgraph, joint));

  base_state_solver = KDLStateSolver(*scene_graph);

  runCompareStateSolver(base_state_solver, state_solver);
  runCompareStateSolverLimits(*scene_graph, base_state_solver);

  std::vector<std::string> joint_names = state_solver.getActiveJointNames();
  SceneState state = state_solver.getState();
  EXPECT_TRUE(std::find(joint_names.begin(), joint_names.end(), subgraph_joint_name) == joint_names.end());
  EXPECT_TRUE(state.link_transforms.find(subgraph->getRoot()) != state.link_transforms.end());
  EXPECT_TRUE(state.joint_transforms.find(subgraph_joint_name) != state.joint_transforms.end());
  EXPECT_TRUE(state.joints.find(subgraph_joint_name) == state.joints.end());

  // Adding twice with the same name should fail
  EXPECT_FALSE(scene_graph->insertSceneGraph(*subgraph, joint));
  EXPECT_FALSE(state_solver.insertSceneGraph(*subgraph, joint));

  base_state_solver = KDLStateSolver(*scene_graph);

  runCompareStateSolver(base_state_solver, state_solver);
  runCompareStateSolverLimits(*scene_graph, base_state_solver);

  // Add subgraph with prefix
  std::string prefix = "prefix_";
  Joint prefix_joint(prefix + subgraph_joint_name);
  prefix_joint.parent_link_name = scene_graph->getRoot();
  prefix_joint.child_link_name = prefix + subgraph->getRoot();
  prefix_joint.type = JointType::FIXED;

  EXPECT_TRUE(scene_graph->insertSceneGraph(*subgraph, prefix_joint, prefix));
  EXPECT_TRUE(state_solver.insertSceneGraph(*subgraph, prefix_joint, prefix));

  base_state_solver = KDLStateSolver(*scene_graph);

  runCompareStateSolver(base_state_solver, state_solver);
  runCompareStateSolverLimits(*scene_graph, base_state_solver);

  joint_names = state_solver.getActiveJointNames();
  state = state_solver.getState();
  EXPECT_TRUE(std::find(joint_names.begin(), joint_names.end(), prefix + subgraph_joint_name) == joint_names.end());
  EXPECT_TRUE(state.link_transforms.find(prefix + subgraph->getRoot()) != state.link_transforms.end());
  EXPECT_TRUE(state.joint_transforms.find(prefix + subgraph_joint_name) != state.joint_transforms.end());
  EXPECT_TRUE(state.joints.find(prefix + subgraph_joint_name) == state.joints.end());

  // Add subgraph with prefix and joint
  prefix = "prefix2_";
  Joint prefix_joint2(prefix + subgraph_joint_name);
  prefix_joint2.parent_link_name = scene_graph->getRoot();
  prefix_joint2.child_link_name = prefix + subgraph->getRoot();
  prefix_joint2.type = JointType::FIXED;

  EXPECT_TRUE(scene_graph->insertSceneGraph(*subgraph, prefix_joint2, prefix));
  EXPECT_TRUE(state_solver.insertSceneGraph(*subgraph, prefix_joint2, prefix));

  base_state_solver = KDLStateSolver(*scene_graph);

  runCompareStateSolver(base_state_solver, state_solver);
  runCompareStateSolverLimits(*scene_graph, base_state_solver);

  joint_names = state_solver.getActiveJointNames();

  EXPECT_TRUE(std::find(joint_names.begin(), joint_names.end(), prefix + subgraph_joint_name) == joint_names.end());
  state = state_solver.getState();
  EXPECT_TRUE(state.link_transforms.find(prefix + subgraph->getRoot()) != state.link_transforms.end());
  EXPECT_TRUE(state.joint_transforms.find(prefix + subgraph_joint_name) != state.joint_transforms.end());
  EXPECT_TRUE(state.joints.find(prefix + subgraph_joint_name) == state.joints.end());
}

template <typename S>
void runChangeJointOriginTest()
{
  // Get the scene graph
  auto scene_graph = getSceneGraph();
  auto state_solver = S(*scene_graph);

  const std::string link_name1 = "link_n1";
  const std::string joint_name1 = "joint_n1";
  Link link_1(link_name1);

  Joint joint_1(joint_name1);
  joint_1.parent_link_name = scene_graph->getRoot();
  joint_1.child_link_name = link_name1;
  joint_1.type = JointType::FIXED;

  EXPECT_TRUE(scene_graph->addLink(link_1, joint_1));
  EXPECT_TRUE(state_solver.addLink(link_1, joint_1));

  KDLStateSolver base_state_solver(*scene_graph);

  runCompareStateSolver(base_state_solver, state_solver);
  runCompareStateSolverLimits(*scene_graph, base_state_solver);

  SceneState state = state_solver.getState();
  EXPECT_TRUE(state.link_transforms.find(link_name1) != state.link_transforms.end());
  EXPECT_TRUE(state.joint_transforms.find(joint_name1) != state.joint_transforms.end());
  EXPECT_TRUE(state.joints.find(joint_name1) == state.joints.end());

  scene_graph->saveDOT(tesseract_common::getTempPath() + "state_solver_before_change_joint_origin_unit.dot");

  Eigen::Isometry3d new_origin = Eigen::Isometry3d::Identity();
  new_origin.translation()(0) += 1.234;

  EXPECT_TRUE(scene_graph->changeJointOrigin(joint_name1, new_origin));
  EXPECT_TRUE(state_solver.changeJointOrigin(joint_name1, new_origin));

  base_state_solver = KDLStateSolver(*scene_graph);

  runCompareStateSolver(base_state_solver, state_solver);
  runCompareStateSolverLimits(*scene_graph, base_state_solver);

  // Check that the origin got updated
  state = state_solver.getState();
  EXPECT_TRUE(state.link_transforms.at(link_name1).isApprox(new_origin));
  EXPECT_TRUE(state.joint_transforms.at(joint_name1).isApprox(new_origin));

  scene_graph->saveDOT(tesseract_common::getTempPath() + "state_solver_after_change_joint_origin_unit.dot");
}

template <typename S>
void runMoveJointTest()
{
  // Get the scene graph
  auto scene_graph = getSceneGraph();
  auto state_solver = S(*scene_graph);

  const std::string link_name1 = "link_n1";
  const std::string link_name2 = "link_n2";
  const std::string joint_name1 = "joint_n1";
  const std::string joint_name2 = "joint_n2";
  Link link_1(link_name1);
  Link link_2(link_name2);

  Joint joint_1(joint_name1);
  joint_1.parent_link_name = scene_graph->getRoot();
  joint_1.child_link_name = link_name1;
  joint_1.type = JointType::FIXED;

  Joint joint_2(joint_name2);
  joint_2.parent_to_joint_origin_transform.translation()(0) = 1.25;
  joint_2.parent_link_name = link_name1;
  joint_2.child_link_name = link_name2;
  joint_2.type = JointType::FIXED;

  EXPECT_TRUE(scene_graph->addLink(link_1, joint_1));
  EXPECT_TRUE(state_solver.addLink(link_1, joint_1));

  KDLStateSolver base_state_solver(*scene_graph);

  runCompareStateSolver(base_state_solver, state_solver);
  runCompareStateSolverLimits(*scene_graph, base_state_solver);

  SceneState state = state_solver.getState();
  EXPECT_TRUE(state.link_transforms.find(link_name1) != state.link_transforms.end());
  EXPECT_TRUE(state.joint_transforms.find(joint_name1) != state.joint_transforms.end());
  EXPECT_TRUE(state.joints.find(joint_name1) == state.joints.end());

  EXPECT_TRUE(scene_graph->addLink(link_2, joint_2));
  EXPECT_TRUE(state_solver.addLink(link_2, joint_2));

  base_state_solver = KDLStateSolver(*scene_graph);

  runCompareStateSolver(base_state_solver, state_solver);

  std::vector<std::string> joint_names = state_solver.getActiveJointNames();
  state = state_solver.getState();
  EXPECT_TRUE(std::find(joint_names.begin(), joint_names.end(), joint_name1) == joint_names.end());
  EXPECT_TRUE(std::find(joint_names.begin(), joint_names.end(), joint_name2) == joint_names.end());
  EXPECT_TRUE(state.link_transforms.find(link_name1) != state.link_transforms.end());
  EXPECT_TRUE(state.joint_transforms.find(joint_name1) != state.joint_transforms.end());
  EXPECT_TRUE(state.joints.find(joint_name1) == state.joints.end());
  EXPECT_TRUE(state.link_transforms.find(link_name2) != state.link_transforms.end());
  EXPECT_TRUE(state.joint_transforms.find(joint_name2) != state.joint_transforms.end());
  EXPECT_TRUE(state.joints.find(joint_name2) == state.joints.end());

  scene_graph->saveDOT(tesseract_common::getTempPath() + "state_solver_before_move_joint_unit.dot");

  EXPECT_TRUE(scene_graph->moveJoint(joint_name1, "tool0"));
  EXPECT_TRUE(state_solver.moveJoint(joint_name1, "tool0"));

  base_state_solver = KDLStateSolver(*scene_graph);

  runCompareStateSolver(base_state_solver, state_solver);
  runCompareStateSolverLimits(*scene_graph, base_state_solver);

  joint_names = state_solver.getActiveJointNames();
  state = state_solver.getState();
  EXPECT_TRUE(std::find(joint_names.begin(), joint_names.end(), joint_name1) == joint_names.end());
  EXPECT_TRUE(std::find(joint_names.begin(), joint_names.end(), joint_name2) == joint_names.end());
  EXPECT_TRUE(state.link_transforms.find(link_name1) != state.link_transforms.end());
  EXPECT_TRUE(state.joint_transforms.find(joint_name1) != state.joint_transforms.end());
  EXPECT_TRUE(state.joints.find(joint_name1) == state.joints.end());
  EXPECT_TRUE(state.link_transforms.find(link_name2) != state.link_transforms.end());
  EXPECT_TRUE(state.joint_transforms.find(joint_name2) != state.joint_transforms.end());
  EXPECT_TRUE(state.joints.find(joint_name2) == state.joints.end());

  scene_graph->saveDOT(tesseract_common::getTempPath() + "state_solver_after_move_joint_unit.dot");
}

template <typename S>
void runMoveLinkTest()
{
  // Get the scene graph
  auto scene_graph = getSceneGraph();
  auto state_solver = S(*scene_graph);

  const std::string link_name1 = "link_n1";
  const std::string link_name2 = "link_n2";
  const std::string joint_name1 = "joint_n1";
  const std::string joint_name2 = "joint_n2";
  Link link_1(link_name1);
  Link link_2(link_name2);

  Joint joint_1(joint_name1);
  joint_1.parent_link_name = scene_graph->getRoot();
  joint_1.child_link_name = link_name1;
  joint_1.type = JointType::FIXED;

  Joint joint_2(joint_name2);
  joint_2.parent_to_joint_origin_transform.translation()(0) = 1.25;
  joint_2.parent_link_name = link_name1;
  joint_2.child_link_name = link_name2;
  joint_2.type = JointType::FIXED;

  EXPECT_TRUE(scene_graph->addLink(link_1, joint_1));
  EXPECT_TRUE(state_solver.addLink(link_1, joint_1));

  KDLStateSolver base_state_solver(*scene_graph);

  runCompareStateSolver(base_state_solver, state_solver);
  runCompareStateSolverLimits(*scene_graph, base_state_solver);

  SceneState state = state_solver.getState();
  EXPECT_TRUE(state.link_transforms.find(link_name1) != state.link_transforms.end());
  EXPECT_TRUE(state.joint_transforms.find(joint_name1) != state.joint_transforms.end());
  EXPECT_TRUE(state.joints.find(joint_name1) == state.joints.end());

  EXPECT_TRUE(scene_graph->addLink(link_2, joint_2));
  EXPECT_TRUE(state_solver.addLink(link_2, joint_2));

  base_state_solver = KDLStateSolver(*scene_graph);

  runCompareStateSolver(base_state_solver, state_solver);
  runCompareStateSolverLimits(*scene_graph, base_state_solver);

  std::vector<std::string> joint_names = state_solver.getActiveJointNames();
  state = state_solver.getState();
  EXPECT_TRUE(std::find(joint_names.begin(), joint_names.end(), joint_name1) == joint_names.end());
  EXPECT_TRUE(std::find(joint_names.begin(), joint_names.end(), joint_name2) == joint_names.end());
  EXPECT_TRUE(state.link_transforms.find(link_name1) != state.link_transforms.end());
  EXPECT_TRUE(state.joint_transforms.find(joint_name1) != state.joint_transforms.end());
  EXPECT_TRUE(state.joints.find(joint_name1) == state.joints.end());
  EXPECT_TRUE(state.link_transforms.find(link_name2) != state.link_transforms.end());
  EXPECT_TRUE(state.joint_transforms.find(joint_name2) != state.joint_transforms.end());
  EXPECT_TRUE(state.joints.find(joint_name2) == state.joints.end());

  scene_graph->saveDOT(tesseract_common::getTempPath() + "state_solver_before_move_link_unit.dot");

  std::string moved_joint_name = joint_name1 + "_moved";
  Joint move_link_joint = joint_1.clone(moved_joint_name);
  move_link_joint.parent_link_name = "tool0";

  EXPECT_TRUE(scene_graph->moveLink(move_link_joint));
  EXPECT_TRUE(state_solver.moveLink(move_link_joint));

  base_state_solver = KDLStateSolver(*scene_graph);

  runCompareStateSolver(base_state_solver, state_solver);
  runCompareStateSolverLimits(*scene_graph, base_state_solver);

  joint_names = state_solver.getActiveJointNames();
  state = state_solver.getState();
  EXPECT_TRUE(std::find(joint_names.begin(), joint_names.end(), joint_name1) == joint_names.end());
  EXPECT_TRUE(std::find(joint_names.begin(), joint_names.end(), moved_joint_name) == joint_names.end());
  EXPECT_TRUE(std::find(joint_names.begin(), joint_names.end(), joint_name2) == joint_names.end());

  EXPECT_TRUE(state.link_transforms.find(link_name1) != state.link_transforms.end());
  EXPECT_TRUE(state.joint_transforms.find(joint_name1) == state.joint_transforms.end());
  EXPECT_TRUE(state.joint_transforms.find(moved_joint_name) != state.joint_transforms.end());
  EXPECT_TRUE(state.joints.find(joint_name1) == state.joints.end());
  EXPECT_TRUE(state.link_transforms.find(link_name2) != state.link_transforms.end());
  EXPECT_TRUE(state.joint_transforms.find(joint_name2) != state.joint_transforms.end());
  EXPECT_TRUE(state.joints.find(joint_name2) == state.joints.end());

  scene_graph->saveDOT(tesseract_common::getTempPath() + "state_solver_after_move_link_unit.dot");
}

template <typename S>
void runChangeJointLimitsTest()
{
  // Get the scene graph
  auto scene_graph = getSceneGraph();
  auto state_solver = S(*scene_graph);

  double new_lower = 1.0;
  double new_upper = 2.0;
  double new_velocity = 3.0;
  double new_acceleration = 4.0;

  scene_graph->changeJointPositionLimits("joint_a1", new_lower, new_upper);
  scene_graph->changeJointVelocityLimits("joint_a1", new_velocity);
  scene_graph->changeJointAccelerationLimits("joint_a1", new_acceleration);

  state_solver.changeJointPositionLimits("joint_a1", new_lower, new_upper);
  state_solver.changeJointVelocityLimits("joint_a1", new_velocity);
  state_solver.changeJointAccelerationLimits("joint_a1", new_acceleration);

  std::vector<std::string> joint_names = state_solver.getActiveJointNames();
  long idx = std::distance(joint_names.begin(), std::find(joint_names.begin(), joint_names.end(), "joint_a1"));
  auto limits = state_solver.getLimits();
  EXPECT_NEAR(limits.joint_limits(idx, 0), new_lower, 1e-5);
  EXPECT_NEAR(limits.joint_limits(idx, 1), new_upper, 1e-5);
  EXPECT_NEAR(limits.velocity_limits(idx), new_velocity, 1e-5);
  EXPECT_NEAR(limits.acceleration_limits(idx), new_acceleration, 1e-5);
}

template <typename S>
void runReplaceJointTest()
{
  {  // Replace joint with same type
    // Get the scene graph
    auto scene_graph = getSceneGraph();
    auto state_solver = S(*scene_graph);

    Joint joint_1("joint_a1");
    joint_1.parent_to_joint_origin_transform.translation()(0) = 1.25;
    joint_1.parent_link_name = "base_link";
    joint_1.child_link_name = "link_1";
    joint_1.type = JointType::FIXED;

    EXPECT_TRUE(scene_graph->removeJoint("joint_a1"));
    EXPECT_TRUE(scene_graph->addJoint(joint_1));
    EXPECT_TRUE(state_solver.replaceJoint(joint_1));

    KDLStateSolver base_state_solver(*scene_graph);

    runCompareStateSolver(base_state_solver, state_solver);
    runCompareStateSolverLimits(*scene_graph, base_state_solver);
  }

  {  // Replace joint which exist but the link does not which should fail
    // Get the scene graph
    auto scene_graph = getSceneGraph();
    auto state_solver = S(*scene_graph);

    Joint joint_1("joint_a1");
    joint_1.parent_to_joint_origin_transform.translation()(0) = 1.25;
    joint_1.parent_link_name = "base_link";
    joint_1.child_link_name = "link_2_does_not_exist";
    joint_1.type = JointType::FIXED;

    EXPECT_FALSE(state_solver.replaceJoint(joint_1));

    KDLStateSolver base_state_solver(*scene_graph);

    runCompareStateSolver(base_state_solver, state_solver);
    runCompareStateSolverLimits(*scene_graph, base_state_solver);
  }

  {  // Replace joint with same type but change transform
    // Get the scene graph
    auto scene_graph = getSceneGraph();
    auto state_solver = S(*scene_graph);

    Joint new_joint_a1 = scene_graph->getJoint("joint_a1")->clone();
    new_joint_a1.parent_to_joint_origin_transform.translation()(0) = 1.25;

    EXPECT_TRUE(scene_graph->removeJoint("joint_a1"));
    EXPECT_TRUE(scene_graph->addJoint(new_joint_a1));
    EXPECT_TRUE(state_solver.replaceJoint(new_joint_a1));

    KDLStateSolver base_state_solver(*scene_graph);

    runCompareStateSolver(base_state_solver, state_solver);
    runCompareStateSolverLimits(*scene_graph, base_state_solver);
  }

  {  // Replace joint with different type (Fixed)
    // Get the scene graph
    auto scene_graph = getSceneGraph();
    auto state_solver = S(*scene_graph);

    Joint new_joint_a1 = scene_graph->getJoint("joint_a1")->clone();
    new_joint_a1.parent_to_joint_origin_transform.translation()(0) = 1.25;
    new_joint_a1.type = JointType::FIXED;

    EXPECT_TRUE(scene_graph->removeJoint("joint_a1"));
    EXPECT_TRUE(scene_graph->addJoint(new_joint_a1));
    EXPECT_TRUE(state_solver.replaceJoint(new_joint_a1));

    KDLStateSolver base_state_solver(*scene_graph);

    runCompareStateSolver(base_state_solver, state_solver);
    runCompareStateSolverLimits(*scene_graph, base_state_solver);
  }

  {  // Replace joint with different type (Continuous)
    // Get the scene graph
    auto scene_graph = getSceneGraph();
    auto state_solver = S(*scene_graph);

    Joint new_joint_a1 = scene_graph->getJoint("joint_a1")->clone();
    new_joint_a1.parent_to_joint_origin_transform.translation()(0) = 1.25;
    new_joint_a1.type = JointType::CONTINUOUS;

    EXPECT_TRUE(scene_graph->removeJoint("joint_a1"));
    EXPECT_TRUE(scene_graph->addJoint(new_joint_a1));
    EXPECT_TRUE(state_solver.replaceJoint(new_joint_a1));

    KDLStateSolver base_state_solver(*scene_graph);

    runCompareStateSolver(base_state_solver, state_solver);
    runCompareStateSolverLimits(*scene_graph, base_state_solver);
  }

  {  // Replace joint with different type (Prismatic)
    // Get the scene graph
    auto scene_graph = getSceneGraph();
    auto state_solver = S(*scene_graph);

    Joint new_joint_a1 = scene_graph->getJoint("joint_a1")->clone();
    new_joint_a1.parent_to_joint_origin_transform.translation()(0) = 1.25;
    new_joint_a1.type = JointType::PRISMATIC;

    EXPECT_TRUE(scene_graph->removeJoint("joint_a1"));
    EXPECT_TRUE(scene_graph->addJoint(new_joint_a1));
    EXPECT_TRUE(state_solver.replaceJoint(new_joint_a1));

    KDLStateSolver base_state_solver(*scene_graph);

    runCompareStateSolver(base_state_solver, state_solver);
    runCompareStateSolverLimits(*scene_graph, base_state_solver);
  }

  {  // Replace joint with different parent which is a replace and move
    // Get the scene graph
    auto scene_graph = getSceneGraph();
    auto state_solver = S(*scene_graph);

    Joint new_joint_a3 = scene_graph->getJoint("joint_a3")->clone();
    new_joint_a3.parent_link_name = "base_link";

    EXPECT_TRUE(state_solver.replaceJoint(new_joint_a3));

    EXPECT_TRUE(scene_graph->removeJoint("joint_a3"));
    EXPECT_TRUE(scene_graph->addJoint(new_joint_a3));

    KDLStateSolver base_state_solver(*scene_graph);

    runCompareStateSolver(base_state_solver, state_solver);
    runCompareStateSolverLimits(*scene_graph, base_state_solver);
  }
}
}  // namespace tesseract_scene_graph::test_suite

#endif  // TESSERACT_STATE_SOLVER_
