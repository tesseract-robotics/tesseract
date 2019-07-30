/**
  These tests test the TrajOptArrayPlanner and the TrajOptFreespacePlanner. They primarily check that the correct types
  of costs and constraints are added when the flags like smooth_velocity are specified. However they are not foolproof.
  They only check that at least one term of the correct type is in the cost or constraint vector. If there should be
  more than one, then it might not be caught. This could be improved in the future, but it is better than nothing.

  Additional features that could be tested in the future
  * Configuration costs added correctly
  * Intermediate waypoints added correctly to freespace
  * coeffs set correctly
  * init info is set correctly
  * Seed trajectory is set correctly
  * callbacks are added correctly
  * Number of steps are obeyed for freespace
  * continuous collision checking flag set correctly

  Last updated: July 15, 2019
  Matthew Powelson
*/

#include <gtest/gtest.h>

// These contain the definitions of the cost types
#include <trajopt/trajectory_costs.hpp>
#include <trajopt_sco/modeling.hpp>
#include <trajopt/collision_terms.hpp>
#include <trajopt/problem_description.hpp>

#include <tesseract_motion_planners/trajopt/trajopt_freespace_planner.h>
#include <tesseract_motion_planners/trajopt/trajopt_array_planner.h>
#include <urdf_parser/urdf_parser.h>

const int NUM_STEPS = 7;

using namespace trajopt;
using namespace tesseract;
using namespace tesseract_environment;
using namespace tesseract_scene_graph;
using namespace tesseract_collision;
using namespace tesseract_motion_planners;

std::string locateResource(const std::string& url)
{
  std::string mod_url = url;
  if (url.find("package://tesseract_support") == 0)
  {
    mod_url.erase(0, strlen("package://tesseract_support"));
    size_t pos = mod_url.find("/");
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

class TesseractPlanningTrajoptUnit : public ::testing::Test
{
protected:
  Tesseract::Ptr tesseract_ptr_;

  void SetUp() override
  {
    ResourceLocatorFn locator = locateResource;
    Tesseract::Ptr tesseract = std::make_shared<Tesseract>();
    boost::filesystem::path urdf_path(std::string(TESSERACT_SUPPORT_DIR) + "/urdf/lbr_iiwa_14_r820.urdf");
    boost::filesystem::path srdf_path(std::string(TESSERACT_SUPPORT_DIR) + "/urdf/lbr_iiwa_14_r820.srdf");
    EXPECT_TRUE(tesseract->init(urdf_path, srdf_path, locator));
    tesseract_ptr_ = tesseract;
  }
};

namespace tesseract_tests
{
/**
 * @brief Test class created to get access to the problem for testing
 */
class TrajOptFreespacePlannerTest : public TrajOptFreespacePlanner
{
public:
  trajopt::TrajOptProb::Ptr getProblem() { return ConstructProblem(*pci_); }
};

/** @brief Test class created to get access to the problem for testing */
class TrajOptArrayPlannerTest : public TrajOptArrayPlanner
{
public:
  trajopt::TrajOptProb::Ptr getProblem() { return ConstructProblem(*pci_); }
};

/** @brief Checks if the vector of base class types contains at least one instance of the Derived type */
template <class Base, class Derived>
bool vectorContainsType(std::vector<Base> vector)
{
  bool contains_type = false;
  for (Base unit : vector)
  {
    Derived* test = dynamic_cast<Derived*>(unit.get());
    contains_type |= (test != nullptr);
  }
  return contains_type;
}

/** @brief Checks if the object passed in is of the derived type */
template <class Base, class Derived>
bool objectIsType(Base unit)
{
  Derived* test = dynamic_cast<Derived*>(unit.get());
  bool contains_type = (test != nullptr);

  return contains_type;
}

}  // namespace tesseract_tests

// This test checks that the boolean flags are adding the correct costs for smoothing and collision
TEST_F(TesseractPlanningTrajoptUnit, TrajoptFreespacePlanner0)
{
  // Set the parameters (Most are being left as defaults)
  TrajOptFreespacePlannerConfig config;
  // These are required
  config.link_ = "tool0";
  config.manipulator_ = "manipulator";
  config.tesseract_ = tesseract_ptr_;
  config.num_steps_ = NUM_STEPS;

  std::vector<std::string> joint_names = tesseract_ptr_->getFwdKinematicsManagerConst()->getFwdKinematicSolver("manipulator")->getJointNames();

  // Specify a JointWaypoint as the start
  config.start_waypoint_ = std::make_shared<JointWaypoint>(std::vector<double>({0, 0, 0, -1.57, 0, 0, 0}), joint_names);

  // Specify a Joint Waypoint as the finish
  config.end_waypoint_ = std::make_shared<JointWaypoint>(std::vector<double>({0, 0, 0, 1.57, 0, 0, 0}), joint_names);

  // Create test planner used for testing problem creation
  tesseract_tests::TrajOptFreespacePlannerTest test_planner;

  // Loop over all combinations of these 4. 0001, 0010, 0011, ... , 1111
  for (uint8_t byte = 0; byte < 16; byte++)
  {
    bool t1 = static_cast<bool>(byte & 0x1);
    bool t2 = static_cast<bool>(byte & 0x2);
    bool t3 = static_cast<bool>(byte & 0x4);
    bool t4 = static_cast<bool>(byte & 0x8);

    config.smooth_velocities_ = t1;
    config.smooth_accelerations_ = t2;
    config.smooth_jerks_ = t3;
    config.collision_check_ = t4;
    test_planner.setConfiguration(config);
    trajopt::TrajOptProb::Ptr prob = test_planner.getProblem();
    EXPECT_EQ((tesseract_tests::vectorContainsType<sco::Cost::Ptr, trajopt::JointVelEqCost>(prob->getCosts())), t1);
    EXPECT_EQ((tesseract_tests::vectorContainsType<sco::Cost::Ptr, trajopt::JointAccEqCost>(prob->getCosts())), t2);
    EXPECT_EQ((tesseract_tests::vectorContainsType<sco::Cost::Ptr, trajopt::JointJerkEqCost>(prob->getCosts())), t3);
    EXPECT_EQ((tesseract_tests::vectorContainsType<sco::Cost::Ptr, trajopt::CollisionCost>(prob->getCosts())), t4);
  }
}

// This test tests freespace motion b/n 2 joint waypoints
TEST_F(TesseractPlanningTrajoptUnit, TrajoptFreespacePlanner1)
{
  // Set the parameters (Most are being left as defaults)
  TrajOptFreespacePlannerConfig config;
  // These are required
  config.link_ = "tool0";
  config.manipulator_ = "manipulator";
  config.tesseract_ = tesseract_ptr_;
  config.num_steps_ = NUM_STEPS;

  std::vector<std::string> joint_names = tesseract_ptr_->getFwdKinematicsManagerConst()->getFwdKinematicSolver("manipulator")->getJointNames();

  // Specify a JointWaypoint as the start
  config.start_waypoint_ = std::make_shared<JointWaypoint>(std::vector<double>({0, 0, 0, -1.57, 0, 0, 0}), joint_names);

  // Specify a Joint Waypoint as the finish
  config.end_waypoint_ = std::make_shared<JointWaypoint>(std::vector<double>({0, 0, 0, 1.57, 0, 0, 0}), joint_names);

  // Create test planner used for testing problem creation
  tesseract_tests::TrajOptFreespacePlannerTest test_planner;
  {
    config.start_waypoint_->setIsCritical(true);
    config.end_waypoint_->setIsCritical(true);
    test_planner.setConfiguration(config);
    trajopt::TrajOptProb::Ptr prob = test_planner.getProblem();
    EXPECT_TRUE((tesseract_tests::vectorContainsType<sco::Constraint::Ptr, trajopt::JointPosEqConstraint>(
        prob->getConstraints())));
    EXPECT_FALSE((tesseract_tests::vectorContainsType<sco::Constraint::Ptr, trajopt::TrajOptConstraintFromErrFunc>(
        prob->getConstraints())));
  }
  {
    config.start_waypoint_->setIsCritical(false);
    config.end_waypoint_->setIsCritical(false);
    test_planner.setConfiguration(config);
    trajopt::TrajOptProb::Ptr prob = test_planner.getProblem();
    EXPECT_TRUE((tesseract_tests::vectorContainsType<sco::Cost::Ptr, trajopt::JointPosEqCost>(prob->getCosts())));
    EXPECT_FALSE(
        (tesseract_tests::vectorContainsType<sco::Cost::Ptr, trajopt::TrajOptCostFromErrFunc>(prob->getCosts())));
  }
}

// This test tests freespace motion b/n 1 joint waypoint and 1 cartesian waypoint
TEST_F(TesseractPlanningTrajoptUnit, TrajoptFreespacePlanner2)
{
  // Create the planner and the responses that will store the results
  TrajOptFreespacePlanner planner;
  PlannerResponse planning_response;

  // Set the parameters (Most are being left as defaults)
  TrajOptFreespacePlannerConfig config;
  // These are required
  config.link_ = "tool0";
  config.manipulator_ = "manipulator";
  config.tesseract_ = tesseract_ptr_;
  config.num_steps_ = NUM_STEPS;

  std::vector<std::string> joint_names = tesseract_ptr_->getFwdKinematicsManagerConst()->getFwdKinematicSolver("manipulator")->getJointNames();

  // Specify a Joint Waypoint as the start
  config.start_waypoint_ = std::make_shared<JointWaypoint>(std::vector<double>({0, 0, 0, -1.57, 0, 0, 0}), joint_names);

  // Specify a Cartesian Waypoint as the finish
  config.end_waypoint_ = std::make_shared<CartesianWaypoint>(Eigen::Vector3d(-.20, .4, 0.2), Eigen::Quaterniond(0, 0, 1.0, 0));

  // Create test planner used for testing problem creation
  tesseract_tests::TrajOptFreespacePlannerTest test_planner;
  {
    config.start_waypoint_->setIsCritical(true);
    config.end_waypoint_->setIsCritical(true);
    test_planner.setConfiguration(config);
    trajopt::TrajOptProb::Ptr prob = test_planner.getProblem();
    EXPECT_TRUE((tesseract_tests::vectorContainsType<sco::Constraint::Ptr, trajopt::JointPosEqConstraint>(
        prob->getConstraints())));
    EXPECT_TRUE((tesseract_tests::vectorContainsType<sco::Constraint::Ptr, trajopt::TrajOptConstraintFromErrFunc>(
        prob->getConstraints())));
  }
  {
    config.start_waypoint_->setIsCritical(false);
    config.end_waypoint_->setIsCritical(false);
    test_planner.setConfiguration(config);
    trajopt::TrajOptProb::Ptr prob = test_planner.getProblem();
    EXPECT_TRUE((tesseract_tests::vectorContainsType<sco::Cost::Ptr, trajopt::JointPosEqCost>(prob->getCosts())));
    EXPECT_TRUE(
        (tesseract_tests::vectorContainsType<sco::Cost::Ptr, trajopt::TrajOptCostFromErrFunc>(prob->getCosts())));
  }
}

// This test tests freespace motion b/n 1 cartesian waypoint and 1 joint waypoint
TEST_F(TesseractPlanningTrajoptUnit, TrajoptFreespacePlanner3)
{
  // Create the planner and the responses that will store the results
  TrajOptFreespacePlanner planner;
  PlannerResponse planning_response;

  // Set the parameters (Most are being left as defaults)
  TrajOptFreespacePlannerConfig config;
  // These are required
  config.link_ = "tool0";
  config.manipulator_ = "manipulator";
  config.tesseract_ = tesseract_ptr_;
  config.num_steps_ = NUM_STEPS;

  std::vector<std::string> joint_names = tesseract_ptr_->getFwdKinematicsManagerConst()->getFwdKinematicSolver("manipulator")->getJointNames();

  // Specify a Cartesian Waypoint as the start
  config.start_waypoint_ = std::make_shared<CartesianWaypoint>(Eigen::Vector3d(-.20, .4, 0.2), Eigen::Quaterniond(0, 0, 1.0, 0));

  // Specify a Joint Waypoint as the finish
  config.end_waypoint_ = std::make_shared<JointWaypoint>(std::vector<double>({0, 0, 0, -1.57, 0, 0, 0}), joint_names);

  // Create test planner used for testing problem creation
  tesseract_tests::TrajOptFreespacePlannerTest test_planner;
  {
    config.start_waypoint_->setIsCritical(true);
    config.end_waypoint_->setIsCritical(true);
    test_planner.setConfiguration(config);
    trajopt::TrajOptProb::Ptr prob = test_planner.getProblem();
    EXPECT_TRUE((tesseract_tests::vectorContainsType<sco::Constraint::Ptr, trajopt::JointPosEqConstraint>(
        prob->getConstraints())));
    EXPECT_TRUE((tesseract_tests::vectorContainsType<sco::Constraint::Ptr, trajopt::TrajOptConstraintFromErrFunc>(
        prob->getConstraints())));
  }
  {
    config.start_waypoint_->setIsCritical(false);
    config.end_waypoint_->setIsCritical(false);
    test_planner.setConfiguration(config);
    trajopt::TrajOptProb::Ptr prob = test_planner.getProblem();
    EXPECT_TRUE((tesseract_tests::vectorContainsType<sco::Cost::Ptr, trajopt::JointPosEqCost>(prob->getCosts())));
    EXPECT_TRUE(
        (tesseract_tests::vectorContainsType<sco::Cost::Ptr, trajopt::TrajOptCostFromErrFunc>(prob->getCosts())));
  }
}

// This test tests freespace motion b/n 2 cartesian waypoints
TEST_F(TesseractPlanningTrajoptUnit, TrajoptFreespacePlanner4)
{
  // Create the planner and the responses that will store the results
  TrajOptFreespacePlanner planner;
  PlannerResponse planning_response;

  // Set the parameters (Most are being left as defaults)
  TrajOptFreespacePlannerConfig config;
  // These are required
  config.link_ = "tool0";
  config.manipulator_ = "manipulator";
  config.tesseract_ = tesseract_ptr_;
  config.num_steps_ = NUM_STEPS;

  // Specify a Cartesian Waypoint as the start
  config.start_waypoint_ = std::make_shared<CartesianWaypoint>(Eigen::Vector3d(-.20, .4, 0.2), Eigen::Quaterniond(0, 0, 1.0, 0));

  // Specify a Cartesian Waypoint as the finish
  config.end_waypoint_ = std::make_shared<CartesianWaypoint>(Eigen::Vector3d(-.20, .4, 0.2), Eigen::Quaterniond(0, 0, 1.0, 0));

  // Create test planner used for testing problem creation
  tesseract_tests::TrajOptFreespacePlannerTest test_planner;
  {
    config.start_waypoint_->setIsCritical(true);
    config.end_waypoint_->setIsCritical(true);
    test_planner.setConfiguration(config);
    trajopt::TrajOptProb::Ptr prob = test_planner.getProblem();
    EXPECT_FALSE((tesseract_tests::vectorContainsType<sco::Constraint::Ptr, trajopt::JointPosEqConstraint>(
        prob->getConstraints())));
    EXPECT_TRUE((tesseract_tests::vectorContainsType<sco::Constraint::Ptr, trajopt::TrajOptConstraintFromErrFunc>(
        prob->getConstraints())));
  }
  {
    config.start_waypoint_->setIsCritical(false);
    config.end_waypoint_->setIsCritical(false);
    test_planner.setConfiguration(config);
    trajopt::TrajOptProb::Ptr prob = test_planner.getProblem();
    EXPECT_FALSE((tesseract_tests::vectorContainsType<sco::Cost::Ptr, trajopt::JointPosEqCost>(prob->getCosts())));
    EXPECT_TRUE(
        (tesseract_tests::vectorContainsType<sco::Cost::Ptr, trajopt::TrajOptCostFromErrFunc>(prob->getCosts())));
  }
}

// ---------------------------------------------------
// ------------ Array Planner ------------------------
// ---------------------------------------------------

// This test checks that the boolean flags are adding the correct costs for smoothing, collision, and cartesian cnts are
// added correctly
TEST_F(TesseractPlanningTrajoptUnit, TrajoptArrayPlanner0)
{
  // Set the parameters (Most are being left as defaults)
  TrajOptArrayPlannerConfig config;
  // These are required
  config.link_ = "tool0";
  config.manipulator_ = "manipulator";
  config.tesseract_ = tesseract_ptr_;

  // These specify the series of points to be optimized
  for (int ind = 0; ind < NUM_STEPS; ind++)
  {
    CartesianWaypoint::Ptr waypoint = std::make_shared<CartesianWaypoint>(Eigen::Vector3d(-0.2 + ind * .02, 0.4, 0.8), Eigen::Quaterniond(0, 1.0, 0, 0));
    waypoint->setIsCritical(true);
    config.target_waypoints_.push_back(waypoint);
  }

  // Create test planner used for testing problem creation
  tesseract_tests::TrajOptArrayPlannerTest test_planner;

  // Loop over all combinations of these 4. 0001, 0010, 0011, ... , 1111
  for (uint8_t byte = 0; byte < 16; byte++)
  {
    bool t1 = static_cast<bool>(byte & 0x1);
    bool t2 = static_cast<bool>(byte & 0x2);
    bool t3 = static_cast<bool>(byte & 0x4);
    bool t4 = static_cast<bool>(byte & 0x8);

    config.smooth_velocities_ = t1;
    config.smooth_accelerations_ = t2;
    config.smooth_jerks_ = t3;
    config.collision_check_ = t4;
    test_planner.setConfiguration(config);
    trajopt::TrajOptProb::Ptr prob = test_planner.getProblem();
    EXPECT_EQ((tesseract_tests::vectorContainsType<sco::Cost::Ptr, trajopt::JointVelEqCost>(prob->getCosts())), t1);
    EXPECT_EQ((tesseract_tests::vectorContainsType<sco::Cost::Ptr, trajopt::JointAccEqCost>(prob->getCosts())), t2);
    EXPECT_EQ((tesseract_tests::vectorContainsType<sco::Cost::Ptr, trajopt::JointJerkEqCost>(prob->getCosts())), t3);
    EXPECT_EQ((tesseract_tests::vectorContainsType<sco::Cost::Ptr, trajopt::CollisionCost>(prob->getCosts())), t4);
  }
  trajopt::TrajOptProb::Ptr prob = test_planner.getProblem();
  EXPECT_FALSE((tesseract_tests::vectorContainsType<sco::Constraint::Ptr, trajopt::JointPosEqConstraint>(
      prob->getConstraints())));
  EXPECT_TRUE((tesseract_tests::vectorContainsType<sco::Constraint::Ptr, trajopt::TrajOptConstraintFromErrFunc>(
      prob->getConstraints())));
  EXPECT_FALSE((tesseract_tests::vectorContainsType<sco::Cost::Ptr, trajopt::JointPosEqCost>(prob->getCosts())));
  EXPECT_FALSE(
      (tesseract_tests::vectorContainsType<sco::Cost::Ptr, trajopt::TrajOptCostFromErrFunc>(prob->getCosts())));
}

// This test checks that the terms are being added correctly for cartesian costs
TEST_F(TesseractPlanningTrajoptUnit, TrajoptArrayPlanner1)
{
  // Set the parameters (Most are being left as defaults)
  TrajOptArrayPlannerConfig config;
  // These are required
  config.link_ = "tool0";
  config.manipulator_ = "manipulator";
  config.tesseract_ = tesseract_ptr_;

  // These specify the series of points to be optimized
  for (int ind = 0; ind < NUM_STEPS; ind++)
  {
    CartesianWaypoint::Ptr waypoint = std::make_shared<CartesianWaypoint>(Eigen::Vector3d(-0.2 + ind * .02, 0.4, 0.8), Eigen::Quaterniond(0, 1.0, 0, 0));
    waypoint->setIsCritical(false);
    config.target_waypoints_.push_back(waypoint);
  }

  // Create test planner used for testing problem creation
  tesseract_tests::TrajOptArrayPlannerTest test_planner;
  test_planner.setConfiguration(config);

  trajopt::TrajOptProb::Ptr prob = test_planner.getProblem();
  EXPECT_FALSE((tesseract_tests::vectorContainsType<sco::Constraint::Ptr, trajopt::JointPosEqConstraint>(
      prob->getConstraints())));
  EXPECT_FALSE((tesseract_tests::vectorContainsType<sco::Constraint::Ptr, trajopt::TrajOptConstraintFromErrFunc>(
      prob->getConstraints())));
  EXPECT_FALSE((tesseract_tests::vectorContainsType<sco::Cost::Ptr, trajopt::JointPosEqCost>(prob->getCosts())));
  EXPECT_TRUE((tesseract_tests::vectorContainsType<sco::Cost::Ptr, trajopt::TrajOptCostFromErrFunc>(prob->getCosts())));
}

// This test checks that the terms are being added correctly for joint cnts
TEST_F(TesseractPlanningTrajoptUnit, TrajoptArrayPlanner2)
{
  // Set the parameters (Most are being left as defaults)
  TrajOptArrayPlannerConfig config;
  // These are required
  config.link_ = "tool0";
  config.manipulator_ = "manipulator";
  config.tesseract_ = tesseract_ptr_;

  std::vector<std::string> joint_names = tesseract_ptr_->getFwdKinematicsManagerConst()->getFwdKinematicSolver("manipulator")->getJointNames();

  // These specify the series of points to be optimized
  for (int ind = 0; ind < NUM_STEPS; ind++)
  {
    JointWaypoint::Ptr waypoint = std::make_shared<JointWaypoint>(std::vector<double>({0, 0, 0, -1.57 + ind * 0.1, 0, 0, 0}), joint_names);
    waypoint->setIsCritical(true);
    config.target_waypoints_.push_back(waypoint);
  }

  // Create test planner used for testing problem creation
  tesseract_tests::TrajOptArrayPlannerTest test_planner;
  test_planner.setConfiguration(config);

  trajopt::TrajOptProb::Ptr prob = test_planner.getProblem();
  EXPECT_TRUE((tesseract_tests::vectorContainsType<sco::Constraint::Ptr, trajopt::JointPosEqConstraint>(
      prob->getConstraints())));
  EXPECT_FALSE((tesseract_tests::vectorContainsType<sco::Constraint::Ptr, trajopt::TrajOptConstraintFromErrFunc>(
      prob->getConstraints())));
  EXPECT_FALSE((tesseract_tests::vectorContainsType<sco::Cost::Ptr, trajopt::JointPosEqCost>(prob->getCosts())));
  EXPECT_FALSE(
      (tesseract_tests::vectorContainsType<sco::Cost::Ptr, trajopt::TrajOptCostFromErrFunc>(prob->getCosts())));
}

// This test checks that the terms are being added correctly for joint costs
TEST_F(TesseractPlanningTrajoptUnit, TrajoptArrayPlanner3)
{
  // Set the parameters (Most are being left as defaults)
  TrajOptArrayPlannerConfig config;
  // These are required
  config.link_ = "tool0";
  config.manipulator_ = "manipulator";
  config.tesseract_ = tesseract_ptr_;

  std::vector<std::string> joint_names = tesseract_ptr_->getFwdKinematicsManagerConst()->getFwdKinematicSolver("manipulator")->getJointNames();

  // These specify the series of points to be optimized
  for (int ind = 0; ind < NUM_STEPS; ind++)
  {
    JointWaypoint::Ptr waypoint = std::make_shared<JointWaypoint>(std::vector<double>({0, 0, 0, -1.57 + ind * 0.1, 0, 0, 0}), joint_names);
    waypoint->setIsCritical(false);
    config.target_waypoints_.push_back(waypoint);
  }

  // Create test planner used for testing problem creation
  tesseract_tests::TrajOptArrayPlannerTest test_planner;
  test_planner.setConfiguration(config);

  trajopt::TrajOptProb::Ptr prob = test_planner.getProblem();
  EXPECT_FALSE((tesseract_tests::vectorContainsType<sco::Constraint::Ptr, trajopt::JointPosEqConstraint>(
      prob->getConstraints())));
  EXPECT_FALSE((tesseract_tests::vectorContainsType<sco::Constraint::Ptr, trajopt::TrajOptConstraintFromErrFunc>(
      prob->getConstraints())));
  EXPECT_TRUE((tesseract_tests::vectorContainsType<sco::Cost::Ptr, trajopt::JointPosEqCost>(prob->getCosts())));
  EXPECT_FALSE(
      (tesseract_tests::vectorContainsType<sco::Cost::Ptr, trajopt::TrajOptCostFromErrFunc>(prob->getCosts())));
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);

  return RUN_ALL_TESTS();
}
