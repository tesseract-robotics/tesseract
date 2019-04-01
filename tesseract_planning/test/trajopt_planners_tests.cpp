#include <gtest/gtest.h>

// These contain the definitions of the cost types
#include <trajopt/trajectory_costs.hpp>
#include <trajopt_sco/modeling.hpp>
#include <trajopt/collision_terms.hpp>

#include <tesseract_planning/trajopt/trajopt_freespace_planner.h>
#include <tesseract_planning/trajopt/trajopt_array_planner.h>
#include <urdf_parser/urdf_parser.h>

#include <tesseract_ros/kdl/kdl_env.h>
#include <ros/package.h>

const int NUM_STEPS = 7;

class TesseractPlanningTrajoptUnit : public ::testing::Test
{
protected:
  tesseract::tesseract_ros::KDLEnvPtr env = std::make_shared<tesseract::tesseract_ros::KDLEnv>();

  void SetUp() override
  {
    ros::Time::init();

    std::string path = ros::package::getPath("tesseract_ros");
    std::ifstream urdf_ifs(path + "/test/urdf/lbr_iiwa_14_r820.urdf");
    std::ifstream srdf_ifs(path + "/test/srdf/lbr_iiwa_14_r820.srdf");
    std::string urdf_xml_string((std::istreambuf_iterator<char>(urdf_ifs)), (std::istreambuf_iterator<char>()));
    std::string srdf_xml_string((std::istreambuf_iterator<char>(srdf_ifs)), (std::istreambuf_iterator<char>()));

    // Parse URDF/SRDF and create environment
    urdf::ModelInterfaceSharedPtr urdf_model = urdf::parseURDF(urdf_xml_string);
    srdf::ModelSharedPtr srdf_model = srdf::ModelSharedPtr(new srdf::Model);
    srdf_model->initString(*urdf_model, srdf_xml_string);
    assert(urdf_model != nullptr);
    assert(env != nullptr);
    bool success = env->init(urdf_model, srdf_model);
    assert(success);
  }
};

namespace tesseract_tests
{
/**
 * @brief Test class created to get access to the problem for testing
 */
class TrajOptFreespacePlannerTest : public tesseract::tesseract_planning::TrajOptFreespacePlanner
{
public:
  trajopt::TrajOptProbPtr getProblem(const tesseract::tesseract_planning::TrajOptFreespacePlannerConfig& config)
  {
    return generateProblem(config);
  }
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
  tesseract::tesseract_planning::TrajOptFreespacePlannerConfig config;
  // These are required
  config.link_ = "tool0";
  config.manipulator_ = "manipulator";
  config.kin_ = env->getManipulator(config.manipulator_);
  config.env_ = env;
  config.num_steps_ = NUM_STEPS;

  // Specify a JointWaypoint as the start
  tesseract::tesseract_planning::JointWaypointPtr start_waypoint =
      std::make_shared<tesseract::tesseract_planning::JointWaypoint>();
  Eigen::VectorXd joint_positions1(7);
  joint_positions1 << 0, 0, 0, -1.57, 0, 0, 0;
  start_waypoint->joint_positions_ = joint_positions1;
  config.start_waypoint_ = start_waypoint;

  // Specify a Joint Waypoint as the finish
  tesseract::tesseract_planning::JointWaypointPtr end_waypoint =
      std::make_shared<tesseract::tesseract_planning::JointWaypoint>();
  Eigen::VectorXd joint_positions2(7);
  joint_positions2 << 0, 0, 0, 1.57, 0, 0, 0;
  end_waypoint->joint_positions_ = joint_positions2;
  config.end_waypoint_ = end_waypoint;

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
    trajopt::TrajOptProbPtr prob = test_planner.getProblem(config);
    EXPECT_EQ((tesseract_tests::vectorContainsType<sco::CostPtr, trajopt::JointVelEqCost>(prob->getCosts())), t1);
    EXPECT_EQ((tesseract_tests::vectorContainsType<sco::CostPtr, trajopt::JointAccEqCost>(prob->getCosts())), t2);
    EXPECT_EQ((tesseract_tests::vectorContainsType<sco::CostPtr, trajopt::JointJerkEqCost>(prob->getCosts())), t3);
    EXPECT_EQ((tesseract_tests::vectorContainsType<sco::CostPtr, trajopt::CollisionCost>(prob->getCosts())), t4);
  }
}

// This test tests freespace motion b/n 2 joint waypoints
TEST_F(TesseractPlanningTrajoptUnit, TrajoptFreespacePlanner1)
{
  // Set the parameters (Most are being left as defaults)
  tesseract::tesseract_planning::TrajOptFreespacePlannerConfig config;
  // These are required
  config.link_ = "tool0";
  config.manipulator_ = "manipulator";
  config.kin_ = env->getManipulator(config.manipulator_);
  config.env_ = env;
  config.num_steps_ = NUM_STEPS;

  // Specify a JointWaypoint as the start
  tesseract::tesseract_planning::JointWaypointPtr start_waypoint =
      std::make_shared<tesseract::tesseract_planning::JointWaypoint>();
  Eigen::VectorXd joint_positions1(7);
  joint_positions1 << 0, 0, 0, -1.57, 0, 0, 0;
  start_waypoint->joint_positions_ = joint_positions1;
  config.start_waypoint_ = start_waypoint;

  // Specify a Joint Waypoint as the finish
  tesseract::tesseract_planning::JointWaypointPtr end_waypoint =
      std::make_shared<tesseract::tesseract_planning::JointWaypoint>();
  Eigen::VectorXd joint_positions2(7);
  joint_positions2 << 0, 0, 0, 1.57, 0, 0, 0;
  end_waypoint->joint_positions_ = joint_positions2;
  config.end_waypoint_ = end_waypoint;

  // Create test planner used for testing problem creation
  tesseract_tests::TrajOptFreespacePlannerTest test_planner;
  {
    config.start_waypoint_->is_critical_ = true;
    config.end_waypoint_->is_critical_ = true;
    trajopt::TrajOptProbPtr prob = test_planner.getProblem(config);
    EXPECT_TRUE((tesseract_tests::vectorContainsType<sco::ConstraintPtr, trajopt::JointPosEqConstraint>(
        prob->getConstraints())));
    EXPECT_FALSE((tesseract_tests::vectorContainsType<sco::ConstraintPtr, trajopt::TrajOptConstraintFromErrFunc>(
        prob->getConstraints())));
  }
  {
    config.start_waypoint_->is_critical_ = false;
    config.end_waypoint_->is_critical_ = false;
    trajopt::TrajOptProbPtr prob = test_planner.getProblem(config);
    EXPECT_TRUE((tesseract_tests::vectorContainsType<sco::CostPtr, trajopt::JointPosEqCost>(prob->getCosts())));
    EXPECT_FALSE(
        (tesseract_tests::vectorContainsType<sco::CostPtr, trajopt::TrajOptCostFromErrFunc>(prob->getCosts())));
  }
}

// This test tests freespace motion b/n 1 joint waypoint and 1 cartesian waypoint
TEST_F(TesseractPlanningTrajoptUnit, TrajoptFreespacePlanner2)
{
  // Create the planner and the responses that will store the results
  tesseract::tesseract_planning::TrajOptFreespacePlanner planner;
  tesseract::tesseract_planning::PlannerResponse planning_response;

  // Set the parameters (Most are being left as defaults)
  tesseract::tesseract_planning::TrajOptFreespacePlannerConfig config;
  // These are required
  config.link_ = "tool0";
  config.manipulator_ = "manipulator";
  config.kin_ = env->getManipulator(config.manipulator_);
  config.env_ = env;
  config.num_steps_ = NUM_STEPS;

  // Specify a Joint Waypoint as the start
  tesseract::tesseract_planning::JointWaypointPtr start_waypoint =
      std::make_shared<tesseract::tesseract_planning::JointWaypoint>();
  Eigen::VectorXd joint_positions1(7);
  joint_positions1 << 0, 0, 0, -1.57, 0, 0, 0;
  start_waypoint->joint_positions_ = joint_positions1;
  config.start_waypoint_ = start_waypoint;

  // Specify a Cartesian Waypoint as the finish
  tesseract::tesseract_planning::CartesianWaypointPtr end_waypoint =
      std::make_shared<tesseract::tesseract_planning::CartesianWaypoint>();
  end_waypoint->cartesian_position_.translation() = Eigen::Vector3d(-.20, .4, 0.2);
  end_waypoint->cartesian_position_.linear() = Eigen::Quaterniond(0, 0, 1.0, 0).toRotationMatrix();
  config.end_waypoint_ = end_waypoint;

  // Create test planner used for testing problem creation
  tesseract_tests::TrajOptFreespacePlannerTest test_planner;
  {
    config.start_waypoint_->is_critical_ = true;
    config.end_waypoint_->is_critical_ = true;
    trajopt::TrajOptProbPtr prob = test_planner.getProblem(config);
    EXPECT_TRUE((tesseract_tests::vectorContainsType<sco::ConstraintPtr, trajopt::JointPosEqConstraint>(
        prob->getConstraints())));
    EXPECT_TRUE((tesseract_tests::vectorContainsType<sco::ConstraintPtr, trajopt::TrajOptConstraintFromErrFunc>(
        prob->getConstraints())));
  }
  {
    config.start_waypoint_->is_critical_ = false;
    config.end_waypoint_->is_critical_ = false;
    trajopt::TrajOptProbPtr prob = test_planner.getProblem(config);
    EXPECT_TRUE((tesseract_tests::vectorContainsType<sco::CostPtr, trajopt::JointPosEqCost>(prob->getCosts())));
    EXPECT_TRUE((tesseract_tests::vectorContainsType<sco::CostPtr, trajopt::TrajOptCostFromErrFunc>(prob->getCosts())));
  }
}

// This test tests freespace motion b/n 1 cartesian waypoint and 1 joint waypoint
TEST_F(TesseractPlanningTrajoptUnit, TrajoptFreespacePlanner3)
{
  // Create the planner and the responses that will store the results
  tesseract::tesseract_planning::TrajOptFreespacePlanner planner;
  tesseract::tesseract_planning::PlannerResponse planning_response;

  // Set the parameters (Most are being left as defaults)
  tesseract::tesseract_planning::TrajOptFreespacePlannerConfig config;
  // These are required
  config.link_ = "tool0";
  config.manipulator_ = "manipulator";
  config.kin_ = env->getManipulator(config.manipulator_);
  config.env_ = env;
  config.num_steps_ = NUM_STEPS;

  // Specify a Cartesian Waypoint as the start
  tesseract::tesseract_planning::CartesianWaypointPtr start_waypoint =
      std::make_shared<tesseract::tesseract_planning::CartesianWaypoint>();
  start_waypoint->cartesian_position_.translation() = Eigen::Vector3d(-.20, .4, 0.2);
  start_waypoint->cartesian_position_.linear() = Eigen::Quaterniond(0, 0, 1.0, 0).toRotationMatrix();
  config.start_waypoint_ = start_waypoint;

  // Specify a Joint Waypoint as the finish
  tesseract::tesseract_planning::JointWaypointPtr end_waypoint =
      std::make_shared<tesseract::tesseract_planning::JointWaypoint>();
  Eigen::VectorXd joint_positions1(7);
  joint_positions1 << 0, 0, 0, -1.57, 0, 0, 0;
  end_waypoint->joint_positions_ = joint_positions1;
  config.end_waypoint_ = end_waypoint;

  // Create test planner used for testing problem creation
  tesseract_tests::TrajOptFreespacePlannerTest test_planner;
  {
    config.start_waypoint_->is_critical_ = true;
    config.end_waypoint_->is_critical_ = true;
    trajopt::TrajOptProbPtr prob = test_planner.getProblem(config);
    EXPECT_TRUE((tesseract_tests::vectorContainsType<sco::ConstraintPtr, trajopt::JointPosEqConstraint>(
        prob->getConstraints())));
    EXPECT_TRUE((tesseract_tests::vectorContainsType<sco::ConstraintPtr, trajopt::TrajOptConstraintFromErrFunc>(
        prob->getConstraints())));
  }
  {
    config.start_waypoint_->is_critical_ = false;
    config.end_waypoint_->is_critical_ = false;
    trajopt::TrajOptProbPtr prob = test_planner.getProblem(config);
    EXPECT_TRUE((tesseract_tests::vectorContainsType<sco::CostPtr, trajopt::JointPosEqCost>(prob->getCosts())));
    EXPECT_TRUE((tesseract_tests::vectorContainsType<sco::CostPtr, trajopt::TrajOptCostFromErrFunc>(prob->getCosts())));
  }
}

// This test tests freespace motion b/n 2 cartesian waypoints
TEST_F(TesseractPlanningTrajoptUnit, TrajoptFreespacePlanner4)
{
  // Create the planner and the responses that will store the results
  tesseract::tesseract_planning::TrajOptFreespacePlanner planner;
  tesseract::tesseract_planning::PlannerResponse planning_response;

  // Set the parameters (Most are being left as defaults)
  tesseract::tesseract_planning::TrajOptFreespacePlannerConfig config;
  // These are required
  config.link_ = "tool0";
  config.manipulator_ = "manipulator";
  config.kin_ = env->getManipulator(config.manipulator_);
  config.env_ = env;
  config.num_steps_ = NUM_STEPS;

  // Specify a Cartesian Waypoint as the start
  tesseract::tesseract_planning::CartesianWaypointPtr start_waypoint =
      std::make_shared<tesseract::tesseract_planning::CartesianWaypoint>();
  start_waypoint->cartesian_position_.translation() = Eigen::Vector3d(-.20, .4, 0.2);
  start_waypoint->cartesian_position_.linear() = Eigen::Quaterniond(0, 0, 1.0, 0).toRotationMatrix();
  config.start_waypoint_ = start_waypoint;

  // Specify a Cartesian Waypoint as the finish
  tesseract::tesseract_planning::CartesianWaypointPtr end_waypoint =
      std::make_shared<tesseract::tesseract_planning::CartesianWaypoint>();
  end_waypoint->cartesian_position_.translation() = Eigen::Vector3d(-.20, .4, 0.2);
  end_waypoint->cartesian_position_.linear() = Eigen::Quaterniond(0, 0, 1.0, 0).toRotationMatrix();
  config.end_waypoint_ = end_waypoint;

  // Create test planner used for testing problem creation
  tesseract_tests::TrajOptFreespacePlannerTest test_planner;
  {
    config.start_waypoint_->is_critical_ = true;
    config.end_waypoint_->is_critical_ = true;
    trajopt::TrajOptProbPtr prob = test_planner.getProblem(config);
    EXPECT_FALSE((tesseract_tests::vectorContainsType<sco::ConstraintPtr, trajopt::JointPosEqConstraint>(
        prob->getConstraints())));
    EXPECT_TRUE((tesseract_tests::vectorContainsType<sco::ConstraintPtr, trajopt::TrajOptConstraintFromErrFunc>(
        prob->getConstraints())));
  }
  {
    config.start_waypoint_->is_critical_ = false;
    config.end_waypoint_->is_critical_ = false;
    trajopt::TrajOptProbPtr prob = test_planner.getProblem(config);
    EXPECT_FALSE((tesseract_tests::vectorContainsType<sco::CostPtr, trajopt::JointPosEqCost>(prob->getCosts())));
    EXPECT_TRUE((tesseract_tests::vectorContainsType<sco::CostPtr, trajopt::TrajOptCostFromErrFunc>(prob->getCosts())));
  }
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);

  return RUN_ALL_TESTS();
}
