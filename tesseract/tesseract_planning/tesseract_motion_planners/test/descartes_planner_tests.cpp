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

#include <tesseract_motion_planners/descartes/descartes_motion_planner.h>
#include <tesseract_motion_planners/descartes/utils.h>
#include <tesseract_motion_planners/core/utils.h>
#include <opw_kinematics/opw_parameters.h>
#include <opw_kinematics/opw_kinematics.h>
#include <descartes_opw/descartes_opw_kinematics.h>
#include <descartes_tesseract/descartes_tesseract_collision_checker.h>
#include <descartes_samplers/evaluators/euclidean_distance_edge_evaluator.h>
#include <descartes_samplers/samplers/axial_symmetric_sampler.h>
#include <urdf_parser/urdf_parser.h>

const int NUM_STEPS = 200;

using namespace tesseract;
using namespace tesseract_environment;
using namespace tesseract_scene_graph;
using namespace tesseract_collision;
using namespace tesseract_motion_planners;
using namespace opw_kinematics;
using namespace descartes_light;
using namespace descartes_core;

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

std::vector<descartes_core::TimingConstraintD> makeTiming(const std::vector<tesseract_motion_planners::Waypoint::Ptr>& path, const double dt)
{
  std::vector<descartes_core::TimingConstraintD> timing(path.size(), dt);
  // TODO(jmeyer): Compute the real time
  // In Descartes land, the timing constraint represents how long the dt is between the previous point and the point
  // associated with this particular constraint. In a trajectory with only one pass the first point is meaningless (?).
  // Here I want to append many passes together so setting the DT to 0.0 is sort of saying: "Hey, take as long
  // as you need to get to here from the last point".

  return timing;
}


DescartesMotionPlannerConfigD createDescartesPlannerConfig(const opw_kinematics::Parameters<double>& params,
                                                           const tesseract::Tesseract::ConstPtr tesseract_ptr,
                                                           const std::string manip,
                                                           const tesseract_kinematics::ForwardKinematics::ConstPtr& kin,
                                                           const Eigen::Isometry3d& tcp,
                                                           const tesseract_environment::EnvState::ConstPtr& current_state,
                                                           const std::vector<tesseract_motion_planners::Waypoint::Ptr> waypoints)
{
  descartes_light::IsValidFn<double> is_valid_fn = std::bind(&descartes_light::isWithinLimits<double>, std::placeholders::_1, kin->getLimits());
  descartes_light::GetRedundantSolutionsFn<double> get_redundant_sol_fn = std::bind(&descartes_light::getRedundantSolutions<double>, std::placeholders::_1, kin->getLimits());

  auto robot_kin = std::make_shared<OPWKinematics<double>>(params, current_state->transforms.at("base_link"), tcp.cast<double>(), is_valid_fn, get_redundant_sol_fn);

  // OPWRailedKinematics expects the joint to be auxiliary axes followed robot joints
  tesseract_environment::AdjacencyMap::Ptr adjacency_map = std::make_shared<tesseract_environment::AdjacencyMap>(tesseract_ptr->getEnvironmentConst()->getSceneGraph(),
                                                                                                                 kin->getActiveLinkNames(),
                                                                                                                 current_state->transforms);
  typename descartes_light::CollisionInterface<double>::Ptr coll_interface = std::make_shared<descartes_light::TesseractCollision<double>>(tesseract_ptr->getEnvironmentConst(), adjacency_map->getActiveLinkNames(), kin->getJointNames());


  // Create Timing Constraint
  std::vector<descartes_core::TimingConstraint<double>> timing = makeTiming(waypoints, std::numeric_limits<double>::max());

  // Create Edge Evaluator
  typename descartes_light::EdgeEvaluator<double>::Ptr edge_computer = std::make_shared<descartes_light::EuclideanDistanceEdgeEvaluator<double>>(kin->numJoints());
  std::vector<typename descartes_light::PositionSampler<double>::Ptr> position_samplers = makeRobotPositionSamplers<double>(waypoints, robot_kin, coll_interface, 60 * M_PI / 180);

  return DescartesMotionPlannerConfigD(tesseract_ptr, manip, coll_interface, robot_kin, edge_computer, timing, position_samplers, waypoints);
}

class TesseractPlanningDescartesUnit : public ::testing::Test
{
protected:
  Tesseract::Ptr tesseract_ptr_;
  opw_kinematics::Parameters<double> opw_params_;

  void SetUp() override
  {
    ResourceLocatorFn locator = locateResource;
    Tesseract::Ptr tesseract = std::make_shared<Tesseract>();
    boost::filesystem::path urdf_path(std::string(TESSERACT_SUPPORT_DIR) + "/urdf/abb_irb2400.urdf");
    boost::filesystem::path srdf_path(std::string(TESSERACT_SUPPORT_DIR) + "/urdf/abb_irb2400.srdf");
    EXPECT_TRUE(tesseract->init(urdf_path, srdf_path, locator));
    tesseract_ptr_ = tesseract;

    opw_params_.a1 = (0.100);
    opw_params_.a2 = (-0.135);
    opw_params_.b =  (0.000);
    opw_params_.c1 = (0.615);
    opw_params_.c2 = (0.705);
    opw_params_.c3 = (0.755);
    opw_params_.c4 = (0.085);

    opw_params_.offsets[2] = -M_PI / 2.0;
  }
};

TEST_F(TesseractPlanningDescartesUnit, DescartesPlannerFixedPoses)
{
  // These specify the series of points to be optimized
  std::vector<tesseract_motion_planners::Waypoint::Ptr> waypoints;
  Eigen::VectorXd x = Eigen::VectorXd::LinSpaced(NUM_STEPS, -0.2, 0.2);
  for (int i = 0; i < x.size(); ++i)
  {
    CartesianWaypoint::Ptr waypoint = std::make_shared<CartesianWaypoint>(Eigen::Vector3d(0.8, x[i], 0.8),
                                                                          Eigen::Quaterniond(0, 0, -1.0, 0));
    waypoint->setIsCritical(true);
    waypoints.push_back(waypoint);
  }

  auto robot_kin = tesseract_ptr_->getFwdKinematicsManagerConst()->getFwdKinematicSolver("manipulator");
  auto current_state = tesseract_ptr_->getEnvironmentConst()->getCurrentState();
  DescartesMotionPlannerConfigD config = createDescartesPlannerConfig(opw_params_, tesseract_ptr_, "manipulator", robot_kin, Eigen::Isometry3d::Identity(), current_state, waypoints);
  config.num_threads = 1;
  DescartesMotionPlanner<double> single_descartes_planner;
  PlannerResponse single_planner_response;
  single_descartes_planner.setConfiguration(config);
  auto single_status = single_descartes_planner.solve(single_planner_response);
  EXPECT_TRUE(single_status);

  for (int i = 0; i < 10; ++i)
  {
    config.num_threads = descartes_light::SolverD::getMaxThreads();
    DescartesMotionPlanner<double> descartes_planner;
    PlannerResponse planner_response;
    descartes_planner.setConfiguration(config);
    auto status = descartes_planner.solve(planner_response);
    EXPECT_TRUE(status);
    for (int j = 0; j < static_cast<int>(waypoints.size()); ++j)
    {
      EXPECT_TRUE(single_planner_response.joint_trajectory.trajectory.row(j).isApprox(planner_response.joint_trajectory.trajectory.row(j), 1e-5));
    }
  }
}

TEST_F(TesseractPlanningDescartesUnit, DescartesPlannerAxialSymetric)
{
  // These specify the series of points to be optimized
  std::vector<tesseract_motion_planners::Waypoint::Ptr> waypoints;
  Eigen::VectorXd x = Eigen::VectorXd::LinSpaced(NUM_STEPS, -0.2, 0.2);
  for (int i = 0; i < x.size(); ++i)
  {
    CartesianWaypoint::Ptr waypoint = std::make_shared<CartesianWaypoint>(Eigen::Vector3d(0.8, x[i], 0.8),
                                                                          Eigen::Quaterniond(0, 0, -1.0, 0));
    waypoint->setIsCritical(true);
    Eigen::VectorXd c(6);
    c << 1, 1, 1, 1, 1, 0;
    waypoint->setCoefficients(c);
    waypoints.push_back(waypoint);
  }

  auto robot_kin = tesseract_ptr_->getFwdKinematicsManagerConst()->getFwdKinematicSolver("manipulator");
  auto current_state = tesseract_ptr_->getEnvironmentConst()->getCurrentState();
  DescartesMotionPlannerConfigD config = createDescartesPlannerConfig(opw_params_, tesseract_ptr_, "manipulator", robot_kin, Eigen::Isometry3d::Identity(), current_state, waypoints);
  config.num_threads = 1;
  DescartesMotionPlanner<double> single_descartes_planner;
  PlannerResponse single_planner_response;
  single_descartes_planner.setConfiguration(config);
  auto single_status = single_descartes_planner.solve(single_planner_response);
  EXPECT_TRUE(single_status);

  for (int i = 0; i < 10; ++i)
  {
    config.num_threads = descartes_light::SolverD::getMaxThreads();
    DescartesMotionPlanner<double> descartes_planner;
    PlannerResponse planner_response;
    descartes_planner.setConfiguration(config);
    auto status = descartes_planner.solve(planner_response);
    EXPECT_TRUE(status);
    for (int j = 0; j < static_cast<int>(waypoints.size()); ++j)
    {
      EXPECT_TRUE(single_planner_response.joint_trajectory.trajectory.row(j).isApprox(planner_response.joint_trajectory.trajectory.row(j), 1e-5));
    }
  }
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);

  return RUN_ALL_TESTS();
}
