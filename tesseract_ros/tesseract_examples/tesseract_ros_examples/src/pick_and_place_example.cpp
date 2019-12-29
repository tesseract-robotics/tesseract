/**
 * @file pick_and_place_example.cpp
 * @brief Pick and place implementation
 *
 * @author Levi Armstrong
 * @date July 22, 2019
 * @version TODO
 * @bug No known bugs
 *
 * @copyright Copyright (c) 2017, Southwest Research Institute
 *
 * @par License
 * Software License Agreement (Apache License)
 * @par
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 * http://www.apache.org/licenses/LICENSE-2.0
 * @par
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */
#include <tesseract_common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <jsoncpp/json/json.h>
#include <ros/ros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract_ros_examples/pick_and_place_example.h>
#include <tesseract_motion_planners/trajopt/config/trajopt_planner_config.h>
#include <tesseract_motion_planners/trajopt/trajopt_motion_planner.h>
#include <tesseract_rosutils/plotting.h>
#include <tesseract_rosutils/utils.h>
#include <tesseract_msgs/ModifyEnvironment.h>
#include <tesseract_msgs/GetEnvironmentChanges.h>
#include <trajopt/plot_callback.hpp>
#include <trajopt/file_write_callback.hpp>
#include <trajopt/problem_description.hpp>
#include <trajopt_utils/config.hpp>
#include <trajopt_utils/logging.hpp>

using namespace trajopt;
using namespace tesseract;
using namespace tesseract_environment;
using namespace tesseract_kinematics;
using namespace tesseract_scene_graph;
using namespace tesseract_collision;
using namespace tesseract_rosutils;

const std::string ROBOT_DESCRIPTION_PARAM = "robot_description"; /**< Default ROS parameter for robot description */
const std::string ROBOT_SEMANTIC_PARAM = "robot_description_semantic"; /**< Default ROS parameter for robot
                                                                          description */
const std::string GET_ENVIRONMENT_CHANGES_SERVICE = "get_tesseract_changes_rviz";
const std::string MODIFY_ENVIRONMENT_SERVICE = "modify_tesseract_rviz";
const bool ENABLE_TIME_COST = false;
const bool ENABLE_VELOCITY_COST = false;
const double OFFSET = 0.005;

namespace tesseract_ros_examples
{
bool PickAndPlaceExample::run()
{
  // Set Log Level
  util::gLogLevel = util::LevelError;

  /////////////
  /// SETUP ///
  /////////////

  // Pull ROS params
  std::string urdf_xml_string, srdf_xml_string, box_parent_link;
  double box_side, box_x, box_y;
  nh_.getParam(ROBOT_DESCRIPTION_PARAM, urdf_xml_string);
  nh_.getParam(ROBOT_SEMANTIC_PARAM, srdf_xml_string);
  nh_.getParam("box_side", box_side);
  nh_.getParam("box_x", box_x);
  nh_.getParam("box_y", box_y);
  nh_.getParam("box_parent_link", box_parent_link);

  // Initialize the environment
  ResourceLocator::Ptr locator = std::make_shared<tesseract_rosutils::ROSResourceLocator>();
  if (!tesseract_->init(urdf_xml_string, srdf_xml_string, locator))
    return false;

  // Create plotting tool
  tesseract_rosutils::ROSPlottingPtr plotter =
      std::make_shared<tesseract_rosutils::ROSPlotting>(tesseract_->getEnvironment());

  if (rviz_)
  {
    // These are used to keep visualization updated
    modify_env_rviz_ = nh_.serviceClient<tesseract_msgs::ModifyEnvironment>(MODIFY_ENVIRONMENT_SERVICE, false);
    get_env_changes_rviz_ =
        nh_.serviceClient<tesseract_msgs::GetEnvironmentChanges>(GET_ENVIRONMENT_CHANGES_SERVICE, false);

    // Check RViz to make sure nothing has changed
    if (!checkRviz())
      return false;
  }
  sleep(20);
  // Set the initial state of the robot
  std::unordered_map<std::string, double> joint_states;
  joint_states["iiwa_joint_1"] = 0.0;
  joint_states["iiwa_joint_2"] = 0.0;
  joint_states["iiwa_joint_3"] = 0.0;
  joint_states["iiwa_joint_4"] = -1.57;
  joint_states["iiwa_joint_5"] = 0.0;
  joint_states["iiwa_joint_6"] = 0.0;
  joint_states["iiwa_joint_7"] = 0.0;
  tesseract_->getEnvironment()->setState(joint_states);

  // Add simulated box to environment
  Link link_box("box");

  Visual::Ptr visual = std::make_shared<Visual>();
  visual->origin = Eigen::Isometry3d::Identity();
  visual->geometry = std::make_shared<tesseract_geometry::Box>(box_side, box_side, box_side);
  link_box.visual.push_back(visual);

  Collision::Ptr collision = std::make_shared<Collision>();
  collision->origin = visual->origin;
  collision->geometry = visual->geometry;
  link_box.collision.push_back(collision);

  Joint joint_box("joint_box");
  joint_box.parent_link_name = box_parent_link;
  joint_box.child_link_name = link_box.getName();
  joint_box.type = JointType::FIXED;
  joint_box.parent_to_joint_origin_transform = Eigen::Isometry3d::Identity();
  joint_box.parent_to_joint_origin_transform.translation() += Eigen::Vector3d(box_x, box_y, (box_side / 2.0) + OFFSET);

  tesseract_->getEnvironment()->addLink(link_box, joint_box);

  if (rviz_)
  {
    // Now update rviz environment
    if (!sendRvizChanges(0))
      return false;
  }

  ////////////
  /// PICK ///
  ////////////

  if (rviz_)
  {
    ROS_ERROR("Press enter to continue");
    std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
  }

  // Choose the manipulator and end effector link
  std::string manip = "Manipulator";
  std::string end_effector = "iiwa_link_ee";

  // Define the final pose (on top of the box)
  Eigen::Isometry3d final_pose;
  Eigen::Quaterniond orientation(0.0, 0.0, 1.0, 0.0);
  final_pose.linear() = orientation.matrix();
  final_pose.translation() += Eigen::Vector3d(box_x, box_y, box_side + 0.77153 + OFFSET);  // Offset for the table

  // Define the approach pose
  Eigen::Isometry3d approach_pose = final_pose;
  approach_pose.translation() += Eigen::Vector3d(0.0, 0.0, 0.15);

  // Create the problem construction info
  trajopt::ProblemConstructionInfo pci(tesseract_);

  pci.basic_info.n_steps = steps_ * 2;
  pci.basic_info.manip = manip;
  pci.basic_info.dt_lower_lim = 2;    // 1/most time
  pci.basic_info.dt_upper_lim = 100;  // 1/least time
  pci.basic_info.start_fixed = true;
  pci.basic_info.use_time = false;

  // Create Kinematic Object
  pci.kin = pci.getManipulator(pci.basic_info.manip);

  pci.init_info.type = trajopt::InitInfo::STATIONARY;
  pci.init_info.dt = 0.5;

  // Add a collision cost
  {
    auto collision = std::make_shared<trajopt::CollisionTermInfo>();
    collision->name = "collision";
    collision->term_type = trajopt::TT_COST;
    collision->continuous = true;
    collision->first_step = 1;
    collision->last_step = pci.basic_info.n_steps - 1;
    collision->info = trajopt::createSafetyMarginDataVector(pci.basic_info.n_steps, 0.005, 50);
    pci.cost_infos.push_back(collision);
  }

  // Add a velocity cost without time to penalize paths that are longer
  {
    std::shared_ptr<trajopt::JointVelTermInfo> jv(new trajopt::JointVelTermInfo);
    jv->targets = std::vector<double>(7, 0.0);
    jv->coeffs = std::vector<double>(7, 5.0);
    jv->term_type = trajopt::TT_COST;
    jv->first_step = 0;
    jv->last_step = pci.basic_info.n_steps - 1;
    jv->name = "joint_velocity_cost";
    pci.cost_infos.push_back(jv);
  }

  // Add a velocity cnt with time to insure that robot dynamics are obeyed
  if (ENABLE_VELOCITY_COST)
  {
    std::shared_ptr<trajopt::JointVelTermInfo> jv(new trajopt::JointVelTermInfo);

    // Taken from iiwa documentation (radians/s) and scaled by 0.8
    std::vector<double> vel_lower_lim{ 1.71 * -0.8, 1.71 * -0.8, 1.75 * -0.8, 2.27 * -0.8,
                                       2.44 * -0.8, 3.14 * -0.8, 3.14 * -0.8 };
    std::vector<double> vel_upper_lim{ 1.71 * 0.8, 1.71 * 0.8, 1.75 * 0.8, 2.27 * 0.8,
                                       2.44 * 0.8, 3.14 * 0.8, 3.14 * 0.8 };

    jv->targets = std::vector<double>(7, 0.0);
    jv->coeffs = std::vector<double>(7, 50.0);
    jv->lower_tols = vel_lower_lim;
    jv->upper_tols = vel_upper_lim;
    jv->term_type = (trajopt::TT_CNT | trajopt::TT_USE_TIME);
    jv->first_step = 0;
    jv->last_step = pci.basic_info.n_steps - 1;
    jv->name = "joint_velocity_cnt";
    pci.cnt_infos.push_back(jv);
  }

  // Add cartesian pose cnt at the approach point
  {
    Eigen::Quaterniond rotation(approach_pose.linear());
    auto pose_constraint = std::make_shared<trajopt::CartPoseTermInfo>();
    pose_constraint->term_type = trajopt::TT_CNT;
    pose_constraint->link = end_effector;
    pose_constraint->timestep = steps_;
    pose_constraint->xyz = approach_pose.translation();

    pose_constraint->wxyz = Eigen::Vector4d(rotation.w(), rotation.x(), rotation.y(), rotation.z());
    pose_constraint->pos_coeffs = Eigen::Vector3d(10.0, 10.0, 10.0);
    pose_constraint->rot_coeffs = Eigen::Vector3d(10.0, 10.0, 10.0);
    pose_constraint->name = "pose_" + std::to_string(steps_);
    pci.cnt_infos.push_back(pose_constraint);
  }

  // Add cartesian pose cnt at the final point
  {
    Eigen::Quaterniond rotation(final_pose.linear());
    auto pose_constraint = std::make_shared<trajopt::CartPoseTermInfo>();
    pose_constraint->term_type = trajopt::TT_CNT;
    pose_constraint->link = end_effector;
    pose_constraint->timestep = 2 * steps_ - 1;
    pose_constraint->xyz = final_pose.translation();

    pose_constraint->wxyz = Eigen::Vector4d(rotation.w(), rotation.x(), rotation.y(), rotation.z());
    pose_constraint->pos_coeffs = Eigen::Vector3d(10.0, 10.0, 10.0);
    pose_constraint->rot_coeffs = Eigen::Vector3d(10.0, 10.0, 10.0);
    pose_constraint->name = "pose_" + std::to_string(2 * steps_ - 1);
    pci.cnt_infos.push_back(pose_constraint);
  }

  // Add a cost on the total time to complete the pick
  if (ENABLE_TIME_COST)
  {
    std::shared_ptr<trajopt::TotalTimeTermInfo> time_cost(new trajopt::TotalTimeTermInfo);
    time_cost->name = "time_cost";
    time_cost->coeff = 5.0;
    time_cost->limit = 0.0;
    time_cost->term_type = trajopt::TT_COST;
    pci.cost_infos.push_back(time_cost);
  }

  // Create the pick problem
  trajopt::TrajOptProb::Ptr pick_prob = ConstructProblem(pci);

  // Set the optimization parameters (Most are being left as defaults)
  tesseract_motion_planners::TrajOptPlannerConfig config(pick_prob);
  config.params.max_iter = 100;

  // Create Plot Callback
  if (plotting_)
  {
    config.callbacks.push_back(PlotCallback(*pick_prob, plotter));
  }

  // Create file write callback discarding any of the file's current contents
  std::shared_ptr<std::ofstream> stream_ptr(new std::ofstream);
  if (write_to_file_)
  {
    std::string path = ros::package::getPath("tesseract_ros_examples") + "/file_output_pick.csv";
    stream_ptr->open(path, std::ofstream::out | std::ofstream::trunc);
    config.callbacks.push_back(trajopt::WriteCallback(stream_ptr, pick_prob));
  }

  // Create the planner and the responses that will store the results
  tesseract_motion_planners::TrajOptMotionPlanner planner;
  tesseract_motion_planners::PlannerResponse planning_response;
  tesseract_motion_planners::PlannerResponse planning_response_place;

  // Set Planner Configuration
  planner.setConfiguration(std::make_shared<tesseract_motion_planners::TrajOptPlannerConfig>(config));

  // Solve problem. Results are stored in the response
  planner.solve(planning_response);

  if (write_to_file_)
    stream_ptr->close();

  // Plot the resulting trajectory
  if (plotting_)
    plotter->plotTrajectory(pick_prob->GetKin()->getJointNames(),
                            planning_response.joint_trajectory.trajectory.leftCols(
                                static_cast<long>(pick_prob->GetKin()->getJointNames().size())));

  std::cout << planning_response.joint_trajectory.trajectory << '\n';

  /////////////
  /// PLACE ///
  /////////////

  if (rviz_)
  {
    ROS_ERROR("Press enter to continue");
    std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
  }

  // Detach the simulated box from the world and attach to the end effector
  Joint joint_box2("joint_box2");
  joint_box2.parent_link_name = end_effector;
  joint_box2.child_link_name = link_box.getName();
  joint_box2.type = JointType::FIXED;
  joint_box2.parent_to_joint_origin_transform = Eigen::Isometry3d::Identity();
  joint_box2.parent_to_joint_origin_transform.translation() += Eigen::Vector3d(0, 0, box_side / 2.0);

  tesseract_->getEnvironment()->moveLink(joint_box2);
  tesseract_->getEnvironment()->addAllowedCollision(link_box.getName(), "iiwa_link_ee", "Never");
  tesseract_->getEnvironment()->addAllowedCollision(link_box.getName(), "iiwa_link_7", "Never");
  tesseract_->getEnvironment()->addAllowedCollision(link_box.getName(), "iiwa_link_6", "Never");
  tesseract_->getEnvironment()->addAllowedCollision(link_box.getName(), end_effector, "Adjacent");

  if (rviz_)
  {
    // Now update rviz environment
    if (!sendRvizChanges(1))
      return false;
  }

  // Set the current state to the last state of the pick trajectory
  tesseract_->getEnvironment()->setState(planning_response.joint_trajectory.joint_names,
                                         planning_response.joint_trajectory.trajectory.bottomRows(1).transpose());

  // Retreat to the approach pose
  Eigen::Isometry3d retreat_pose = approach_pose;

  // Define some place locations.
  Eigen::Isometry3d bottom_right_shelf, bottom_left_shelf, middle_right_shelf, middle_left_shelf, top_right_shelf,
      top_left_shelf;
  bottom_right_shelf.linear() = Eigen::Quaterniond(0, 0, 0.7071068, 0.7071068).matrix();
  bottom_right_shelf.translation() = Eigen::Vector3d(0.148856, 0.73085, 0.906);
  bottom_left_shelf.linear() = Eigen::Quaterniond(0, 0, 0.7071068, 0.7071068).matrix();
  bottom_left_shelf.translation() = Eigen::Vector3d(-0.148856, 0.73085, 0.906);
  middle_right_shelf.linear() = Eigen::Quaterniond(0, 0, 0.7071068, 0.7071068).matrix();
  middle_right_shelf.translation() = Eigen::Vector3d(0.148856, 0.73085, 1.16);
  middle_left_shelf.linear() = Eigen::Quaterniond(0, 0, 0.7071068, 0.7071068).matrix();
  middle_left_shelf.translation() = Eigen::Vector3d(-0.148856, 0.73085, 1.16);
  top_right_shelf.linear() = Eigen::Quaterniond(0, 0, 0.7071068, 0.7071068).matrix();
  top_right_shelf.translation() = Eigen::Vector3d(0.148856, 0.73085, 1.414);
  top_left_shelf.linear() = Eigen::Quaterniond(0, 0, 0.7071068, 0.7071068).matrix();
  top_left_shelf.translation() = Eigen::Vector3d(-0.148856, 0.73085, 1.414);

  // Set the target pose to middle_left_shelf
  final_pose = middle_left_shelf;

  // Setup approach for place move
  approach_pose = final_pose;
  approach_pose.translation() += Eigen::Vector3d(0.0, -0.25, 0);

  // Create the problem construction info
  trajopt::ProblemConstructionInfo pci_place(tesseract_);

  pci_place.basic_info.n_steps = steps_ * 3;
  pci_place.basic_info.manip = manip;
  pci_place.basic_info.dt_lower_lim = 2;    // 1/most time
  pci_place.basic_info.dt_upper_lim = 100;  // 1/least time
  pci_place.basic_info.start_fixed = true;
  pci_place.basic_info.use_time = false;

  // Create Kinematic Object
  pci_place.kin = pci_place.getManipulator(pci_place.basic_info.manip);

  pci_place.init_info.type = trajopt::InitInfo::STATIONARY;
  pci_place.init_info.dt = 0.5;

  // Add a collision cost
  {
    auto collision = std::make_shared<trajopt::CollisionTermInfo>();
    collision->name = "collision";
    collision->term_type = trajopt::TT_COST;
    collision->continuous = true;
    collision->first_step = 1;
    collision->last_step = pci_place.basic_info.n_steps - 1;
    collision->info = trajopt::createSafetyMarginDataVector(pci_place.basic_info.n_steps, 0.005, 50);
    pci_place.cost_infos.push_back(collision);
  }

  // Add a velocity cost without time to penalize paths that are longer
  {
    auto jv = std::make_shared<trajopt::JointVelTermInfo>();
    jv->targets = std::vector<double>(7, 0.0);
    jv->coeffs = std::vector<double>(7, 5.0);
    jv->term_type = trajopt::TT_COST;
    jv->first_step = 0;
    jv->last_step = pci_place.basic_info.n_steps - 1;
    jv->name = "joint_velocity_cost";
    pci_place.cost_infos.push_back(jv);
  }

  // Add a velocity cnt with time to insure that robot dynamics are obeyed
  if (ENABLE_VELOCITY_COST)
  {
    std::shared_ptr<trajopt::JointVelTermInfo> jv(new trajopt::JointVelTermInfo);

    // Taken from iiwa documentation (radians/s) and scaled by 0.8
    std::vector<double> vel_lower_lim{ 1.71 * -0.8, 1.71 * -0.8, 1.75 * -0.8, 2.27 * -0.8,
                                       2.44 * -0.8, 3.14 * -0.8, 3.14 * -0.8 };
    std::vector<double> vel_upper_lim{ 1.71 * 0.8, 1.71 * 0.8, 1.75 * 0.8, 2.27 * 0.8,
                                       2.44 * 0.8, 3.14 * 0.8, 3.14 * 0.8 };

    jv->targets = std::vector<double>(7, 0.0);
    jv->coeffs = std::vector<double>(7, 50.0);
    jv->lower_tols = vel_lower_lim;
    jv->upper_tols = vel_upper_lim;
    jv->term_type = (trajopt::TT_CNT | trajopt::TT_USE_TIME);
    jv->first_step = 0;
    jv->last_step = pci_place.basic_info.n_steps - 1;
    jv->name = "joint_velocity_cnt";
    pci_place.cnt_infos.push_back(jv);
  }

  // Add cartesian pose cnt at the retreat point
  Eigen::Quaterniond rotation(retreat_pose.linear());
  auto pose_constraint = std::make_shared<trajopt::CartPoseTermInfo>();
  pose_constraint->term_type = trajopt::TT_CNT;
  pose_constraint->link = end_effector;
  pose_constraint->timestep = steps_ - 1;
  pose_constraint->xyz = retreat_pose.translation();

  pose_constraint->wxyz = Eigen::Vector4d(rotation.w(), rotation.x(), rotation.y(), rotation.z());
  pose_constraint->pos_coeffs = Eigen::Vector3d(10.0, 10.0, 10.0);
  pose_constraint->rot_coeffs = Eigen::Vector3d(10.0, 10.0, 10.0);
  pose_constraint->name = "pose_" + std::to_string(steps_ - 1);
  pci_place.cnt_infos.push_back(pose_constraint);

  // Add cartesian pose cnt at the final point
  int steps = 3 * steps_ - 2 * steps_;
  for (int index = 0; index < steps; index++)
  {
    Eigen::Quaterniond rotation(final_pose.linear());
    auto pose_constraint = std::make_shared<trajopt::CartPoseTermInfo>();
    pose_constraint->term_type = trajopt::TT_CNT;
    pose_constraint->link = end_effector;
    pose_constraint->timestep = 2 * steps_ + index;
    pose_constraint->xyz = approach_pose.translation();
    pose_constraint->xyz.y() = approach_pose.translation().y() + 0.25 / (steps - 1) * index;

    pose_constraint->wxyz = Eigen::Vector4d(rotation.w(), rotation.x(), rotation.y(), rotation.z());
    pose_constraint->pos_coeffs = Eigen::Vector3d(10.0, 10.0, 10.0);
    pose_constraint->rot_coeffs = Eigen::Vector3d(10.0, 10.0, 10.0);
    pose_constraint->name = "pose_" + std::to_string(2 * steps_ + index);
    pci_place.cnt_infos.push_back(pose_constraint);
  }

  // Add a cost on the total time to complete the pick
  if (ENABLE_TIME_COST)
  {
    std::shared_ptr<trajopt::TotalTimeTermInfo> time_cost(new trajopt::TotalTimeTermInfo);
    time_cost->name = "time_cost";
    time_cost->coeff = 5.0;
    time_cost->term_type = trajopt::TT_COST;
    pci_place.cost_infos.push_back(time_cost);
  }

  // Create the place problem
  trajopt::TrajOptProb::Ptr place_prob = ConstructProblem(pci_place);

  // Set the optimization parameters
  tesseract_motion_planners::TrajOptPlannerConfig config_place(place_prob);
  config_place.params.max_iter = 100;

  // Create Plot Callback
  if (plotting_)
  {
    config_place.callbacks.push_back(PlotCallback(*place_prob, plotter));
  }

  // Create file write callback discarding any of the file's current contents
  std::shared_ptr<std::ofstream> stream_ptr_place(new std::ofstream);
  if (write_to_file_)
  {
    std::string path = ros::package::getPath("pick_and_place") + "/file_output_place.csv";
    stream_ptr->open(path, std::ofstream::out | std::ofstream::trunc);
    config_place.callbacks.push_back(trajopt::WriteCallback(stream_ptr_place, place_prob));
  }

  // Set Planner Configuration
  planner.setConfiguration(std::make_shared<tesseract_motion_planners::TrajOptPlannerConfig>(config_place));

  // Solve problem
  planner.solve(planning_response_place);

  if (write_to_file_)
    stream_ptr_place->close();

  // Plot the resulting trajectory
  if (plotting_)
    plotter->plotTrajectory(planning_response_place.joint_trajectory.joint_names,
                            planning_response_place.joint_trajectory.trajectory.leftCols(
                                static_cast<long>(place_prob->GetKin()->getJointNames().size())));

  std::cout << planning_response_place.joint_trajectory.trajectory << '\n';

  ROS_INFO("Done");
  return true;
}
}  // namespace tesseract_ros_examples
