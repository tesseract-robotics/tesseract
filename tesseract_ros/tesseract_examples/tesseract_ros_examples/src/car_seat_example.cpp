/**
 * @file car_seat_example.cpp
 * @brief Car seat example implementation
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

#include <tesseract_ros_examples/car_seat_example.h>
#include <tesseract_environment/core/utils.h>
#include <tesseract_rosutils/plotting.h>
#include <tesseract_rosutils/utils.h>
#include <trajopt/plot_callback.hpp>
#include <trajopt/problem_description.hpp>
#include <trajopt_utils/config.hpp>
#include <trajopt_utils/logging.hpp>
#include <tesseract_geometry/mesh_parser.h>
#include <memory>

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

namespace tesseract_ros_examples
{
CarSeatExample::CarSeatExample(const ros::NodeHandle& nh, bool plotting, bool rviz)
  : Example(plotting, rviz), nh_(nh), env_current_revision_(0)
{
  locator_ = std::make_shared<tesseract_rosutils::ROSResourceLocator>();
}

void CarSeatExample::addSeats()
{
  for (int i = 0; i < 3; ++i)
  {
    Link link_seat("seat_" + std::to_string(i + 1));

    Visual::Ptr visual = std::make_shared<Visual>();
    visual->origin = Eigen::Isometry3d::Identity();
    visual->geometry =
        tesseract_geometry::createMeshFromResource<tesseract_geometry::Mesh>(locator_->locateResource("package://"
                                                                                                      "tesseract_ros_"
                                                                                                      "examples/meshes/"
                                                                                                      "car_seat/visual/"
                                                                                                      "seat.dae"),
                                                                             Eigen::Vector3d(1, 1, 1),
                                                                             true)[0];
    link_seat.visual.push_back(visual);

    for (int m = 1; m <= 10; ++m)
    {
      std::vector<tesseract_geometry::Mesh::Ptr> meshes =
          tesseract_geometry::createMeshFromResource<tesseract_geometry::Mesh>(
              locator_->locateResource("package://tesseract_ros_examples/"
                                       "meshes/car_seat/collision/seat_" +
                                       std::to_string(m) + ".stl"));
      for (auto& mesh : meshes)
      {
        Collision::Ptr collision = std::make_shared<Collision>();
        collision->origin = visual->origin;
        collision->geometry = makeConvexMesh(*mesh);
        link_seat.collision.push_back(collision);
      }
    }

    Joint joint_seat("joint_seat_" + std::to_string(i + 1));
    joint_seat.parent_link_name = "world";
    joint_seat.child_link_name = link_seat.getName();
    joint_seat.type = JointType::FIXED;
    joint_seat.parent_to_joint_origin_transform = Eigen::AngleAxisd(0.0, Eigen::Vector3d::UnitX()) *
                                                  Eigen::AngleAxisd(0.0, Eigen::Vector3d::UnitY()) *
                                                  Eigen::AngleAxisd(3.14159, Eigen::Vector3d::UnitZ());
    joint_seat.parent_to_joint_origin_transform.translation() = Eigen::Vector3d(0.5 + i, 2.15, 0.45);

    tesseract_->getEnvironment()->addLink(link_seat, joint_seat);
  }
}

// void CarSeatExample::addCar()
//{
//  AttachableObjectPtr obj(new AttachableObject());
//  std::shared_ptr<shapes::Mesh>
//  visual_mesh(shapes::createMeshFromResource("package://tesseract_ros_examples/meshes/car_seat/visual/car.dae"));
//  std::shared_ptr<shapes::Mesh>
//  collision_mesh(shapes::createMeshFromResource("package://tesseract_ros_examples/meshes/car_seat/collision/car.stl"));
//  Eigen::Isometry3d seat_pose;
//  seat_pose.setIdentity();

//  obj->name = "car";
//  obj->visual.shapes.push_back(visual_mesh);
//  obj->visual.shape_poses.push_back(seat_pose);
//  obj->collision.shapes.push_back(collision_mesh);
//  obj->collision.shape_poses.push_back(seat_pose);
//  obj->collision.collision_object_types.push_back(CollisionObjectType::UseShapeType);

//  env_->addAttachableObject(obj);

//  AttachedBodyInfo attached_body;
//  attached_body.object_name = obj->name;
//  attached_body.parent_link_name = "world";
//  attached_body.transform.setIdentity();
//  attached_body.transform.translation() = Eigen::Vector3d(0.0, -0.2, 0.0);
//  attached_body.touch_links = {"base_link", "carriage", "cell_logo",
//  "conveyor", "fence", "link_l", "link_s", "rail"};

//  env_->attachBody(attached_body);
//}

std::unordered_map<std::string, std::unordered_map<std::string, double>> CarSeatExample::getPredefinedPosition()
{
  std::unordered_map<std::string, std::unordered_map<std::string, double>> result;

  std::unordered_map<std::string, double> default_pos;
  default_pos["carriage_rail"] = 1.0;
  default_pos["joint_b"] = 0.0;
  default_pos["joint_e"] = 0.0;
  default_pos["joint_l"] = 0.0;
  default_pos["joint_r"] = 0.0;
  default_pos["joint_s"] = -1.5707;
  default_pos["joint_t"] = 0.0;
  default_pos["joint_u"] = -1.5707;
  result["Default"] = default_pos;

  std::unordered_map<std::string, double> pick1;
  pick1["carriage_rail"] = 2.22;
  pick1["joint_b"] = 0.45;
  pick1["joint_e"] = 0.0;
  pick1["joint_l"] = 0.53;
  pick1["joint_r"] = 0.0;
  pick1["joint_s"] = -3.14;
  pick1["joint_t"] = -0.29;
  pick1["joint_u"] = -1.49;
  result["Pick1"] = pick1;

  std::unordered_map<std::string, double> pick2;
  pick2["carriage_rail"] = 1.22;
  pick2["joint_b"] = 0.45;
  pick2["joint_e"] = 0.0;
  pick2["joint_l"] = 0.53;
  pick2["joint_r"] = 0.0;
  pick2["joint_s"] = -3.14;
  pick2["joint_t"] = -0.29;
  pick2["joint_u"] = -1.49;
  result["Pick2"] = pick2;

  std::unordered_map<std::string, double> pick3;
  pick3["carriage_rail"] = 0.22;
  pick3["joint_b"] = 0.45;
  pick3["joint_e"] = 0.0;
  pick3["joint_l"] = 0.53;
  pick3["joint_r"] = 0.0;
  pick3["joint_s"] = -3.14;
  pick3["joint_t"] = -0.29;
  pick3["joint_u"] = -1.49;
  result["Pick3"] = pick3;

  std::unordered_map<std::string, double> place1;
  place1["carriage_rail"] = 4.10529;
  place1["joint_b"] = 0.608497;
  place1["joint_e"] = 0.0167816;
  place1["joint_l"] = 0.869957;
  place1["joint_r"] = 0.0502274;
  place1["joint_s"] = -0.0394713;
  place1["joint_t"] = -0.318406;
  place1["joint_u"] = -1.30834;
  result["Place1"] = place1;

  std::unordered_map<std::string, double> home;
  home["carriage_rail"] = 0.0;
  home["joint_b"] = 0.0;
  home["joint_e"] = 0.0;
  home["joint_l"] = 0.0;
  home["joint_r"] = 0.0;
  home["joint_s"] = 0.0;
  home["joint_t"] = 0.0;
  home["joint_u"] = 0.0;
  result["Home"] = home;

  return result;
}

std::vector<double> CarSeatExample::getPositionVector(const ForwardKinematics::ConstPtr& kin,
                                                      const std::unordered_map<std::string, double>& pos)
{
  std::vector<double> result;
  for (const auto& joint_name : kin->getJointNames())
    result.push_back(pos.at(joint_name));

  return result;
}

Eigen::VectorXd CarSeatExample::getPositionVectorXd(const ForwardKinematics::ConstPtr& kin,
                                                    const std::unordered_map<std::string, double>& pos)
{
  Eigen::VectorXd result;
  result.resize(kin->numJoints());
  int cnt = 0;
  for (const auto& joint_name : kin->getJointNames())
    result[cnt++] = pos.at(joint_name);

  return result;
}

std::shared_ptr<ProblemConstructionInfo> CarSeatExample::cppMethod(const std::string& start, const std::string& finish)
{
  std::shared_ptr<ProblemConstructionInfo> pci = std::make_shared<ProblemConstructionInfo>(tesseract_);

  // Populate Basic Info
  pci->basic_info.n_steps = 50;
  pci->basic_info.manip = "manipulator";
  pci->basic_info.start_fixed = true;
  pci->basic_info.use_time = false;

  pci->opt_info.max_iter = 200;
  pci->opt_info.min_approx_improve = 1e-3;
  pci->opt_info.min_trust_box_size = 1e-3;

  // Create Kinematic Object
  pci->kin = pci->getManipulator(pci->basic_info.manip);

  // Populate Init Info
  Eigen::VectorXd start_pos = getPositionVectorXd(pci->kin, saved_positions_[start]);

  std::vector<double> joint_pose = getPositionVector(pci->kin, saved_positions_[finish]);
  pci->init_info.type = InitInfo::GIVEN_TRAJ;
  pci->init_info.data = start_pos.transpose().replicate(pci->basic_info.n_steps, 1);
  for (auto i = 0; i < start_pos.size(); ++i)
    pci->init_info.data.col(i) =
        Eigen::VectorXd::LinSpaced(pci->basic_info.n_steps, start_pos[i], joint_pose[static_cast<size_t>(i)]);

  // Populate Cost Info
  auto joint_vel = std::make_shared<JointVelTermInfo>();
  joint_vel->coeffs = std::vector<double>(8, 1.0);
  joint_vel->targets = std::vector<double>(8, 0.0);
  joint_vel->first_step = 0;
  joint_vel->last_step = pci->basic_info.n_steps - 1;
  joint_vel->name = "joint_vel";
  joint_vel->term_type = TT_COST;
  pci->cost_infos.push_back(joint_vel);

  auto joint_acc = std::make_shared<JointAccTermInfo>();
  joint_acc->coeffs = std::vector<double>(8, 5.0);
  joint_acc->targets = std::vector<double>(8, 0.0);
  joint_acc->first_step = 0;
  joint_acc->last_step = pci->basic_info.n_steps - 1;
  joint_acc->name = "joint_acc";
  joint_acc->term_type = TT_COST;
  pci->cost_infos.push_back(joint_acc);

  auto joint_jerk = std::make_shared<JointJerkTermInfo>();
  joint_jerk->coeffs = std::vector<double>(8, 5.0);
  joint_jerk->targets = std::vector<double>(8, 0.0);
  joint_jerk->first_step = 0;
  joint_jerk->last_step = pci->basic_info.n_steps - 1;
  joint_jerk->name = "joint_jerk";
  joint_jerk->term_type = TT_COST;
  pci->cost_infos.push_back(joint_jerk);

  auto collision = std::make_shared<CollisionTermInfo>();
  collision->name = "collision";
  collision->term_type = TT_CNT;
  collision->continuous = true;
  collision->first_step = 0;
  collision->last_step = pci->basic_info.n_steps - 2;
  collision->info = createSafetyMarginDataVector(pci->basic_info.n_steps, 0.0001, 40);
  pci->cnt_infos.push_back(collision);

  // Create place pose constraint
  std::shared_ptr<JointPosTermInfo> jpos(new JointPosTermInfo);
  jpos->term_type = TT_CNT;
  jpos->name = finish;
  jpos->first_step = pci->basic_info.n_steps - 1;
  jpos->last_step = pci->basic_info.n_steps - 1;
  jpos->targets = joint_pose;
  pci->cnt_infos.push_back(jpos);

  return pci;
}

bool CarSeatExample::run()
{
  // Initial setup
  std::string urdf_xml_string, srdf_xml_string;
  nh_.getParam(ROBOT_DESCRIPTION_PARAM, urdf_xml_string);
  nh_.getParam(ROBOT_SEMANTIC_PARAM, srdf_xml_string);

  if (!tesseract_->init(urdf_xml_string, srdf_xml_string, locator_))
    return false;

  // These are used to keep visualization updated
  if (rviz_)
  {
    modify_env_rviz_ = nh_.serviceClient<tesseract_msgs::ModifyEnvironment>(MODIFY_ENVIRONMENT_SERVICE, false);
    get_env_changes_rviz_ =
        nh_.serviceClient<tesseract_msgs::GetEnvironmentChanges>(GET_ENVIRONMENT_CHANGES_SERVICE, false);

    // Check RViz to make sure nothing has changed
    if (!checkRviz())
      return false;
  }

  // Create plotting tool
  tesseract_rosutils::ROSPlottingPtr plotter =
      std::make_shared<tesseract_rosutils::ROSPlotting>(tesseract_->getEnvironment());

  // Get predefined positions
  saved_positions_ = getPredefinedPosition();

  //  // Add the car as a detailed mesh
  //  addCar();

  // Put three seats on the conveyor
  addSeats();

  if (rviz_)
  {
    // Now update rviz environment
    if (!sendRvizChanges(static_cast<unsigned long>(env_current_revision_)))
      return false;
  }

  // Store current revision
  env_current_revision_ = tesseract_->getEnvironment()->getRevision();

  // Move to home position
  tesseract_->getEnvironment()->setState(saved_positions_["Home"]);

  // Set Log Level
  util::gLogLevel = util::LevelError;

  // Solve Trajectory
  ROS_INFO("Car Seat Demo Started");

  // Go pick up first seat
  ros::Time tStart;
  std::shared_ptr<ProblemConstructionInfo> pci;
  TrajOptProb::Ptr prob;

  pci = cppMethod("Home", "Pick1");
  prob = ConstructProblem(*pci);
  sco::BasicTrustRegionSQP pick1_opt(prob);
  if (plotting_)
    pick1_opt.addCallback(PlotCallback(*prob, plotter));

  pick1_opt.initialize(trajToDblVec(prob->GetInitTraj()));
  tStart = ros::Time::now();
  pick1_opt.optimize();
  ROS_INFO("Pick seat #1 planning time: %.3f", (ros::Time::now() - tStart).toSec());

  if (plotting_)
    plotter->clear();

  // Plot the trajectory
  plotter->plotTrajectory(prob->GetKin()->getJointNames(), getTraj(pick1_opt.x(), prob->GetVars()));

  std::vector<ContactResultMap> collisions;
  tesseract_environment::StateSolver::Ptr state_solver = prob->GetEnv()->getStateSolver();
  ContinuousContactManager::Ptr manager = prob->GetEnv()->getContinuousContactManager();
  AdjacencyMap::Ptr adjacency_map =
      std::make_shared<tesseract_environment::AdjacencyMap>(prob->GetEnv()->getSceneGraph(),
                                                            prob->GetKin()->getActiveLinkNames(),
                                                            prob->GetEnv()->getCurrentState()->transforms);

  manager->setActiveCollisionObjects(adjacency_map->getActiveLinkNames());
  manager->setContactDistanceThreshold(0);

  bool found = checkTrajectory(
      collisions, *manager, *state_solver, prob->GetKin()->getJointNames(), getTraj(pick1_opt.x(), prob->GetVars()));

  ROS_INFO((found) ? ("Pick seat #1 trajectory is in collision") : ("Pick seat #1 trajectory is collision free"));

  // Get the state of at the end of pick 1 trajectory
  EnvState::Ptr state = tesseract_->getEnvironment()->getState(
      prob->GetKin()->getJointNames(), getPositionVectorXd(prob->GetKin(), saved_positions_["Pick1"]));

  // Now we to detach seat_1 and attach it to the robot end_effector
  Joint joint_seat_1_robot("joint_seat_1_robot");
  joint_seat_1_robot.parent_link_name = "end_effector";
  joint_seat_1_robot.child_link_name = "seat_1";
  joint_seat_1_robot.type = JointType::FIXED;
  joint_seat_1_robot.parent_to_joint_origin_transform =
      state->transforms["end_effector"].inverse() * state->transforms["seat_1"];

  tesseract_->getEnvironment()->moveLink(joint_seat_1_robot);
  tesseract_->getEnvironment()->addAllowedCollision("seat_1", "end_effector", "Adjacent");
  tesseract_->getEnvironment()->addAllowedCollision("seat_1", "cell_logo", "Never");
  tesseract_->getEnvironment()->addAllowedCollision("seat_1", "fence", "Never");
  tesseract_->getEnvironment()->addAllowedCollision("seat_1", "link_b", "Never");
  tesseract_->getEnvironment()->addAllowedCollision("seat_1", "link_r", "Never");
  tesseract_->getEnvironment()->addAllowedCollision("seat_1", "link_t", "Never");

  if (rviz_)
  {
    // Now update rviz environment
    if (!sendRvizChanges(static_cast<unsigned long>(env_current_revision_)))
      return false;
  }

  // Store current revision
  env_current_revision_ = tesseract_->getEnvironment()->getRevision();

  pci = cppMethod("Pick1", "Place1");
  prob = ConstructProblem(*pci);
  sco::BasicTrustRegionSQP place1_opt(prob);
  if (plotting_)
    place1_opt.addCallback(PlotCallback(*prob, plotter));

  place1_opt.initialize(trajToDblVec(prob->GetInitTraj()));
  tStart = ros::Time::now();
  place1_opt.optimize();
  ROS_INFO("Place seat #1 planning time: %.3f", (ros::Time::now() - tStart).toSec());

  if (plotting_)
    plotter->clear();

  // Plot the trajectory
  plotter->plotTrajectory(prob->GetKin()->getJointNames(), getTraj(place1_opt.x(), prob->GetVars()));

  manager = prob->GetEnv()->getContinuousContactManager();
  adjacency_map = std::make_shared<tesseract_environment::AdjacencyMap>(prob->GetEnv()->getSceneGraph(),
                                                                        prob->GetKin()->getActiveLinkNames(),
                                                                        prob->GetEnv()->getCurrentState()->transforms);

  manager->setActiveCollisionObjects(adjacency_map->getActiveLinkNames());
  manager->setContactDistanceThreshold(0);
  collisions.clear();
  found = checkTrajectory(
      collisions, *manager, *state_solver, prob->GetKin()->getJointNames(), getTraj(place1_opt.x(), prob->GetVars()));

  ROS_INFO((found) ? ("Place seat #1 trajectory is in collision") : ("Place seat #1 trajectory is collision free"));

  // Get the state of at the end of place 1 trajectory
  state = tesseract_->getEnvironment()->getState(prob->GetKin()->getJointNames(),
                                                 getPositionVectorXd(prob->GetKin(), saved_positions_["Place1"]));

  // Now we to detach seat_1 and attach it to the robot end_effector
  Joint joint_seat_1_car("joint_seat_1_car");
  joint_seat_1_car.parent_link_name = "car";
  joint_seat_1_car.child_link_name = "seat_1";
  joint_seat_1_car.type = JointType::FIXED;
  joint_seat_1_car.parent_to_joint_origin_transform = state->transforms["car"].inverse() * state->transforms["seat_1"];

  tesseract_->getEnvironment()->moveLink(joint_seat_1_car);

  if (rviz_)
  {
    // Now update rviz environment
    if (!sendRvizChanges(static_cast<unsigned long>(env_current_revision_)))
      return false;
  }

  // Store current revision
  env_current_revision_ = tesseract_->getEnvironment()->getRevision();

  pci = cppMethod("Place1", "Pick2");
  prob = ConstructProblem(*pci);
  sco::BasicTrustRegionSQP pick2_opt(prob);
  if (plotting_)
    pick2_opt.addCallback(PlotCallback(*prob, plotter));

  pick2_opt.initialize(trajToDblVec(prob->GetInitTraj()));
  tStart = ros::Time::now();
  pick2_opt.optimize();
  ROS_INFO("Pick seat #2 planning time: %.3f", (ros::Time::now() - tStart).toSec());

  if (plotting_)
    plotter->clear();

  // Plot the trajectory
  plotter->plotTrajectory(prob->GetKin()->getJointNames(), getTraj(pick2_opt.x(), prob->GetVars()));

  manager = prob->GetEnv()->getContinuousContactManager();
  adjacency_map = std::make_shared<tesseract_environment::AdjacencyMap>(prob->GetEnv()->getSceneGraph(),
                                                                        prob->GetKin()->getActiveLinkNames(),
                                                                        prob->GetEnv()->getCurrentState()->transforms);

  manager->setActiveCollisionObjects(adjacency_map->getActiveLinkNames());
  manager->setContactDistanceThreshold(0);
  collisions.clear();
  found = checkTrajectory(
      collisions, *manager, *state_solver, prob->GetKin()->getJointNames(), getTraj(pick2_opt.x(), prob->GetVars()));

  ROS_INFO((found) ? ("Pick seat #2 trajectory is in collision") : ("Pick seat #2 trajectory is collision free"));

  // Remove the seat in the car.
  tesseract_->getEnvironment()->removeJoint("joint_seat_1_car");

  // Now we to detach seat_1 and attach it to the robot end_effector
  Joint joint_seat_2_robot("joint_seat_2_robot");
  joint_seat_2_robot.parent_link_name = "end_effector";
  joint_seat_2_robot.child_link_name = "seat_2";
  joint_seat_2_robot.type = JointType::FIXED;
  joint_seat_2_robot.parent_to_joint_origin_transform =
      state->transforms["end_effector"].inverse() * state->transforms["seat_2"];

  tesseract_->getEnvironment()->moveLink(joint_seat_2_robot);

  if (rviz_)
  {
    // Now update rviz environment
    if (!sendRvizChanges(static_cast<unsigned long>(env_current_revision_)))
      return false;
  }

  // Store current revision
  env_current_revision_ = tesseract_->getEnvironment()->getRevision();

  return true;
}

}  // namespace tesseract_ros_examples
// int main(int argc, char **argv)
//{
//  // Set up ROS.
//  ros::init(argc, argv, "rss_demo_node");
//  ros::NodeHandle nh;

//  ros::AsyncSpinner spinner(4);
//  spinner.start();

//  std::vector<moveit_msgs::CollisionObject> collision_objects;
//  MoveGroup group("robot_rail");

//  // Add all three seats to the world environment
//  moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
//  planning_scene_monitor::PlanningSceneMonitor
//  planning_scene_monitor("robot_description");
//  planning_scene_monitor.startSceneMonitor("/move_group/monitored_planning_scene");

//  collision_objects = createCollisionObjects();
//  planning_scene_interface.applyCollisionObjects(collision_objects);

//  group.setPlanningTime(30.0);
//  group.setStartState(planning_scene_monitor.getPlanningScene()->getCurrentState());
//  group.setNamedTarget("Pick1");

//  MoveGroup::Plan plan;
//  MoveItErrorCode e;

//  do {
//    e = group.plan(plan);
//  } while(e != MoveItErrorCode::SUCCESS);

//  group.execute(plan);

//  sleep_t.sleep();
//  std::string id = "seat_1";
//  std::string eff_link = "end_effector";
//  std::vector<std::string> touch_links = {eff_link, "cell_logo", "fence",
//  "link_b", "link_r", "link_t", id};

//  group.attachObject(id, eff_link, touch_links);
//  sleep_t.sleep();

//  group.setStartState(planning_scene_monitor.getPlanningScene()->getCurrentState());
//  group.setNamedTarget("Place1");

//  do {
//    e = group.plan(plan);
//  } while(e != MoveItErrorCode::SUCCESS);

//  group.execute(plan);

//  sleep_t.sleep();
//  group.detachObject(id);
//  planning_scene_interface.removeCollisionObjects({id});
//  sleep_t.sleep();
//  group.setStartState(planning_scene_monitor.getPlanningScene()->getCurrentState());
//  group.setNamedTarget("Pick2");

//  e = group.plan(plan);
//  if (e = MoveItErrorCode::SUCCESS)
//    group.execute(plan);

//  sleep_t.sleep();
//  id = "seat_2";
//  group.attachObject(id, eff_link, touch_links);
//  sleep_t.sleep();
//  group.setStartState(planning_scene_monitor.getPlanningScene()->getCurrentState());
//  group.setNamedTarget("Place1");

//  e = group.plan(plan);
//  if (e = MoveItErrorCode::SUCCESS)
//    group.execute(plan);

//  sleep_t.sleep();
//  group.detachObject(id);
//  planning_scene_interface.removeCollisionObjects({id});
//  sleep_t.sleep();
//  group.setStartState(*group.getCurrentState());
//  group.setNamedTarget("Pick3");

//  e = group.plan(plan);
//  if (e = MoveItErrorCode::SUCCESS)
//    group.execute(plan);

//  sleep_t.sleep();
//  id = "seat_3";
//  group.attachObject(id, eff_link, touch_links);
//  sleep_t.sleep();
//  group.setStartState(planning_scene_monitor.getPlanningScene()->getCurrentState());
//  group.setNamedTarget("Place1");

//  e = group.plan(plan);
//  if (e = MoveItErrorCode::SUCCESS)
//    group.execute(plan);

//  sleep_t.sleep();
//  group.detachObject(id);
//  planning_scene_interface.removeCollisionObjects({id});
//  sleep_t.sleep();
//  group.setStartState(planning_scene_monitor.getPlanningScene()->getCurrentState());
//  group.setNamedTarget("Pick2");

//  e = group.plan(plan);
//  if (e = MoveItErrorCode::SUCCESS)
//    group.execute(plan);

//}
