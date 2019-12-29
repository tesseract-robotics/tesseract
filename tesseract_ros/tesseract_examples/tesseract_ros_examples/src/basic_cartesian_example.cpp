/**
 * @file basic_cartesian_plan.cpp
 * @brief Basic cartesian example implementation
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
#include <octomap_ros/conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud_conversion.h>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract_ros_examples/basic_cartesian_example.h>
#include <tesseract_environment/core/utils.h>
#include <tesseract_rosutils/plotting.h>
#include <tesseract_rosutils/utils.h>
#include <trajopt/plot_callback.hpp>
#include <trajopt_utils/config.hpp>
#include <trajopt_utils/logging.hpp>

using namespace trajopt;
using namespace tesseract;
using namespace tesseract_environment;
using namespace tesseract_scene_graph;
using namespace tesseract_collision;
using namespace tesseract_rosutils;

const std::string ROBOT_DESCRIPTION_PARAM = "robot_description"; /**< Default ROS parameter for robot description */
const std::string ROBOT_SEMANTIC_PARAM =
    "robot_description_semantic"; /**< Default ROS parameter for robot description */
const std::string TRAJOPT_DESCRIPTION_PARAM =
    "trajopt_description"; /**< Default ROS parameter for trajopt description */
const std::string GET_ENVIRONMENT_CHANGES_SERVICE = "get_tesseract_changes_rviz";
const std::string MODIFY_ENVIRONMENT_SERVICE = "modify_tesseract_rviz";

namespace tesseract_ros_examples
{
TrajOptProb::Ptr BasicCartesianExample::jsonMethod()
{
  std::string trajopt_config;

  nh_.getParam(TRAJOPT_DESCRIPTION_PARAM, trajopt_config);

  Json::Value root;
  Json::Reader reader;
  bool parse_success = reader.parse(trajopt_config, root);
  if (!parse_success)
  {
    ROS_FATAL("Failed to load trajopt json file from ros parameter");
  }

  return ConstructProblem(root, tesseract_);
}

bool BasicCartesianExample::addPointCloud()
{
  // Create octomap and add it to the local environment
  pcl::PointCloud<pcl::PointXYZ> full_cloud;
  double delta = 0.05;
  auto length = static_cast<int>(1 / delta);

  for (int x = 0; x < length; ++x)
    for (int y = 0; y < length; ++y)
      for (int z = 0; z < length; ++z)
        full_cloud.push_back(pcl::PointXYZ(-0.5f + static_cast<float>(x * delta),
                                           -0.5f + static_cast<float>(y * delta),
                                           -0.5f + static_cast<float>(z * delta)));

  sensor_msgs::PointCloud2 pointcloud_msg;
  pcl::toROSMsg(full_cloud, pointcloud_msg);

  octomap::Pointcloud octomap_data;
  octomap::pointCloud2ToOctomap(pointcloud_msg, octomap_data);
  std::shared_ptr<octomap::OcTree> octree = std::make_shared<octomap::OcTree>(2 * delta);
  octree->insertPointCloud(octomap_data, octomap::point3d(0, 0, 0));

  // Add octomap to environment
  Link link_octomap("octomap_attached");

  Visual::Ptr visual = std::make_shared<Visual>();
  visual->origin = Eigen::Isometry3d::Identity();
  visual->origin.translation() = Eigen::Vector3d(1, 0, 0);
  visual->geometry = std::make_shared<tesseract_geometry::Octree>(octree, tesseract_geometry::Octree::BOX);
  link_octomap.visual.push_back(visual);

  Collision::Ptr collision = std::make_shared<Collision>();
  collision->origin = visual->origin;
  collision->geometry = visual->geometry;
  link_octomap.collision.push_back(collision);

  Joint joint_octomap("joint_octomap_attached");
  joint_octomap.parent_link_name = "base_link";
  joint_octomap.child_link_name = link_octomap.getName();
  joint_octomap.type = JointType::FIXED;

  return tesseract_->getEnvironment()->addLink(link_octomap, joint_octomap);
}

TrajOptProb::Ptr BasicCartesianExample::cppMethod()
{
  ProblemConstructionInfo pci(tesseract_);

  // Populate Basic Info
  pci.basic_info.n_steps = steps_;
  pci.basic_info.manip = "manipulator";
  pci.basic_info.start_fixed = false;
  pci.basic_info.use_time = false;
  //  pci.basic_info.dofs_fixed

  // Create Kinematic Object
  pci.kin = pci.getManipulator(pci.basic_info.manip);

  // Populate Init Info
  EnvState::ConstPtr current_state = pci.env->getCurrentState();
  Eigen::VectorXd start_pos;
  start_pos.resize(pci.kin->numJoints());
  int cnt = 0;
  for (const auto& j : pci.kin->getJointNames())
  {
    start_pos[cnt] = current_state->joints.at(j);
    ++cnt;
  }

  pci.init_info.type = InitInfo::STATIONARY;
  pci.init_info.data = start_pos.transpose().replicate(pci.basic_info.n_steps, 1);

  // Populate Cost Info
  auto jv = std::make_shared<JointVelTermInfo>();
  jv->coeffs = std::vector<double>(7, 5.0);
  jv->targets = std::vector<double>(7, 0.0);
  jv->first_step = 0;
  jv->last_step = pci.basic_info.n_steps - 1;
  jv->name = "joint_vel";
  jv->term_type = TT_COST;
  pci.cost_infos.push_back(jv);

  auto collision = std::make_shared<CollisionTermInfo>();
  collision->name = "collision";
  collision->term_type = TT_COST;
  collision->continuous = false;
  collision->first_step = 0;
  collision->last_step = pci.basic_info.n_steps - 1;
  collision->info = createSafetyMarginDataVector(pci.basic_info.n_steps, 0.025, 20);

  // Populate Constraints
  double delta = 0.5 / pci.basic_info.n_steps;
  for (auto i = 0; i < pci.basic_info.n_steps; ++i)
  {
    auto pose = std::make_shared<CartPoseTermInfo>();
    pose->term_type = TT_CNT;
    pose->name = "waypoint_cart_" + std::to_string(i);
    pose->link = "tool0";
    pose->timestep = i;
    pose->xyz = Eigen::Vector3d(0.5, -0.2 + delta * i, 0.62);
    pose->wxyz = Eigen::Vector4d(0.0, 0.0, 1.0, 0.0);
    pose->pos_coeffs = Eigen::Vector3d(10, 10, 10);
    pose->rot_coeffs = Eigen::Vector3d(10, 10, 10);
    pci.cnt_infos.push_back(pose);
  }

  return ConstructProblem(pci);
}

bool BasicCartesianExample::run()
{
  // Initial setup
  std::string urdf_xml_string, srdf_xml_string;
  nh_.getParam(ROBOT_DESCRIPTION_PARAM, urdf_xml_string);
  nh_.getParam(ROBOT_SEMANTIC_PARAM, srdf_xml_string);

  ResourceLocator::Ptr locator = std::make_shared<tesseract_rosutils::ROSResourceLocator>();
  if (!tesseract_->init(urdf_xml_string, srdf_xml_string, locator))
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

  // Create octomap and add it to the local environment
  if (!addPointCloud())
    return false;

  if (rviz_)
  {
    // Now update rviz environment
    if (!sendRvizChanges(0))
      return false;
  }

  // Create plotting tool
  ROSPlottingPtr plotter = std::make_shared<ROSPlotting>(tesseract_->getEnvironment());

  // Set the robot initial state
  std::unordered_map<std::string, double> ipos;
  ipos["joint_a1"] = -0.4;
  ipos["joint_a2"] = 0.2762;
  ipos["joint_a3"] = 0.0;
  ipos["joint_a4"] = -1.3348;
  ipos["joint_a5"] = 0.0;
  ipos["joint_a6"] = 1.4959;
  ipos["joint_a7"] = 0.0;

  tesseract_->getEnvironment()->setState(ipos);

  // Set Log Level
  util::gLogLevel = util::LevelError;

  // Setup Problem
  TrajOptProb::Ptr prob;
  if (method_ == "cpp")
    prob = cppMethod();
  else
    prob = jsonMethod();

  // Solve Trajectory
  ROS_INFO("basic cartesian plan example");

  std::vector<ContactResultMap> collisions;
  tesseract_environment::StateSolver::Ptr state_solver = prob->GetEnv()->getStateSolver();
  ContinuousContactManager::Ptr manager = prob->GetEnv()->getContinuousContactManager();
  AdjacencyMap::Ptr adjacency_map =
      std::make_shared<tesseract_environment::AdjacencyMap>(prob->GetEnv()->getSceneGraph(),
                                                            prob->GetKin()->getActiveLinkNames(),
                                                            prob->GetEnv()->getCurrentState()->transforms);

  manager->setActiveCollisionObjects(adjacency_map->getActiveLinkNames());
  manager->setContactDistanceThreshold(0);
  collisions.clear();
  bool found =
      checkTrajectory(collisions, *manager, *state_solver, prob->GetKin()->getJointNames(), prob->GetInitTraj());

  ROS_INFO((found) ? ("Initial trajectory is in collision") : ("Initial trajectory is collision free"));

  sco::BasicTrustRegionSQP opt(prob);
  if (plotting_)
  {
    opt.addCallback(PlotCallback(*prob, plotter));
  }

  opt.initialize(trajToDblVec(prob->GetInitTraj()));
  ros::Time tStart = ros::Time::now();
  opt.optimize();
  ROS_INFO("planning time: %.3f", (ros::Time::now() - tStart).toSec());

  if (plotting_)
    plotter->clear();

  collisions.clear();
  found = checkTrajectory(
      collisions, *manager, *state_solver, prob->GetKin()->getJointNames(), getTraj(opt.x(), prob->GetVars()));

  ROS_INFO((found) ? ("Final trajectory is in collision") : ("Final trajectory is collision free"));

  plotter->plotTrajectory(prob->GetKin()->getJointNames(), getTraj(opt.x(), prob->GetVars()));

  return true;
}

}  // namespace tesseract_ros_examples
