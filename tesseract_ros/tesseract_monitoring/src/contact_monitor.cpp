/*
 * Copyright 2020 Southwest Research Institute
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include <tesseract_common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <ros/ros.h>
#include <ros/package.h>
#include <sensor_msgs/JointState.h>
#include <tesseract_msgs/ContactResultVector.h>
#include <tesseract_msgs/ModifyEnvironment.h>
#include <tesseract_msgs/ComputeContactResultVector.h>
#include <visualization_msgs/MarkerArray.h>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <boost/thread/mutex.hpp>
#include <boost/thread/thread.hpp>

#include <tesseract_collision/core/discrete_contact_manager.h>
#include <tesseract/tesseract.h>
#include <tesseract_scene_graph/graph.h>
#include <tesseract_scene_graph/utils.h>
#include <tesseract_scene_graph/parser/srdf_parser.h>
#include <tesseract_urdf/urdf_parser.h>
#include <tesseract_environment/kdl/kdl_env.h>
#include <tesseract_environment/core/utils.h>
#include <tesseract_rosutils/utils.h>
#include <tesseract_rosutils/plotting.h>

// Stuff for the contact monitor
const std::string ROBOT_DESCRIPTION_PARAM = "robot_description"; /**< Default ROS parameter for robot description */
const double DEFAULT_CONTACT_DISTANCE = 0.1;

namespace contact_monitor
{

class ContactMonitor
{
private:
  tesseract::Tesseract::Ptr tess_;
  std::vector<std::string> monitored_link_names_;
  tesseract_collision::ContactTestType type_;
  double contact_distance_;
  bool publish_environment_;
  bool publish_markers_;
  tesseract_collision::DiscreteContactManager::Ptr manager_;
  ros::Subscriber joint_states_sub_;
  ros::Publisher contact_results_pub_;
  ros::Publisher environment_pub_;
  ros::Publisher contact_marker_pub_;
  ros::Subscriber environment_diff_sub_;
  ros::ServiceServer modify_env_service_;
  ros::ServiceServer compute_contact_results_;
  boost::mutex modify_mutex_;
  boost::shared_ptr<sensor_msgs::JointState> current_joint_states_;
  boost::condition_variable current_joint_states_evt_;

public:

  /**
 * @brief Compute collision results and publish results.
 *
 * This also publishes environment and contact markers if correct flags are enabled for visualization and debuging.
 */
  void computeCollisionReportThread()
  {
    while (!ros::isShuttingDown())
    {
      boost::shared_ptr<sensor_msgs::JointState> msg = nullptr;
      tesseract_collision::ContactResultMap contacts;
      tesseract_msgs::ContactResultVector contacts_msg;
      // Limit the lock
      {
        boost::mutex::scoped_lock lock(modify_mutex_);
        if (!current_joint_states_)
        {
          current_joint_states_evt_.wait(lock);
        }

        if (!current_joint_states_)
          continue;

        msg = current_joint_states_;
        current_joint_states_.reset();

        contacts.clear();
        contacts_msg.contacts.clear();

        tess_->getEnvironment()->setState(msg->name, msg->position);
        tesseract_environment::EnvState::ConstPtr state = tess_->getEnvironment()->getCurrentState();

        manager_->setCollisionObjectsTransform(state->transforms);
        manager_->contactTest(contacts, type_);
      }

      if (publish_environment_)
      {
        tesseract_msgs::TesseractState state_msg;
        tesseract_rosutils::toMsg(state_msg, *(tess_->getEnvironment()));
        environment_pub_.publish(state_msg);
      }

      tesseract_collision::ContactResultVector contacts_vector;
      tesseract_collision::flattenResults(std::move(contacts), contacts_vector);
      Eigen::VectorXd safety_distance(contacts_vector.size());
      contacts_msg.contacts.reserve(contacts_vector.size());
      for (std::size_t i = 0; i < contacts_vector.size(); ++i)
      {
        safety_distance[static_cast<long>(i)] = contact_distance_;
        tesseract_msgs::ContactResult contact_msg;
        tesseract_rosutils::toMsg(contact_msg, contacts_vector[i], msg->header.stamp);
        contacts_msg.contacts.push_back(contact_msg);
      }
      contact_results_pub_.publish(contacts_msg);

      if (publish_markers_)
      {
        int id_counter = 0;
        visualization_msgs::MarkerArray marker_msg =
            tesseract_rosutils::ROSPlotting::getContactResultsMarkerArrayMsg(id_counter,
                                                                             tess_->getEnvironment()->getSceneGraph()->getRoot(),
                                                                             "contact_monitor",
                                                                             msg->header.stamp,
                                                                             monitored_link_names_,
                                                                             contacts_vector,
                                                                             safety_distance);
        contact_marker_pub_.publish(marker_msg);
      }
    }
  }

  void callbackJointState(boost::shared_ptr<sensor_msgs::JointState> msg)
  {
    boost::mutex::scoped_lock lock(modify_mutex_);
    current_joint_states_ = std::move(msg);
    current_joint_states_evt_.notify_all();
  }

  bool callbackModifyTesseractEnv(tesseract_msgs::ModifyEnvironmentRequest& request,
                                  tesseract_msgs::ModifyEnvironmentResponse& response)
  {
    boost::mutex::scoped_lock lock(modify_mutex_);
    response.success = tesseract_rosutils::processMsg(*(tess_->getEnvironment()), request.commands);
    response.revision = static_cast<unsigned long>(tess_->getEnvironmentConst()->getRevision());

    // Create a new manager
    std::vector<std::string> active = manager_->getActiveCollisionObjects();
    double contact_distance = manager_->getContactDistanceThreshold();
    tesseract_collision::IsContactAllowedFn fn = manager_->getIsContactAllowedFn();

    manager_ = tess_->getEnvironment()->getDiscreteContactManager();
    manager_->setActiveCollisionObjects(active);
    manager_->setContactDistanceThreshold(contact_distance);
    manager_->setIsContactAllowedFn(fn);

    return true;
  }

  bool callbackComputeContactResultVector(tesseract_msgs::ComputeContactResultVectorRequest& request,
                                          tesseract_msgs::ComputeContactResultVectorResponse& response)
  {
    tesseract_collision::ContactResultMap contact_results;

    // Limit the lock
    {
      boost::mutex::scoped_lock lock(modify_mutex_);

      tess_->getEnvironment()->setState(request.joint_states.name, request.joint_states.position);
      tesseract_environment::EnvState::ConstPtr state = tess_->getEnvironment()->getCurrentState();

      manager_->setCollisionObjectsTransform(state->transforms);
      manager_->contactTest(contact_results, type_);
    }

    tesseract_collision::ContactResultVector contacts_vector;
    tesseract_collision::flattenResults(std::move(contact_results), contacts_vector);
    response.collision_result.contacts.reserve(contacts_vector.size());
    for (const auto& contact : contacts_vector)
    {
      tesseract_msgs::ContactResult contact_msg;
      tesseract_rosutils::toMsg(contact_msg, contact, request.joint_states.header.stamp);
      response.collision_result.contacts.push_back(contact_msg);
    }
    response.success = true;

    return true;
  }

  void callbackTesseractEnvDiff(const tesseract_msgs::TesseractStatePtr& state)
  {
    boost::mutex::scoped_lock lock(modify_mutex_);
    if (!tesseract_rosutils::processMsg(*(tess_->getEnvironment()), *state))
    {
      ROS_ERROR("Invalid TesseractState diff message");
    }

    // Create a new manager
    std::vector<std::string> active = manager_->getActiveCollisionObjects();
    double contact_distance = manager_->getContactDistanceThreshold();
    tesseract_collision::IsContactAllowedFn fn = manager_->getIsContactAllowedFn();

    manager_ = tess_->getEnvironment()->getDiscreteContactManager();
    manager_->setActiveCollisionObjects(active);
    manager_->setContactDistanceThreshold(contact_distance);
    manager_->setIsContactAllowedFn(fn);

    return;
  }

  ContactMonitor(const tesseract::Tesseract::Ptr& tess,
                 ros::NodeHandle& nh,
                 ros::NodeHandle& pnh,
                 const std::vector<std::string>& monitored_link_names,
                 const tesseract_collision::ContactTestType& type,
                 const double contact_distance,
                 const bool publish_environment = false,
                 const bool publish_markers = false
      )
    : tess_ (tess)
    , monitored_link_names_ (monitored_link_names)
    , type_ (type)
    , contact_distance_ (contact_distance)
    , publish_environment_ (publish_environment)
    , publish_markers_ (publish_markers)
  {
    if (tess_ == nullptr)
    {
      ROS_ERROR("Null pointer passed for tesseract object.  Not setting up contact monitor.");
      return;
    }
    manager_ = tess->getEnvironment()->getDiscreteContactManager();
    if (manager_ == nullptr)
    {
      ROS_ERROR("Discrete contact manager is a null pointer.  Not setting up contact monitor.");
      return;
    }
    manager_->setActiveCollisionObjects(monitored_link_names);
    manager_->setContactDistanceThreshold(contact_distance);

    joint_states_sub_ = nh.subscribe("joint_states", 1, &ContactMonitor::callbackJointState, this);
    contact_results_pub_ = pnh.advertise<tesseract_msgs::ContactResultVector>("contact_results", 1, true);
    modify_env_service_ = pnh.advertiseService("modify_environment", &ContactMonitor::callbackModifyTesseractEnv, this);

    if (publish_environment_)
      environment_pub_ = pnh.advertise<tesseract_msgs::TesseractState>("tesseract", 100, false);

    if (publish_markers_)
      contact_marker_pub_ = pnh.advertise<visualization_msgs::MarkerArray>("contact_results_markers", 1, true);

    compute_contact_results_ = pnh.advertiseService("compute_contact_results", &ContactMonitor::callbackComputeContactResultVector, this);

    environment_diff_sub_ = pnh.subscribe("tesseract_diff", 100, &ContactMonitor::callbackTesseractEnvDiff, this);

    return;
  }

  ~ContactMonitor()
  {
    current_joint_states_evt_.notify_all();
    return;
  }

};

} // end namespace contact monitor

int main(int argc, char** argv)
{
  ros::init(argc, argv, "tesseract_contact_monitoring");
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");

  tesseract_scene_graph::SceneGraph::Ptr scene_graph;
  tesseract_scene_graph::SRDFModel::Ptr srdf_model;
  std::string robot_description;
  bool publish_environment;
  bool publish_markers;

  pnh.param<std::string>("robot_description", robot_description, ROBOT_DESCRIPTION_PARAM);
  pnh.param<bool>("publish_environment", publish_environment, false);
  pnh.param<bool>("publish_markers", publish_markers, false);

  // Initial setup
  std::string urdf_xml_string, srdf_xml_string;
  if (!nh.hasParam(robot_description))
  {
    ROS_ERROR("Failed to find parameter: %s", robot_description.c_str());
    return -1;
  }

  if (!nh.hasParam(robot_description + "_semantic"))
  {
    ROS_ERROR("Failed to find parameter: %s", (robot_description + "_semantic").c_str());
    return -1;
  }

  nh.getParam(robot_description, urdf_xml_string);
  nh.getParam(robot_description + "_semantic", srdf_xml_string);

  tesseract::Tesseract::Ptr tess = std::make_shared<tesseract::Tesseract>();
  tesseract_scene_graph::ResourceLocator::Ptr locator = std::make_shared<tesseract_rosutils::ROSResourceLocator>();
  if (!tess->init(urdf_xml_string, srdf_xml_string, locator))
  {
    ROS_ERROR("Failed to initialize environment.");
    return -1;
  }

  double contact_distance;
  pnh.param<double>("contact_distance", contact_distance, DEFAULT_CONTACT_DISTANCE);

  std::vector<std::string> monitored_link_names;
  monitored_link_names = tess->getEnvironment()->getLinkNames();
  if (pnh.hasParam("monitor_links"))
    pnh.getParam("monitor_links", monitored_link_names);

  if (monitored_link_names.empty())
    monitored_link_names = tess->getEnvironment()->getLinkNames();

  int contact_test_type = 2;
  if (pnh.hasParam("contact_test_type"))
    pnh.getParam("contact_test_type", contact_test_type);

  if (contact_test_type < 0 || contact_test_type > 3)
  {
    ROS_WARN("Request type must be 0, 1, 2 or 3. Setting to 2(ALL)!");
    contact_test_type = 2;
  }
  tesseract_collision::ContactTestType type = static_cast<tesseract_collision::ContactTestType>(contact_test_type);

  contact_monitor::ContactMonitor cm(tess,
                                     nh,
                                     pnh,
                                     monitored_link_names,
                                     type,
                                     contact_distance,
                                     publish_environment,
                                     publish_markers);

  boost::thread t (&contact_monitor::ContactMonitor::computeCollisionReportThread, &cm);

  ROS_INFO("Contact Monitor Running!");

  ros::spin();

  return 0;
}

