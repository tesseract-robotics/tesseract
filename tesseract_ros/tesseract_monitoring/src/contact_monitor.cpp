#include <tesseract_environment/core/macros.h>
TESSERACT_ENVIRONMENT_IGNORE_WARNINGS_PUSH
#include <pluginlib/class_loader.hpp>
#include <ros/ros.h>
#include <ros/package.h>
#include <sensor_msgs/JointState.h>
#include <tesseract_msgs/ContactResultVector.h>
#include <tesseract_msgs/ModifyEnvironment.h>
#include <tesseract_msgs/ComputeContactResultVector.h>
TESSERACT_ENVIRONMENT_IGNORE_WARNINGS_POP

#include <tesseract_collision/core/discrete_contact_manager.h>
#include <tesseract/tesseract.h>
#include <tesseract_scene_graph/graph.h>
#include <tesseract_scene_graph/utils.h>
#include <tesseract_scene_graph/parser/urdf_parser.h>
#include <tesseract_scene_graph/parser/srdf_parser.h>
#include <tesseract_environment/kdl/kdl_env.h>
#include <tesseract_environment/core/utils.h>
#include <tesseract_rosutils/utils.h>

using namespace tesseract;
using namespace tesseract_environment;
using namespace tesseract_rosutils;
using namespace tesseract_collision;
using namespace tesseract_scene_graph;

typedef pluginlib::ClassLoader<DiscreteContactManager> DiscreteContactManagerPluginLoader;
typedef std::shared_ptr<DiscreteContactManagerPluginLoader> DiscreteContactManagerPluginLoaderPtr;

const std::string ROBOT_DESCRIPTION_PARAM = "robot_description"; /**< Default ROS parameter for robot description */

const double DEFAULT_CONTACT_DISTANCE = 0.1;

static Tesseract::Ptr tess;
static DiscreteContactManagerPtr manager;
static ros::Subscriber joint_states_sub;
static ros::Publisher contact_results_pub;
static ros::Publisher environment_pub;
static ros::Subscriber environment_diff_sub;
static ros::ServiceServer modify_env_service;
static ros::ServiceServer compute_contact_results;
static ContactTestType type;
static ContactResultMap contacts;
static tesseract_msgs::ContactResultVector contacts_msg;
static bool publish_environment;
static boost::mutex modify_mutex;
static DiscreteContactManagerPluginLoaderPtr discrete_manager_loader; /**< The discrete contact manager loader */


void callbackJointState(const sensor_msgs::JointState::ConstPtr& msg)
{
  boost::mutex::scoped_lock(modify_mutex);
  contacts.clear();
  contacts_msg.contacts.clear();

  tess->getEnvironment()->setState(msg->name, msg->position);
  EnvStateConstPtr state = tess->getEnvironment()->getCurrentState();

  manager->setCollisionObjectsTransform(state->transforms);
  manager->contactTest(contacts, type);

  if (publish_environment)
  {
    tesseract_msgs::TesseractState state_msg;
    toMsg(state_msg, *(tess->getEnvironment()));
    environment_pub.publish(state_msg);
  }

  ContactResultVector contacts_vector;
  tesseract_collision::flattenResults(std::move(contacts), contacts_vector);
  contacts_msg.contacts.reserve(contacts_vector.size());
  for (const auto& contact : contacts_vector)
  {
    tesseract_msgs::ContactResult contact_msg;
    toMsg(contact_msg, contact, msg->header.stamp);
    contacts_msg.contacts.push_back(contact_msg);
  }
  contact_results_pub.publish(contacts_msg);
}

bool callbackModifyTesseractEnv(tesseract_msgs::ModifyEnvironmentRequest& request,
                                tesseract_msgs::ModifyEnvironmentResponse& response)
{
  boost::mutex::scoped_lock(modify_mutex);
  response.success = processMsg(*(tess->getEnvironment()), request.commands);

  // Create a new manager
  std::vector<std::string> active = manager->getActiveCollisionObjects();
  double contact_distance = manager->getContactDistanceThreshold();
  IsContactAllowedFn fn = manager->getIsContactAllowedFn();

  manager = tess->getEnvironment()->getDiscreteContactManager();
  manager->setActiveCollisionObjects(active);
  manager->setContactDistanceThreshold(contact_distance);
  manager->setIsContactAllowedFn(fn);

  return true;
}

bool callbackComputeContactResultVector(tesseract_msgs::ComputeContactResultVectorRequest& request,
                                        tesseract_msgs::ComputeContactResultVectorResponse& response)
{
  ContactResultMap contacts;

  boost::mutex::scoped_lock(modify_mutex);

  tess->getEnvironment()->setState(request.joint_states.name, request.joint_states.position);
  EnvStateConstPtr state = tess->getEnvironment()->getCurrentState();

  manager->setCollisionObjectsTransform(state->transforms);
  manager->contactTest(contacts, type);

  ContactResultVector contacts_vector;
  tesseract_collision::flattenResults(std::move(contacts), contacts_vector);
  response.collision_result.contacts.reserve(contacts_vector.size());
  for (const auto& contact : contacts_vector)
  {
    tesseract_msgs::ContactResult contact_msg;
    toMsg(contact_msg, contact, request.joint_states.header.stamp);
    response.collision_result.contacts.push_back(contact_msg);
  }
  response.success = true;

  return true;
}

void callbackTesseractEnvDiff(const tesseract_msgs::TesseractStatePtr& state)
{
//  if (!state->is_diff)
//    return;

  boost::mutex::scoped_lock(modify_mutex);
  if (!processMsg(*(tess->getEnvironment()), *state))
  {
    ROS_ERROR("Invalid TesseractState diff message");
  }

  // Create a new manager
  std::vector<std::string> active = manager->getActiveCollisionObjects();
  double contact_distance = manager->getContactDistanceThreshold();
  IsContactAllowedFn fn = manager->getIsContactAllowedFn();

  manager = tess->getEnvironment()->getDiscreteContactManager();
  manager->setActiveCollisionObjects(active);
  manager->setContactDistanceThreshold(contact_distance);
  manager->setIsContactAllowedFn(fn);
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "tesseract_contact_monitoring");
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");


  SceneGraphPtr scene_graph;
  SRDFModelPtr srdf_model;
  std::string robot_description;
  std::string plugin;

  pnh.param<std::string>("robot_description", robot_description, ROBOT_DESCRIPTION_PARAM);
  pnh.param<bool>("publish_environment", publish_environment, false);
//  if (pnh.hasParam("plugin"))
//  {
//    discrete_manager_loader.reset(new DiscreteContactManagerPluginLoader("tesseract_collision", "tesseract_collision::DiscreteContactManager"));
//    pnh.getParam("plugin", plugin);
//    if (discrete_manager_loader->isClassAvailable(plugin))
//    {
//      ROS_ERROR("Failed to load tesseract contact checker plugin: %s.", plugin.c_str());
//      return -1;
//    }
//    auto fn = [&]() -> DiscreteContactManagerPtr { return ::discrete_manager_loader->createUniqueInstance(plugin); };
//    env->registerDiscreteContactManager(plugin, fn);
//    env->setActiveDiscreteContactManager(plugin);
//  }

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

  tess = std::make_shared<tesseract::Tesseract>();
  ResourceLocatorFn locator = tesseract_rosutils::locateResource;
  if (!tess->init(urdf_xml_string, srdf_xml_string, locator))
  {
    ROS_ERROR("Failed to initialize environment.");
    return -1;
  }

  // Setup request information
  std::vector<std::string> link_names;
  double contact_distance;

  pnh.param<double>("contact_distance", contact_distance, DEFAULT_CONTACT_DISTANCE);

  link_names = tess->getEnvironment()->getLinkNames();
  if (pnh.hasParam("monitor_links"))
    pnh.getParam("monitor_links", link_names);

  if (link_names.empty())
    link_names = tess->getEnvironment()->getLinkNames();

  int contact_test_type = 2;
  if (pnh.hasParam("contact_test_type"))
    pnh.getParam("contact_test_type", contact_test_type);

  if (contact_test_type < 0 || contact_test_type > 3)
  {
    ROS_WARN("Request type must be 0, 1, 2 or 3. Setting to 2(ALL)!");
    contact_test_type = 2;
  }
  type = static_cast<ContactTestType>(contact_test_type);

  manager = tess->getEnvironment()->getDiscreteContactManager();
  manager->setActiveCollisionObjects(link_names);
  manager->setContactDistanceThreshold(contact_distance);

  joint_states_sub = nh.subscribe("joint_states", 1, &callbackJointState);
  contact_results_pub = pnh.advertise<tesseract_msgs::ContactResultVector>("contact_results", 1, true);
  modify_env_service =
      pnh.advertiseService<tesseract_msgs::ModifyEnvironmentRequest, tesseract_msgs::ModifyEnvironmentResponse>(
          "modify_environment", &callbackModifyTesseractEnv);

  if (publish_environment)
    environment_pub = pnh.advertise<tesseract_msgs::TesseractState>("tesseract", 100, false);

  compute_contact_results = pnh.advertiseService<tesseract_msgs::ComputeContactResultVectorRequest,
                                                 tesseract_msgs::ComputeContactResultVectorResponse>(
      "compute_contact_results", &callbackComputeContactResultVector);

  environment_diff_sub = pnh.subscribe("tesseract_diff", 100, &callbackTesseractEnvDiff);

  ROS_INFO("Contact Monitor Running!");

  ros::spin();

  return 0;
}
