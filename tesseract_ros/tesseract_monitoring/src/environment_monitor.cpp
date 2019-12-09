/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2011, Willow Garage, Inc.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of Willow Garage nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

/* Author: Ioan Sucan */

#include <tesseract_common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <ros/console.h>
#include <dynamic_reconfigure/server.h>
#include <memory>
#include <tesseract_monitoring/EnvironmentMonitorDynamicReconfigureConfig.h>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract_environment/kdl/kdl_env.h>
#include <tesseract_environment/core/utils.h>
#include <tesseract_monitoring/environment_monitor.h>
#include <tesseract_kinematics/core/utils.h>
#include <tesseract_kinematics/kdl/kdl_fwd_kin_chain.h>
#include <tesseract_kinematics/kdl/kdl_fwd_kin_tree.h>
#include <tesseract_scene_graph/utils.h>

class DynamicReconfigureImpl
{
public:
  DynamicReconfigureImpl(tesseract_monitoring::EnvironmentMonitor* owner)
    : owner_(owner), dynamic_reconfigure_server_(ros::NodeHandle(decideNamespace(owner->getName())))
  {
    dynamic_reconfigure_server_.setCallback(
        boost::bind(&DynamicReconfigureImpl::dynamicReconfigureCallback, this, _1, _2));
  }

private:
  // make sure we do not advertise the same service multiple times, in case we
  // use multiple PlanningSceneMonitor
  // instances in a process
  static std::string decideNamespace(const std::string& name)
  {
    std::string ns = "~/" + name;
    std::replace(ns.begin(), ns.end(), ' ', '_');
    std::transform(ns.begin(), ns.end(), ns.begin(), ::tolower);
    if (ros::service::exists(ns + "/set_parameters", false))
    {
      unsigned int c = 1;
      while (ros::service::exists(ns + boost::lexical_cast<std::string>(c) + "/set_parameters", false))
        c++;
      ns += boost::lexical_cast<std::string>(c);
    }
    return ns;
  }

  void dynamicReconfigureCallback(tesseract_monitoring::EnvironmentMonitorDynamicReconfigureConfig& config,
                                  uint32_t /*level*/)
  {
    using namespace tesseract_monitoring;
    EnvironmentMonitor::EnvironmentUpdateType event = EnvironmentMonitor::UPDATE_NONE;
    if (config.publish_geometry_updates)
      event = static_cast<EnvironmentMonitor::EnvironmentUpdateType>(event | EnvironmentMonitor::UPDATE_GEOMETRY);
    if (config.publish_state_updates)
      event = static_cast<EnvironmentMonitor::EnvironmentUpdateType>(event | EnvironmentMonitor::UPDATE_STATE);
    if (config.publish_transforms_updates)
      event = static_cast<EnvironmentMonitor::EnvironmentUpdateType>(event | EnvironmentMonitor::UPDATE_TRANSFORMS);
    if (config.publish_environment)
    {
      owner_->setEnvironmentPublishingFrequency(config.publish_environment_hz);
      owner_->startPublishingEnvironment(event);
    }
    else
      owner_->stopPublishingEnvironment();
  }

  tesseract_monitoring::EnvironmentMonitor* owner_;
  dynamic_reconfigure::Server<tesseract_monitoring::EnvironmentMonitorDynamicReconfigureConfig>
      dynamic_reconfigure_server_;
};

namespace tesseract_monitoring
{
static const std::string LOGNAME = "environment_monitor";
const std::string EnvironmentMonitor::DEFAULT_JOINT_STATES_TOPIC = "joint_states";
const std::string EnvironmentMonitor::DEFAULT_GET_ENVIRONMENT_CHANGES_SERVICE = "get_tesseract_changes";
const std::string EnvironmentMonitor::DEFAULT_GET_ENVIRONMENT_INFORMATION_SERVICE = "get_tesseract_information";
const std::string EnvironmentMonitor::DEFAULT_MODIFY_ENVIRONMENT_SERVICE = "modify_tesseract";
const std::string EnvironmentMonitor::DEFAULT_SAVE_SCENE_GRAPH_SERVICE = "save_scene_graph";
const std::string EnvironmentMonitor::MONITORED_ENVIRONMENT_TOPIC = "monitored_tesseract";

EnvironmentMonitor::EnvironmentMonitor(const std::string& robot_description,
                                       std::string name,
                                       std::string discrete_plugin,
                                       std::string continuous_plugin)
  : monitor_name_(std::move(name))
  , discrete_plugin_name_(std::move(discrete_plugin))
  , continuous_plugin_name_(std::move(continuous_plugin))
  , nh_("~")
{
  // Initial setup
  std::string urdf_xml_string, srdf_xml_string;
  if (!root_nh_.hasParam(robot_description))
  {
    ROS_ERROR("Failed to find parameter: %s", robot_description.c_str());
    return;
  }

  if (!root_nh_.hasParam(robot_description + "_semantic"))
  {
    ROS_ERROR("Failed to find parameter: %s", (robot_description + "_semantic").c_str());
    return;
  }

  root_nh_.getParam(robot_description, urdf_xml_string);
  root_nh_.getParam(robot_description + "_semantic", srdf_xml_string);

  tesseract_ = std::make_shared<tesseract::Tesseract>();
  tesseract_scene_graph::ResourceLocator::Ptr locator = std::make_shared<tesseract_rosutils::ROSResourceLocator>();
  if (!tesseract_->init(urdf_xml_string, srdf_xml_string, locator))
    return;

  initialize();
}

EnvironmentMonitor::EnvironmentMonitor(tesseract::Tesseract::Ptr tesseract,
                                       std::string name,
                                       std::string discrete_plugin,
                                       std::string continuous_plugin)
  : monitor_name_(std::move(name))
  , discrete_plugin_name_(std::move(discrete_plugin))
  , continuous_plugin_name_(std::move(continuous_plugin))
  , tesseract_(std::move(tesseract))
  , nh_("~")
{
  initialize();
}

EnvironmentMonitor::~EnvironmentMonitor()
{
  stopPublishingEnvironment();
  stopStateMonitor();

  delete reconfigure_impl_;
  current_state_monitor_.reset();
  tesseract_.reset();
}

void EnvironmentMonitor::initialize()
{
  enforce_next_state_update_ = false;

  if (monitor_name_.empty())
    monitor_name_ = "tesseract_monitor";

  if (!tesseract_->isInitialized())
  {
    ROS_FATAL_NAMED(LOGNAME, "Faild to initalize environment monitor");
    return;
  }

  try
  {
    discrete_manager_loader_.reset(
        new DiscreteContactManagerPluginLoader("tesseract_collision", "tesseract_collision::DiscreteContactManager"));
    for (auto plugin : discrete_manager_loader_->getDeclaredClasses())
    {
      auto fn = [&]() -> tesseract_collision::DiscreteContactManager::Ptr {
        return discrete_manager_loader_->createUniqueInstance(plugin);
      };
      tesseract_->getEnvironment()->registerDiscreteContactManager(discrete_manager_loader_->getClassType(plugin), fn);

      ROS_INFO("Discrete Contact Monitor Registered: %s", discrete_manager_loader_->getClassType(plugin).c_str());
    }

    // The tesseract sets a default so it is ok if one is not provided here.
    if (!discrete_plugin_name_.empty())
    {
      if (discrete_manager_loader_->isClassAvailable(discrete_plugin_name_))
      {
        ROS_ERROR("Failed to set default tesseract contact checker plugin: %s.", discrete_plugin_name_.c_str());
      }
      else
      {
        tesseract_->getEnvironment()->setActiveDiscreteContactManager(discrete_plugin_name_);
      }
    }

    continuous_manager_loader_.reset(new ContinuousContactManagerPluginLoader("tesseract_collision",
                                                                              "tesseract_collision::"
                                                                              "ContinuousContactManager"));
    for (auto plugin : continuous_manager_loader_->getDeclaredClasses())
    {
      auto fn = [&]() -> tesseract_collision::ContinuousContactManager::Ptr {
        return continuous_manager_loader_->createUniqueInstance(plugin);
      };
      tesseract_->getEnvironment()->registerContinuousContactManager(continuous_manager_loader_->getClassType(plugin),
                                                                     fn);

      ROS_INFO("Continuous Contact Monitor Registered: %s", continuous_manager_loader_->getClassType(plugin).c_str());
    }

    if (!continuous_plugin_name_.empty())
    {
      if (continuous_manager_loader_->isClassAvailable(continuous_plugin_name_))
      {
        ROS_ERROR("Failed to set default tesseract contact checker plugin: %s.", continuous_plugin_name_.c_str());
      }
      else
      {
        tesseract_->getEnvironment()->setActiveContinuousContactManager(continuous_plugin_name_);
      }
    }
  }
  catch (int& /*e*/)
  {
    ROS_ERROR_NAMED(LOGNAME, "Failed to load tesseract contact managers plugin");
    tesseract_.reset();
  }

  publish_environment_frequency_ = 2.0;
  new_environment_update_ = UPDATE_NONE;

  last_update_time_ = last_robot_motion_time_ = ros::Time::now();
  last_robot_state_update_wall_time_ = ros::WallTime::now();
  dt_state_update_ = ros::WallDuration(0.1);

  state_update_pending_ = false;
  state_update_timer_ = nh_.createWallTimer(dt_state_update_,
                                            &EnvironmentMonitor::stateUpdateTimerCallback,
                                            this,
                                            false,   // not a oneshot timer
                                            false);  // do not start the timer yet

  reconfigure_impl_ = new DynamicReconfigureImpl(this);

  modify_environment_server_ =
      nh_.advertiseService(DEFAULT_MODIFY_ENVIRONMENT_SERVICE, &EnvironmentMonitor::modifyEnvironmentCallback, this);

  get_environment_changes_server_ = nh_.advertiseService(
      DEFAULT_GET_ENVIRONMENT_CHANGES_SERVICE, &EnvironmentMonitor::getEnvironmentChangesCallback, this);

  get_environment_information_server_ = nh_.advertiseService(
      DEFAULT_GET_ENVIRONMENT_INFORMATION_SERVICE, &EnvironmentMonitor::getEnvironmentInformationCallback, this);

  save_scene_graph_server_ =
      nh_.advertiseService(DEFAULT_SAVE_SCENE_GRAPH_SERVICE, &EnvironmentMonitor::saveSceneGraphCallback, this);
}

void EnvironmentMonitor::stopPublishingEnvironment()
{
  if (publish_environment_)
  {
    std::unique_ptr<boost::thread> copy;
    copy.swap(publish_environment_);
    new_environment_update_condition_.notify_all();
    copy->join();
    stopPublishingEnvironment();
    environment_publisher_.shutdown();
    ROS_INFO_NAMED(LOGNAME, "Stopped publishing maintained environment.");
  }
}

void EnvironmentMonitor::startPublishingEnvironment(EnvironmentUpdateType update_type,
                                                    const std::string& environment_topic)
{
  publish_update_types_ = update_type;
  if (!publish_environment_ && tesseract_->isInitialized())
  {
    environment_publisher_ = nh_.advertise<tesseract_msgs::TesseractState>(environment_topic, 100, false);
    ROS_INFO_NAMED(LOGNAME, "Publishing maintained environment on '%s'", environment_topic.c_str());
    publish_environment_ =
        std::make_unique<boost::thread>(boost::bind(&EnvironmentMonitor::environmentPublishingThread, this));
  }
}

void EnvironmentMonitor::environmentPublishingThread()
{
  ROS_DEBUG_NAMED(LOGNAME, "Started environment state publishing thread ...");

  // publish the full planning scene
  tesseract_msgs::TesseractState start_msg;
  tesseract_rosutils::toMsg(start_msg, *(tesseract_->getEnvironment()));

  environment_publisher_.publish(start_msg);
  ros::Duration(1.5).sleep();
  environment_publisher_.publish(start_msg);

  ROS_DEBUG_NAMED(LOGNAME, "Published the Tesseract Environment State for: '%s'", start_msg.id.c_str());

  do
  {
    tesseract_msgs::TesseractState msg;
    bool publish_msg = false;
    ros::Rate rate(publish_environment_frequency_);
    {
      boost::unique_lock<boost::shared_mutex> ulock(scene_update_mutex_);
      while (new_environment_update_ == UPDATE_NONE && publish_environment_)
        new_environment_update_condition_.wait(ulock);
      if (new_environment_update_ != UPDATE_NONE)
      {
        if ((publish_update_types_ & new_environment_update_) || new_environment_update_ == UPDATE_ENVIRONMENT)
        {
          tesseract_rosutils::toMsg(msg, *(tesseract_->getEnvironment()));

          // also publish timestamp of this robot_state
          msg.joint_state.header.stamp = last_robot_motion_time_;
          publish_msg = true;
        }
        new_environment_update_ = UPDATE_NONE;
      }
    }

    if (publish_msg)
    {
      rate.reset();
      environment_publisher_.publish(msg);
      rate.sleep();
    }
  } while (publish_environment_);
}

void EnvironmentMonitor::getMonitoredTopics(std::vector<std::string>& topics) const
{
  topics.clear();
  if (current_state_monitor_)
  {
    const std::string& t = current_state_monitor_->getMonitoredTopic();
    if (!t.empty())
      topics.push_back(t);
  }
}

void EnvironmentMonitor::triggerEnvironmentUpdateEvent(EnvironmentUpdateType update_type)
{
  // do not modify update functions while we are calling them
  boost::recursive_mutex::scoped_lock lock(update_lock_);

  for (auto& update_callback : update_callbacks_)
    update_callback(update_type);
  new_environment_update_ = static_cast<EnvironmentUpdateType>(new_environment_update_ | update_type);
  new_environment_update_condition_.notify_all();
}

void EnvironmentMonitor::newStateCallback(const tesseract_msgs::TesseractStateConstPtr& env)
{
  if (!tesseract_->getEnvironment())
    return;

  EnvironmentUpdateType upd = UPDATE_ENVIRONMENT;
  std::string old_scene_name;
  {
    boost::unique_lock<boost::shared_mutex> ulock(scene_update_mutex_);

    last_update_time_ = ros::Time::now();
    last_robot_motion_time_ = env->joint_state.header.stamp;
    ROS_DEBUG_STREAM_NAMED(LOGNAME,
                           "environment update " << fmod(last_update_time_.toSec(), 10.)
                                                 << " robot stamp: " << fmod(last_robot_motion_time_.toSec(), 10.));
    old_scene_name = tesseract_->getEnvironment()->getName();
    tesseract_rosutils::processMsg(tesseract_->getEnvironment(), *env);
  }

  upd = UPDATE_NONE;
  if (!tesseract_rosutils::isMsgEmpty(env->joint_state) || !tesseract_rosutils::isMsgEmpty(env->multi_dof_joint_state))
    upd = static_cast<EnvironmentUpdateType>(upd | UPDATE_STATE);

  triggerEnvironmentUpdateEvent(upd);
}

bool EnvironmentMonitor::applyEnvironmentCommandsMessage(
    const std::string& id,
    int revision,
    const std::vector<tesseract_msgs::EnvironmentCommand>& commands)
{
  if (!tesseract_->getEnvironment() || id != tesseract_->getEnvironment()->getName() ||
      revision != tesseract_->getEnvironment()->getRevision())
    return false;

  bool result;

  EnvironmentUpdateType upd = UPDATE_ENVIRONMENT;
  std::string old_scene_name;
  {
    boost::unique_lock<boost::shared_mutex> ulock(scene_update_mutex_);
    result = tesseract_rosutils::processMsg(*(tesseract_->getEnvironment()), commands);
  }

  // if we have a diff, try to more accuratelly determine the update type
  //  if (env_msg.is_diff)
  //  {
  //    //    bool no_other_scene_upd = (env_msg.name.empty() || env_msg.name ==
  //    //    old_scene_name) &&
  //    //                               env_msg.allowed_collision_matrix.entry_names.empty();

  //    // TODO: Levi Need to add back allowed collision matrix
  //    bool no_other_scene_upd = (env_msg.name.empty() || env_msg.name == old_scene_name);

  //    if (no_other_scene_upd)
  //    {
  //      upd = UPDATE_NONE;
  //      if (!env_msg.attachable_objects.empty() || !env_msg.attached_bodies.empty())
  //        upd = static_cast<EnvironmentUpdateType>(upd | UPDATE_GEOMETRY);

  //      //      if (!env.fixed_frame_transforms.empty())
  //      //        upd = (EnvironmentUpdateType)((int)upd |
  //      //        (int)UPDATE_TRANSFORMS);

  //      if (!tesseract_ros::isMsgEmpty(env_msg.multi_dof_joint_state) ||
  //          !tesseract_ros::isMsgEmpty(env_msg.multi_dof_joint_state))
  //        upd = static_cast<EnvironmentUpdateType>(upd | UPDATE_STATE);
  //    }
  //  }
  triggerEnvironmentUpdateEvent(upd);
  return result;
}

// void EnvironmentMonitor::newPlanningSceneWorldCallback(
//    const moveit_msgs::PlanningSceneWorldConstPtr& world)
//{
//  if (scene_)
//  {
//    updateFrameTransforms();
//    {
//      boost::unique_lock<boost::shared_mutex> ulock(scene_update_mutex_);
//      last_update_time_ = ros::Time::now();
//      scene_->getWorldNonConst()->clearObjects();
//      scene_->processPlanningSceneWorldMsg(*world);
//      if (octomap_monitor_)
//      {
//        if (world->octomap.octomap.data.empty())
//        {
//          octomap_monitor_->getOcTreePtr()->lockWrite();
//          octomap_monitor_->getOcTreePtr()->clear();
//          octomap_monitor_->getOcTreePtr()->unlockWrite();
//        }
//      }
//    }
//    triggerSceneUpdateEvent(UPDATE_SCENE);
//  }
//}

// void EnvironmentMonitor::collisionObjectFailTFCallback(
//    const moveit_msgs::CollisionObjectConstPtr& obj,
//    tf::filter_failure_reasons::FilterFailureReason reason)
//{
//  // if we just want to remove objects, the frame does not matter
//  if (reason == tf::filter_failure_reasons::EmptyFrameID && obj->operation ==
//  moveit_msgs::CollisionObject::REMOVE)
//    collisionObjectCallback(obj);
//}

// void EnvironmentMonitor::attachableObjectCallback(const
// tesseract_msgs::AttachableObjectConstPtr &ao_msg)
//{
//  if (env_)
//  {
//    {
//      boost::unique_lock<boost::shared_mutex> ulock(scene_update_mutex_);
//      last_update_time_ = ros::Time::now();

//      tesseract_ros::processAttachableObjectMsg(env_, *ao_msg);
//    }
//    triggerEnvironmentUpdateEvent(UPDATE_GEOMETRY);
//  }
//}

// void EnvironmentMonitor::attachedBodyInfoCallback(const
// tesseract_msgs::AttachedBodyInfoConstPtr &ab_info_msg)
//{
//  if (env_)
//  {
//    {
//      boost::unique_lock<boost::shared_mutex> ulock(scene_update_mutex_);
//      last_update_time_ = ros::Time::now();

//      tesseract_ros::processAttachedBodyInfoMsg(env_, *ab_info_msg);
//    }
//    triggerEnvironmentUpdateEvent(UPDATE_GEOMETRY);
//  }
//}

bool EnvironmentMonitor::saveSceneGraphCallback(tesseract_msgs::SaveSceneGraphRequest& req,
                                                tesseract_msgs::SaveSceneGraphResponse& res)
{
  auto env = tesseract_->getEnvironment();
  res.success = !(env == nullptr);
  env->getSceneGraph()->saveDOT(req.filepath);
  res.id = env->getName();
  res.revision = static_cast<unsigned long>(env->getRevision());

  return true;
}

bool EnvironmentMonitor::waitForCurrentState(const ros::Time& t, double wait_time)
{
  if (t.isZero())
    return false;
  ros::WallTime start = ros::WallTime::now();
  ros::WallDuration timeout(wait_time);

  ROS_DEBUG_NAMED(LOGNAME, "sync robot state to: %.3fs", fmod(t.toSec(), 10.));

  if (current_state_monitor_)
  {
    // Wait for next robot update in state monitor. Those updates are only
    // passed to PSM when robot actually moved!
    enforce_next_state_update_ = true;  // enforce potential updates to be directly applied
    bool success = current_state_monitor_->waitForCurrentState(t, wait_time);
    enforce_next_state_update_ = false;  // back to normal throttling behavior,
                                         // not applying incoming updates
                                         // immediately

    /* If the robot doesn't move, we will never receive an update from CSM in
       planning scene.
       As we ensured that an update, if it is triggered by CSM, is directly
       passed to the scene,
       we can early return true here (if we successfully received a CSM update).
       Otherwise return false. */
    if (success)
      return true;

    ROS_WARN_NAMED(LOGNAME, "Failed to fetch current robot state.");
    return false;
  }

  // Sometimes there is no state monitor. In this case state updates are
  // received as part of scene updates only.
  // However, scene updates are only published if the robot actually moves.
  // Hence we need a timeout!
  // As publishing planning scene updates is throttled (2Hz by default), a 1s
  // timeout is a suitable default.
  boost::shared_lock<boost::shared_mutex> lock(scene_update_mutex_);
  ros::Time prev_robot_motion_time = last_robot_motion_time_;
  while (last_robot_motion_time_ < t &&  // Wait until the state update actually reaches the scene.
         timeout > ros::WallDuration())
  {
    ROS_DEBUG_STREAM_NAMED(LOGNAME, "last robot motion: " << (t - last_robot_motion_time_).toSec() << " ago");
    new_environment_update_condition_.wait_for(lock, boost::chrono::nanoseconds(timeout.toNSec()));
    timeout -= ros::WallTime::now() - start;  // compute remaining wait_time
  }
  bool success = last_robot_motion_time_ >= t;
  // suppress warning if we received an update at all
  if (!success && prev_robot_motion_time != last_robot_motion_time_)
    ROS_WARN_NAMED(
        LOGNAME, "Maybe failed to update robot state, time diff: %.3fs", (t - last_robot_motion_time_).toSec());

  ROS_DEBUG_STREAM_NAMED(LOGNAME,
                         "sync done: robot motion: " << (t - last_robot_motion_time_).toSec()
                                                     << " scene update: " << (t - last_update_time_).toSec());
  return success;
}

void EnvironmentMonitor::lockEnvironmentRead() { scene_update_mutex_.lock_shared(); }
void EnvironmentMonitor::unlockEnvironmentRead() { scene_update_mutex_.unlock_shared(); }
void EnvironmentMonitor::lockEnvironmentWrite() { scene_update_mutex_.lock(); }
void EnvironmentMonitor::unlockEnvironmentWrite() { scene_update_mutex_.unlock(); }

void EnvironmentMonitor::startStateMonitor(const std::string& joint_states_topic)
{
  stopStateMonitor();
  if (tesseract_->getEnvironment())
  {
    if (!current_state_monitor_)
      current_state_monitor_.reset(
          new CurrentStateMonitor(tesseract_->getEnvironment(), tesseract_->getFwdKinematicsManager(), root_nh_));

    current_state_monitor_->addUpdateCallback(boost::bind(&EnvironmentMonitor::onStateUpdate, this, _1));
    current_state_monitor_->startStateMonitor(joint_states_topic);

    {
      boost::mutex::scoped_lock lock(state_pending_mutex_);
      if (!dt_state_update_.isZero())
        state_update_timer_.start();
    }
  }
  else
  {
    ROS_ERROR_NAMED(LOGNAME, "Cannot monitor robot state because planning scene is not configured");
  }
}

void EnvironmentMonitor::stopStateMonitor()
{
  if (current_state_monitor_)
    current_state_monitor_->stopStateMonitor();

  // stop must be called with state_pending_mutex_ unlocked to avoid deadlock
  state_update_timer_.stop();
  {
    boost::mutex::scoped_lock lock(state_pending_mutex_);
    state_update_pending_ = false;
  }
}

void EnvironmentMonitor::onStateUpdate(const sensor_msgs::JointStateConstPtr& /* joint_state */)
{
  const ros::WallTime& n = ros::WallTime::now();
  ros::WallDuration dt = n - last_robot_state_update_wall_time_;

  bool update = enforce_next_state_update_;
  {
    boost::mutex::scoped_lock lock(state_pending_mutex_);

    if (dt < dt_state_update_ && !update)
    {
      state_update_pending_ = true;
    }
    else
    {
      state_update_pending_ = false;
      last_robot_state_update_wall_time_ = n;
      update = true;
    }
  }
  // run the state update with state_pending_mutex_ unlocked
  if (update)
    updateEnvironmentWithCurrentState();
}

void EnvironmentMonitor::stateUpdateTimerCallback(const ros::WallTimerEvent& /*event*/)
{
  if (state_update_pending_)
  {
    bool update = false;

    const ros::WallTime& n = ros::WallTime::now();
    ros::WallDuration dt = n - last_robot_state_update_wall_time_;

    {
      // lock for access to dt_state_update_ and state_update_pending_
      boost::mutex::scoped_lock lock(state_pending_mutex_);
      if (state_update_pending_ && dt >= dt_state_update_)
      {
        state_update_pending_ = false;
        last_robot_state_update_wall_time_ = ros::WallTime::now();
        update = true;
        ROS_DEBUG_STREAM_NAMED(LOGNAME,
                               "performPendingStateUpdate: " << fmod(last_robot_state_update_wall_time_.toSec(), 10));
      }
    }

    // run the state update with state_pending_mutex_ unlocked
    if (update)
    {
      updateEnvironmentWithCurrentState();
      ROS_DEBUG_NAMED(LOGNAME, "performPendingStateUpdate done");
    }
  }
}

void EnvironmentMonitor::setStateUpdateFrequency(double hz)
{
  bool update = false;
  if (hz > std::numeric_limits<double>::epsilon())
  {
    boost::mutex::scoped_lock lock(state_pending_mutex_);
    dt_state_update_.fromSec(1.0 / hz);
    state_update_timer_.setPeriod(dt_state_update_);
    state_update_timer_.start();
  }
  else
  {
    // stop must be called with state_pending_mutex_ unlocked to avoid deadlock
    state_update_timer_.stop();
    boost::mutex::scoped_lock lock(state_pending_mutex_);
    dt_state_update_ = ros::WallDuration(0, 0);
    if (state_update_pending_)
      update = true;
  }
  ROS_INFO_NAMED(LOGNAME, "Updating internal planning scene state at most every %lf seconds", dt_state_update_.toSec());

  if (update)
    updateEnvironmentWithCurrentState();
}

void EnvironmentMonitor::updateEnvironmentWithCurrentState()
{
  if (current_state_monitor_)
  {
    std::vector<std::string> missing;
    if (!current_state_monitor_->haveCompleteState(missing) &&
        (ros::Time::now() - current_state_monitor_->getMonitorStartTime()).toSec() > 1.0)
    {
      std::string missing_str = boost::algorithm::join(missing, ", ");
      ROS_WARN_THROTTLE_NAMED(
          1, LOGNAME, "The complete state of the robot is not yet known.  Missing %s", missing_str.c_str());
    }

    {
      boost::unique_lock<boost::shared_mutex> ulock(scene_update_mutex_);
      last_update_time_ = last_robot_motion_time_ = current_state_monitor_->getCurrentStateTime();
      ROS_DEBUG_STREAM_NAMED(LOGNAME, "robot state update " << fmod(last_robot_motion_time_.toSec(), 10.));

      tesseract_->getEnvironment()->setState(current_state_monitor_->getCurrentState()->joints);
    }
    triggerEnvironmentUpdateEvent(UPDATE_STATE);
  }
  else
    ROS_ERROR_THROTTLE_NAMED(1, LOGNAME, "State monitor is not active. Unable to set the planning scene state");
}

void EnvironmentMonitor::addUpdateCallback(const boost::function<void(EnvironmentUpdateType)>& fn)
{
  boost::recursive_mutex::scoped_lock lock(update_lock_);
  if (fn)
    update_callbacks_.push_back(fn);
}

void EnvironmentMonitor::clearUpdateCallbacks()
{
  boost::recursive_mutex::scoped_lock lock(update_lock_);
  update_callbacks_.clear();
}

void EnvironmentMonitor::setEnvironmentPublishingFrequency(double hz)
{
  publish_environment_frequency_ = hz;
  ROS_DEBUG_NAMED(
      LOGNAME, "Maximum frquency for publishing an environment is now %lf Hz", publish_environment_frequency_);
}

bool EnvironmentMonitor::modifyEnvironmentCallback(tesseract_msgs::ModifyEnvironmentRequest& req,
                                                   tesseract_msgs::ModifyEnvironmentResponse& res)
{
  res.success = applyEnvironmentCommandsMessage(req.id, static_cast<int>(req.revision), req.commands);
  res.revision = static_cast<unsigned long>(tesseract_->getEnvironmentConst()->getRevision());
  return res.success;
}

bool EnvironmentMonitor::getEnvironmentChangesCallback(tesseract_msgs::GetEnvironmentChangesRequest& req,
                                                       tesseract_msgs::GetEnvironmentChangesResponse& res)
{
  if (static_cast<int>(req.revision) > tesseract_->getEnvironment()->getRevision())
  {
    res.success = false;
    return false;
  }

  res.id = tesseract_->getEnvironment()->getName();
  res.revision = static_cast<unsigned long>(tesseract_->getEnvironment()->getRevision());
  if (!tesseract_rosutils::toMsg(res.commands, tesseract_->getEnvironment()->getCommandHistory(), req.revision))
  {
    res.success = false;
    return false;
  }

  res.success = true;
  return res.success;
}

bool EnvironmentMonitor::getEnvironmentInformationCallback(tesseract_msgs::GetEnvironmentInformationRequest& req,
                                                           tesseract_msgs::GetEnvironmentInformationResponse& res)
{
  res.id = tesseract_->getEnvironment()->getName();
  res.revision = static_cast<unsigned long>(tesseract_->getEnvironment()->getRevision());

  if (req.flags & tesseract_msgs::GetEnvironmentInformationRequest::COMMAND_HISTORY)
  {
    if (!tesseract_rosutils::toMsg(res.command_history, tesseract_->getEnvironment()->getCommandHistory(), 0))
    {
      res.success = false;
      return false;
    }
  }

  if (req.flags & tesseract_msgs::GetEnvironmentInformationRequest::LINK_LIST)
  {
    for (const auto& link : tesseract_->getEnvironmentConst()->getSceneGraph()->getLinks())
    {
      tesseract_msgs::Link msg;
      if (!tesseract_rosutils::toMsg(msg, *link))
      {
        res.success = false;
        return false;
      }
      res.links.push_back(msg);
    }
  }

  if (req.flags & tesseract_msgs::GetEnvironmentInformationRequest::JOINT_LIST)
  {
    for (const auto& joint : tesseract_->getEnvironmentConst()->getSceneGraph()->getJoints())
    {
      tesseract_msgs::Joint msg;
      if (!tesseract_rosutils::toMsg(msg, *joint))
      {
        res.success = false;
        return false;
      }
      res.joints.push_back(msg);
    }
  }

  if (req.flags & tesseract_msgs::GetEnvironmentInformationRequest::LINK_NAMES)
  {
    for (const auto& link : tesseract_->getEnvironmentConst()->getLinkNames())
    {
      res.link_names.push_back(link);
    }
  }

  if (req.flags & tesseract_msgs::GetEnvironmentInformationRequest::JOINT_NAMES)
  {
    for (const auto& joint : tesseract_->getEnvironmentConst()->getJointNames())
    {
      res.joint_names.push_back(joint);
    }
  }

  if (req.flags & tesseract_msgs::GetEnvironmentInformationRequest::ACTIVE_LINK_NAMES)
  {
    for (const auto& link : tesseract_->getEnvironmentConst()->getActiveLinkNames())
    {
      res.active_link_names.push_back(link);
    }
  }

  if (req.flags & tesseract_msgs::GetEnvironmentInformationRequest::ACTIVE_JOINT_NAMES)
  {
    for (const auto& joint : tesseract_->getEnvironmentConst()->getActiveJointNames())
    {
      res.active_joint_names.push_back(joint);
    }
  }

  if (req.flags & tesseract_msgs::GetEnvironmentInformationRequest::LINK_TRANSFORMS)
  {
    for (const auto& link_pair : tesseract_->getEnvironmentConst()->getCurrentState()->transforms)
    {
      res.link_transforms.names.push_back(link_pair.first);
      geometry_msgs::Pose pose;
      tf::poseEigenToMsg(link_pair.second, pose);
      res.link_transforms.transforms.push_back(pose);
    }
  }

  if (req.flags & tesseract_msgs::GetEnvironmentInformationRequest::JOINT_TRANSFORMS)
  {
    tesseract_environment::EnvState::ConstPtr current_state = tesseract_->getEnvironmentConst()->getCurrentState();
    for (const auto& joint : tesseract_->getEnvironmentConst()->getSceneGraph()->getJoints())
    {
      res.joint_transforms.names.push_back(joint->getName());
      geometry_msgs::Pose pose;
      tf::poseEigenToMsg(
          current_state->transforms.at(joint->parent_link_name) * joint->parent_to_joint_origin_transform, pose);
      res.joint_transforms.transforms.push_back(pose);
    }
  }

  if (req.flags & tesseract_msgs::GetEnvironmentInformationRequest::ALLOWED_COLLISION_MATRIX)
  {
    if (!tesseract_rosutils::toMsg(res.allowed_collision_matrix,
                                   *tesseract_->getEnvironmentConst()->getAllowedCollisionMatrix()))
    {
      res.success = false;
      return false;
    }
  }

  res.success = true;
  return true;
}

}  // namespace tesseract_monitoring
