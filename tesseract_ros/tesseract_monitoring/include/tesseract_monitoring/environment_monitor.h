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

#ifndef TESSERACT_MONITORING_ENVIRONMENT_H
#define TESSERACT_MONITORING_ENVIRONMENT_H

#include <tesseract_common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <pluginlib/class_loader.hpp>
#include <ros/ros.h>
#include <tf/tf.h>
#include <tf/message_filter.h>
#include <message_filters/subscriber.h>
#include <boost/noncopyable.hpp>
#include <boost/thread/shared_mutex.hpp>
#include <boost/thread/recursive_mutex.hpp>
#include <memory>
#include <tesseract_msgs/TesseractState.h>
#include <tesseract_msgs/ModifyEnvironment.h>
#include <tesseract_msgs/GetEnvironmentChanges.h>
#include <tesseract_msgs/GetEnvironmentInformation.h>
#include <tesseract_msgs/SaveSceneGraph.h>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract_collision/core/discrete_contact_manager.h>
#include <tesseract_collision/core/continuous_contact_manager.h>
#include <tesseract_environment/core/environment.h>
#include <tesseract_monitoring/current_state_monitor.h>
#include <tesseract_rosutils/utils.h>
#include <tesseract_monitoring/environment_monitor.h>
#include <tesseract_scene_graph/graph.h>
#include <tesseract_scene_graph/parser/srdf_parser.h>
#include <tesseract_urdf/urdf_parser.h>
#include <tesseract_kinematics/core/forward_kinematics.h>

class DynamicReconfigureImpl;

namespace tesseract_monitoring
{
using DiscreteContactManagerPluginLoader = pluginlib::ClassLoader<tesseract_collision::DiscreteContactManager>;
using DiscreteContactManagerPluginLoaderPtr = std::shared_ptr<DiscreteContactManagerPluginLoader>;

using ContinuousContactManagerPluginLoader = pluginlib::ClassLoader<tesseract_collision::ContinuousContactManager>;
using ContinuousContactManagerPluginLoaderPtr = std::shared_ptr<ContinuousContactManagerPluginLoader>;

/**
 * @brief TesseractMonitor
 * Subscribes to the topic \e tesseract_environment */
class EnvironmentMonitor : private boost::noncopyable
{
public:
  enum EnvironmentUpdateType
  {
    /** \brief No update */
    UPDATE_NONE = 0,

    /** \brief The state in the monitored environment was updated */
    UPDATE_STATE = 1,

    /** \brief The maintained set of fixed transforms in the monitored environment was updated */
    UPDATE_TRANSFORMS = 2,

    /** \brief The geometry of the environment was updated. This includes receiving new octomaps, collision objects,
       attached
       objects, environment geometry, etc. */
    UPDATE_GEOMETRY = 4,

    /** \brief The entire environment was updated */
    UPDATE_ENVIRONMENT = 8 + UPDATE_STATE + UPDATE_TRANSFORMS + UPDATE_GEOMETRY
  };

  /// The name of the topic used by default for receiving joint states
  static const std::string DEFAULT_JOINT_STATES_TOPIC;  // "/joint_states"

  /// The name of the service used by default for requesting tesseract environment change history
  static const std::string DEFAULT_GET_ENVIRONMENT_CHANGES_SERVICE;  // "/get_tesseract_changes"

  /// The name of the service used by default for requesting tesseract environment information
  static const std::string DEFAULT_GET_ENVIRONMENT_INFORMATION_SERVICE;  // "/get_tesseract_information"

  /// The name of the service used by default for setting the full tesseract environment state
  static const std::string DEFAULT_MODIFY_ENVIRONMENT_SERVICE;  // "/modify_tesseract"

  /// The name of the service used by default for saving the scene graph as a DOT
  static const std::string DEFAULT_SAVE_SCENE_GRAPH_SERVICE;  //"/save_scene_graph"

  /// The name of the topic used by default for publishing the monitored tesseract environment (this is without "/" in
  /// the name, so the topic is prefixed by the node name)
  static const std::string MONITORED_ENVIRONMENT_TOPIC;  // "/monitored_tesseract"

  /** @brief Constructor
   *  @param robot_description The name of the ROS parameter that contains the URDF (in string format)
   *  @param tf A pointer to a tf::Transformer
   *  @param name A name identifying this planning scene monitor
   */
  EnvironmentMonitor(const std::string& robot_description,
                     std::string name,
                     std::string discrete_plugin = "",
                     std::string continuous_plugin = "");

  /** @brief Constructor
   *  @param rml A pointer to a kinematic model loader
   *  @param tf A pointer to a tf::Transformer
   *  @param name A name identifying this planning scene monitor
   */
  EnvironmentMonitor(tesseract::Tesseract::Ptr tesseract,
                     std::string name,
                     std::string discrete_plugin = "",
                     std::string continuous_plugin = "");

  ~EnvironmentMonitor();
  EnvironmentMonitor(const EnvironmentMonitor&) = delete;
  EnvironmentMonitor& operator=(const EnvironmentMonitor&) = delete;
  EnvironmentMonitor(EnvironmentMonitor&&) = delete;
  EnvironmentMonitor& operator=(EnvironmentMonitor&&) = delete;

  /** \brief Get the name of this monitor */
  const std::string& getName() const { return monitor_name_; }
  const tesseract_scene_graph::SceneGraph::ConstPtr& getSceneGraph() const
  {
    return tesseract_->getEnvironment()->getSceneGraph();
  }
  const tesseract_scene_graph::SRDFModel::ConstPtr& getSRDF() const { return tesseract_->getSRDFModel(); }
  /** @brief <b>Avoid this function!</b>  Returns an @b
   *         unsafe pointer to the current planning scene.
   * @warning Most likely you do not want to call this function
   *          directly.  PlanningSceneMonitor has a background thread
   *          which repeatedly updates and clobbers various contents
   *          of its internal PlanningScene instance.  This function
   *          just returns a pointer to that dynamic internal object.
   *          The correct thing is usually to use a
   *          LockedPlanningSceneRO or LockedPlanningSceneRW, which
   *          locks the PlanningSceneMonitor and provides safe access
   *          to the PlanningScene object.
   * @see LockedPlanningSceneRO
   * @see LockedPlanningSceneRW.
   * @return A pointer to the current planning scene.*/
  const tesseract_environment::Environment::Ptr& getEnvironment() { return tesseract_->getEnvironment(); }
  /*! @brief <b>Avoid this function!</b>  Returns an @b
   *         unsafe pointer to the current planning scene.
   * @copydetails PlanningSceneMonitor::getPlanningScene() */
  const tesseract_environment::Environment::ConstPtr& getEnvironment() const
  {
    return tesseract_->getEnvironmentConst();
  }

  /**
   * @brief Get Tesseract Non Const
   * @return A shared point to a tesseract object
   */
  const tesseract::Tesseract::Ptr& getTesseract() { return tesseract_; }

  /**
   * @brief Get Tesseract Const
   * @return A shared point to a const tesseract object
   */
  tesseract::Tesseract::ConstPtr getTesseractConst() const { return tesseract_; }

  /** @brief Return true if the scene \e scene can be updated directly
      or indirectly by this monitor. This function will return true if
      the pointer of the scene is the same as the one maintained,
      or if a parent of the scene is the one maintained. */
  bool updatesEnvironment(const tesseract_environment::Environment::ConstPtr& env) const;

  /** @brief Return true if the scene \e scene can be updated directly
      or indirectly by this monitor. This function will return true if
      the pointer of the scene is the same as the one maintained,
      or if a parent of the scene is the one maintained. */
  bool updatesEnvironment(const tesseract_environment::Environment::Ptr& env) const;

  /** @brief Get the stored robot description
   *  @return An instance of the stored robot description*/
  const std::string& getURDFDescription() const { return robot_description_; }

  /** \brief Start publishing the maintained environment. The first message set out is a complete environment.
      Diffs are sent afterwards on updates specified by the \e event bitmask. For UPDATE_ENVIRONMENT, the full
     environment is always
     sent. */
  void startPublishingEnvironment(EnvironmentUpdateType update_type,
                                  const std::string& environment_topic = MONITORED_ENVIRONMENT_TOPIC);

  /** \brief Stop publishing the maintained environment. */
  void stopPublishingEnvironment();

  /** \brief Set the maximum frequency at which environment are being published */
  void setEnvironmentPublishingFrequency(double hz);

  /** \brief Get the maximum frequency at which environment are published (Hz) */
  double getEnvironmentPublishingFrequency() const { return publish_environment_frequency_; }
  /** @brief Get the stored instance of the stored current state monitor
   *  @return An instance of the stored current state monitor*/
  const CurrentStateMonitorPtr& getStateMonitor() const { return current_state_monitor_; }
  CurrentStateMonitorPtr& getStateMonitorNonConst() { return current_state_monitor_; }
  /** @brief Start the current state monitor
      @param joint_states_topic the topic to listen to for joint states
      @param attached_objects_topic the topic to listen to for attached collision objects */
  void startStateMonitor(const std::string& joint_states_topic = DEFAULT_JOINT_STATES_TOPIC);

  /** @brief Stop the state monitor*/
  void stopStateMonitor();

  /** @brief Update the scene using the monitored state. This function is automatically called when an update to the
     current state is received (if startStateMonitor() has been called).
      The updates are throttled to a maximum update frequency however, which is set by setStateUpdateFrequency(). */
  void updateEnvironmentWithCurrentState();

  /** @brief Update the scene using the monitored state at a specified frequency, in Hz. This function has an effect
     only when updates from the CurrentStateMonitor are received at a higher frequency.
      In that case, the updates are throttled down, so that they do not exceed a maximum update frequency specified
     here.
      @param hz the update frequency. By default this is 10Hz. */
  void setStateUpdateFrequency(double hz);

  /** @brief Get the maximum frequency (Hz) at which the current state of the planning scene is updated.*/
  double getStateUpdateFrequency() const
  {
    if (!dt_state_update_.isZero())
      return 1.0 / dt_state_update_.toSec();

    return 0.0;
  }

  /** @brief Add a function to be called when an update to the scene is received */
  void addUpdateCallback(const boost::function<void(EnvironmentUpdateType)>& fn);

  /** @brief Clear the functions to be called when an update to the scene is received */
  void clearUpdateCallbacks();

  /** \brief Return the time when the last update was made to the planning scene (by \e any monitor) */
  const ros::Time& getLastUpdateTime() const { return last_update_time_; }

  /** @brief This function is called every time there is a change to the planning scene */
  void triggerEnvironmentUpdateEvent(EnvironmentUpdateType update_type);

  /** \brief Wait for robot state to become more recent than time t.
   *
   * If there is no state monitor active, there will be no scene updates.
   * Hence, you can specify a timeout to wait for those updates. Default is 1s.
   */
  bool waitForCurrentState(const ros::Time& t, double wait_time = 1.);

  /** \brief Lock the scene for reading (multiple threads can lock for reading at the same time) */
  void lockEnvironmentRead();

  /** \brief Unlock the scene from reading (multiple threads can lock for reading at the same time) */
  void unlockEnvironmentRead();

  /** \brief Lock the scene for writing (only one thread can lock for writing and no other thread can lock for reading)
   */
  void lockEnvironmentWrite();

  /** \brief Lock the scene from writing (only one thread can lock for writing and no other thread can lock for reading)
   */
  void unlockEnvironmentWrite();

  void clearOctomap();

  void getMonitoredTopics(std::vector<std::string>& topics) const;

protected:
  /** @brief Initialize the planning scene monitor
   *  @param scene The scene instance to fill with data (an instance is allocated if the one passed in is not allocated)
   */
  void initialize();

  /// The name of this scene monitor
  std::string monitor_name_;
  std::string discrete_plugin_name_;
  std::string continuous_plugin_name_;

  DiscreteContactManagerPluginLoaderPtr discrete_manager_loader_;
  ContinuousContactManagerPluginLoaderPtr continuous_manager_loader_;

  tesseract::Tesseract::Ptr tesseract_;
  boost::shared_mutex scene_update_mutex_;  /// mutex for stored scene
  ros::Time last_update_time_;              /// Last time the state was updated
  ros::Time last_robot_motion_time_;        /// Last time the robot has moved
  bool enforce_next_state_update_;          /// flag to enforce immediate state update in onStateUpdate()

  ros::NodeHandle nh_;
  ros::NodeHandle root_nh_;
  std::string robot_description_;

  // variables for planning scene publishing
  ros::Publisher environment_publisher_;
  std::unique_ptr<boost::thread> publish_environment_;
  double publish_environment_frequency_;
  EnvironmentUpdateType publish_update_types_;
  EnvironmentUpdateType new_environment_update_;
  boost::condition_variable_any new_environment_update_condition_;

  // host a service for modifying the environment
  ros::ServiceServer modify_environment_server_;

  // host a service for getting the environment changes
  ros::ServiceServer get_environment_changes_server_;

  // host a service for getting the environment information
  ros::ServiceServer get_environment_information_server_;

  // host a service for saving the scene graph to a DOT file
  ros::ServiceServer save_scene_graph_server_;

  // include a current state monitor
  CurrentStateMonitorPtr current_state_monitor_;

  /// lock access to update_callbacks_
  boost::recursive_mutex update_lock_;
  std::vector<boost::function<void(EnvironmentUpdateType)> > update_callbacks_;  /// List of callbacks to trigger when
                                                                                 /// updates
                                                                                 /// are received

private:
  void getUpdatedFrameTransforms(std::vector<geometry_msgs::TransformStamped>& transforms);

  // publish environment update diffs (runs in its own thread)
  void environmentPublishingThread();

  // called by current_state_monitor_ when robot state (as monitored on joint state topic) changes
  void onStateUpdate(const sensor_msgs::JointStateConstPtr& joint_state);

  // called by state_update_timer_ when a state update it pending
  void stateUpdateTimerCallback(const ros::WallTimerEvent& event);

  // Callback for a new state msg
  void newStateCallback(const tesseract_msgs::TesseractStateConstPtr& env);

  /** @brief Callback for modifying the environment via service request */
  bool modifyEnvironmentCallback(tesseract_msgs::ModifyEnvironmentRequest& req,
                                 tesseract_msgs::ModifyEnvironmentResponse& res);

  /** @brief Callback for get the environment changes via service request */
  bool getEnvironmentChangesCallback(tesseract_msgs::GetEnvironmentChangesRequest& req,
                                     tesseract_msgs::GetEnvironmentChangesResponse& res);

  /** @brief Callback for get the environment information via service request */
  bool getEnvironmentInformationCallback(tesseract_msgs::GetEnvironmentInformationRequest& req,
                                         tesseract_msgs::GetEnvironmentInformationResponse& res);

  /** @brief Callback to save the scene graph to a DOT via a service request */
  bool saveSceneGraphCallback(tesseract_msgs::SaveSceneGraphRequest& req, tesseract_msgs::SaveSceneGraphResponse& res);

  // Called when new service request is called to modify the environment.
  bool applyEnvironmentCommandsMessage(const std::string& id,
                                       int revision,
                                       const std::vector<tesseract_msgs::EnvironmentCommand>& commands);

  // Lock for state_update_pending_ and dt_state_update_
  boost::mutex state_pending_mutex_;

  /// True when we need to update the RobotState from current_state_monitor_
  // This field is protected by state_pending_mutex_
  volatile bool state_update_pending_;

  /// the amount of time to wait in between updates to the robot state
  // This field is protected by state_pending_mutex_
  ros::WallDuration dt_state_update_;

  /// the amount of time to wait when looking up transforms
  // Setting this to a non-zero value resolves issues when the sensor data is
  // arriving so fast that it is preceding the transform state.
  ros::Duration shape_transform_cache_lookup_wait_time_;

  /// timer for state updates.
  // Check if last_state_update_ is true and if so call updateSceneWithCurrentState()
  // Not safe to access from callback functions.
  ros::WallTimer state_update_timer_;

  /// Last time the state was updated from current_state_monitor_
  // Only access this from callback functions (and constructor)
  ros::WallTime last_robot_state_update_wall_time_;

  DynamicReconfigureImpl* reconfigure_impl_;
};
using EnvironmentMonitorPtr = std::shared_ptr<EnvironmentMonitor>;
using EnvironmentMonitorConstPtr = std::shared_ptr<const EnvironmentMonitor>;

/** \brief This is a convenience class for obtaining access to an
 *         instance of a locked Environment.
 *
 * Instances of this class can be used almost exactly like instances
 * of a ROSBasicEnvPtr because of the typecast operator and
 * "operator->" functions.  Therefore you will often see code like this:
 * @code
 *   environment_monitor::LockedEnvironmentRO ls(environment_monitor);
 *   const tesseract_ros::ROSBasicEnvPtr& env = ls->getEnvironment();
 * @endcode

 * The function "getEnvironment()" is a member of EnvironmentMonitor and not
 * a member of this class.  However because of the "operator->" here
 * which returns a ROSBasicEnvConstPtr, this works.
 *
 * Any number of these "ReadOnly" locks can exist at a given time.
 * The intention is that users which only need to read from the
 * ROSBasicEnvPtr will use these and will thus not interfere with each
 * other.
 *
 * @see LockedEnvironmentRW */
class LockedEnvironmentRO
{
public:
  LockedEnvironmentRO(EnvironmentMonitorPtr environment_monitor) : env_monitor_(std::move(environment_monitor))
  {
    initialize(true);
  }

  const EnvironmentMonitorPtr& getEnvironmentMonitor() { return env_monitor_; }
  operator bool() const { return env_monitor_ && env_monitor_->getEnvironment(); }
  operator const tesseract_environment::Environment::ConstPtr&() const
  {
    return static_cast<const EnvironmentMonitor*>(env_monitor_.get())->getEnvironment();
  }

  const tesseract_environment::Environment::ConstPtr& operator->() const
  {
    return static_cast<const EnvironmentMonitor*>(env_monitor_.get())->getEnvironment();
  }

protected:
  LockedEnvironmentRO(EnvironmentMonitorPtr environment_monitor, bool read_only)
    : env_monitor_(std::move(environment_monitor))
  {
    initialize(read_only);
  }

  void initialize(bool read_only)
  {
    if (env_monitor_)
      lock_.reset(new SingleUnlock(env_monitor_.get(), read_only));
  }

  // we use this struct so that lock/unlock are called only once
  // even if the LockedPlanningScene instance is copied around
  struct SingleUnlock
  {
    SingleUnlock(EnvironmentMonitor* environment_monitor, bool read_only)
      : env_monitor_(environment_monitor), read_only_(read_only)
    {
      if (read_only)
        env_monitor_->lockEnvironmentRead();
      else
        env_monitor_->lockEnvironmentWrite();
    }
    ~SingleUnlock()
    {
      if (read_only_)
        env_monitor_->unlockEnvironmentRead();
      else
        env_monitor_->unlockEnvironmentWrite();
    }
    SingleUnlock(const SingleUnlock&) = delete;
    SingleUnlock& operator=(const SingleUnlock&) = delete;
    SingleUnlock(SingleUnlock&&) = delete;
    SingleUnlock& operator=(SingleUnlock&&) = delete;

    EnvironmentMonitor* env_monitor_;
    bool read_only_;
  };
  using SingleUnlockPtr = std::shared_ptr<SingleUnlock>;
  using SingleUnlockConstPtr = std::shared_ptr<const SingleUnlock>;

  EnvironmentMonitorPtr env_monitor_;
  SingleUnlockPtr lock_;
};

/** \brief This is a convenience class for obtaining access to an
 *         instance of a locked Environment.
 *
 * Instances of this class can be used almost exactly like instances
 * of a EnvironmentMonitorPtr because of the typecast operator and
 * "operator->" functions.  Therefore you will often see code like this:
 * @code
 *   environment_monitor::LockedEnvironmentRW ls(environment_monitor);
 *   const tesseract_ros::ROSBasicEnvPtr& env = ls->getEnvironment();
 * @endcode

 * The function "getEnvironment()" is a member of EnvironmentMonitor and not
 * a member of this class.  However because of the "operator->" here
 * which returns a ROSBasicEnvPtr, this works.
 *
 * Only one of these "ReadWrite" locks can exist at a given time.  The
 * intention is that users which need to write to the ROSBasicEnv
 * will use these, preventing other writers and readers from locking
 * the same PlanningScene at the same time.
 *
 * @see LockedEnvironmentRO */
class LockedEnvironmentRW : public LockedEnvironmentRO
{
public:
  LockedEnvironmentRW(const EnvironmentMonitorPtr& environment_monitor)
    : LockedEnvironmentRO(environment_monitor, false)
  {
  }

  operator const tesseract_environment::Environment::Ptr&() { return env_monitor_->getEnvironment(); }
  const tesseract_environment::Environment::Ptr& operator->() { return env_monitor_->getEnvironment(); }
};
}  // namespace tesseract_monitoring

#endif
