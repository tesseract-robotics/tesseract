/**
 * @file environment_monitor.h
 * @brief Tesseract Environment Monitor Interface Class.
 *
 * @author Levi Armstrong
 * @date March 30, 2022
 * @version TODO
 * @bug No known bugs
 *
 * @copyright Copyright (c) 2022, Levi Armstrong
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
#ifndef TESSERACT_ENVIRONMENT_ENVIRONMENT_MONITOR_H
#define TESSERACT_ENVIRONMENT_ENVIRONMENT_MONITOR_H

#include <tesseract_environment/environment.h>
#include <chrono>

namespace tesseract_environment
{
enum class MonitoredEnvironmentMode : int
{
  /**
   * @brief The default behavior when monitoring another environment is the following.
   *
   * Case1: If the revision is greater than, it will call a service of the monitored environment to get the changes and
   *        apply them.
   * Case2: If the revision is less than, it will reinitialize the environment and request the remaining changes and
   *        apply them.
   */
  DEFAULT = 0,

  /**
   * @brief The synchronized behavior when monitoring another environment is the following.
   *
   * @warning Currently this works best if there is only one monitor which is synchronized because currently if two
   *          environment monitors are synchronized, it currently does not have a way to reason about which ones
   *          should get applied. Need to research approach for this type of system.
   *
   * Case1: If the revision is greater than, it will call a service of the monitored environment to get the changes and
   *        apply them.
   * Case2: If the revision is less than, it will call a service of the monitored environment to apply the new changes.
   */
  SYNCHRONIZED = 1
};

/** @brief Tesseract Environment Monitor Interface Class */
class EnvironmentMonitor
{
public:
  using Ptr = std::shared_ptr<EnvironmentMonitor>;
  using ConstPtr = std::shared_ptr<const EnvironmentMonitor>;
  using UPtr = std::unique_ptr<EnvironmentMonitor>;
  using ConstUPtr = std::unique_ptr<const EnvironmentMonitor>;

  /**
   * @brief Constructor
   * @param monitor_namespace A name identifying this monitor, must be unique
   */
  EnvironmentMonitor(std::string monitor_namespace) : monitor_namespace_(monitor_namespace) {}

  /**
   * @brief Constructor
   * @param env The environment to use internal to the monitor
   * @param monitor_namespace A name identifying this monitor, must be unique
   */
  EnvironmentMonitor(tesseract_environment::Environment::Ptr env, std::string monitor_namespace)
    : env_(std::move(env)), monitor_namespace_(monitor_namespace)
  {
  }

  virtual ~EnvironmentMonitor() = default;

  /** \brief Get the namespace of this monitor */
  virtual const std::string& getNamespace() const { return monitor_namespace_; }

  /**
   * @brief Returns an @b threadsafe reference to the current environment.
   * @details Modification should only be made if this monitor is the master. If this is monitoring another environment
   * the local changes will get removed on the next update cycle. Recommend using the TesseractMonitorInterface to apply
   * commands to the monitored environment until the todo below is implemented.
   * @return The current environment.
   */
  virtual tesseract_environment::Environment& environment() { return *env_; }

  /**
   * @brief Returns an @b threadsafe const reference to the current environment.
   * @return The current environment.*/
  virtual const tesseract_environment::Environment& environment() const { return *env_; }

  /**
   * @brief Returns an @b threadsafe shared pointer to the current environment.
   * @details Modification should only be made if this monitor is the master. If this is monitoring another environment
   * the local changes will get removed on the next update cycle. Recommend using the TesseractMonitorInterface to apply
   * commands to the monitored environment until the todo below is implemented.
   * @return The current environment.
   */
  virtual tesseract_environment::Environment::Ptr getEnvironment() { return env_; }

  /**
   * @brief Returns an @b threadsafe const shared point to the current environment.
   * @return The current environment.
   */
  virtual tesseract_environment::Environment::ConstPtr getEnvironment() const { return env_; }

  /**
   * @brief Wait for connection to upstream environment
   * @param duration The time to wait in seconds before returning, if zero it waits indefinitely
   * @return True if it has connected to upstream environment, otherwise false
   */
  virtual bool waitForConnection(std::chrono::duration<double> duration = std::chrono::seconds(0)) const = 0;

  /** @brief Start publishing the maintained environment. */
  virtual void startPublishingEnvironment() = 0;

  /** @brief Stop publishing the maintained environment. */
  virtual void stopPublishingEnvironment() = 0;

  /** @brief Set the maximum frequency at which environment are being published */
  virtual void setEnvironmentPublishingFrequency(double hz) = 0;

  /** @brief Get the maximum frequency at which environment are published (Hz) */
  virtual double getEnvironmentPublishingFrequency() const = 0;

  /**
   * @brief Start the current state monitor
   * @param joint_states_topic the topic to listen to for joint states
   * @param publish_tf If true, TFs will be published for each joint. Default: true
   */
  virtual void startStateMonitor(const std::string& joint_states_topic, bool publish_tf = true) = 0;

  /** @brief Stop the state monitor*/
  virtual void stopStateMonitor() = 0;

  /**
   * @brief Update the scene using the monitored state at a specified frequency, in Hz. This function has an effect only
   * when updates from the CurrentStateMonitor are received at a higher frequency. In that case, the updates are
   * throttled down, so that they do not exceed a maximum update frequency specified here.
   * @param hz the update frequency. By default this is 10Hz.
   */
  virtual void setStateUpdateFrequency(double hz = 10) = 0;

  /** @brief Get the maximum frequency (Hz) at which the current state of the planning scene is updated.*/
  virtual double getStateUpdateFrequency() const = 0;

  /**
   * @brief Update the scene using the monitored state. This function is automatically called when an update to the
   * current state is received (if startStateMonitor() has been called). The updates are throttled to a maximum update
   * frequency however, which is set by setStateUpdateFrequency().
   */
  virtual void updateEnvironmentWithCurrentState() = 0;

  /**
   * @brief Start the monitoring of an environment topic
   * @param monitored_namespace The namespace of the environment to monitor
   */
  virtual void startMonitoringEnvironment(const std::string& monitored_namespace,
                                          MonitoredEnvironmentMode mode = MonitoredEnvironmentMode::DEFAULT) = 0;

  /** \brief Stop monitoring the external environment. */
  virtual void stopMonitoringEnvironment() = 0;

  /**
   * @brief Wait for robot state to become more recent than time t.
   * @param duration The time to wait in seconds
   * @details If there is no state monitor active, there will be no scene updates.
   * Hence, you can specify a timeout to wait for those updates. Default is 1s.
   */
  virtual bool waitForCurrentState(std::chrono::duration<double> duration = std::chrono::seconds(1)) = 0;

  /** @brief Shutdown advertised services */
  virtual void shutdown() = 0;

protected:
  tesseract_environment::Environment::Ptr env_;
  std::string monitor_namespace_;
  MonitoredEnvironmentMode mode_{ MonitoredEnvironmentMode::DEFAULT };
};

}  // namespace tesseract_environment
#endif  // TESSERACT_ENVIRONMENT_ENVIRONMENT_MONITOR_H
