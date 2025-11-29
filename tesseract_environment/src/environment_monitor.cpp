/**
 * @file environment_monitor.cpp
 * @brief Tesseract Environment Monitor Interface Class.
 *
 * @author Levi Armstrong
 * @date March 30, 2022
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

#include <tesseract_environment/environment_monitor.h>
#include <tesseract_environment/environment.h>

#include <tesseract_kinematics/core/joint_group.h>
#include <tesseract_kinematics/core/kinematic_group.h>

namespace tesseract_environment
{
EnvironmentMonitor::EnvironmentMonitor(std::string monitor_namespace) : monitor_namespace_(std::move(monitor_namespace))
{
}

EnvironmentMonitor::EnvironmentMonitor(std::shared_ptr<Environment> env, std::string monitor_namespace)
  : env_(std::move(env)), monitor_namespace_(std::move(monitor_namespace))
{
}

const std::string& EnvironmentMonitor::getNamespace() const { return monitor_namespace_; }

Environment& EnvironmentMonitor::environment() { return *env_; }

const Environment& EnvironmentMonitor::environment() const { return *env_; }

std::shared_ptr<Environment> EnvironmentMonitor::getEnvironment() { return env_; }

std::shared_ptr<const Environment> EnvironmentMonitor::getEnvironment() const { return env_; }
}  // namespace tesseract_environment
