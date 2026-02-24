/**
 * @file fwd.h
 * @brief Tesseract Environment Forward Declarations.
 *
 * @author Levi Armstrong
 * @date February 21, 2024
 *
 * @copyright Copyright (c) 2024, Levi Armstrong
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
#ifndef TESSERACT_ENVIRONMENT_FWD_H
#define TESSERACT_ENVIRONMENT_FWD_H

#include <cstdint>

namespace tesseract::environment
{
enum class Events : std::uint8_t;
enum class CommandType : std::int8_t;
enum class MonitoredEnvironmentMode : std::uint8_t;
enum class ModifyAllowedCollisionsType : std::uint8_t;

class Environment;
class EnvironmentCache;
class EnvironmentMonitorInterface;
class EnvironmentMonitor;
struct Event;
class Command;
class AddLinkCommand;
class MoveLinkCommand;
class MoveJointCommand;
class RemoveLinkCommand;
class RemoveJointCommand;
class ReplaceJointCommand;
class ChangeLinkOriginCommand;
class ChangeJointOriginCommand;
class ChangeLinkCollisionEnabledCommand;
class ChangeLinkVisibilityCommand;
class ModifyAllowedCollisionsCommand;
class RemoveAllowedCollisionLinkCommand;
class AddSceneGraphCommand;
class ChangeJointPositionLimitsCommand;
class ChangeJointVelocityLimitsCommand;
class ChangeJointAccelerationLimitsCommand;
class AddKinematicsInformationCommand;
class ChangeCollisionMarginsCommand;
class AddContactManagersPluginInfoCommand;
class SetActiveContinuousContactManagerCommand;
class SetActiveDiscreteContactManagerCommand;
class AddTrajectoryLinkCommand;
}  // namespace tesseract::environment

#endif  // TESSERACT_ENVIRONMENT_FWD_H
