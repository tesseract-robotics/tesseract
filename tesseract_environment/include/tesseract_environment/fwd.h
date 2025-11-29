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

namespace tesseract_environment
{
class Environment;
class EnvironmentCache;
class EnvironmentMonitorInterface;
class EnvironmentMonitor;
enum class MonitoredEnvironmentMode;
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
enum class ModifyAllowedCollisionsType;
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
}  // namespace tesseract_environment

#endif  // TESSERACT_ENVIRONMENT_FWD_H
