/**
 * @file types.h
 * @brief Tesseracts Collision Forward Declarations
 *
 * @author Levi Armstrong
 * @date February 17, 2024
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
#ifndef TESSERACT_COLLISION_CORE_TESSERACT_COLLISION_FWD_H
#define TESSERACT_COLLISION_CORE_TESSERACT_COLLISION_FWD_H

#include <cstdint>

namespace tesseract::collision
{
// types.h
enum class ContinuousCollisionType : std::uint8_t;
enum class ContactTestType : std::uint8_t;
struct ContactResult;
class ContactResultMap;
struct ContactRequest;
struct ContactTestData;
enum class CollisionEvaluatorType : std::uint8_t;
enum class CollisionCheckProgramType : std::uint8_t;
enum class ACMOverrideType : std::uint8_t;
struct ContactManagerConfig;
struct CollisionCheckConfig;
struct ContactTrajectorySubstepResults;
struct ContactTrajectoryStepResults;
struct ContactTrajectoryResults;
class ContactResultValidator;

// contact_managers_plugin_factory.h
class DiscreteContactManagerFactory;
class ContinuousContactManagerFactory;
class ContactManagersPluginFactory;

// convex_decomposition.h
class ConvexDecomposition;

// discrete_contact_manager.h
class DiscreteContactManager;

// continuous_contact_manager.h
class ContinuousContactManager;
}  // namespace tesseract::collision

#endif  // TESSERACT_COLLISION_CORE_TESSERACT_COLLISION_FWD_H
