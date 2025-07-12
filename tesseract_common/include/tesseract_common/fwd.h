/**
 * @file tesseract_common_fwd.h
 * @brief Tesseract common package forward declarations
 *
 * @author Levi Armstrong
 * @author Matthew Powelson
 * @date November 15, 2020
 * @version TODO
 * @bug No known bugs
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
#ifndef TESSERACT_COMMON_TESSERACT_COMMON_FWD_H
#define TESSERACT_COMMON_TESSERACT_COMMON_FWD_H

#include <cstdint>

namespace tesseract_common
{
// types.h
struct PluginInfo;
struct PluginInfoContainer;
struct ProfilesPluginInfo;
struct KinematicsPluginInfo;
struct ContactManagersPluginInfo;
struct TaskComposerPluginInfo;
struct CalibrationInfo;
struct PairHash;

// any_poly.h
class AnyInterface;
class AnyPoly;

// allowed_collision_matrix
class AllowedCollisionMatrix;

// contact_allowed_validator
class ContactAllowedValidator;
class ACMContactAllowedValidator;
class CombinedContactAllowedValidator;

// collision_margin_data.h
enum class CollisionMarginPairOverrideType : std::uint8_t;
class CollisionMarginData;
class CollisionMarginPairData;

// joint_state.h
class JointState;
class JointTrajectory;

// kinematic_limits.h
struct KinematicLimits;

// manipulator_info.h
struct ManipulatorInfo;

// resource_locator.h
class Resource;
class ResourceLocator;

// serialization.h
struct Serialization;

// timer.h
class Timer;

// Profile Dictionary
class Profile;
class ProfileDictionary;
}  // namespace tesseract_common

#endif  // TESSERACT_COMMON_TESSERACT_COMMON_FWD_H
