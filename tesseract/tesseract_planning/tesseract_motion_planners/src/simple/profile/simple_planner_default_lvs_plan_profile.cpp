/**
 * @file simple_planner_default_lvs_plan_profile.cpp
 * @brief
 *
 * @author Tyler Marr
 * @date Septemeber 16, 2020
 * @version TODO
 * @bug No known bugs
 *
 * @copyright Copyright (c) 2020, Southwest Research Institute
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

#include <tesseract_motion_planners/simple/profile/simple_planner_default_lvs_plan_profile.h>
#include <tesseract_motion_planners/simple/step_generators/lvs_interpolation.h>

namespace tesseract_planning
{
SimplePlannerDefaultLVSPlanProfile::SimplePlannerDefaultLVSPlanProfile(double state_longest_valid_segment_length,
                                                                       double translation_longest_valid_segment_length,
                                                                       double rotation_longest_valid_segment_length,
                                                                       int min_steps)
  : state_longest_valid_segment_length_(state_longest_valid_segment_length)
  , translation_longest_valid_segment_length_(translation_longest_valid_segment_length)
  , rotation_longest_valid_segment_length_(rotation_longest_valid_segment_length)
  , min_steps_(min_steps)
{
  apply();
}

void SimplePlannerDefaultLVSPlanProfile::apply()
{
  // Bind the Freespace functions
  joint_joint_freespace = [this](const JointWaypoint& one,
                                 const JointWaypoint& two,
                                 const PlanInstruction& three,
                                 const PlannerRequest& four,
                                 const ManipulatorInfo& five) {
    return LVSInterpolateStateWaypoint(one,
                                       two,
                                       three,
                                       four,
                                       five,
                                       this->state_longest_valid_segment_length_,
                                       this->translation_longest_valid_segment_length_,
                                       this->rotation_longest_valid_segment_length_,
                                       this->min_steps_);
  };
  joint_cart_freespace = [this](const JointWaypoint& one,
                                const CartesianWaypoint& two,
                                const PlanInstruction& three,
                                const PlannerRequest& four,
                                const ManipulatorInfo& five) {
    return LVSInterpolateStateWaypoint(one,
                                       two,
                                       three,
                                       four,
                                       five,
                                       this->state_longest_valid_segment_length_,
                                       this->translation_longest_valid_segment_length_,
                                       this->rotation_longest_valid_segment_length_,
                                       this->min_steps_);
  };
  cart_joint_freespace = [this](const CartesianWaypoint& one,
                                const JointWaypoint& two,
                                const PlanInstruction& three,
                                const PlannerRequest& four,
                                const ManipulatorInfo& five) {
    return LVSInterpolateStateWaypoint(one,
                                       two,
                                       three,
                                       four,
                                       five,
                                       this->state_longest_valid_segment_length_,
                                       this->translation_longest_valid_segment_length_,
                                       this->rotation_longest_valid_segment_length_,
                                       this->min_steps_);
  };
  cart_cart_freespace = [this](const CartesianWaypoint& one,
                               const CartesianWaypoint& two,
                               const PlanInstruction& three,
                               const PlannerRequest& four,
                               const ManipulatorInfo& five) {
    return LVSInterpolateStateWaypoint(one,
                                       two,
                                       three,
                                       four,
                                       five,
                                       this->state_longest_valid_segment_length_,
                                       this->translation_longest_valid_segment_length_,
                                       this->rotation_longest_valid_segment_length_,
                                       this->min_steps_);
  };

  // Bind the Linear functions
  joint_joint_linear = [this](const JointWaypoint& one,
                              const JointWaypoint& two,
                              const PlanInstruction& three,
                              const PlannerRequest& four,
                              const ManipulatorInfo& five) {
    return LVSInterpolateStateWaypoint(one,
                                       two,
                                       three,
                                       four,
                                       five,
                                       this->state_longest_valid_segment_length_,
                                       this->translation_longest_valid_segment_length_,
                                       this->rotation_longest_valid_segment_length_,
                                       this->min_steps_);
  };
  joint_cart_linear = [this](const JointWaypoint& one,
                             const CartesianWaypoint& two,
                             const PlanInstruction& three,
                             const PlannerRequest& four,
                             const ManipulatorInfo& five) {
    return LVSInterpolateStateWaypoint(one,
                                       two,
                                       three,
                                       four,
                                       five,
                                       this->state_longest_valid_segment_length_,
                                       this->translation_longest_valid_segment_length_,
                                       this->rotation_longest_valid_segment_length_,
                                       this->min_steps_);
  };
  cart_joint_linear = [this](const CartesianWaypoint& one,
                             const JointWaypoint& two,
                             const PlanInstruction& three,
                             const PlannerRequest& four,
                             const ManipulatorInfo& five) {
    return LVSInterpolateStateWaypoint(one,
                                       two,
                                       three,
                                       four,
                                       five,
                                       this->state_longest_valid_segment_length_,
                                       this->translation_longest_valid_segment_length_,
                                       this->rotation_longest_valid_segment_length_,
                                       this->min_steps_);
  };
  cart_cart_linear = [this](const CartesianWaypoint& one,
                            const CartesianWaypoint& two,
                            const PlanInstruction& three,
                            const PlannerRequest& four,
                            const ManipulatorInfo& five) {
    return LVSInterpolateStateWaypoint(one,
                                       two,
                                       three,
                                       four,
                                       five,
                                       this->state_longest_valid_segment_length_,
                                       this->translation_longest_valid_segment_length_,
                                       this->rotation_longest_valid_segment_length_,
                                       this->min_steps_);
  };
}

const double& SimplePlannerDefaultLVSPlanProfile::getStateLongestValidSegmentLength()
{
  return state_longest_valid_segment_length_;
}
void SimplePlannerDefaultLVSPlanProfile::setStateLongestValidSegmentLength(double state_longest_valid_segment_length)
{
  state_longest_valid_segment_length_ = state_longest_valid_segment_length;
  apply();
}

const double& SimplePlannerDefaultLVSPlanProfile::getTranslationLongestValidSegmentLength()
{
  return translation_longest_valid_segment_length_;
}
void SimplePlannerDefaultLVSPlanProfile::setTranslationLongestValidSegmentLength(
    double translation_longest_valid_segment_length)
{
  translation_longest_valid_segment_length_ = translation_longest_valid_segment_length;
  apply();
}

const double& SimplePlannerDefaultLVSPlanProfile::getRotationLongestValidSegmentLength()
{
  return rotation_longest_valid_segment_length_;
}
void SimplePlannerDefaultLVSPlanProfile::setRotationLongestValidSegmentLength(
    double rotation_longest_valid_segment_length)
{
  rotation_longest_valid_segment_length_ = rotation_longest_valid_segment_length;
  apply();
}

const int& SimplePlannerDefaultLVSPlanProfile::getMinSteps() { return min_steps_; }
void SimplePlannerDefaultLVSPlanProfile::setMinSteps(int min_steps)
{
  min_steps_ = min_steps;
  apply();
}
}  // namespace tesseract_planning
