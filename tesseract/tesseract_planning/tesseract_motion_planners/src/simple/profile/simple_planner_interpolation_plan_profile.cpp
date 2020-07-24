/**
 * @file simple_planner_interpolation_plan_profile.cpp
 * @brief
 *
 * @author Matthew Powelson
 * @date July 23, 2020
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

#include <tesseract_motion_planners/simple/profile/simple_planner_interpolation_plan_profile.h>
#include <tesseract_motion_planners/simple/step_generators/fixed_size_interpolation.h>

namespace tesseract_planning
{
SimplePlannerInterpolationPlanProfile::SimplePlannerInterpolationPlanProfile(int freespace_steps, int cartesian_steps)
  : freespace_steps_(freespace_steps), cartesian_steps_(cartesian_steps)
{
  apply();
}

void SimplePlannerInterpolationPlanProfile::apply()
{
  // Bind the Freespace functions
  joint_joint_freespace = [this](const JointWaypoint& one,
                                 const JointWaypoint& two,
                                 const PlanInstruction& three,
                                 const PlannerRequest& four) {
    return fixedSizeJointInterpolation(one, two, three, four, this->freespace_steps_);
  };
  joint_cart_freespace = [this](const JointWaypoint& one,
                                const CartesianWaypoint& two,
                                const PlanInstruction& three,
                                const PlannerRequest& four) {
    return fixedSizeJointInterpolation(one, two, three, four, this->freespace_steps_);
  };
  cart_joint_freespace = [this](const CartesianWaypoint& one,
                                const JointWaypoint& two,
                                const PlanInstruction& three,
                                const PlannerRequest& four) {
    return fixedSizeJointInterpolation(one, two, three, four, this->freespace_steps_);
  };
  cart_cart_freespace = [this](const CartesianWaypoint& one,
                               const CartesianWaypoint& two,
                               const PlanInstruction& three,
                               const PlannerRequest& four) {
    return fixedSizeJointInterpolation(one, two, three, four, this->freespace_steps_);
  };

  // Bind the Linear functions
  joint_joint_linear = [this](const JointWaypoint& one,
                              const JointWaypoint& two,
                              const PlanInstruction& three,
                              const PlannerRequest& four) {
    return fixedSizeCartesianInterpolation(one, two, three, four, this->cartesian_steps_);
  };
  joint_cart_linear = [this](const JointWaypoint& one,
                             const CartesianWaypoint& two,
                             const PlanInstruction& three,
                             const PlannerRequest& four) {
    return fixedSizeCartesianInterpolation(one, two, three, four, this->cartesian_steps_);
  };
  cart_joint_linear = [this](const CartesianWaypoint& one,
                             const JointWaypoint& two,
                             const PlanInstruction& three,
                             const PlannerRequest& four) {
    return fixedSizeCartesianInterpolation(one, two, three, four, this->cartesian_steps_);
  };
  cart_cart_linear = [this](const CartesianWaypoint& one,
                            const CartesianWaypoint& two,
                            const PlanInstruction& three,
                            const PlannerRequest& four) {
    return fixedSizeCartesianInterpolation(one, two, three, four, this->cartesian_steps_);
  };
}

const int& SimplePlannerInterpolationPlanProfile::getFreespaceSteps() { return freespace_steps_; }
void SimplePlannerInterpolationPlanProfile::setFreespaceSteps(int freespace_steps)
{
  freespace_steps_ = freespace_steps;
  apply();
}

const int& SimplePlannerInterpolationPlanProfile::getCartesianSteps() { return cartesian_steps_; }
void SimplePlannerInterpolationPlanProfile::setCartesianSteps(int cartesian_steps)
{
  cartesian_steps_ = cartesian_steps;
  apply();
}
}  // namespace tesseract_planning
