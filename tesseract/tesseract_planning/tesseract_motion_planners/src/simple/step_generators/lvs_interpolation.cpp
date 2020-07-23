/**
 * @file lvs_interpolation.h
 * @brief
 *
 * @author Levi Armstrong
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
#ifndef TESSERACT_MOTION_PLANNERS_LVS_INTERPOLATION_H
#define TESSERACT_MOTION_PLANNERS_LVS_INTERPOLATION_H

#include <tesseract_common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <vector>
#include <memory>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract_motion_planners/core/types.h>
#include <tesseract_motion_planners/core/utils.h>
#include <tesseract_motion_planners/simple/step_generators/lvs_interpolation.h>

namespace tesseract_planning
{
CompositeInstruction
LVSJointInterpolation(const JointWaypoint&, const JointWaypoint&, const PlanInstruction&, const PlannerRequest&)
{
  CONSOLE_BRIDGE_logError("LVSJointInterpolation with Joint/Joint not yet implemented. Pull requests welcome");

  return CompositeInstruction();
}

CompositeInstruction
LVSJointInterpolation(const JointWaypoint&, const CartesianWaypoint&, const PlanInstruction&, const PlannerRequest&)
{
  CONSOLE_BRIDGE_logError("LVSJointInterpolation with Joint/Cart not yet implemented. Pull requests welcome");

  return CompositeInstruction();
}

CompositeInstruction
LVSJointInterpolation(const CartesianWaypoint&, const JointWaypoint&, const PlanInstruction&, const PlannerRequest&)
{
  CONSOLE_BRIDGE_logError("LVSJointInterpolation with Cart/Joint not yet implemented. Pull requests welcome");

  return CompositeInstruction();
}

CompositeInstruction
LVSJointInterpolation(const CartesianWaypoint&, const CartesianWaypoint&, const PlanInstruction&, const PlannerRequest&)
{
  CONSOLE_BRIDGE_logError("LVSJointInterpolation with Cart/Cart not yet implemented. Pull requests welcome");

  return CompositeInstruction();
}

CompositeInstruction
LVSLinearInterpolation(const JointWaypoint&, const JointWaypoint&, const PlanInstruction&, const PlannerRequest&)
{
  CONSOLE_BRIDGE_logError("LVSLinearInterpolation with Joint/Joint not yet implemented. Pull requests welcome");

  return CompositeInstruction();
}

CompositeInstruction
LVSLinearInterpolation(const JointWaypoint&, const CartesianWaypoint&, const PlanInstruction&, const PlannerRequest&)
{
  CONSOLE_BRIDGE_logError("LVSLinearInterpolation with Joint/Cart not yet implemented. Pull requests welcome");

  return CompositeInstruction();
}

CompositeInstruction
LVSLinearInterpolation(const CartesianWaypoint&, const JointWaypoint&, const PlanInstruction&, const PlannerRequest&)
{
  CONSOLE_BRIDGE_logError("LVSLinearInterpolation with Cart/Joint not yet implemented. Pull requests welcome");

  return CompositeInstruction();
}

CompositeInstruction LVSLinearInterpolation(const CartesianWaypoint&,
                                            const CartesianWaypoint&,
                                            const PlanInstruction&,
                                            const PlannerRequest&)
{
  CONSOLE_BRIDGE_logError("LVSLinearInterpolation with Cart/Cart not yet implemented. Pull requests welcome");

  return CompositeInstruction();
}
}  // namespace tesseract_planning

#endif  // TESSERACT_MOTION_PLANNERS_SIMPLE_PROFILE_H
