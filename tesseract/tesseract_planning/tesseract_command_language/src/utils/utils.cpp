/**
 * @file utils.cpp
 * @brief
 *
 * @author Levi Armstrong
 * @date June 15, 2020
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
#include <tesseract_common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <algorithm>
#include <console_bridge/console.h>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract_command_language/utils/utils.h>
#include <tesseract_command_language/instruction_type.h>
#include <tesseract_command_language/joint_waypoint.h>
#include <tesseract_command_language/cartesian_waypoint.h>
#include <tesseract_command_language/state_waypoint.h>

namespace tesseract_planning
{
const Eigen::VectorXd& getJointPosition(const Waypoint& waypoint)
{
  if (isJointWaypoint(waypoint))
    return *(waypoint.cast_const<JointWaypoint>());

  if (isStateWaypoint(waypoint))
    return waypoint.cast_const<StateWaypoint>()->position;

  throw std::runtime_error("Unsupported waypoint type.");
}

Eigen::VectorXd getJointPosition(const std::vector<std::string>& joint_names, const Waypoint& waypoint)
{
  Eigen::VectorXd jv;
  std::vector<std::string> jn;
  if (isJointWaypoint(waypoint))
  {
    const auto* jwp = waypoint.cast_const<JointWaypoint>();
    jv = *jwp;
    jn = jwp->joint_names;
  }
  else if (isStateWaypoint(waypoint))
  {
    const auto* swp = waypoint.cast_const<StateWaypoint>();
    jv = swp->position;
    jn = swp->joint_names;
  }
  else
  {
    throw std::runtime_error("Unsupported waypoint type.");
  }

  if (jn.size() == joint_names.size())
    throw std::runtime_error("Joint name sizes do not match!");

  if (joint_names == jn)
    return jv;

  Eigen::VectorXd output = jv;
  for (std::size_t i = 0; i < joint_names.size(); ++i)
  {
    if (joint_names[i] == jn[i])
      continue;

    auto it = std::find(jn.begin(), jn.end(), joint_names[i]);
    if (it == jn.end())
      throw std::runtime_error("Joint names do not match!");

    long idx = std::distance(jn.begin(), it);
    output(static_cast<long>(i)) = jv(static_cast<long>(idx));
  }

  return output;
}

bool setJointPosition(Waypoint& waypoint, const Eigen::Ref<const Eigen::VectorXd>& position)
{
  if (isJointWaypoint(waypoint))
    *waypoint.cast<JointWaypoint>() = position;
  else if (isStateWaypoint(waypoint))
    waypoint.cast<StateWaypoint>()->position = position;
  else
    return false;

  return true;
}

bool isWithinJointLimits(const Waypoint& wp, const Eigen::Ref<const Eigen::MatrixX2d>& limits)
{
  if (isJointWaypoint(wp) || isStateWaypoint(wp))
  {
    Eigen::VectorXd cmd_pos;
    try
    {
      cmd_pos = getJointPosition(wp);
    }
    catch (std::exception& e)
    {
      CONSOLE_BRIDGE_logWarn("getJointPosition threw %s", e.what());
      return false;
    }

    // Check input validity
    if (limits.rows() != cmd_pos.size())
    {
      CONSOLE_BRIDGE_logWarn(
          "Invalid limits when clamping Waypoint. Waypoint size: %d, Limits size: %d", cmd_pos.size(), limits.rows());
      return false;
    }

    // Is it within the limits?
    if ((limits.col(0).array() > cmd_pos.array()).any())
      return false;
    if ((limits.col(1).array() < cmd_pos.array()).any())
      return false;
  }

  return true;
}

bool clampToJointLimits(Waypoint& wp, const Eigen::Ref<const Eigen::MatrixX2d>& limits, double max_deviation)
{
  Eigen::VectorXd deviation_vec = Eigen::VectorXd::Ones(limits.rows()) * max_deviation;
  return clampToJointLimits(wp, limits, deviation_vec);
}

bool clampToJointLimits(Waypoint& wp,
                        const Eigen::Ref<const Eigen::MatrixX2d>& limits,
                        const Eigen::Ref<const Eigen::VectorXd>& max_deviation)
{
  if (isJointWaypoint(wp) || isStateWaypoint(wp))
  {
    Eigen::VectorXd cmd_pos;
    try
    {
      cmd_pos = getJointPosition(wp);
    }
    catch (std::exception& e)
    {
      CONSOLE_BRIDGE_logWarn("getJointPosition threw %s", e.what());
      return false;
    }

    // Check input validity
    if (limits.rows() != cmd_pos.size())
    {
      CONSOLE_BRIDGE_logWarn(
          "Invalid limits when clamping Waypoint. Waypoint size: %d, Limits size: %d", limits.rows(), cmd_pos.size());
      return false;
    }
    if (limits.rows() != max_deviation.size())
    {
      CONSOLE_BRIDGE_logWarn("Invalid max deviation given when clamping Waypoint. Waypoint size: %d, max deviation "
                             "size: %d",
                             limits.rows(),
                             max_deviation.size());
      return false;
    }

    if (((cmd_pos.array() - limits.col(1).array()) > max_deviation.array()).any())
      return false;
    if ((-(cmd_pos.array() - limits.col(0).array()) > max_deviation.array()).any())
      return false;

    CONSOLE_BRIDGE_logDebug("Clamping Waypoint to joint limits");
    Eigen::VectorXd new_position = cmd_pos;
    new_position = new_position.cwiseMax(limits.col(0));
    new_position = new_position.cwiseMin(limits.col(1));
    return setJointPosition(wp, new_position);
  }

  return true;
}

void generateSkeletonSeedHelper(CompositeInstruction& composite_instructions)
{
  for (auto& i : composite_instructions)
  {
    if (isCompositeInstruction(i))
    {
      generateSkeletonSeedHelper(*(i.cast<CompositeInstruction>()));
    }
    else if (isPlanInstruction(i))
    {
      CompositeInstruction ci;
      const auto* pi = i.cast<PlanInstruction>();
      ci.setProfile(pi->getProfile());
      ci.setDescription(pi->getDescription());
      ci.setManipulatorInfo(pi->getManipulatorInfo());

      i = ci;
    }
  }
}

CompositeInstruction generateSkeletonSeed(const CompositeInstruction& composite_instructions)
{
  CompositeInstruction seed = composite_instructions;
  generateSkeletonSeedHelper(seed);
  return seed;
}

}  // namespace tesseract_planning
