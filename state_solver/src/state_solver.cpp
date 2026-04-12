/**
 * @file state_solver.cpp
 * @brief Default implementations of ID-based setState/getState overloads.
 *
 * @author Levi Armstrong
 *
 * @copyright Copyright (c) 2017, Southwest Research Institute
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
#include <tesseract/state_solver/state_solver.h>
#include <tesseract/scene_graph/scene_state.h>

namespace tesseract::scene_graph
{
void StateSolver::setState(const SceneState::JointValues& joint_values,
                           const tesseract::common::JointIdTransformMap& floating_joint_values)
{
  std::unordered_map<std::string, double> str_map;
  str_map.reserve(joint_values.size());
  for (const auto& [id, val] : joint_values)
    str_map[id.name()] = val;
  setState(str_map, floating_joint_values);
}

void StateSolver::setState(const std::vector<tesseract::common::JointId>& joint_ids,
                           const Eigen::Ref<const Eigen::VectorXd>& joint_values,
                           const tesseract::common::JointIdTransformMap& floating_joint_values)
{
  std::vector<std::string> names;
  names.reserve(joint_ids.size());
  for (const auto& id : joint_ids)
    names.push_back(id.name());
  setState(names, joint_values, floating_joint_values);
}

SceneState StateSolver::getState(const SceneState::JointValues& joint_values,
                                 const tesseract::common::JointIdTransformMap& floating_joint_values) const
{
  std::unordered_map<std::string, double> str_map;
  str_map.reserve(joint_values.size());
  for (const auto& [id, val] : joint_values)
    str_map[id.name()] = val;
  return getState(str_map, floating_joint_values);
}

SceneState StateSolver::getState(const std::vector<tesseract::common::JointId>& joint_ids,
                                 const Eigen::Ref<const Eigen::VectorXd>& joint_values,
                                 const tesseract::common::JointIdTransformMap& floating_joint_values) const
{
  std::vector<std::string> names;
  names.reserve(joint_ids.size());
  for (const auto& id : joint_ids)
    names.push_back(id.name());
  return getState(names, joint_values, floating_joint_values);
}
}  // namespace tesseract::scene_graph
