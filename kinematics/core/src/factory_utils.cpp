/**
 * @file factory_utils.cpp
 * @brief Shared YAML-parsing helpers for kinematics plugin factories.
 *
 * @author Roelof Oomen
 * @date May 1, 2026
 *
 * @copyright Copyright (c) 2026
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

#include <tesseract/kinematics/factory_utils.h>
#include <tesseract/kinematics/yaml_extensions.h>
#include <tesseract/common/utils.h>
#include <tesseract/scene_graph/graph.h>
#include <tesseract/scene_graph/joint.h>

#include <cmath>
#include <map>

namespace tesseract::kinematics
{
std::map<tesseract::common::JointId, JointSampleSpec>
parseSampleResolutionMap(const YAML::Node& sample_res_node,
                         const tesseract::scene_graph::SceneGraph& scene_graph,
                         const std::string& block_label)
{
  std::map<tesseract::common::JointId, JointSampleSpec> sample_res_map;

  for (const auto& entry : sample_res_node)
  {
    const auto psr = entry.as<PositionerSampleResolution>();
    const tesseract::common::JointId joint_id(psr.name);
    const std::string& jn = joint_id.name();

    auto jnt = scene_graph.getJoint(joint_id);
    if (jnt == nullptr)
      throw std::runtime_error(tesseract::common::strFormat(
          "'%s' failed to find joint '%s' in scene graph!", block_label.c_str(), jn.c_str()));

    // A continuous joint is unbounded: default to one full turn and accept any explicit range.
    if (jnt->type == tesseract::scene_graph::JointType::CONTINUOUS)
    {
      const double range_min = psr.min.value_or(-M_PI);
      const double range_max = psr.max.value_or(M_PI);
      if (range_min > range_max)
        throw std::runtime_error(tesseract::common::strFormat(
            "'%s' joint '%s' sample range is not valid!", block_label.c_str(), jn.c_str()));
      sample_res_map.insert_or_assign(joint_id, JointSampleSpec{ psr.value, range_min, range_max });
      continue;
    }

    if (jnt->limits == nullptr)
      throw std::runtime_error(
          tesseract::common::strFormat("'%s' joint '%s' has no limits!", block_label.c_str(), jn.c_str()));

    const double range_min = psr.min.value_or(jnt->limits->lower);
    const double range_max = psr.max.value_or(jnt->limits->upper);

    if (range_min < jnt->limits->lower)
      throw std::runtime_error(tesseract::common::strFormat(
          "'%s' joint '%s' sample range minimum is less than joint minimum!", block_label.c_str(), jn.c_str()));
    if (range_max > jnt->limits->upper)
      throw std::runtime_error(tesseract::common::strFormat(
          "'%s' joint '%s' sample range maximum is greater than joint maximum!", block_label.c_str(), jn.c_str()));
    if (range_min > range_max)
      throw std::runtime_error(
          tesseract::common::strFormat("'%s' joint '%s' sample range is not valid!", block_label.c_str(), jn.c_str()));

    sample_res_map.insert_or_assign(joint_id, JointSampleSpec{ psr.value, range_min, range_max });
  }

  return sample_res_map;
}

SampleGridConfig toSampleGridConfig(const std::map<tesseract::common::JointId, JointSampleSpec>& sample_res_map,
                                    const std::vector<tesseract::common::JointId>& joint_ids,
                                    const std::string& block_label)
{
  if (sample_res_map.size() != joint_ids.size())
    throw std::runtime_error(tesseract::common::strFormat("'%s' has incorrect number of joints!", block_label.c_str()));

  const auto n = static_cast<Eigen::Index>(joint_ids.size());
  SampleGridConfig out;
  out.range.resize(n, 2);
  out.resolution.resize(n);
  for (Eigen::Index i = 0; i < n; ++i)
  {
    const auto& jn = joint_ids[static_cast<std::size_t>(i)];
    auto it = sample_res_map.find(jn);
    if (it == sample_res_map.end())
      throw std::runtime_error(
          tesseract::common::strFormat("'%s' missing joint '%s'!", block_label.c_str(), jn.name().c_str()));

    out.resolution(i) = it->second.resolution;
    out.range(i, 0) = it->second.min;
    out.range(i, 1) = it->second.max;
  }

  return out;
}

}  // namespace tesseract::kinematics
