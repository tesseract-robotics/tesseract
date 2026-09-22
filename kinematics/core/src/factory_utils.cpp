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
#include <tesseract/common/utils.h>
#include <tesseract/common/yaml_extensions.h>
#include <tesseract/scene_graph/graph.h>
#include <tesseract/scene_graph/joint.h>

#include <map>

namespace tesseract::kinematics
{
tesseract::common::PluginInfo parsePluginInfo(const YAML::Node& sub_node,
                                              const std::string& error_prefix,
                                              const std::string& block_label)
{
  try
  {
    return sub_node.as<tesseract::common::PluginInfo>();
  }
  catch (const std::exception& e)
  {
    // The converter does not know which block of the factory config it was handed, and a factory
    // config typically has several. Name it, otherwise the message cannot be acted on.
    throw std::runtime_error(error_prefix + ", '" + block_label + "': " + e.what());
  }
}

std::map<tesseract::common::JointId, std::array<double, 3>>
parseSampleResolutionMap(const YAML::Node& sample_res_node,
                         const tesseract::scene_graph::SceneGraph& scene_graph,
                         const std::string& error_prefix,
                         const std::string& block_label)
{
  const std::string block = error_prefix + ", '" + block_label + "' ";

  // [resolution, min, max] per joint id, before reordering onto the chain's joint order.
  std::map<tesseract::common::JointId, std::array<double, 3>> sample_res_map;

  for (auto it = sample_res_node.begin(); it != sample_res_node.end(); ++it)
  {
    const YAML::Node& joint = *it;
    std::array<double, 3> values{ 0, 0, 0 };

    tesseract::common::JointId joint_id;
    if (YAML::Node n = joint["name"])
      joint_id = tesseract::common::JointId(n.as<std::string>());
    else
      throw std::runtime_error(block + "missing 'name' entry!");

    if (YAML::Node n = joint["value"])
      values[0] = n.as<double>();
    else
      throw std::runtime_error(block + "missing 'value' entry!");

    auto jnt = scene_graph.getJoint(joint_id);
    if (jnt == nullptr)
      throw std::runtime_error(block + "failed to find joint in scene graph!");
    if (jnt->limits == nullptr)
      throw std::runtime_error(
          tesseract::common::strFormat("%sjoint '%s' has no limits!", block.c_str(), joint_id.name().c_str()));

    values[1] = jnt->limits->lower;
    values[2] = jnt->limits->upper;

    if (YAML::Node min = joint["min"])
      values[1] = min.as<double>();
    if (YAML::Node max = joint["max"])
      values[2] = max.as<double>();

    if (values[1] < jnt->limits->lower)
      throw std::runtime_error(block + "sample range minimum is less than joint minimum!");
    if (values[2] > jnt->limits->upper)
      throw std::runtime_error(block + "sample range maximum is greater than joint maximum!");
    if (values[1] > values[2])
      throw std::runtime_error(block + "sample range is not valid!");

    sample_res_map[joint_id] = values;
  }

  return sample_res_map;
}

SampleGridConfig toSampleGridConfig(const std::map<tesseract::common::JointId, std::array<double, 3>>& sample_res_map,
                                    const std::vector<tesseract::common::JointId>& joint_ids,
                                    const std::string& error_prefix,
                                    const std::string& block_label)
{
  const std::string block = error_prefix + ", '" + block_label + "' ";

  if (sample_res_map.size() != joint_ids.size())
    throw std::runtime_error(block + "has incorrect number of joints!");

  const auto n = static_cast<Eigen::Index>(joint_ids.size());
  SampleGridConfig out;
  out.range.resize(n, 2);
  out.resolution.resize(n);
  for (Eigen::Index i = 0; i < n; ++i)
  {
    const auto& jn = joint_ids[static_cast<std::size_t>(i)];
    auto it = sample_res_map.find(jn);
    if (it == sample_res_map.end())
      throw std::runtime_error(tesseract::common::strFormat("%smissing joint '%s'!", block.c_str(), jn.name().c_str()));

    out.resolution(i) = it->second[0];
    out.range(i, 0) = it->second[1];
    out.range(i, 1) = it->second[2];
  }

  return out;
}

}  // namespace tesseract::kinematics
