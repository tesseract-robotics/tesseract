/**
 * @file factory_utils.h
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
#ifndef TESSERACT_KINEMATICS_FACTORY_UTILS_H
#define TESSERACT_KINEMATICS_FACTORY_UTILS_H

#include <tesseract/common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <array>
#include <map>
#include <string>
#include <vector>
#include <Eigen/Core>
#include <yaml-cpp/yaml.h>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract/common/plugin_info.h>
#include <tesseract/scene_graph/fwd.h>

namespace tesseract::kinematics
{
/**
 * @brief Parse a `<sub_node>: { class: ..., config: ... }` block into a PluginInfo.
 * @details Decoding is delegated to the PluginInfo YAML converter in tesseract::common; this
 *          wrapper only annotates the failure with which block of a factory config was at fault.
 * @param sub_node      The YAML node for the sub-block (e.g. config["positioner"]).
 * @param error_prefix  Prefix used in thrown error messages (e.g. "ROPInvKinFactory").
 * @param block_label   Human-readable name of the block (e.g. "positioner") used in error messages.
 * @throws std::runtime_error if the block does not decode to a PluginInfo (e.g. `class` missing).
 */
tesseract::common::PluginInfo parsePluginInfo(const YAML::Node& sub_node,
                                              const std::string& error_prefix,
                                              const std::string& block_label);

/**
 * @brief Parse a YAML "*_sample_resolution" sequence into a name -> [resolution, min, max] map.
 * @details For each entry: requires `name` and `value`; uses joint limits from @p scene_graph as
 *          [min, max] defaults; allows optional `min`/`max` overrides; validates that
 *          [min, max] lies within the joint's full limits and is non-empty.
 *
 *          Factories should call this before constructing the sampled chain's forward kinematics:
 *          it is what rejects a joint whose limits are null, and chain construction dereferences
 *          those limits. Feed the result to toSampleGridConfig() once the chain's joint order is
 *          known.
 * @param sample_res_node The YAML sequence node.
 * @param scene_graph     Used to look up each joint's limits.
 * @param error_prefix    Prefix used in thrown error messages.
 * @param block_label     Human-readable name of the block (e.g. "tool_sample_resolution").
 * @throws std::runtime_error on any of the malformed-input or out-of-range conditions above.
 */
std::map<std::string, std::array<double, 3>>
parseSampleResolutionMap(const YAML::Node& sample_res_node,
                         const tesseract::scene_graph::SceneGraph& scene_graph,
                         const std::string& error_prefix,
                         const std::string& block_label);

/** @brief A sampled chain's per-joint discretisation, ordered to match that chain's joint order. */
struct SampleGridConfig
{
  Eigen::MatrixX2d range;     /**< @brief One row per joint: [min, max] */
  Eigen::VectorXd resolution; /**< @brief One entry per joint */
};

/**
 * @brief Reorder a parseSampleResolutionMap() result onto a chain's joint order, producing the
 *        range/resolution pair that buildSampleGrid() consumes.
 * @param sample_res_map Output of parseSampleResolutionMap().
 * @param joint_names    Joint order of the sampled chain, from its forward kinematics.
 * @param error_prefix   Prefix used in thrown error messages.
 * @param block_label    Human-readable name of the block (e.g. "tool_sample_resolution").
 * @throws std::runtime_error if the map does not describe exactly @p joint_names.
 */
SampleGridConfig toSampleGridConfig(const std::map<std::string, std::array<double, 3>>& sample_res_map,
                                    const std::vector<std::string>& joint_names,
                                    const std::string& error_prefix,
                                    const std::string& block_label);

}  // namespace tesseract::kinematics

#endif  // TESSERACT_KINEMATICS_FACTORY_UTILS_H
