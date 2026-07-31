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
#include <map>
#include <string>
#include <vector>
#include <Eigen/Core>
#include <yaml-cpp/yaml.h>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract/common/types.h>
#include <tesseract/scene_graph/fwd.h>

namespace tesseract::kinematics
{
/** @brief One joint's sampling discretisation, as parsed from a "*_sample_resolution" entry. */
struct JointSampleSpec
{
  double resolution; /**< @brief Sample spacing */
  double min;        /**< @brief Lower bound of the sampled range */
  double max;        /**< @brief Upper bound of the sampled range */
};

/**
 * @brief Parse a "*_sample_resolution" sequence into a joint id -> discretisation map.
 * @details Each entry decodes as a PositionerSampleResolution, which supplies `name` and `value`
 *          and leaves `min`/`max` optional. This function applies the parts that need the scene
 *          graph: joint limits as the [min, max] defaults, and validation that [min, max] lies
 *          within the joint's full limits and is non-empty. A CONTINUOUS joint is unbounded: it
 *          defaults to [-pi, pi], accepts any ordered explicit range, and needs no `limits`.
 *
 *          Feed the result to toSampleGridConfig() once the sampled chain's joint order is known.
 * @param sample_res_node The YAML sequence node, e.g. config.at("tool_sample_resolution").getValue().
 * @param scene_graph     Used to look up each joint's limits.
 * @param block_label     Name of the config block, quoted into error messages (e.g.
 *                        "tool_sample_resolution"). The reporting factory's identity is supplied by
 *                        whoever catches, so it is not repeated here.
 * @throws std::runtime_error on any of the malformed-input or out-of-range conditions above.
 */
std::map<tesseract::common::JointId, JointSampleSpec>
parseSampleResolutionMap(const YAML::Node& sample_res_node,
                         const tesseract::scene_graph::SceneGraph& scene_graph,
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
 * @param joint_ids      Joint order of the sampled chain, from its forward kinematics.
 * @param block_label    Name of the config block, quoted into error messages.
 * @throws std::runtime_error if the map does not describe exactly @p joint_ids.
 */
SampleGridConfig toSampleGridConfig(const std::map<tesseract::common::JointId, JointSampleSpec>& sample_res_map,
                                    const std::vector<tesseract::common::JointId>& joint_ids,
                                    const std::string& block_label);

}  // namespace tesseract::kinematics

#endif  // TESSERACT_KINEMATICS_FACTORY_UTILS_H
