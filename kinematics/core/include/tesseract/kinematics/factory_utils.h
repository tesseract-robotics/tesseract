/**
 * @file factory_utils.h
 * @brief Shared YAML-parsing helpers for kinematics plugin factories.
 */
#ifndef TESSERACT_KINEMATICS_FACTORY_UTILS_H
#define TESSERACT_KINEMATICS_FACTORY_UTILS_H

#include <tesseract/common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <array>
#include <map>
#include <string>
#include <yaml-cpp/yaml.h>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract/common/plugin_info.h>
#include <tesseract/scene_graph/fwd.h>

namespace tesseract::kinematics
{
/**
 * @brief Parse a `<sub_node>: { class: ..., config: ... }` block into a PluginInfo.
 * @param sub_node      The YAML node for the sub-block (e.g. config["positioner"]).
 * @param error_prefix  Prefix used in thrown error messages (e.g. "ROPInvKinFactory").
 * @param block_label   Human-readable name of the block (e.g. "positioner") used in error messages.
 * @throws std::runtime_error if `class` is missing.
 */
tesseract::common::PluginInfo parsePluginInfo(const YAML::Node& sub_node,
                                              const std::string& error_prefix,
                                              const std::string& block_label);

/**
 * @brief Parse a YAML "*_sample_resolution" sequence into a name -> [value, min, max] map.
 * @details For each entry: requires `name` and `value`; uses joint limits from @p scene_graph as
 *          [min, max] defaults; allows optional `min`/`max` overrides; validates that
 *          [min, max] lies within the joint's full limits and is non-empty.
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

}  // namespace tesseract::kinematics

#endif  // TESSERACT_KINEMATICS_FACTORY_UTILS_H
