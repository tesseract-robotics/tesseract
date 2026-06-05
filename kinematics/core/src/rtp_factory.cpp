/**
 * @file rtp_factory.cpp
 * @brief Robot with Tool Positioner Inverse kinematics Factory implementation.
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
#include <tesseract/kinematics/rtp_factory.h>
#include <tesseract/kinematics/rtp_inv_kin.h>
#include <tesseract/kinematics/forward_kinematics.h>
#include <tesseract/kinematics/factory_utils.h>
#include <tesseract/scene_graph/graph.h>
#include <tesseract/scene_graph/joint.h>

#include <console_bridge/console.h>

#include <optional>

namespace tesseract::kinematics
{
std::unique_ptr<InverseKinematics> RTPInvKinFactory::create(const std::string& solver_name,
                                                            const tesseract::scene_graph::SceneGraph& scene_graph,
                                                            const tesseract::scene_graph::SceneState& scene_state,
                                                            const KinematicsPluginFactory& plugin_factory,
                                                            const YAML::Node& config) const
{
  ForwardKinematics::UPtr fwd_kin;
  InverseKinematics::UPtr inv_kin;
  std::optional<double> m_reach_explicit;
  Eigen::MatrixX2d sample_range;
  Eigen::VectorXd sample_res;

  try
  {
    if (YAML::Node n = config["manipulator_reach"])
      m_reach_explicit = n.as<double>();

    std::map<std::string, std::array<double, 3>> sample_res_map;
    if (YAML::Node sample_res_node = config["tool_sample_resolution"])
    {
      sample_res_map =
          parseSampleResolutionMap(sample_res_node, scene_graph, "RTPInvKinFactory", "tool_sample_resolution");
    }
    else
    {
      throw std::runtime_error("RTPInvKinFactory, missing 'tool_sample_resolution' entry!");
    }

    if (YAML::Node tool = config["tool_positioner"])
    {
      auto p_info = parsePluginInfo(tool, "RTPInvKinFactory", "tool_positioner");
      fwd_kin = plugin_factory.createFwdKin(p_info.class_name, p_info, scene_graph, scene_state);
      if (fwd_kin == nullptr)
        throw std::runtime_error("RTPInvKinFactory, failed to create tool forward kinematics!");
      if (sample_res_map.size() != static_cast<std::size_t>(fwd_kin->numJoints()))
        throw std::runtime_error("RTPInvKinFactory, tool sample resolution has incorrect number of joints!");
    }
    else
    {
      throw std::runtime_error("RTPInvKinFactory, missing 'tool_positioner' entry!");
    }

    sample_range.resize(fwd_kin->numJoints(), 2);
    sample_res.resize(fwd_kin->numJoints());
    std::vector<std::string> joint_names = fwd_kin->getJointNames();
    for (Eigen::Index i = 0; i < static_cast<Eigen::Index>(joint_names.size()); ++i)
    {
      const auto& jn = joint_names[static_cast<std::size_t>(i)];
      auto it = sample_res_map.find(jn);
      if (it == sample_res_map.end())
        throw std::runtime_error("RTPInvKinFactory, tool sample resolution missing joint '" + jn + "'!");

      sample_res(i) = it->second[0];
      sample_range(i, 0) = it->second[1];
      sample_range(i, 1) = it->second[2];
    }

    if (YAML::Node manipulator = config["manipulator"])
    {
      auto m_info = parsePluginInfo(manipulator, "RTPInvKinFactory", "manipulator");
      inv_kin = plugin_factory.createInvKin(m_info.class_name, m_info, scene_graph, scene_state);
      if (inv_kin == nullptr)
        throw std::runtime_error("RTPInvKinFactory, failed to create manipulator inverse kinematics!");
    }
    else
    {
      throw std::runtime_error("RTPInvKinFactory, missing 'manipulator' entry!");
    }

    if (m_reach_explicit.has_value())
    {
      return std::make_unique<RTPInvKin>(scene_graph,
                                         scene_state,
                                         std::move(inv_kin),
                                         *m_reach_explicit,
                                         std::move(fwd_kin),
                                         sample_range,
                                         sample_res,
                                         solver_name);
    }
    return std::make_unique<RTPInvKin>(
        scene_graph, scene_state, std::move(inv_kin), std::move(fwd_kin), sample_range, sample_res, solver_name);
  }
  catch (const std::exception& e)
  {
    CONSOLE_BRIDGE_logError("RTPInvKinFactory: Failed to create solver from config! Details: %s", e.what());
    return nullptr;
  }
}

PLUGIN_ANCHOR_IMPL(RTPInvKinFactoriesAnchor)

}  // namespace tesseract::kinematics

// NOLINTNEXTLINE(cppcoreguidelines-avoid-non-const-global-variables)
TESSERACT_ADD_INV_KIN_PLUGIN(tesseract::kinematics::RTPInvKinFactory, RTPInvKinFactory);
