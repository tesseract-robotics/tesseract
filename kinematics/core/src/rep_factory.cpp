/**
 * @file rep_factory.h
 * @brief Robot with External Positioner Inverse kinematics Factory.
 *
 * @author Levi Armstrong
 * @date Aug 30, 2021
 *
 * @copyright Copyright (c) 2021, Southwest Research Institute
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
#include <tesseract/kinematics/rep_factory.h>
#include <tesseract/kinematics/rep_inv_kin.h>
#include <tesseract/kinematics/forward_kinematics.h>
#include <tesseract/kinematics/factory_utils.h>
#include <tesseract/scene_graph/graph.h>
#include <tesseract/scene_graph/joint.h>

#include <console_bridge/console.h>
#include <optional>

namespace tesseract::kinematics
{
std::unique_ptr<InverseKinematics> REPInvKinFactory::create(const std::string& solver_name,
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
    // Get Reach (optional: when omitted, the ctor will derive it from the chain)
    if (YAML::Node n = config["manipulator_reach"])
      m_reach_explicit = n.as<double>();

    // Get positioner sample resolution
    std::map<std::string, std::array<double, 3>> sample_res_map;
    if (YAML::Node sample_res_node = config["positioner_sample_resolution"])
    {
      sample_res_map = parseSampleResolutionMap(sample_res_node, scene_graph,
                                                "REPInvKinFactory", "positioner_sample_resolution");
    }
    else
    {
      throw std::runtime_error("REPInvKinFactory, missing 'positioner_sample_resolution' entry!");
    }

    // Get Positioner
    if (YAML::Node positioner = config["positioner"])
    {
      auto p_info = parsePluginInfo(positioner, "REPInvKinFactory", "positioner");
      fwd_kin = plugin_factory.createFwdKin(p_info.class_name, p_info, scene_graph, scene_state);
      if (fwd_kin == nullptr)
        throw std::runtime_error("REPInvKinFactory, failed to create positioner forward kinematics!");
      if (sample_res_map.size() != static_cast<std::size_t>(fwd_kin->numJoints()))
        throw std::runtime_error("REPInvKinFactory, positioner sample resolution has incorrect number of joints!");
    }
    else
    {
      throw std::runtime_error("REPInvKinFactory, missing 'positioner' entry!");
    }

    // Load Positioner Resolution and Range
    sample_range.resize(fwd_kin->numJoints(), 2);
    sample_res.resize(fwd_kin->numJoints());
    std::vector<std::string> joint_names = fwd_kin->getJointNames();
    for (Eigen::Index i = 0; i < static_cast<Eigen::Index>(joint_names.size()); ++i)
    {
      const auto& jn = joint_names[static_cast<std::size_t>(i)];
      auto it = sample_res_map.find(jn);
      if (it == sample_res_map.end())
        throw std::runtime_error("REPInvKinFactory, positioner sample resolution missing joint '" + jn + "'!");

      sample_res(i) = it->second[0];
      sample_range(i, 0) = it->second[1];
      sample_range(i, 1) = it->second[2];
    }

    // Get Manipulator
    if (YAML::Node manipulator = config["manipulator"])
    {
      auto m_info = parsePluginInfo(manipulator, "REPInvKinFactory", "manipulator");
      inv_kin = plugin_factory.createInvKin(m_info.class_name, m_info, scene_graph, scene_state);
      if (inv_kin == nullptr)
        throw std::runtime_error("REPInvKinFactory, failed to create manipulator inverse kinematics!");
    }
    else
    {
      throw std::runtime_error("REPInvKinFactory, missing 'manipulator' entry!");
    }

    if (m_reach_explicit.has_value())
    {
      return std::make_unique<REPInvKin>(scene_graph,
                                          scene_state,
                                          std::move(inv_kin),
                                          *m_reach_explicit,
                                          std::move(fwd_kin),
                                          sample_range,
                                          sample_res,
                                          solver_name);
    }
    return std::make_unique<REPInvKin>(
        scene_graph, scene_state, std::move(inv_kin), std::move(fwd_kin), sample_range, sample_res, solver_name);
  }
  catch (const std::exception& e)
  {
    CONSOLE_BRIDGE_logError("REPInvKinFactory: Failed to parse yaml config data! Details: %s", e.what());
    return nullptr;
  }
}

PLUGIN_ANCHOR_IMPL(REPInvKinFactoriesAnchor)

}  // namespace tesseract::kinematics

// NOLINTNEXTLINE(cppcoreguidelines-avoid-non-const-global-variables)
TESSERACT_ADD_INV_KIN_PLUGIN(tesseract::kinematics::REPInvKinFactory, REPInvKinFactory);
