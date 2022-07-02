/**
 * @file kdl_factories.h
 * @brief Tesseract KDL Factories.
 *
 * @author Levi Armstrong
 * @date Aug 27, 2021
 * @version TODO
 * @bug No known bugs
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

#include <tesseract_kinematics/kdl/kdl_factories.h>
#include <tesseract_kinematics/kdl/kdl_fwd_kin_chain.h>
#include <tesseract_kinematics/kdl/kdl_inv_kin_chain_lma.h>
#include <tesseract_kinematics/kdl/kdl_inv_kin_chain_nr.h>

namespace tesseract_kinematics
{
ForwardKinematics::UPtr KDLFwdKinChainFactory::create(const std::string& solver_name,
                                                      const tesseract_scene_graph::SceneGraph& scene_graph,
                                                      const tesseract_scene_graph::SceneState& /*scene_state*/,
                                                      const KinematicsPluginFactory& /*plugin_factory*/,
                                                      const YAML::Node& config) const
{
  std::string base_link;
  std::string tip_link;

  try
  {
    if (YAML::Node n = config["base_link"])
      base_link = n.as<std::string>();
    else
      throw std::runtime_error("KDLFwdKinChainFactory, missing 'base_link' entry");

    if (YAML::Node n = config["tip_link"])
      tip_link = n.as<std::string>();
    else
      throw std::runtime_error("KDLFwdKinChainFactory, missing 'tip_link' entry");
  }
  catch (const std::exception& e)
  {
    CONSOLE_BRIDGE_logError("KDLFwdKinChainFactory: Failed to parse yaml config data! Details: %s", e.what());
    return nullptr;
  }

  return std::make_unique<KDLFwdKinChain>(scene_graph, base_link, tip_link, solver_name);
}

InverseKinematics::UPtr KDLInvKinChainLMAFactory::create(const std::string& solver_name,
                                                         const tesseract_scene_graph::SceneGraph& scene_graph,
                                                         const tesseract_scene_graph::SceneState& /*scene_state*/,
                                                         const KinematicsPluginFactory& /*plugin_factory*/,
                                                         const YAML::Node& config) const
{
  std::string base_link;
  std::string tip_link;

  try
  {
    if (YAML::Node n = config["base_link"])
      base_link = n.as<std::string>();
    else
      throw std::runtime_error("KDLInvKinChainLMAFactory, missing 'base_link' entry");

    if (YAML::Node n = config["tip_link"])
      tip_link = n.as<std::string>();
    else
      throw std::runtime_error("KDLInvKinChainLMAFactory, missing 'tip_link' entry");
  }
  catch (const std::exception& e)
  {
    CONSOLE_BRIDGE_logError("KDLInvKinChainLMAFactory: Failed to parse yaml config data! Details: %s", e.what());
    return nullptr;
  }

  return std::make_unique<KDLInvKinChainLMA>(scene_graph, base_link, tip_link, solver_name);
}

InverseKinematics::UPtr KDLInvKinChainNRFactory::create(const std::string& solver_name,
                                                        const tesseract_scene_graph::SceneGraph& scene_graph,
                                                        const tesseract_scene_graph::SceneState& /*scene_state*/,
                                                        const KinematicsPluginFactory& /*plugin_factory*/,
                                                        const YAML::Node& config) const
{
  std::string base_link;
  std::string tip_link;

  try
  {
    if (YAML::Node n = config["base_link"])
      base_link = n.as<std::string>();
    else
      throw std::runtime_error("KDLInvKinChainNRFactory, missing 'base_link' entry");

    if (YAML::Node n = config["tip_link"])
      tip_link = n.as<std::string>();
    else
      throw std::runtime_error("KDLInvKinChainNRFactory, missing 'tip_link' entry");
  }
  catch (const std::exception& e)
  {
    CONSOLE_BRIDGE_logError("KDLInvKinChainNRFactory: Failed to parse yaml config data! Details: %s", e.what());
    return nullptr;
  }

  return std::make_unique<KDLInvKinChainNR>(scene_graph, base_link, tip_link, solver_name);
}

TESSERACT_PLUGIN_ANCHOR_IMPL(KDLFactoriesAnchor)

}  // namespace tesseract_kinematics

// NOLINTNEXTLINE(cppcoreguidelines-avoid-non-const-global-variables)
TESSERACT_ADD_FWD_KIN_PLUGIN(tesseract_kinematics::KDLFwdKinChainFactory, KDLFwdKinChainFactory);
// NOLINTNEXTLINE(cppcoreguidelines-avoid-non-const-global-variables)
TESSERACT_ADD_INV_KIN_PLUGIN(tesseract_kinematics::KDLInvKinChainLMAFactory, KDLInvKinChainLMAFactory);
// NOLINTNEXTLINE(cppcoreguidelines-avoid-non-const-global-variables)
TESSERACT_ADD_INV_KIN_PLUGIN(tesseract_kinematics::KDLInvKinChainNRFactory, KDLInvKinChainNRFactory);
