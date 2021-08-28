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
#include <tesseract_kinematics/kdl/kdl_fwd_kin_tree.h>
#include <tesseract_kinematics/kdl/kdl_inv_kin_chain_lma.h>
#include <tesseract_kinematics/kdl/kdl_inv_kin_chain_nr.h>

namespace tesseract_kinematics
{
ForwardKinematics::UPtr KDLFwdKinChainFactory::create(const std::string& name,
                                                      const tesseract_scene_graph::SceneGraph& scene_graph,
                                                      const tesseract_scene_graph::SceneState& /*scene_state*/,
                                                      const KinematicsPluginFactory& /*plugin_factory*/,
                                                      const YAML::Node& config) const
{
  std::string base_link;
  std::string tip_link;

  try
  {
    base_link = config["base_link"].as<std::string>();
    tip_link = config["tip_link"].as<std::string>();
  }
  catch (...)
  {
    CONSOLE_BRIDGE_logError("KDLFwdKinChainFactory: Failed to parse yaml config data!");
    return nullptr;
  }

  return std::make_unique<KDLFwdKinChain>(name, scene_graph, base_link, tip_link);
}

// ForwardKinematics::UPtr KDLFwdKinTreePlugin::create(const std::string& name,
//                                 const tesseract_scene_graph::SceneGraph& scene_graph,
//                                 const tesseract_scene_graph::SceneState& scene_state,
//                                 const YAML::Node& config) const
//{

//}

InverseKinematics::UPtr KDLInvKinChainLMAFactory::create(const std::string& name,
                                                         const tesseract_scene_graph::SceneGraph& scene_graph,
                                                         const tesseract_scene_graph::SceneState& /*scene_state*/,
                                                         const KinematicsPluginFactory& /*plugin_factory*/,
                                                         const YAML::Node& config) const
{
  std::string base_link;
  std::string tip_link;

  try
  {
    base_link = config["base_link"].as<std::string>();
    tip_link = config["tip_link"].as<std::string>();
  }
  catch (...)
  {
    CONSOLE_BRIDGE_logError("KDLInvKinChainLMAFactory: Failed to parse yaml config data!");
    return nullptr;
  }

  return std::make_unique<KDLInvKinChainLMA>(name, scene_graph, base_link, tip_link);
}

InverseKinematics::UPtr KDLInvKinChainNRFactory::create(const std::string& name,
                                                        const tesseract_scene_graph::SceneGraph& scene_graph,
                                                        const tesseract_scene_graph::SceneState& /*scene_state*/,
                                                        const KinematicsPluginFactory& /*plugin_factory*/,
                                                        const YAML::Node& config) const
{
  std::string base_link;
  std::string tip_link;

  try
  {
    base_link = config["base_link"].as<std::string>();
    tip_link = config["tip_link"].as<std::string>();
  }
  catch (...)
  {
    CONSOLE_BRIDGE_logError("KDLInvKinChainNRFactory: Failed to parse yaml config data!");
    return nullptr;
  }

  return std::make_unique<KDLInvKinChainNR>(name, scene_graph, base_link, tip_link);
}

}  // namespace tesseract_kinematics

TESSERACT_ADD_PLUGIN(tesseract_kinematics::KDLFwdKinChainFactory, KDLFwdKinChainFactory);
TESSERACT_ADD_PLUGIN(tesseract_kinematics::KDLInvKinChainLMAFactory, KDLInvKinChainLMAFactory);
TESSERACT_ADD_PLUGIN(tesseract_kinematics::KDLInvKinChainNRFactory, KDLInvKinChainNRFactory);
