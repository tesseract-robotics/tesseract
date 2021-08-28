/**
 * @file opw_factory.h
 * @brief Tesseract OPW Inverse kinematics Factory
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

#include <tesseract_kinematics/opw/opw_factory.h>
#include <tesseract_kinematics/opw/opw_inv_kin.h>

namespace tesseract_kinematics
{
InverseKinematics::UPtr OPWInvKinFactory::create(const std::string& name,
                                                 const tesseract_scene_graph::SceneGraph& scene_graph,
                                                 const tesseract_scene_graph::SceneState& /*scene_state*/,
                                                 const KinematicsPluginFactory& /*plugin_factory*/,
                                                 const YAML::Node& config) const
{
  std::string base_link;
  std::string tip_link;
  opw_kinematics::Parameters<double> params;
  tesseract_scene_graph::ShortestPath path;

  try
  {
    base_link = config["base_link"].as<std::string>();
    tip_link = config["tip_link"].as<std::string>();

    const YAML::Node& opw_params = config["params"];
    params.a1 = opw_params["a1"].as<double>();
    params.a2 = opw_params["a2"].as<double>();
    params.b = opw_params["b"].as<double>();
    params.c1 = opw_params["c1"].as<double>();
    params.c2 = opw_params["c2"].as<double>();
    params.c3 = opw_params["c3"].as<double>();
    params.c4 = opw_params["c4"].as<double>();

    if (YAML::Node offsets = opw_params["offsets"])
      params.offsets = offsets.as<std::array<double, 6>>();

    if (YAML::Node sign_corrections = opw_params["sign_corrections"])
      params.sign_corrections = sign_corrections.as<std::array<signed char, 6>>();

    path = scene_graph.getShortestPath(base_link, tip_link);
  }
  catch (...)
  {
    CONSOLE_BRIDGE_logError("OPWInvKinFactory: Failed to parse yaml config data!");
    return nullptr;
  }

  return std::make_unique<OPWInvKin>(name, params, base_link, tip_link, path.active_joints);
}
}  // namespace tesseract_kinematics

TESSERACT_ADD_PLUGIN(tesseract_kinematics::OPWInvKinFactory, OPWInvKinFactory);
