/**
 * @file ur_factory.cpp
 * @brief Tesseract UR Inverse kinematics Factory
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

#include <tesseract_common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <string>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract_kinematics/ur/ur_factory.h>
#include <tesseract_kinematics/ur/ur_inv_kin.h>
#include <tesseract_kinematics/core/types.h>

namespace tesseract_kinematics
{
InverseKinematics::UPtr URInvKinFactory::create(const std::string& solver_name,
                                                const tesseract_scene_graph::SceneGraph& scene_graph,
                                                const tesseract_scene_graph::SceneState& /*scene_state*/,
                                                const KinematicsPluginFactory& /*plugin_factory*/,
                                                const YAML::Node& config) const
{
  std::string base_link;
  std::string tip_link;
  tesseract_kinematics::URParameters params;
  tesseract_scene_graph::ShortestPath path;

  try
  {
    if (YAML::Node n = config["base_link"])
      base_link = n.as<std::string>();
    else
      throw std::runtime_error("URInvKinFactory, missing 'base_link' entry");

    if (YAML::Node n = config["tip_link"])
      tip_link = n.as<std::string>();
    else
      throw std::runtime_error("URInvKinFactory, missing 'tip_link' entry");

    if (YAML::Node model = config["model"])
    {
      auto model_str = model.as<std::string>();
      if (model_str == "UR3")
        params = UR3Parameters;
      else if (model_str == "UR5")
        params = UR5Parameters;
      else if (model_str == "UR10")
        params = UR10Parameters;
      else if (model_str == "UR3e")
        params = UR3eParameters;
      else if (model_str == "UR5e")
        params = UR5eParameters;
      else if (model_str == "UR10e")
        params = UR10eParameters;
      else
      {
        CONSOLE_BRIDGE_logError("URInvKinFactory: Invalid model!");
        return nullptr;
      }
    }
    else
    {
      if (YAML::Node ur_params = config["params"])
      {
        if (YAML::Node n = ur_params["d1"])
          params.d1 = n.as<double>();
        else
          throw std::runtime_error("URInvKinFactory, 'params' missing 'd1' entry");

        if (YAML::Node n = ur_params["a2"])
          params.a2 = n.as<double>();
        else
          throw std::runtime_error("URInvKinFactory, 'params' missing 'a2' entry");

        if (YAML::Node n = ur_params["a3"])
          params.a3 = n.as<double>();
        else
          throw std::runtime_error("URInvKinFactory, 'params' missing 'a3' entry");

        if (YAML::Node n = ur_params["d4"])
          params.d4 = n.as<double>();
        else
          throw std::runtime_error("URInvKinFactory, 'params' missing 'd4' entry");

        if (YAML::Node n = ur_params["d5"])
          params.d5 = n.as<double>();
        else
          throw std::runtime_error("URInvKinFactory, 'params' missing 'd5' entry");

        if (YAML::Node n = ur_params["d6"])
          params.d6 = n.as<double>();
        else
          throw std::runtime_error("URInvKinFactory, 'params' missing 'd6' entry");
      }
      else
      {
        throw std::runtime_error("URInvKinFactory, missing 'params' or 'model' entry");
      }
    }

    path = scene_graph.getShortestPath(base_link, tip_link);
  }
  catch (const std::exception& e)
  {
    CONSOLE_BRIDGE_logError("URInvKinFactory: Failed to parse yaml config data! Details: %s", e.what());
    return nullptr;
  }

  return std::make_unique<URInvKin>(params, base_link, tip_link, path.active_joints, solver_name);
}

TESSERACT_PLUGIN_ANCHOR_IMPL(URFactoriesAnchor)

}  // namespace tesseract_kinematics

// NOLINTNEXTLINE(cppcoreguidelines-avoid-non-const-global-variables)
TESSERACT_ADD_INV_KIN_PLUGIN(tesseract_kinematics::URInvKinFactory, URInvKinFactory);
