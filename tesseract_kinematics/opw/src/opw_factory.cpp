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
InverseKinematics::UPtr OPWInvKinFactory::create(const std::string& solver_name,
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
    if (YAML::Node n = config["base_link"])
      base_link = n.as<std::string>();
    else
      throw std::runtime_error("OPWInvKinFactory, missing 'base_link' entry");

    if (YAML::Node n = config["tip_link"])
      tip_link = n.as<std::string>();
    else
      throw std::runtime_error("OPWInvKinFactory, missing 'tip_link' entry");

    if (YAML::Node opw_params = config["params"])
    {
      if (YAML::Node n = opw_params["a1"])
        params.a1 = n.as<double>();
      else
        throw std::runtime_error("OPWInvKinFactory, 'params' missing 'a1' entry");

      if (YAML::Node n = opw_params["a2"])
        params.a2 = n.as<double>();
      else
        throw std::runtime_error("OPWInvKinFactory, 'params' missing 'a2' entry");

      if (YAML::Node n = opw_params["b"])
        params.b = n.as<double>();
      else
        throw std::runtime_error("OPWInvKinFactory, 'params' missing 'b' entry");

      if (YAML::Node n = opw_params["c1"])
        params.c1 = n.as<double>();
      else
        throw std::runtime_error("OPWInvKinFactory, 'params' missing 'c1' entry");

      if (YAML::Node n = opw_params["c2"])
        params.c2 = n.as<double>();
      else
        throw std::runtime_error("OPWInvKinFactory, 'params' missing 'c2' entry");

      if (YAML::Node n = opw_params["c3"])
        params.c3 = n.as<double>();
      else
        throw std::runtime_error("OPWInvKinFactory, 'params' missing 'c3' entry");

      if (YAML::Node n = opw_params["c4"])
        params.c4 = n.as<double>();
      else
        throw std::runtime_error("OPWInvKinFactory, 'params' missing 'c4' entry");

      if (YAML::Node offsets = opw_params["offsets"])
      {
        auto o = offsets.as<std::vector<double>>();
        if (o.size() != 6)
          throw std::runtime_error("OPWInvKinFactory, offsets should have six elements!");

        std::copy(o.begin(), o.end(), params.offsets.begin());
      }

      if (YAML::Node sign_corrections = opw_params["sign_corrections"])
      {
        auto sc = sign_corrections.as<std::vector<int>>();
        if (sc.size() != 6)
          throw std::runtime_error("OPWInvKinFactory, sign_corrections should have six elements!");

        for (std::size_t i = 0; i < sc.size(); ++i)
        {
          if (sc[i] == 1)
            params.sign_corrections[i] = 1;
          else if (sc[i] == -1)
            params.sign_corrections[i] = -1;
          else
            throw std::runtime_error("OPWInvKinFactory, sign_corrections can only contain 1 or -1");
        }
      }
    }
    else
    {
      throw std::runtime_error("OPWInvKinFactory, missing 'params' entry");
    }

    path = scene_graph.getShortestPath(base_link, tip_link);
  }
  catch (const std::exception& e)
  {
    CONSOLE_BRIDGE_logError("OPWInvKinFactory: Failed to parse yaml config data! Details: %s", e.what());
    return nullptr;
  }

  return std::make_unique<OPWInvKin>(params, base_link, tip_link, path.active_joints, solver_name);
}

TESSERACT_PLUGIN_ANCHOR_IMPL(OPWFactoriesAnchor)

}  // namespace tesseract_kinematics

// NOLINTNEXTLINE(cppcoreguidelines-avoid-non-const-global-variables)
TESSERACT_ADD_INV_KIN_PLUGIN(tesseract_kinematics::OPWInvKinFactory, OPWInvKinFactory);
