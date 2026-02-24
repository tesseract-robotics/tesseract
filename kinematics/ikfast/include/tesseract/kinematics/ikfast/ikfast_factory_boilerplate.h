/**
 * @file ikfast_factory_boilerplate.h
 * @brief Tesseract IKFast Factory Boilerplate.
 *
 * @author Michael Ripperger, Roelof Oomen
 * @date July 19, 2023
 *
 * @copyright Copyright (c) 2023, Southwest Research Institute
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
#ifndef TESSERACT_KINEMATICS_IKFAST_FACTORY_BOILERPLATE_H
#define TESSERACT_KINEMATICS_IKFAST_FACTORY_BOILERPLATE_H

#include <tesseract/kinematics/kinematics_plugin_factory.h>
#include <tesseract/kinematics/ikfast/impl/ikfast_inv_kin.hpp>
#include <tesseract/scene_graph/graph.h>

namespace tesseract::kinematics
{
class IKFastInvKinFactory : public InvKinFactory
{
public:
  InverseKinematics::UPtr create(const std::string& solver_name,
                                 const tesseract::scene_graph::SceneGraph& scene_graph,
                                 const tesseract::scene_graph::SceneState& /*scene_state*/,
                                 const KinematicsPluginFactory& /*plugin_factory*/,
                                 const YAML::Node& config) const override
  {
    std::string base_link;
    std::string tip_link;
    std::size_t n_joints = 0;
    std::vector<std::string> active_joints;
    std::vector<std::vector<double>> free_joint_states;
    try
    {
      if (YAML::Node n = config["base_link"])
        base_link = n.as<std::string>();
      else
        throw std::runtime_error("IKFastInvKinFactory, missing 'base_link' entry");

      if (YAML::Node n = config["tip_link"])
        tip_link = n.as<std::string>();
      else
        throw std::runtime_error("IKFastInvKinFactory, missing 'tip_link' entry");

      if (YAML::Node n = config["n_joints"])
        n_joints = n.as<std::size_t>();
      else
        throw std::runtime_error("IKFastInvKinFactory, missing 'n_joints' entry");

      // Get the active joints in between the base link and tip link
      active_joints = scene_graph.getShortestPath(base_link, tip_link).active_joints;

      std::size_t free_joints_required = active_joints.size() - n_joints;
      std::map<std::size_t, std::vector<double>> free_joint_states_map;
      // Get the free joint states
      if (YAML::Node free_joint_states_node = config["free_joint_states"])
      {
        if (free_joints_required == 0)
          throw std::runtime_error("IKFastInvKinFactory, entry 'free_joint_states' exists but no free joints exist");

        for (std::size_t idx = 0; idx < free_joint_states_node.size(); ++idx)
        {
          // Check the joints specification
          if (free_joint_states_node[idx].size() != free_joints_required)
          {
            std::stringstream ss;
            ss << "IKFastInvKinFactory, Number of active joints (" << active_joints.size()
               << ") must equal the sum of the number of nominal joints (" << n_joints
               << ") and the number of free joints (" << free_joint_states_map.size() << ")";
            throw std::runtime_error(ss.str());
          }
          free_joint_states_map[idx] = free_joint_states_node[idx].as<std::vector<double>>();
        }
      }
      else
      {
        if (free_joints_required > 0)
        {
          std::stringstream ss;
          ss << "IKFastInvKinFactory, missing 'free_joint_states' entry, but states for " << free_joints_required
             << " free joints required";
          throw std::runtime_error(ss.str());
        }
        CONSOLE_BRIDGE_logDebug("IKFastInvKinFactory: No 'free_joint_states' entry found, none required");
      }

      free_joint_states.reserve(free_joint_states_map.size());
      std::transform(free_joint_states_map.begin(),
                     free_joint_states_map.end(),
                     std::back_inserter(free_joint_states),
                     [](const std::pair<const std::size_t, std::vector<double>>& pair) { return pair.second; });
    }
    catch (const std::exception& e)
    {
      CONSOLE_BRIDGE_logError("IKFastInvKinFactory: Failed to parse yaml config data! Details: %s", e.what());
      return nullptr;
    }

    return std::make_unique<IKFastInvKin>(base_link, tip_link, active_joints, solver_name, free_joint_states);
  }
};

}  // namespace tesseract::kinematics

#endif  // TESSERACT_KINEMATICS_IKFAST_FACTORY_BOILERPLATE_H
