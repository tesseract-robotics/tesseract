/**
 * @file rop_factory.cpp
 * @brief Robot on Positioner Inverse kinematics factory.
 *
 * @author Levi Armstrong
 * @date Aug 30, 2021
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

#include <tesseract_kinematics/core/rop_factory.h>
#include <tesseract_kinematics/core/rop_inv_kin.h>

namespace tesseract_kinematics
{
InverseKinematics::UPtr ROPInvKinFactory::create(const std::string& solver_name,
                                                 const tesseract_scene_graph::SceneGraph& scene_graph,
                                                 const tesseract_scene_graph::SceneState& scene_state,
                                                 const KinematicsPluginFactory& plugin_factory,
                                                 const YAML::Node& config) const
{
  ForwardKinematics::UPtr fwd_kin;
  InverseKinematics::UPtr inv_kin;
  double m_reach{ 0 };
  Eigen::MatrixX2d sample_range;
  Eigen::VectorXd sample_res;

  try
  {
    // Get Reach
    if (YAML::Node n = config["manipulator_reach"])
      m_reach = n.as<double>();
    else
      throw std::runtime_error("ROPInvKinFactory, missing 'manipulator_reach' entry!");

    // Get positioner sample resolution
    std::map<std::string, std::array<double, 3>> sample_res_map;
    if (YAML::Node sample_res_node = config["positioner_sample_resolution"])
    {
      for (auto it = sample_res_node.begin(); it != sample_res_node.end(); ++it)
      {
        const YAML::Node& joint = *it;
        std::array<double, 3> values{ 0, 0, 0 };

        std::string joint_name;
        if (YAML::Node n = joint["name"])
          joint_name = n.as<std::string>();
        else
          throw std::runtime_error("ROPInvKinFactory, 'positioner_sample_resolution' missing 'name' entry!");

        if (YAML::Node n = joint["value"])
          values[0] = n.as<double>();
        else
          throw std::runtime_error("ROPInvKinFactory, 'positioner_sample_resolution' missing 'value' entry!");

        auto jnt = scene_graph.getJoint(joint_name);
        if (jnt == nullptr)
          throw std::runtime_error("ROPInvKinFactory, 'positioner_sample_resolution' failed to find joint in scene "
                                   "graph!");

        values[1] = jnt->limits->lower;
        values[2] = jnt->limits->upper;

        if (YAML::Node min = joint["min"])
          values[1] = min.as<double>();

        if (YAML::Node max = joint["max"])
          values[2] = max.as<double>();

        if (values[1] < jnt->limits->lower)
          throw std::runtime_error("ROPInvKinFactory, sample range minimum is less than joint minimum!");

        if (values[2] > jnt->limits->upper)
          throw std::runtime_error("ROPInvKinFactory, sample range maximum is greater than joint maximum!");

        if (values[1] > values[2])
          throw std::runtime_error("ROPInvKinFactory, sample range is not valid!");

        sample_res_map[joint_name] = values;
      }
    }
    else
    {
      throw std::runtime_error("ROPInvKinFactory, missing 'positioner_sample_resolution' entry!");
    }

    // Get Positioner
    if (YAML::Node positioner = config["positioner"])
    {
      tesseract_common::PluginInfo p_info;
      if (YAML::Node n = positioner["class"])
        p_info.class_name = n.as<std::string>();
      else
        throw std::runtime_error("ROPInvKinFactory, 'positioner' missing 'class' entry!");

      if (YAML::Node n = positioner["config"])
        p_info.config = n;

      fwd_kin = plugin_factory.createFwdKin(p_info.class_name, p_info, scene_graph, scene_state);
      if (fwd_kin == nullptr)
        throw std::runtime_error("ROPInvKinFactory, failed to create positioner forward kinematics!");

      if (sample_res_map.size() != static_cast<std::size_t>(fwd_kin->numJoints()))
        throw std::runtime_error("ROPInvKinFactory, positioner sample resolution has incorrect number of joints!");
    }
    else
    {
      throw std::runtime_error("ROPInvKinFactory, missing 'positioner' entry!");
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
        throw std::runtime_error("ROPInvKinFactory, positioner sample resolution missing joint '" + jn + "'!");

      sample_res(i) = it->second[0];
      sample_range(i, 0) = it->second[1];
      sample_range(i, 1) = it->second[2];
    }

    // Get Manipulator
    if (YAML::Node manipulator = config["manipulator"])
    {
      tesseract_common::PluginInfo m_info;
      if (YAML::Node n = manipulator["class"])
        m_info.class_name = n.as<std::string>();
      else
        throw std::runtime_error("ROPInvKinFactory, 'manipulator' missing 'class' entry!");

      if (YAML::Node n = manipulator["config"])
        m_info.config = n;

      inv_kin = plugin_factory.createInvKin(m_info.class_name, m_info, scene_graph, scene_state);
      if (inv_kin == nullptr)
        throw std::runtime_error("ROPInvKinFactory, failed to create positioner forward kinematics!");
    }
    else
    {
      throw std::runtime_error("ROPInvKinFactory, missing 'manipulator' entry!");
    }
  }
  catch (const std::exception& e)
  {
    CONSOLE_BRIDGE_logError("ROPInvKinFactory: Failed to parse yaml config data! Details: %s", e.what());
    return nullptr;
  }

  return std::make_unique<ROPInvKin>(
      scene_graph, scene_state, std::move(inv_kin), m_reach, std::move(fwd_kin), sample_range, sample_res, solver_name);
}

TESSERACT_PLUGIN_ANCHOR_IMPL(ROPInvKinFactoriesAnchor)

}  // namespace tesseract_kinematics

// NOLINTNEXTLINE(cppcoreguidelines-avoid-non-const-global-variables)
TESSERACT_ADD_INV_KIN_PLUGIN(tesseract_kinematics::ROPInvKinFactory, ROPInvKinFactory);
