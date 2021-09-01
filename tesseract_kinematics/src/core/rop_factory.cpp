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
InverseKinematics::UPtr ROPInvKinFactory::create(const std::string& name,
                                                 const tesseract_scene_graph::SceneGraph& scene_graph,
                                                 const tesseract_scene_graph::SceneState& scene_state,
                                                 const KinematicsPluginFactory& plugin_factory,
                                                 const YAML::Node& config) const
{
  ForwardKinematics::UPtr fwd_kin;
  InverseKinematics::UPtr inv_kin;
  double m_reach;
  Eigen::MatrixX2d sample_range;
  Eigen::VectorXd sample_res;

  try
  {
    // Get Reach
    if (!config["manipulator_reach"])
      throw std::runtime_error("ROPInvKinFactory, manipulator_reach node is missing!");

    m_reach = config["manipulator_reach"].as<double>();

    // Get positioner sample resolution
    if (!config["positioner_sample_resolution"])
      throw std::runtime_error("ROPInvKinFactory, positioner_sample_resolution node is missing!");

    const YAML::Node& sample_res_node = config["positioner_sample_resolution"];
    std::map<std::string, std::array<double, 3>> sample_res_map;
    for (auto it = sample_res_node.begin(); it != sample_res_node.end(); ++it)
    {
      const YAML::Node& joint = *it;
      std::array<double, 3> values;
      std::string joint_name = joint["name"].as<std::string>();
      values[0] = joint["value"].as<double>();

      auto jnt = scene_graph.getJoint(joint_name);
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

    // Get Positioner
    if (!config["positioner"])
      throw std::runtime_error("ROPInvKinFactory, positioner node is missing!");

    const YAML::Node& positioner = config["positioner"];

    KinematicsPluginInfo p_info;
    p_info.group = name + "_positioner";
    p_info.class_name = positioner["class"].as<std::string>();
    p_info.name = p_info.class_name;
    p_info.config = positioner["config"];

    fwd_kin = plugin_factory.createFwdKin(p_info, scene_graph, scene_state);
    if (fwd_kin == nullptr)
      throw std::runtime_error("ROPInvKinFactory, failed to create positioner forward kinematics!");

    // Check size of sample resolution to make sure it matches the number of joints
    if (sample_res_map.size() != static_cast<std::size_t>(fwd_kin->numJoints()))
      throw std::runtime_error("ROPInvKinFactory, positioner sample resolution has incorrect number of joints!");

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
    if (!config["manipulator"])
      throw std::runtime_error("ROPInvKinFactory, manipulator node is missing!");

    const YAML::Node& manipulator = config["manipulator"];

    KinematicsPluginInfo m_info;
    m_info.group = name + "_manipulator";
    m_info.class_name = manipulator["class"].as<std::string>();
    m_info.name = p_info.class_name;
    m_info.config = manipulator["config"];

    inv_kin = plugin_factory.createInvKin(m_info, scene_graph, scene_state);
    if (inv_kin == nullptr)
      throw std::runtime_error("ROPInvKinFactory, failed to create positioner forward kinematics!");
  }
  catch (const std::exception& e)
  {
    CONSOLE_BRIDGE_logError("ROPInvKinFactory: Failed to parse yaml config data! Details: %s", e.what());
    return nullptr;
  }

  std::string solver_name = DEFAULT_ROP_INV_KIN_SOLVER_NAME;
  if (YAML::Node solver_name_node = config["solver_name"])
    solver_name = solver_name_node.as<std::string>();

  return std::make_unique<ROPInvKin>(name,
                                     scene_graph,
                                     scene_state,
                                     std::move(inv_kin),
                                     m_reach,
                                     std::move(fwd_kin),
                                     sample_range,
                                     sample_res,
                                     solver_name);
}
}  // namespace tesseract_kinematics

TESSERACT_ADD_PLUGIN(tesseract_kinematics::ROPInvKinFactory, ROPInvKinFactory);
