/**
 * @file rop_factory.cpp
 * @brief Robot on Positioner Inverse kinematics factory.
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

#include <tesseract/kinematics/rop_factory.h>
#include <tesseract/kinematics/rop_inv_kin.h>
#include <tesseract/kinematics/forward_kinematics.h>
#include <tesseract/kinematics/yaml_extensions.h>
#include <tesseract/scene_graph/graph.h>
#include <tesseract/scene_graph/joint.h>

#include <tesseract/common/schema_registration.h>
#include <tesseract/common/property_tree.h>

#include <console_bridge/console.h>

namespace
{
tesseract::common::PropertyTree ropInvKinFactorySchema()
{
  using namespace tesseract::common;

  static const std::string kItemType = "tesseract::kinematics::PositionerSampleResolution";

  // clang-format off
  return PropertyTreeBuilder()
    .attribute(property_attribute::TYPE, property_type::CONTAINER)
    .doubleNum("manipulator_reach").required().done()
    .customType("positioner_sample_resolution",
          property_type::createList(kItemType)).required().done()
    .customType("positioner", "tesseract::kinematics::FwdKinFactory")
      .required().acceptsDerivedTypes().validator(validateCustomType).done()
    .customType("manipulator", "tesseract::kinematics::InvKinFactory")
      .required().acceptsDerivedTypes().validator(validateCustomType).done()
    .build();
  // clang-format on
}
}  // namespace

namespace tesseract::kinematics
{
tesseract::common::PropertyTree ROPInvKinFactory::schema() const { return ropInvKinFactorySchema(); }

std::unique_ptr<InverseKinematics> ROPInvKinFactory::create(const std::string& solver_name,
                                                            const tesseract::scene_graph::SceneGraph& scene_graph,
                                                            const tesseract::scene_graph::SceneState& scene_state,
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
      for (const auto& entry : sample_res_node)
      {
        auto psr = entry.as<PositionerSampleResolution>();

        auto jnt = scene_graph.getJoint(psr.name);
        if (jnt == nullptr)
          throw std::runtime_error("ROPInvKinFactory, 'positioner_sample_resolution' failed to find joint '" +
                                   psr.name + "' in scene graph!");

        double range_min = psr.min.value_or(jnt->limits->lower);
        double range_max = psr.max.value_or(jnt->limits->upper);

        if (range_min < jnt->limits->lower)
          throw std::runtime_error("ROPInvKinFactory, sample range minimum is less than joint minimum!");

        if (range_max > jnt->limits->upper)
          throw std::runtime_error("ROPInvKinFactory, sample range maximum is greater than joint maximum!");

        if (range_min > range_max)
          throw std::runtime_error("ROPInvKinFactory, sample range is not valid!");

        sample_res_map[psr.name] = { psr.value, range_min, range_max };
      }
    }
    else
    {
      throw std::runtime_error("ROPInvKinFactory, missing 'positioner_sample_resolution' entry!");
    }

    // Get Positioner
    if (YAML::Node positioner = config["positioner"])
    {
      tesseract::common::PluginInfo p_info;
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
      tesseract::common::PluginInfo m_info;
      if (YAML::Node n = manipulator["class"])
        m_info.class_name = n.as<std::string>();
      else
        throw std::runtime_error("ROPInvKinFactory, 'manipulator' missing 'class' entry!");

      if (YAML::Node n = manipulator["config"])
        m_info.config = n;

      inv_kin = plugin_factory.createInvKin(m_info.class_name, m_info, scene_graph, scene_state);
      if (inv_kin == nullptr)
        throw std::runtime_error("ROPInvKinFactory, failed to create positioner inverse kinematics!");
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

PLUGIN_ANCHOR_IMPL(ROPInvKinFactoriesAnchor)

}  // namespace tesseract::kinematics

// NOLINTNEXTLINE(cppcoreguidelines-avoid-non-const-global-variables)
TESSERACT_ADD_INV_KIN_PLUGIN(tesseract::kinematics::ROPInvKinFactory, ROPInvKinFactory);
TESSERACT_SCHEMA_REGISTER(ROPInvKinFactory, ropInvKinFactorySchema);
TESSERACT_SCHEMA_REGISTER_DERIVED_TYPE(tesseract::kinematics::InvKinFactory, ROPInvKinFactory);
