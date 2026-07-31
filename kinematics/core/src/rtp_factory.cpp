/**
 * @file rtp_factory.cpp
 * @brief Robot with Tool Positioner Inverse kinematics Factory implementation.
 *
 * @author Roelof Oomen
 * @date May 1, 2026
 *
 * @copyright Copyright (c) 2026
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
#include <tesseract/kinematics/rtp_factory.h>
#include <tesseract/kinematics/rtp_inv_kin.h>
#include <tesseract/kinematics/forward_kinematics.h>
#include <tesseract/kinematics/factory_utils.h>
#include <tesseract/scene_graph/graph.h>
#include <tesseract/scene_graph/joint.h>

#include <tesseract/common/property_tree.h>
#include <tesseract/common/schema_registration.h>
#include <tesseract/common/yaml_extensions.h>

namespace
{
tesseract::common::PropertyTree rtpInvKinFactorySchema()
{
  using namespace tesseract::common;

  static const std::string kItemType = "tesseract::kinematics::PositionerSampleResolution";

  // manipulator_reach is deliberately not required: when absent it is derived from the manipulator
  // chain's reach upper bound, which is what RTPInvKin's shorter constructor does.
  // clang-format off
  return PropertyTreeBuilder()
    .attribute(property_attribute::TYPE, property_type::CONTAINER)
    .float64("manipulator_reach").done()
    .customType("tool_sample_resolution",
          property_type::createList(kItemType)).required().done()
    .customType("tool_positioner", "tesseract::kinematics::FwdKinFactory")
      .required().acceptsDerivedTypes().done()
    .customType("manipulator", "tesseract::kinematics::InvKinFactory")
      .required().acceptsDerivedTypes().done()
    .build();
  // clang-format on
}
}  // namespace

namespace tesseract::kinematics
{
tesseract::common::PropertyTree RTPInvKinFactory::schema() const { return rtpInvKinFactorySchema(); }

std::unique_ptr<InverseKinematics> RTPInvKinFactory::createImpl(const std::string& solver_name,
                                                                const tesseract::scene_graph::SceneGraph& scene_graph,
                                                                const tesseract::scene_graph::SceneState& scene_state,
                                                                const KinematicsPluginFactory& plugin_factory,
                                                                const tesseract::common::PropertyTree& config) const
{
  const auto sample_res_map =
      parseSampleResolutionMap(config.at("tool_sample_resolution").getValue(), scene_graph, "tool_sample_resolution");

  const auto p_info = config.at("tool_positioner").as<tesseract::common::PluginInfo>();
  ForwardKinematics::UPtr fwd_kin = plugin_factory.createFwdKin(p_info.class_name, p_info, scene_graph, scene_state);
  if (fwd_kin == nullptr)
    throw std::runtime_error("RTPInvKinFactory, failed to create tool forward kinematics!");

  const SampleGridConfig grid = toSampleGridConfig(sample_res_map, fwd_kin->getJointIds(), "tool_sample_resolution");

  const auto m_info = config.at("manipulator").as<tesseract::common::PluginInfo>();
  InverseKinematics::UPtr inv_kin = plugin_factory.createInvKin(m_info.class_name, m_info, scene_graph, scene_state);
  if (inv_kin == nullptr)
    throw std::runtime_error("RTPInvKinFactory, failed to create manipulator inverse kinematics!");

  // An absent manipulator_reach selects the constructor that derives the reach from the
  // manipulator chain instead.
  if (const auto* reach = config.find("manipulator_reach"); reach != nullptr && !reach->isNull())
  {
    return std::make_unique<RTPInvKin>(scene_graph,
                                       scene_state,
                                       std::move(inv_kin),
                                       reach->as<double>(),
                                       std::move(fwd_kin),
                                       grid.range,
                                       grid.resolution,
                                       solver_name);
  }
  return std::make_unique<RTPInvKin>(
      scene_graph, scene_state, std::move(inv_kin), std::move(fwd_kin), grid.range, grid.resolution, solver_name);
}

PLUGIN_ANCHOR_IMPL(RTPInvKinFactoriesAnchor)

}  // namespace tesseract::kinematics

// NOLINTNEXTLINE(cppcoreguidelines-avoid-non-const-global-variables)
TESSERACT_ADD_INV_KIN_PLUGIN(tesseract::kinematics::RTPInvKinFactory, RTPInvKinFactory);
TESSERACT_SCHEMA_REGISTER(RTPInvKinFactory, rtpInvKinFactorySchema);
TESSERACT_SCHEMA_REGISTER_DERIVED_TYPE(tesseract::kinematics::InvKinFactory, RTPInvKinFactory);
