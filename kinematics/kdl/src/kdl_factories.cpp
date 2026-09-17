/**
 * @file kdl_factories.h
 * @brief Tesseract KDL Factories.
 *
 * @author Levi Armstrong
 * @date Aug 27, 2021
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

#include <tesseract/kinematics/kdl/kdl_factories.h>
#include <tesseract/kinematics/kdl/kdl_fwd_kin_chain.h>
#include <tesseract/kinematics/kdl/kdl_inv_kin_chain_lma.h>
#include <tesseract/kinematics/kdl/kdl_inv_kin_chain_nr.h>
#include <tesseract/kinematics/kdl/kdl_inv_kin_chain_nr_jl.h>

#include <tesseract/scene_graph/graph.h>
#include <tesseract/scene_graph/scene_state.h>

#include <tesseract/common/schema_registration.h>
#include <tesseract/common/property_tree.h>

#include <console_bridge/console.h>

namespace
{
tesseract::common::PropertyTree kdlFwdKinChainFactorySchema()
{
  using namespace tesseract::common;
  // clang-format off
  return PropertyTreeBuilder()
      .attribute(property_attribute::TYPE, property_type::CONTAINER)
      .string("base_link").required().minimumLength(1).done()
      .string("tip_link").required().minimumLength(1).done()
      .build();
  // clang-format on
}

tesseract::common::PropertyTree kdlInvKinChainLMAFactorySchema()
{
  using namespace tesseract::common;
  // clang-format off
  return PropertyTreeBuilder()
      .attribute(property_attribute::TYPE, property_type::CONTAINER)
      .string("base_link").required().minimumLength(1).done()
      .string("tip_link").required().minimumLength(1).done()
      .customType("task_weights", property_type::createList(property_type::FLOAT64, 6)).done()
      .float64("eps").done()
      .int32("max_iterations").done()
      .float64("eps_joints").done()
      .build();
  // clang-format on
}

tesseract::common::PropertyTree kdlInvKinChainNRFactorySchema()
{
  using namespace tesseract::common;
  // clang-format off
  return PropertyTreeBuilder()
      .attribute(property_attribute::TYPE, property_type::CONTAINER)
      .string("base_link").required().minimumLength(1).done()
      .string("tip_link").required().minimumLength(1).done()
      .float64("velocity_eps").done()
      .int32("velocity_iterations").done()
      .float64("position_eps").done()
      .int32("position_iterations").done()
      .build();
  // clang-format on
}
}  // namespace

namespace tesseract::kinematics
{
tesseract::common::PropertyTree KDLFwdKinChainFactory::schema() const { return kdlFwdKinChainFactorySchema(); }

tesseract::common::PropertyTree KDLInvKinChainLMAFactory::schema() const { return kdlInvKinChainLMAFactorySchema(); }

tesseract::common::PropertyTree KDLInvKinChainNRFactory::schema() const { return kdlInvKinChainNRFactorySchema(); }

tesseract::common::PropertyTree KDLInvKinChainNR_JLFactory::schema() const { return kdlInvKinChainNRFactorySchema(); }

std::unique_ptr<ForwardKinematics>
KDLFwdKinChainFactory::createImpl(const std::string& solver_name,
                                  const tesseract::scene_graph::SceneGraph& scene_graph,
                                  const tesseract::scene_graph::SceneState& /*scene_state*/,
                                  const KinematicsPluginFactory& /*plugin_factory*/,
                                  const tesseract::common::PropertyTree& config) const
{
  const common::LinkId base_link(config.at("base_link").as<std::string>());
  const common::LinkId tip_link(config.at("tip_link").as<std::string>());
  return std::make_unique<KDLFwdKinChain>(scene_graph, base_link, tip_link, solver_name);
}

std::unique_ptr<InverseKinematics>
KDLInvKinChainLMAFactory::createImpl(const std::string& solver_name,
                                     const tesseract::scene_graph::SceneGraph& scene_graph,
                                     const tesseract::scene_graph::SceneState& /*scene_state*/,
                                     const KinematicsPluginFactory& /*plugin_factory*/,
                                     const tesseract::common::PropertyTree& config) const
{
  const common::LinkId base_link(config.at("base_link").as<std::string>());
  const common::LinkId tip_link(config.at("tip_link").as<std::string>());
  KDLInvKinChainLMA::Config kdl_config;

  if (const auto* value = config.find("task_weights"); value != nullptr && !value->isNull())
    kdl_config.task_weights = value->as<std::array<double, 6>>();
  if (const auto* value = config.find("eps"); value != nullptr && !value->isNull())
    kdl_config.eps = value->as<double>();
  if (const auto* value = config.find("max_iterations"); value != nullptr && !value->isNull())
    kdl_config.max_iterations = value->as<int>();
  if (const auto* value = config.find("eps_joints"); value != nullptr && !value->isNull())
    kdl_config.eps_joints = value->as<double>();

  return std::make_unique<KDLInvKinChainLMA>(scene_graph, base_link, tip_link, kdl_config, solver_name);
}

std::unique_ptr<InverseKinematics>
KDLInvKinChainNRFactory::createImpl(const std::string& solver_name,
                                    const tesseract::scene_graph::SceneGraph& scene_graph,
                                    const tesseract::scene_graph::SceneState& /*scene_state*/,
                                    const KinematicsPluginFactory& /*plugin_factory*/,
                                    const tesseract::common::PropertyTree& config) const
{
  const common::LinkId base_link(config.at("base_link").as<std::string>());
  const common::LinkId tip_link(config.at("tip_link").as<std::string>());
  KDLInvKinChainNR::Config kdl_config;

  if (const auto* value = config.find("velocity_eps"); value != nullptr && !value->isNull())
    kdl_config.vel_eps = value->as<double>();
  if (const auto* value = config.find("velocity_iterations"); value != nullptr && !value->isNull())
    kdl_config.vel_iterations = value->as<int>();
  if (const auto* value = config.find("position_eps"); value != nullptr && !value->isNull())
    kdl_config.pos_eps = value->as<double>();
  if (const auto* value = config.find("position_iterations"); value != nullptr && !value->isNull())
    kdl_config.pos_iterations = value->as<int>();

  return std::make_unique<KDLInvKinChainNR>(scene_graph, base_link, tip_link, kdl_config, solver_name);
}

std::unique_ptr<InverseKinematics>
KDLInvKinChainNR_JLFactory::createImpl(const std::string& solver_name,
                                       const tesseract::scene_graph::SceneGraph& scene_graph,
                                       const tesseract::scene_graph::SceneState& /*scene_state*/,
                                       const KinematicsPluginFactory& /*plugin_factory*/,
                                       const tesseract::common::PropertyTree& config) const
{
  const common::LinkId base_link(config.at("base_link").as<std::string>());
  const common::LinkId tip_link(config.at("tip_link").as<std::string>());
  KDLInvKinChainNR_JL::Config kdl_config;

  if (const auto* value = config.find("velocity_eps"); value != nullptr && !value->isNull())
    kdl_config.vel_eps = value->as<double>();
  if (const auto* value = config.find("velocity_iterations"); value != nullptr && !value->isNull())
    kdl_config.vel_iterations = value->as<int>();
  if (const auto* value = config.find("position_eps"); value != nullptr && !value->isNull())
    kdl_config.pos_eps = value->as<double>();
  if (const auto* value = config.find("position_iterations"); value != nullptr && !value->isNull())
    kdl_config.pos_iterations = value->as<int>();

  return std::make_unique<KDLInvKinChainNR_JL>(scene_graph, base_link, tip_link, kdl_config, solver_name);
}

PLUGIN_ANCHOR_IMPL(KDLFactoriesAnchor)

}  // namespace tesseract::kinematics

// NOLINTNEXTLINE(cppcoreguidelines-avoid-non-const-global-variables)
TESSERACT_ADD_FWD_KIN_PLUGIN(tesseract::kinematics::KDLFwdKinChainFactory, KDLFwdKinChainFactory);
// NOLINTNEXTLINE(cppcoreguidelines-avoid-non-const-global-variables)
TESSERACT_ADD_INV_KIN_PLUGIN(tesseract::kinematics::KDLInvKinChainLMAFactory, KDLInvKinChainLMAFactory);
// NOLINTNEXTLINE(cppcoreguidelines-avoid-non-const-global-variables)
TESSERACT_ADD_INV_KIN_PLUGIN(tesseract::kinematics::KDLInvKinChainNRFactory, KDLInvKinChainNRFactory);
// NOLINTNEXTLINE(cppcoreguidelines-avoid-non-const-global-variables)
TESSERACT_ADD_INV_KIN_PLUGIN(tesseract::kinematics::KDLInvKinChainNR_JLFactory, KDLInvKinChainNR_JLFactory);

TESSERACT_SCHEMA_REGISTER(KDLFwdKinChainFactory, kdlFwdKinChainFactorySchema);
TESSERACT_SCHEMA_REGISTER(KDLInvKinChainLMAFactory, kdlInvKinChainLMAFactorySchema);
TESSERACT_SCHEMA_REGISTER(KDLInvKinChainNRFactory, kdlInvKinChainNRFactorySchema);
TESSERACT_SCHEMA_REGISTER(KDLInvKinChainNR_JLFactory, kdlInvKinChainNRFactorySchema);

TESSERACT_SCHEMA_REGISTER_DERIVED_TYPE(tesseract::kinematics::FwdKinFactory, KDLFwdKinChainFactory);
TESSERACT_SCHEMA_REGISTER_DERIVED_TYPE(tesseract::kinematics::InvKinFactory, KDLInvKinChainLMAFactory);
TESSERACT_SCHEMA_REGISTER_DERIVED_TYPE(tesseract::kinematics::InvKinFactory, KDLInvKinChainNRFactory);
TESSERACT_SCHEMA_REGISTER_DERIVED_TYPE(tesseract::kinematics::InvKinFactory, KDLInvKinChainNR_JLFactory);
