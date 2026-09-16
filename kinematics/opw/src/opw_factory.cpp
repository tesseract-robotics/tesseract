/**
 * @file opw_factory.h
 * @brief Tesseract OPW Inverse kinematics Factory
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

#include <tesseract/kinematics/opw/opw_factory.h>
#include <tesseract/kinematics/opw/opw_inv_kin.h>

#include <tesseract/scene_graph/graph.h>
#include <tesseract/scene_graph/scene_state.h>

#include <tesseract/common/schema_registration.h>
#include <tesseract/common/property_tree.h>

#include <console_bridge/console.h>

namespace
{
void validateSignCorrections(const tesseract::common::PropertyTree& node,
                             const std::string& path,
                             std::vector<std::string>& errors)
{
  if (!node.getValue().IsSequence())
    return;

  for (std::size_t index = 0; index < node.getValue().size(); ++index)
  {
    try
    {
      const int value = node.getValue()[index].as<int>();
      if (value != -1 && value != 1)
        errors.push_back(path + "[" + std::to_string(index) + "]: value must be -1 or 1");
    }
    catch (const std::exception& e)
    {
      errors.push_back(path + "[" + std::to_string(index) + "]: " + e.what());
    }
  }
}

tesseract::common::PropertyTree opwInvKinFactorySchema()
{
  using namespace tesseract::common;
  // clang-format off
  return PropertyTreeBuilder()
      .attribute(property_attribute::TYPE, property_type::CONTAINER)
      .string("base_link").required().minimumLength(1).done()
      .string("tip_link").required().minimumLength(1).done()
      .container("params").required()
          .float64("a1").required().done()
          .float64("a2").required().done()
          .float64("b").required().done()
          .float64("c1").required().done()
          .float64("c2").required().done()
          .float64("c3").required().done()
          .float64("c4").required().done()
          .customType("offsets", property_type::createList(property_type::FLOAT64, 6)).done()
          .customType("sign_corrections", property_type::createList(property_type::INT32, 6))
            .validator(validateSignCorrections).done()
      .done()
      .build();
  // clang-format on
}
}  // namespace

namespace tesseract::kinematics
{
tesseract::common::PropertyTree OPWInvKinFactory::schema() const { return opwInvKinFactorySchema(); }

std::unique_ptr<InverseKinematics>
OPWInvKinFactory::createImpl(const std::string& solver_name,
                             const tesseract::scene_graph::SceneGraph& scene_graph,
                             const tesseract::scene_graph::SceneState& /*scene_state*/,
                             const KinematicsPluginFactory& /*plugin_factory*/,
                             const tesseract::common::PropertyTree& config) const
{
  const common::LinkId base_link(config.at("base_link").as<std::string>());
  const common::LinkId tip_link(config.at("tip_link").as<std::string>());
  opw_kinematics::Parameters<double> params;
  const auto& opw_params = config.at("params");
  params.a1 = opw_params.at("a1").as<double>();
  params.a2 = opw_params.at("a2").as<double>();
  params.b = opw_params.at("b").as<double>();
  params.c1 = opw_params.at("c1").as<double>();
  params.c2 = opw_params.at("c2").as<double>();
  params.c3 = opw_params.at("c3").as<double>();
  params.c4 = opw_params.at("c4").as<double>();

  if (const auto* value = opw_params.find("offsets"); value != nullptr && !value->isNull())
  {
    const auto offsets = value->as<std::vector<double>>();
    std::copy(offsets.begin(), offsets.end(), params.offsets.begin());
  }
  if (const auto* value = opw_params.find("sign_corrections"); value != nullptr && !value->isNull())
  {
    const auto sign_corrections = value->as<std::vector<int>>();
    std::copy(sign_corrections.begin(), sign_corrections.end(), params.sign_corrections.begin());
  }

  const auto path = scene_graph.getShortestPath(base_link, tip_link);
  return std::make_unique<OPWInvKin>(params, base_link, tip_link, path.active_joints, solver_name);
}

PLUGIN_ANCHOR_IMPL(OPWFactoriesAnchor)

}  // namespace tesseract::kinematics

// NOLINTNEXTLINE(cppcoreguidelines-avoid-non-const-global-variables)
TESSERACT_ADD_INV_KIN_PLUGIN(tesseract::kinematics::OPWInvKinFactory, OPWInvKinFactory);
TESSERACT_SCHEMA_REGISTER(OPWInvKinFactory, opwInvKinFactorySchema);
TESSERACT_SCHEMA_REGISTER_DERIVED_TYPE(tesseract::kinematics::InvKinFactory, OPWInvKinFactory);
