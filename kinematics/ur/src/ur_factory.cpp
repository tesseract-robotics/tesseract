/**
 * @file ur_factory.cpp
 * @brief Tesseract UR Inverse kinematics Factory
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

#include <tesseract/common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <string>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract/kinematics/ur/ur_factory.h>
#include <tesseract/kinematics/ur/ur_inv_kin.h>
#include <tesseract/kinematics/types.h>

#include <tesseract/scene_graph/graph.h>
#include <tesseract/scene_graph/scene_state.h>

#include <tesseract/common/schema_registration.h>
#include <tesseract/common/property_tree.h>

#include <tesseract/common/logging.h>

namespace
{
tesseract::common::PropertyTree urInvKinFactorySchema()
{
  using namespace tesseract::common;
  // clang-format off
  return PropertyTreeBuilder()
    .attribute(property_attribute::TYPE, property_type::CONTAINER)
    .string("base_link").required().minimumLength(1).done()
    .string("tip_link").required().minimumLength(1).done()
    .beginOneOf()
      .container("by_model")
        .string("model").required().minimumLength(1)
          .enumValues({"UR3", "UR5", "UR10", "UR3e", "UR5e", "UR10e"}).done()
      .done()
      .container("by_params")
        .container("params").required()
          .float64("d1").required().done()
          .float64("a2").required().done()
          .float64("a3").required().done()
          .float64("d4").required().done()
          .float64("d5").required().done()
          .float64("d6").required().done()
        .done()
      .done()
    .endOneOf()
    .build();
  // clang-format on
}
}  // namespace

namespace tesseract::kinematics
{
tesseract::common::PropertyTree URInvKinFactory::schema() const { return urInvKinFactorySchema(); }

std::unique_ptr<InverseKinematics>
URInvKinFactory::createImpl(const std::string& solver_name,
                            const tesseract::scene_graph::SceneGraph& scene_graph,
                            const tesseract::scene_graph::SceneState& /*scene_state*/,
                            const KinematicsPluginFactory& /*plugin_factory*/,
                            const tesseract::common::PropertyTree& config) const
{
  const common::LinkId base_link(config.at("base_link").as<std::string>());
  const common::LinkId tip_link(config.at("tip_link").as<std::string>());
  tesseract::kinematics::URParameters params;

  if (const auto* model = config.find("model"); model != nullptr && !model->isNull())
  {
    const auto model_name = model->as<std::string>();
    if (model_name == "UR3")
      params = UR3Parameters;
    else if (model_name == "UR5")
      params = UR5Parameters;
    else if (model_name == "UR10")
      params = UR10Parameters;
    else if (model_name == "UR3e")
      params = UR3eParameters;
    else if (model_name == "UR5e")
      params = UR5eParameters;
    else
      params = UR10eParameters;
  }
  else
  {
    const auto& ur_params = config.at("params");
    params.d1 = ur_params.at("d1").as<double>();
    params.a2 = ur_params.at("a2").as<double>();
    params.a3 = ur_params.at("a3").as<double>();
    params.d4 = ur_params.at("d4").as<double>();
    params.d5 = ur_params.at("d5").as<double>();
    params.d6 = ur_params.at("d6").as<double>();
  }

  const auto path = scene_graph.getShortestPath(base_link, tip_link);
  return std::make_unique<URInvKin>(params, base_link, tip_link, path.active_joints, solver_name);
}

PLUGIN_ANCHOR_IMPL(URFactoriesAnchor)

}  // namespace tesseract::kinematics

// NOLINTNEXTLINE(cppcoreguidelines-avoid-non-const-global-variables)
TESSERACT_ADD_INV_KIN_PLUGIN(tesseract::kinematics::URInvKinFactory, URInvKinFactory);
TESSERACT_SCHEMA_REGISTER(URInvKinFactory, urInvKinFactorySchema);
TESSERACT_SCHEMA_REGISTER_DERIVED_TYPE(tesseract::kinematics::InvKinFactory, URInvKinFactory);
