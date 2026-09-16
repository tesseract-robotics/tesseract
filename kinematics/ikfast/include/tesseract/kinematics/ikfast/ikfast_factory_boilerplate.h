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

#include <tesseract/common/property_tree.h>
#include <tesseract/kinematics/kinematics_plugin_factory.h>
#include <tesseract/kinematics/ikfast/impl/ikfast_inv_kin.hpp>
#include <tesseract/scene_graph/graph.h>

namespace tesseract::kinematics
{
class IKFastInvKinFactory : public InvKinFactory
{
public:
  tesseract::common::PropertyTree schema() const override
  {
    using namespace tesseract::common;
    // clang-format off
    return PropertyTreeBuilder()
        .attribute(property_attribute::TYPE, property_type::CONTAINER)
        .string("base_link").required().minimumLength(1).done()
        .string("tip_link").required().minimumLength(1).done()
        .uint64("n_joints").required().minimum(1).done()
        .customType("free_joint_states",
                    property_type::createList(property_type::createList(property_type::FLOAT64))).done()
        .build();
    // clang-format on
  }

protected:
  InverseKinematics::UPtr createImpl(const std::string& solver_name,
                                     const tesseract::scene_graph::SceneGraph& scene_graph,
                                     const tesseract::scene_graph::SceneState& /*scene_state*/,
                                     const KinematicsPluginFactory& /*plugin_factory*/,
                                     const tesseract::common::PropertyTree& config) const override
  {
    const common::LinkId base_link(config.at("base_link").as<std::string>());
    const common::LinkId tip_link(config.at("tip_link").as<std::string>());
    const auto n_joints = config.at("n_joints").as<std::size_t>();
    const auto active_joints = scene_graph.getShortestPath(base_link, tip_link).active_joints;
    if (active_joints.size() < n_joints)
      throw std::runtime_error("IKFastInvKinFactory, nominal joint count exceeds the active joint count");

    const std::size_t free_joints_required = active_joints.size() - n_joints;
    std::vector<std::vector<double>> free_joint_states;
    if (const auto* value = config.find("free_joint_states"); value != nullptr && !value->isNull())
    {
      if (free_joints_required == 0)
        throw std::runtime_error("IKFastInvKinFactory, entry 'free_joint_states' exists but no free joints exist");

      free_joint_states = value->as<std::vector<std::vector<double>>>();
      for (const auto& state : free_joint_states)
      {
        if (state.size() != free_joints_required)
        {
          std::stringstream ss;
          ss << "IKFastInvKinFactory, Number of active joints (" << active_joints.size()
             << ") must equal the sum of the number of nominal joints (" << n_joints
             << ") and the number of free joints (" << state.size() << ")";
          throw std::runtime_error(ss.str());
        }
      }
    }
    else if (free_joints_required > 0)
    {
      std::stringstream ss;
      ss << "IKFastInvKinFactory, missing 'free_joint_states' entry, but states for " << free_joints_required
         << " free joints required";
      throw std::runtime_error(ss.str());
    }

    return std::make_unique<IKFastInvKin>(base_link, tip_link, active_joints, solver_name, free_joint_states);
  }
};

}  // namespace tesseract::kinematics

#endif  // TESSERACT_KINEMATICS_IKFAST_FACTORY_BOILERPLATE_H
