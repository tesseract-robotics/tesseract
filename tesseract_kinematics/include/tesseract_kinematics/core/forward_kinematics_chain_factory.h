/**
 * @file forward_kinematics_chain_factory.h
 * @brief Forward kinematics Chain Abstract factory.
 *
 * @author Levi Armstrong
 * @date April 15, 2018
 * @version TODO
 * @bug No known bugs
 *
 * @copyright Copyright (c) 2013, Southwest Research Institute
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
#ifndef TESSERACT_KINEMATICS_FORWARD_KINEMATICS_CHAIN_FACTORY_H
#define TESSERACT_KINEMATICS_FORWARD_KINEMATICS_CHAIN_FACTORY_H

#include <tesseract_kinematics/core/forward_kinematics.h>
#include <tesseract_scene_graph/graph.h>
#include <tesseract_scene_graph/scene_state.h>

#ifdef SWIG
%shared_ptr(tesseract_kinematics::FwdKinChainFactory)
#endif  // SWIG

namespace tesseract_kinematics
{
class FwdKinChainFactory
{
public:
  using Ptr = std::shared_ptr<FwdKinChainFactory>;
  using ConstPtr = std::shared_ptr<const FwdKinChainFactory>;
  using UPtr = std::unique_ptr<FwdKinChainFactory>;
  using ConstUPtr = std::unique_ptr<const FwdKinChainFactory>;

  FwdKinChainFactory() = default;
  virtual ~FwdKinChainFactory() = default;
  FwdKinChainFactory(const FwdKinChainFactory&) = default;
  FwdKinChainFactory& operator=(const FwdKinChainFactory&) = default;
  FwdKinChainFactory(FwdKinChainFactory&&) = default;
  FwdKinChainFactory& operator=(FwdKinChainFactory&&) = default;

  /**
   * @brief Get the name of the factory
   * @return The name
   */
  virtual const std::string& getName() const = 0;

  /**
   * @brief Create Forward Kinematics Chain Object
   * @param name The name of the kinematic chain
   * @param scene_graph The Tesseract Scene Graph
   * @param scene_state The state of the scene graph
   * @param base_link The name of the base link for the kinematic chain
   * @param tip_link The name of the tip link for the kinematic chain
   * @return If failed to create, nullptr is returned.
   */
  virtual ForwardKinematics::UPtr create(const std::string& name,
                                         const tesseract_scene_graph::SceneGraph& scene_graph,
                                         const tesseract_scene_graph::SceneState& scene_state,
                                         const std::string& base_link,
                                         const std::string& tip_link) const = 0;
};

}  // namespace tesseract_kinematics
#endif  // TESSERACT_KINEMATICS_FORWARD_KINEMATICS_CHAIN_FACTORY_H
