/**
 * @file forward_kinematics_tree_factory.h
 * @brief Forward kinematics Tree Abstract factory.
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
#ifndef TESSERACT_KINEMATICS_FORWARD_KINEMATICS_TREE_FACTORY_H
#define TESSERACT_KINEMATICS_FORWARD_KINEMATICS_TREE_FACTORY_H
#include <tesseract_kinematics/core/forward_kinematics.h>
#include <tesseract_scene_graph/graph.h>
#include <tesseract_scene_graph/scene_state.h>

#ifdef SWIG
%shared_ptr(tesseract_kinematics::FwdKinTreeFactory)
#endif  // SWIG

namespace tesseract_kinematics
{
class FwdKinTreeFactory
{
public:
  using Ptr = std::shared_ptr<FwdKinTreeFactory>;
  using ConstPtr = std::shared_ptr<const FwdKinTreeFactory>;
  using UPtr = std::unique_ptr<FwdKinTreeFactory>;
  using ConstUPtr = std::unique_ptr<const FwdKinTreeFactory>;

  FwdKinTreeFactory() = default;
  virtual ~FwdKinTreeFactory() = default;
  FwdKinTreeFactory(const FwdKinTreeFactory&) = default;
  FwdKinTreeFactory& operator=(const FwdKinTreeFactory&) = default;
  FwdKinTreeFactory(FwdKinTreeFactory&&) = default;
  FwdKinTreeFactory& operator=(FwdKinTreeFactory&&) = default;

  /**
   * @brief Get the name of the factory
   * @return The name
   */
  virtual const std::string& getName() const = 0;

  /**
   * @brief Create Forward Kinematics Tree Object
   * @param name The name of the kinematic chain
   * @param scene_graph The Tesseract Scene Graph
   * @param scene_state The state of the scene graph
   * @param joint_names The joint names to construct the tree from
   * @return If failed to create, nullptr is returned.
   */
  virtual ForwardKinematics::UPtr create(const std::string& name,
                                         const tesseract_scene_graph::SceneGraph& scene_graph,
                                         const tesseract_scene_graph::SceneState& scene_state,
                                         const std::vector<std::string>& joint_names) const = 0;
};

}  // namespace tesseract_kinematics
#endif  // TESSERACT_KINEMATICS_FORWARD_KINEMATICS_TREE_FACTORY_H
