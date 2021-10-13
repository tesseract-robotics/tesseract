/**
 * @file inverse_kinematics_factory.h
 * @brief Inverse kinematics Abstract factory.
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
#ifndef TESSERACT_KINEMATICS_INVERSE_KINEMATICS_FACTORY_H
#define TESSERACT_KINEMATICS_INVERSE_KINEMATICS_FACTORY_H

#include <tesseract_kinematics/core/inverse_kinematics.h>
#include <tesseract_scene_graph/graph.h>
#include <tesseract_scene_graph/scene_state.h>

#ifdef SWIG
%shared_ptr(tesseract_kinematics::InverseKinematicsFactory)
#endif  // SWIG

namespace tesseract_kinematics
{
enum class InverseKinematicsFactoryType
{
  CHAIN = 0,
  TREE = 1,
  GRAPH = 2
};

class InverseKinematicsFactory
{
public:
  using Ptr = std::shared_ptr<InverseKinematicsFactory>;
  using ConstPtr = std::shared_ptr<const InverseKinematicsFactory>;

  InverseKinematicsFactory() = default;
  virtual ~InverseKinematicsFactory() = default;
  InverseKinematicsFactory(const InverseKinematicsFactory&) = default;
  InverseKinematicsFactory& operator=(const InverseKinematicsFactory&) = default;
  InverseKinematicsFactory(InverseKinematicsFactory&&) = default;
  InverseKinematicsFactory& operator=(InverseKinematicsFactory&&) = default;

  /**
   * @brief Get the name of the factory
   * @return The name
   */
  virtual const std::string& getName() const = 0;

  /**
   * @brief Get the type of Inverse Kinematic factory
   * @return Type {CHAIN, TREE, GRAPH}
   */
  virtual InverseKinematicsFactoryType getType() const = 0;

  /**
   * @brief Create Inverse Kinematics Chain Object
   * This only need to be implemented if of type chain
   * @param name The name of the kinematic chain
   * @param scene_graph The Tesseract Scene Graph
   * @param scene_state The state of the scene graph
   * @param base_link The name of the base link for the kinematic chain
   * @param tip_link The name of the tip link for the kinematic chain
   * @return If failed to create, nullptr is returned.
   */
  virtual InverseKinematics::UPtr create(const std::string& /*name*/,                               // NOLINT
                                         const tesseract_scene_graph::SceneGraph& /*scene_graph*/,  // NOLINT
                                         const tesseract_scene_graph::SceneState& /*scene_state*/,  // NOLINT
                                         const std::string& /*base_link*/,                          // NOLINT
                                         const std::string& /*tip_link*/) const                     // NOLINT
  {
    return nullptr;
  }

  /**
   * @brief Create Inverse Kinematics Chain Object
   * This only need to be implemented if of type chain
   * @param name The name of the kinematic chain
   * @param scene_graph The Tesseract Scene Graph
   * @param scene_state The state of the scene graph
   * @param chains The chains that make up the kinematic chain
   * @return If failed to create, nullptr is returned.
   */
  virtual InverseKinematics::UPtr
  create(const std::string& /*name*/,                                               // NOLINT
         const tesseract_scene_graph::SceneGraph& /*scene_graph*/,                  // NOLINT
         const tesseract_scene_graph::SceneState& /*scene_state*/,                  // NOLINT
         const std::vector<std::pair<std::string, std::string>>& /*chains*/) const  // NOLINT
  {
    return nullptr;
  }

  /**
   * @brief Create Inverse Kinematics Tree Object
   * This only need to be implemented if of type chain
   * @param name The name of the kinematic chain
   * @param scene_graph The tesseract scene graph
   * @param scene_state The state of the scene graph
   * @param joint_names The list of active joints to be considered
   * @return If failed to create, nullptr is returned.
   */
  virtual InverseKinematics::UPtr create(const std::string& /*name*/,                               // NOLINT
                                         const tesseract_scene_graph::SceneGraph& /*scene_graph*/,  // NOLINT
                                         const tesseract_scene_graph::SceneState& /*scene_state*/,  // NOLINT
                                         const std::vector<std::string>& /*joint_names*/) const     // NOLINT
  {
    return nullptr;
  }
};

}  // namespace tesseract_kinematics

#endif  // TESSERACT_KINEMATICS_INVERSE_KINEMATICS_FACTORY_H
