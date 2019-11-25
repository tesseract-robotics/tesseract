/**
 * @file forward_kinematics_factory.h
 * @brief Forward kinematics Abstract factory.
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
#ifndef TESSERACT_KINEMATICS_FORWARD_KINEMATICS_FACTORY_H
#define TESSERACT_KINEMATICS_FORWARD_KINEMATICS_FACTORY_H

#include <tesseract_kinematics/core/forward_kinematics.h>
#include <tesseract_scene_graph/graph.h>

namespace tesseract_kinematics
{
enum class ForwardKinematicsFactoryType
{
  CHAIN = 0,
  TREE = 1,
  GRAPH = 2
};

class ForwardKinematicsFactory
{
public:
  using Ptr = std::shared_ptr<ForwardKinematicsFactory>;
  using ConstPtr = std::shared_ptr<const ForwardKinematicsFactory>;

  ForwardKinematicsFactory() = default;
  virtual ~ForwardKinematicsFactory() = default;
  ForwardKinematicsFactory(const ForwardKinematicsFactory&) = default;
  ForwardKinematicsFactory& operator=(const ForwardKinematicsFactory&) = default;
  ForwardKinematicsFactory(ForwardKinematicsFactory&&) = default;
  ForwardKinematicsFactory& operator=(ForwardKinematicsFactory&&) = default;

  /**
   * @brief Get the name of the factory
   * @return The name
   */
  virtual const std::string& getName() const = 0;

  /**
   * @brief Get the type of Forward Kinematic factory
   * @return Type {CHAIN, TREE, GRAPH}
   */
  virtual ForwardKinematicsFactoryType getType() const = 0;

  /**
   * @brief Create Forward Kinematics Chain Object
   * This only need to be implemented if of type chain
   * @param scene_graph The Tesseract Scene Graph
   * @param base_link The name of the base link for the kinematic chain
   * @param tip_link The name of the tip link for the kinematic chain
   * @param name The name of the kinematic chain
   * @return True if init() completes successfully
   */
  virtual ForwardKinematics::Ptr create(tesseract_scene_graph::SceneGraph::ConstPtr /*scene_graph*/,  // NOLINT
                                        const std::string& /*base_link*/,                             // NOLINT
                                        const std::string& /*tip_link*/,                              // NOLINT
                                        const std::string /*name*/) const                             // NOLINT
  {
    return nullptr;
  }

  /**
   * @brief Create Forward Kinematics Tree Object
   * This only need to be implemented if of type chain
   * @param scene_graph The tesseract scene graph
   * @param joint_names The list of active joints to be considered
   * @param name The name of the kinematic chain
   * @param start_state The initial start state for the tree. This should inlclude all joints in the scene graph
   * @return True if init() completes successfully
   */
  virtual ForwardKinematics::Ptr create(tesseract_scene_graph::SceneGraph::ConstPtr /*scene_graph*/,  // NOLINT
                                        const std::vector<std::string>& /*joint_names*/,              // NOLINT
                                        const std::string /*name*/,                                   // NOLINT
                                        std::unordered_map<std::string, double> /*start_state*/ =
                                            std::unordered_map<std::string, double>()) const  // NOLINT
  {
    return nullptr;
  }
};

}  // namespace tesseract_kinematics
#endif  // TESSERACT_KINEMATICS_FORWARD_KINEMATICS_FACTORY_H
