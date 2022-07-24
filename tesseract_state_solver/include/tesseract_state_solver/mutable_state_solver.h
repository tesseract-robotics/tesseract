/**
 * @file mutable_state_solver.h
 * @brief Tesseract Scene Graph Mutable State Solver Interface .
 *
 * @author Levi Armstrong
 * @date Dec 18, 2017
 * @version TODO
 * @bug No known bugs
 *
 * @copyright Copyright (c) 2017, Southwest Research Institute
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
#ifndef TESSERACT_STATE_SOLVER_MUTABLE_STATE_SOLVER_H
#define TESSERACT_STATE_SOLVER_MUTABLE_STATE_SOLVER_H

#include <tesseract_state_solver/state_solver.h>

namespace tesseract_scene_graph
{
/**
 * @brief A mutable state solver allows you to reconfigure the solver's links and joints
 */
class MutableStateSolver : public StateSolver
{
public:
  using Ptr = std::shared_ptr<MutableStateSolver>;
  using ConstPtr = std::shared_ptr<const MutableStateSolver>;
  using UPtr = std::unique_ptr<MutableStateSolver>;
  using ConstUPtr = std::unique_ptr<const MutableStateSolver>;

  MutableStateSolver() = default;
  ~MutableStateSolver() override = default;
  MutableStateSolver(const MutableStateSolver&) = default;
  MutableStateSolver& operator=(const MutableStateSolver&) = default;
  MutableStateSolver(MutableStateSolver&&) = default;
  MutableStateSolver& operator=(MutableStateSolver&&) = default;

  /**
   * @brief Set the state solver revision number
   * @param revision The revision number to assign
   */
  virtual void setRevision(int revision) = 0;

  /**
   * @brief Get the state solver revision number
   * @return revision number
   */
  virtual int getRevision() const = 0;

  /**
   * @brief Adds a link/joint to the solver
   * @param link The link to be added to the graph
   * @param joint The associated joint to be added to the graph
   * @return Return False if a link with the same name allready exists, otherwise true
   */
  virtual bool addLink(const Link& link, const Joint& joint) = 0;

  /**
   * @brief Move a link
   * @param joint The associated joint that defines the move
   * @return Return False if link does not exist or if joint name already exists, otherwise true
   */
  virtual bool moveLink(const Joint& joint) = 0;

  /**
   * @brief Removes a link from the graph
   *
   * Note: this will remove all inbound and outbound edges
   *
   * @param name Name of the link to be removed
   * @return Return False if a link does not exists, otherwise true
   */
  virtual bool removeLink(const std::string& name) = 0;

  /**
   * @brief Replace and existing joint with the provided one
   * @param joint The replacement joint
   * @return Return False if a joint does not exists, otherwise true
   */
  virtual bool replaceJoint(const Joint& joint) = 0;

  /**
   * @brief Removes a joint from the graph
   * @param name Name of the joint to be removed
   * @return Return False if a joint does not exists, otherwise true
   */
  virtual bool removeJoint(const std::string& name) = 0;

  /**
   * @brief Move joint to new parent link
   * @param name Name of the joint to move
   * @param parent_link Name of parent link to move to
   * @return Returns true if successful, otherwise false.
   */
  virtual bool moveJoint(const std::string& name, const std::string& parent_link) = 0;

  /**
   * @brief Changes the "origin" transform of the joint and recomputes the associated edge
   * @param name Name of the joint to be changed
   * @param new_origin The new transform associated with the joint
   * @return True if successful.
   */
  virtual bool changeJointOrigin(const std::string& name, const Eigen::Isometry3d& new_origin) = 0;

  /**
   * @brief Changes the position limits associated with a joint
   * @param joint_name Name of the joint to be updated
   * @param limits New position limits to be set as the joint limits
   * @returnTrue if successful.
   */
  virtual bool changeJointPositionLimits(const std::string& name, double lower, double upper) = 0;

  /**
   * @brief Changes the velocity limits associated with a joint
   * @param joint_name Name of the joint to be updated
   * @param limits New velocity limits to be set as the joint limits
   * @return
   */
  virtual bool changeJointVelocityLimits(const std::string& name, double limit) = 0;

  /**
   * @brief Changes the acceleration limits associated with a joint
   * @param joint_name Name of the joint to be updated
   * @param limits New acceleration limits to be set as the joint limits
   * @return
   */
  virtual bool changeJointAccelerationLimits(const std::string& name, double limit) = 0;

  /**
   * @brief Merge a scene into the current solver
   * @param scene_graph Const ref to the graph to be merged
   * @param joint The joint that connects current scene with the inserted scene
   * @param prefix string Will be prepended to every link and joint of the merged scene
   * @return Return False if any link or joint name collides with current solver, otherwise True
   * The prefix argument is meant to allow adding multiple copies of the same subgraph with different names
   */
  virtual bool insertSceneGraph(const SceneGraph& scene_graph, const Joint& joint, const std::string& prefix = "") = 0;
};
}  // namespace tesseract_scene_graph

#endif  // TESSERACT_STATE_SOLVER_MUTABLE_STATE_SOLVER_H
