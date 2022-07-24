/**
 * @file state_solver.h
 * @brief Tesseract Scene Graph State Solver Interface.
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
#ifndef TESSERACT_STATE_SOLVER_STATE_SOLVER_H
#define TESSERACT_STATE_SOLVER_STATE_SOLVER_H

#include <tesseract_common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <vector>
#include <string>
#include <memory>
#include <unordered_map>
#include <Eigen/Geometry>
#include <Eigen/Core>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract_scene_graph/graph.h>
#include <tesseract_scene_graph/scene_state.h>
#include <tesseract_common/types.h>

namespace tesseract_scene_graph
{
class StateSolver
{
public:
  using Ptr = std::shared_ptr<StateSolver>;
  using ConstPtr = std::shared_ptr<const StateSolver>;
  using UPtr = std::unique_ptr<StateSolver>;
  using ConstUPtr = std::unique_ptr<const StateSolver>;

  StateSolver() = default;
  virtual ~StateSolver() = default;
  StateSolver(const StateSolver&) = default;
  StateSolver& operator=(const StateSolver&) = default;
  StateSolver(StateSolver&&) = default;
  StateSolver& operator=(StateSolver&&) = default;

  /**
   * @brief This should clone the object so it may be used in a multi threaded application where each thread would
   * clone the solver.
   * @return A clone of the object.
   */
  virtual StateSolver::UPtr clone() const = 0;

  /**
   * @brief Set the current state of the solver
   * @details This must be the same size and order as what is returned by getJointNames
   * @param joint_values The joint values
   */
  virtual void setState(const Eigen::Ref<const Eigen::VectorXd>& joint_values) = 0;

  /**
   * @brief Set the current state of the solver
   *
   * After updating the current state these function must call currentStateChanged() which
   * will update the contact managers transforms
   *
   */
  virtual void setState(const std::unordered_map<std::string, double>& joint_values) = 0;
  virtual void setState(const std::vector<std::string>& joint_names,
                        const Eigen::Ref<const Eigen::VectorXd>& joint_values) = 0;

  /**
   * @brief Get the state of the solver given the joint values
   * @details This must be the same size and order as what is returned by getJointNames
   * @param joint_values The joint values
   */
  virtual SceneState getState(const Eigen::Ref<const Eigen::VectorXd>& joint_values) const = 0;

  /**
   * @brief Get the state of the scene for a given set or subset of joint values.
   *
   * This does not change the internal state of the solver.
   *
   * @param joints A map of joint names to joint values to change.
   * @return A the state of the environment
   */
  virtual SceneState getState(const std::unordered_map<std::string, double>& joint_values) const = 0;
  virtual SceneState getState(const std::vector<std::string>& joint_names,
                              const Eigen::Ref<const Eigen::VectorXd>& joint_values) const = 0;

  /**
   * @brief Get the current state of the scene
   * @return The current state
   */
  virtual SceneState getState() const = 0;

  /**
   * @brief Get the jacobian of the solver given the joint values
   * @details This must be the same size and order as what is returned by getJointNames
   * @param joint_values The joint values
   * @param link_name The link name to calculate the jacobian
   */
  virtual Eigen::MatrixXd getJacobian(const Eigen::Ref<const Eigen::VectorXd>& joint_values,
                                      const std::string& link_name) const = 0;

  /**
   * @brief Get the jacobian of the scene for a given set or subset of joint values.
   *
   *    * This does not return the jacobian based on the provided joint names. It is order based
   * on the order returned from getJointNames
   *
   * This does not change the internal state of the solver.
   *
   * @param joints A map of joint names to joint values to change.
   * @param link_name The link name to calculate the jacobian
   * @return A the state of the environment
   */
  virtual Eigen::MatrixXd getJacobian(const std::unordered_map<std::string, double>& joint_values,
                                      const std::string& link_name) const = 0;
  virtual Eigen::MatrixXd getJacobian(const std::vector<std::string>& joint_names,
                                      const Eigen::Ref<const Eigen::VectorXd>& joint_values,
                                      const std::string& link_name) const = 0;

  /**
   * @brief Get the random state of the environment
   * @return Environment state
   */
  virtual SceneState getRandomState() const = 0;

  /**
   * @brief Get the vector of joint names
   * @return A vector of joint names
   */
  virtual std::vector<std::string> getJointNames() const = 0;

  /**
   * @brief Get the vector of joint names which align with the limits
   * @return A vector of joint names
   */
  virtual std::vector<std::string> getActiveJointNames() const = 0;

  /**
   * @brief Get the base link name
   * @return The base link name
   */
  virtual std::string getBaseLinkName() const = 0;

  /**
   * @brief Get the vector of link names
   * @return A vector of link names
   */
  virtual std::vector<std::string> getLinkNames() const = 0;

  /**
   * @brief Get the vector of active link names
   * @return A vector of active link names
   */
  virtual std::vector<std::string> getActiveLinkNames() const = 0;

  /**
   * @brief Get a vector of static link names in the environment
   * @return A vector of static link names
   */
  virtual std::vector<std::string> getStaticLinkNames() const = 0;

  /**
   * @brief Check if link is an active link
   * @param link_name The link name to check
   * @return True if active, otherwise false
   */
  virtual bool isActiveLinkName(const std::string& link_name) const = 0;

  /**
   * @brief Check if link name exists
   * @param link_name The link name to check for
   * @return True if it exists, otherwise false
   */
  virtual bool hasLinkName(const std::string& link_name) const = 0;

  /**
   * @brief Get all of the links transforms
   * @details Order should be the same as getLinkNames()
   * @return Get a vector of transforms for all links.
   */
  virtual tesseract_common::VectorIsometry3d getLinkTransforms() const = 0;

  /**
   * @brief Get the transform corresponding to the link.
   * @return Transform and is identity when no transform is available.
   */
  virtual Eigen::Isometry3d getLinkTransform(const std::string& link_name) const = 0;

  /**
   * @brief Get transform between two links using the current state
   * @param from_link_name The link name the transform should be relative to
   * @param to_link_name The link name to get transform
   * @return The relative transform = inv(Transform(from_link_name)) * Transform(to_link_name)
   */
  virtual Eigen::Isometry3d getRelativeLinkTransform(const std::string& from_link_name,
                                                     const std::string& to_link_name) const = 0;

  /**
   * @brief Getter for kinematic limits
   * @return The kinematic limits
   */
  virtual tesseract_common::KinematicLimits getLimits() const = 0;
};
}  // namespace tesseract_scene_graph

#endif  // TESSERACT_STATE_SOLVER_STATE_SOLVER_H
