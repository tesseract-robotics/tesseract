/**
 * @file state_solver.h
 * @brief Tesseract Scene Graph State Solver Interface.
 *
 * @author Levi Armstrong
 * @date Dec 18, 2017
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

#include <tesseract/common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <vector>
#include <string>
#include <memory>
#include <unordered_map>
#include <Eigen/Geometry>
#include <Eigen/Core>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract/common/fwd.h>
#include <tesseract/common/types.h>
#include <tesseract/scene_graph/fwd.h>
#include <tesseract/scene_graph/scene_state.h>
#include <tesseract/common/eigen_types.h>

namespace tesseract::scene_graph
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
  virtual void setState(const Eigen::Ref<const Eigen::VectorXd>& joint_values,
                        const tesseract::common::JointIdTransformMap& floating_joint_values = {}) = 0;

  /** @brief Set the current state from a string-keyed map (delegates to JointValues overload) */
  virtual void setState(const std::unordered_map<std::string, double>& joint_values,
                        const tesseract::common::JointIdTransformMap& floating_joint_values = {})
  {
    SceneState::JointValues id_map;
    for (const auto& [name, val] : joint_values)
      id_map[tesseract::common::JointId(name)] = val;
    setState(id_map, floating_joint_values);
  }

  /** @brief Set the current state from string joint names + values (delegates to JointId overload) */
  virtual void setState(const std::vector<std::string>& joint_names,
                        const Eigen::Ref<const Eigen::VectorXd>& joint_values,
                        const tesseract::common::JointIdTransformMap& floating_joint_values = {})
  {
    setState(tesseract::common::toIds<tesseract::common::JointId>(joint_names), joint_values, floating_joint_values);
  }

  /**
   * @brief Set the current state of the floating joint values
   * @param floating_joint_values The floating joint values to set
   */
  virtual void setState(const tesseract::common::JointIdTransformMap& floating_joint_values) = 0;

  /**
   * @brief Set the current state using a JointId-keyed map (avoids string-to-ID conversion)
   * @param joint_values A map of JointId to joint values to change
   * @param floating_joint_values The floating joint origin transform
   */
  /** @brief Set the current state using a JointId-keyed map */
  virtual void setState(const SceneState::JointValues& joint_values,
                        const tesseract::common::JointIdTransformMap& floating_joint_values = {}) = 0;

  /** @brief Set the current state using a vector of JointIds */
  virtual void setState(const std::vector<tesseract::common::JointId>& joint_ids,
                        const Eigen::Ref<const Eigen::VectorXd>& joint_values,
                        const tesseract::common::JointIdTransformMap& floating_joint_values = {}) = 0;

  /**
   * @brief Get the state of the solver given the joint values
   * @details This must be the same size and order as what is returned by getJointNames
   * @param joint_values The joint values
   * @param floating_joint_values The floating joint origin transform
   */
  virtual SceneState getState(const Eigen::Ref<const Eigen::VectorXd>& joint_values,
                              const tesseract::common::JointIdTransformMap& floating_joint_values = {}) const = 0;

  /**
   * @brief Get the state of the scene for a given set or subset of joint values.
   *
   * This does not change the internal state of the solver.
   *
   * @param joints A map of joint names to joint values to change.
   * @param floating_joint_values The floating joint origin transform
   * @return A the state of the environment
   */
  /** @brief Get the state from a string-keyed map (delegates to JointValues overload) */
  virtual SceneState getState(const std::unordered_map<std::string, double>& joint_values,
                              const tesseract::common::JointIdTransformMap& floating_joint_values = {}) const
  {
    SceneState::JointValues id_map;
    for (const auto& [name, val] : joint_values)
      id_map[tesseract::common::JointId(name)] = val;
    return getState(id_map, floating_joint_values);
  }

  /** @brief Get the state from string joint names + values (delegates to JointId overload) */
  virtual SceneState getState(const std::vector<std::string>& joint_names,
                              const Eigen::Ref<const Eigen::VectorXd>& joint_values,
                              const tesseract::common::JointIdTransformMap& floating_joint_values = {}) const
  {
    return getState(tesseract::common::toIds<tesseract::common::JointId>(joint_names), joint_values,
                    floating_joint_values);
  }
  /**
   * @brief Get the state given floating joint values
   * @param floating_joint_values The floating joint values to leverage
   * @return A the state of the environment
   */
  virtual SceneState getState(const tesseract::common::JointIdTransformMap& floating_joint_values) const = 0;

  /**
   * @brief Get the state using a JointId-keyed map (avoids string-to-ID conversion)
   * @param joint_values A map of JointId to joint values to change
   * @param floating_joint_values The floating joint origin transform
   * @return The state of the environment
   */
  /** @brief Get the state using a JointId-keyed map */
  virtual SceneState getState(const SceneState::JointValues& joint_values,
                              const tesseract::common::JointIdTransformMap& floating_joint_values = {}) const = 0;

  /** @brief Get the state using a vector of JointIds */
  virtual SceneState getState(const std::vector<tesseract::common::JointId>& joint_ids,
                              const Eigen::Ref<const Eigen::VectorXd>& joint_values,
                              const tesseract::common::JointIdTransformMap& floating_joint_values = {}) const = 0;

  /** @brief Get link transforms using string joint names (delegates to JointId overload) */
  virtual void getLinkTransforms(tesseract::common::LinkIdTransformMap& link_transforms,
                                 const std::vector<std::string>& joint_names,
                                 const Eigen::Ref<const Eigen::VectorXd>& joint_values) const
  {
    getLinkTransforms(link_transforms, tesseract::common::toIds<tesseract::common::JointId>(joint_names), joint_values);
  }

  /**
   * @brief Get link transforms using joint IDs instead of names
   * @param link_transforms The link_transforms to populate with data.
   * @param joint_ids A list of joint IDs to change.
   * @param joint_values The joint values
   */
  virtual void getLinkTransforms(tesseract::common::LinkIdTransformMap& link_transforms,
                                 const std::vector<tesseract::common::JointId>& joint_ids,
                                 const Eigen::Ref<const Eigen::VectorXd>& joint_values) const = 0;

  /**
   * @brief Get the current state of the scene
   * @return The current state
   */
  virtual SceneState getState() const = 0;

  /** @brief Get the jacobian from a string-keyed map (delegates to JointId overload) */
  virtual Eigen::MatrixXd
  getJacobian(const std::unordered_map<std::string, double>& joint_values,
              const std::string& link_name,
              const tesseract::common::JointIdTransformMap& floating_joint_values = {}) const
  {
    std::vector<tesseract::common::JointId> ids;
    Eigen::VectorXd vals(static_cast<Eigen::Index>(joint_values.size()));
    Eigen::Index i = 0;
    for (const auto& [name, val] : joint_values)
    {
      ids.push_back(tesseract::common::JointId(name));
      vals(i++) = val;
    }
    return getJacobian(ids, vals, tesseract::common::LinkId(link_name), floating_joint_values);
  }

  /** @brief Get the jacobian from string joint names (delegates to JointId overload) */
  virtual Eigen::MatrixXd
  getJacobian(const std::vector<std::string>& joint_names,
              const Eigen::Ref<const Eigen::VectorXd>& joint_values,
              const std::string& link_name,
              const tesseract::common::JointIdTransformMap& floating_joint_values = {}) const
  {
    return getJacobian(tesseract::common::toIds<tesseract::common::JointId>(joint_names), joint_values,
                       tesseract::common::LinkId(link_name), floating_joint_values);
  }

  /**
   * @brief Get the jacobian for a link identified by LinkId
   * @details This must be the same size and order as what is returned by getJointNames
   * @param joint_values The joint values
   * @param link_id The link ID to calculate the jacobian for
   * @param floating_joint_values The floating joint origin transform
   */
  virtual Eigen::MatrixXd
  getJacobian(const Eigen::Ref<const Eigen::VectorXd>& joint_values,
              const tesseract::common::LinkId& link_id,
              const tesseract::common::JointIdTransformMap& floating_joint_values = {}) const = 0;

  /**
   * @brief Get the jacobian for a given set or subset of joint IDs.
   * @param joint_ids A list of joint IDs to change.
   * @param joint_values The joint values
   * @param link_id The link ID to calculate the jacobian for
   * @param floating_joint_values The floating joint origin transform
   */
  virtual Eigen::MatrixXd
  getJacobian(const std::vector<tesseract::common::JointId>& joint_ids,
              const Eigen::Ref<const Eigen::VectorXd>& joint_values,
              const tesseract::common::LinkId& link_id,
              const tesseract::common::JointIdTransformMap& floating_joint_values = {}) const = 0;

  /**
   * @brief Get the random state of the environment
   * @return Environment state
   */
  virtual SceneState getRandomState() const = 0;

  /** @brief Get the vector of joint names (delegates to getJointIds) */
  virtual std::vector<std::string> getJointNames() const { return tesseract::common::toNames(getJointIds()); }

  /** @brief Get the vector of joint IDs */
  virtual std::vector<tesseract::common::JointId> getJointIds() const = 0;

  /** @brief Get the vector of floating joint names (delegates to getFloatingJointIds) */
  virtual std::vector<std::string> getFloatingJointNames() const
  {
    return tesseract::common::toNames(getFloatingJointIds());
  }

  /** @brief Get the vector of floating joint IDs */
  virtual std::vector<tesseract::common::JointId> getFloatingJointIds() const = 0;

  /** @brief Get the vector of active joint names (delegates to getActiveJointIds) */
  virtual std::vector<std::string> getActiveJointNames() const
  {
    return tesseract::common::toNames(getActiveJointIds());
  }

  /** @brief Get the vector of active joint IDs which align with the limits */
  virtual std::vector<tesseract::common::JointId> getActiveJointIds() const = 0;

  /** @brief Get the base link ID */
  virtual tesseract::common::LinkId getBaseLinkId() const = 0;

  /** @brief Get the vector of link names (delegates to getLinkIds) */
  virtual std::vector<std::string> getLinkNames() const { return tesseract::common::toNames(getLinkIds()); }

  /** @brief Get the vector of link IDs */
  virtual std::vector<tesseract::common::LinkId> getLinkIds() const = 0;

  /** @brief Get the vector of active link names (delegates to getActiveLinkIds) */
  virtual std::vector<std::string> getActiveLinkNames() const
  {
    return tesseract::common::toNames(getActiveLinkIds());
  }

  /** @brief Get the vector of active link IDs */
  virtual std::vector<tesseract::common::LinkId> getActiveLinkIds() const = 0;

  /** @brief Get a vector of static link names (delegates to getStaticLinkIds) */
  virtual std::vector<std::string> getStaticLinkNames() const
  {
    return tesseract::common::toNames(getStaticLinkIds());
  }

  /** @brief Get a vector of static link IDs */
  virtual std::vector<tesseract::common::LinkId> getStaticLinkIds() const = 0;

  /**
   * @brief Check if link is an active link
   * @param link_name The link id to check
   * @return True if active, otherwise false
   */
  virtual bool isActiveLinkId(const tesseract::common::LinkId& link_id) const = 0;

  /**
   * @brief Check if link id exists
   * @param link_name The link id to check for
   * @return True if it exists, otherwise false
   */
  virtual bool hasLinkId(const tesseract::common::LinkId& link_id) const = 0;

  /**
   * @brief Get all of the links transforms
   * @details Order should be the same as getLinkNames()
   * @return Get a vector of transforms for all links.
   */
  virtual tesseract::common::VectorIsometry3d getLinkTransforms() const = 0;

  /**
   * @brief Get the transform corresponding to the link.
   * @return Transform and is identity when no transform is available.
   */
  virtual Eigen::Isometry3d getLinkTransform(const tesseract::common::LinkId& link_id) const = 0;

  /**
   * @brief Get transform between two links using the current state
   * @param from_link_name The link id the transform should be relative to
   * @param to_link_name The link id to get transform
   * @return The relative transform = inv(Transform(from_link_id)) * Transform(to_link_id)
   */
  virtual Eigen::Isometry3d getRelativeLinkTransform(const tesseract::common::LinkId& from_link_id,
                                                     const tesseract::common::LinkId& to_link_id) const = 0;

  /**
   * @brief Getter for kinematic limits
   * @return The kinematic limits
   */
  virtual tesseract::common::KinematicLimits getLimits() const = 0;
};
}  // namespace tesseract::scene_graph

#endif  // TESSERACT_STATE_SOLVER_STATE_SOLVER_H
