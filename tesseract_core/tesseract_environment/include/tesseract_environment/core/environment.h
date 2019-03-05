/**
 * @file Environment.h
 * @brief Tesseract Environment.
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
#ifndef TESSERACT_ENVIRONMENT_ENVIRONMENT_H
#define TESSERACT_ENVIRONMENT_ENVIRONMENT_H
#include <tesseract_environment/core/macros.h>
TESSERACT_ENVIRONMENT_IGNORE_WARNINGS_PUSH
#include <vector>
#include <string>
TESSERACT_ENVIRONMENT_IGNORE_WARNINGS_POP

#include <tesseract_environment/core/types.h>
#include <tesseract_collision/core/discrete_contact_manager.h>
#include <tesseract_collision/core/continuous_contact_manager.h>

namespace tesseract_environment
{
class Environment
{
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  virtual ~Environment() = default;
  /** @brief Give the environment a name */
  virtual void setName(const std::string& name) = 0;

  /** @brief Get the name of the environment
   *
   * This may be empty, if so check urdf name
   */
  virtual const std::string& getName() const = 0;

  /** @brief Set the current state of the environment */
  virtual void setState(const std::unordered_map<std::string, double>& joints) = 0;
  virtual void setState(const std::vector<std::string>& joint_names, const std::vector<double>& joint_values) = 0;
  virtual void setState(const std::vector<std::string>& joint_names,
                        const Eigen::Ref<const Eigen::VectorXd>& joint_values) = 0;

  /** @brief Get the current state of the environment */
  virtual EnvStateConstPtr getState() const = 0;

  /**
   * @brief Get the state of the environment for a given set or subset of joint values.
   *
   * This does not change the internal state of the environment.
   *
   * @param joints A map of joint names to joint values to change.
   * @return A the state of the environment
   */
  virtual EnvStatePtr getState(const std::unordered_map<std::string, double>& joints) const = 0;
  virtual EnvStatePtr getState(const std::vector<std::string>& joint_names,
                               const std::vector<double>& joint_values) const = 0;
  virtual EnvStatePtr getState(const std::vector<std::string>& joint_names,
                               const Eigen::Ref<const Eigen::VectorXd>& joint_values) const = 0;

  /**
   * @brief Adds a link to the environment
   * @param link The link to be added to the graph
   * @return Return False if a link with the same name allready exists, otherwise true
   */
  virtual bool addLink(tesseract_scene_graph::LinkPtr link) = 0;

  /**
   * @brief Removes a link from the environment
   *
   *        Parent joint and all child components (links/joints) should be removed
   *
   * @param name Name of the link to be removed
   * @return Return False if a link does not exists, otherwise true
   */
  virtual bool removeLink(const std::string& name) = 0;

  /**
   * @brief Move a link in the environment
   *
   *        This should delete the parent joint of the child link. All child links and joints follow.
   *
   * @param joint The new joint.
   * @return Return False if a link does not exists or has no parent joint, otherwise true
   */
  virtual bool moveLink(tesseract_scene_graph::JointPtr joint) = 0;

  /**
   * @brief Get a link in the environment
   * @param name The name of the link
   * @return Return nullptr if link name does not exists, otherwise a pointer to the link
   */
  virtual tesseract_scene_graph::LinkConstPtr getLink(const std::string& name) const = 0;

  /**
   * @brief Adds joint to the
   * @param joint The joint to be added
   * @return Return False if parent or child link does not exists and if joint name already exists in the graph, otherwise true
   */
  virtual bool addJoint(tesseract_scene_graph::JointPtr joint) = 0;

  /**
   * @brief Removes a joint from the environment
   *
   *        All child components (links/joints) should be removed
   *
   * @param name Name of the joint to be removed
   * @return Return False if a joint does not exists, otherwise true
   */
  virtual bool removeJoint(const std::string& name) = 0;

  /**
   * @brief Move a joint from one link to another
   *
   *        All child links & joints should follow
   *
   * @param joint_name The name of the joint to move
   * @param new_parent_link The name of the link to move to.
   * @return Return False if parent_link does not exists, otherwise true
   */
  virtual bool moveJoint(const std::string& joint_name, const std::string& parent_link) = 0;


  /**
   * @brief Get a joint in the environment
   * @param name The name of the joint
   * @return Return nullptr if joint name does not exists, otherwise a pointer to the joint
   */
  virtual tesseract_scene_graph::JointConstPtr getJoint(const std::string& name) const = 0;

  /**
   * @brief Get a vector of joint names in the environment
   * @return A vector of joint names
   */
  virtual std::vector<std::string> getJointNames() const = 0;

  /**
   * @brief Get the current state of the environment
   *
   * Order should be the same as getJointNames()
   *
   * @return A vector of joint values
   */
  virtual Eigen::VectorXd getCurrentJointValues() const = 0;

  /**
   * @brief Get the root link name
   * @return String
   */
  virtual const std::string& getRootLinkName() const = 0;

  /**
   * @brief Get a vector of link names in the environment
   * @return A vector of link names
   */
  virtual std::vector<std::string> getLinkNames() const = 0;

  /**
   * @brief Get a vector of active link names in the environment
   * @return A vector of active link names
   */
  virtual std::vector<std::string> getActiveLinkNames() const = 0;

  /**
   * @brief Get all of the links transforms
   *
   * Order should be the same as getLinkNames()
   *
   * @return Get a vector of transforms for all links in the environment.
   */
  virtual VectorIsometry3d getLinkTransforms() const = 0;

  /**
   * @brief Get the transform corresponding to the link.
   * @return Transform and is identity when no transform is available.
   */
  virtual const Eigen::Isometry3d& getLinkTransform(const std::string& link_name) const = 0;

  /** @brief Get the allowed collision matrix */
  virtual AllowedCollisionMatrixConstPtr getAllowedCollisionMatrix() const = 0;

  /** @brief Get the allowed collision matrix */
  virtual AllowedCollisionMatrixPtr getAllowedCollisionMatrixNonConst() = 0;

  /** @brief Get the active function for determining if two links are allowed to be in collision */
  virtual tesseract_collision::IsContactAllowedFn getIsContactAllowedFn() const = 0;

  /** @brief Set the active function for determining if two links are allowed to be in collision */
  virtual void setIsContactAllowedFn(tesseract_collision::IsContactAllowedFn fn) = 0;

  /**
   * @brief Set the discrete contact manager
   *
   * This method should clear the contents of the manager and reload it with the objects
   * in the environment.
   */
  virtual bool setDiscreteContactManager(tesseract_collision::DiscreteContactManagerConstPtr manager) = 0;

  /** @brief Get a copy of the environments discrete contact manager */
  virtual tesseract_collision::DiscreteContactManagerPtr getDiscreteContactManager() const = 0;

  /**
   * @brief Set the continuous contact manager
   *
   * This method should clear the contents of the manager and reload it with the objects
   * in the environment.
   */
  virtual bool setContinuousContactManager(tesseract_collision::ContinuousContactManagerConstPtr manager) = 0;

  /** @brief Get a copy of the environments continuous contact manager */
  virtual tesseract_collision::ContinuousContactManagerPtr getContinuousContactManager() const = 0;

};  // class BasicEnvBase

typedef std::shared_ptr<Environment> EnvironmentPtr;
typedef std::shared_ptr<const Environment> EnvironmentConstPtr;

/**
 * @brief continuousCollisionCheckTrajectory Should perform a continuous collision check over the trajectory
 * and stop on first collision.
 * @param manager A continuous contact manager
 * @param env The environment
 * @param joint_names JointNames corresponding to the values in traj (must be in same order)
 * @param link_names Name of the links to calculate collision data for.
 * @param traj The joint values at each time step
 * @param contacts A vector of vector of ContactMap where each indicie corrisponds to a timestep
 * @param first_only Indicates if it should return on first contact
 * @return True if collision was found, otherwise false.
 */
//inline bool continuousCollisionCheckTrajectory(tesseract_collision::ContinuousContactManager& manager,
//                                               const tesseract_environment::Environment& env,
//                                               const tesseract_kinematics::ForwardKinematics& kin,
//                                               const Eigen::Ref<const TrajArray>& traj,
//                                               std::vector<tesseract_collision::ContactResultMap>& contacts,
//                                               bool first_only = true)
//{
//  bool found = false;
//  const std::vector<std::string>& joint_names = kin.getJointNames();
//  const std::vector<std::string>& link_names = kin.getLinkNames();

//  contacts.reserve(static_cast<size_t>(traj.rows() - 1));
//  for (int iStep = 0; iStep < traj.rows() - 1; ++iStep)
//  {
//    tesseract_collision::ContactResultMap collisions;

//    EnvStatePtr state0 = env.getState(joint_names, traj.row(iStep));
//    EnvStatePtr state1 = env.getState(joint_names, traj.row(iStep + 1));

//    for (const auto& link_name : link_names)
//      manager.setCollisionObjectsTransform(link_name, state0->transforms[link_name], state1->transforms[link_name]);

//    manager.contactTest(collisions, tesseract_collision::ContactTestTypes::FIRST);

//    if (collisions.size() > 0)
//      found = true;

//    contacts.push_back(collisions);

//    if (found && first_only)
//      break;
//  }

//  return found;
//}

}  // namespace tesseract_environment

#endif  // TESSERACT_ENVIRONMENT_ENVIRONMENT_H
