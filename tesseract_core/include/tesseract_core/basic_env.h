/**
 * @file basic_env.h
 * @brief Basic low-level environment with collision and distance functions.
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
#ifndef TESSERACT_CORE_BASIC_ENV_H
#define TESSERACT_CORE_BASIC_ENV_H
#include <tesseract_core/macros.h>
TESSERACT_IGNORE_WARNINGS_PUSH
#include <vector>
#include <string>
TESSERACT_IGNORE_WARNINGS_POP

#include <tesseract_core/basic_types.h>
#include <tesseract_core/basic_kin.h>
#include <tesseract_core/discrete_contact_manager_base.h>
#include <tesseract_core/continuous_contact_manager_base.h>

namespace tesseract
{
class BasicEnv
{
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  virtual ~BasicEnv() = default;
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
   * @brief hasManipulator Check if a manipulator exist in the environment
   * @param manipulator_name Name of the manipulator
   * @return True if it exists otherwise false
   */
  virtual bool hasManipulator(const std::string& manipulator_name) const = 0;

  /**
   * @brief getManipulatorKin Get a kinematic object for the provided manipulator name.
   * @param manipulator_name Name of the manipulator
   * @return BasicKinPtr
   */
  virtual BasicKinConstPtr getManipulator(const std::string& manipulator_name) const = 0;

  /**
   * @brief A a manipulator as a kinematic chain
   * @param base_link The base link of the chain
   * @param tip_link The tip link of the chain
   * @param name The name of the manipulator. This must be unique.
   * @return true if successfully created, otherwise false.
   */
  virtual bool addManipulator(const std::string& base_link,
                              const std::string& tip_link,
                              const std::string& manipulator_name) = 0;

  /**
   * @brief A a manipulator as a set of joints
   * @param joint_names The list of joint names that make up the manipulator (used for non chains)
   * @param name The name of the manipulator. This must be unique.
   * @return true if successfully created, otherwise false.
   */
  virtual bool addManipulator(const std::vector<std::string>& joint_names, const std::string& manipulator_name) = 0;

  /**
   * @brief Add object so it may be attached/detached.
   *
   * This object is not part of the environment until attached to a link.
   *
   * @param attachable_object The object information
   */
  virtual void addAttachableObject(const AttachableObjectConstPtr attachable_object) = 0;

  /**
   * @brief Remove object from list of available objects to be attached
   *
   * This will not remove any bodies using the object.
   *
   * @param name The name of the object to be removed
   */
  virtual void removeAttachableObject(const std::string& name) = 0;

  /**
   * @brief Get a map of available attachable objects
   * @return A map of attachable objects
   */
  virtual const AttachableObjectConstPtrMap& getAttachableObjects() const = 0;

  /** @brief This will remove all attachable objects */
  virtual void clearAttachableObjects() = 0;

  /**
   * @brief Get attached body
   * @param name The name of the body
   * @return AttachedBody
   */
  virtual const AttachedBodyInfo& getAttachedBody(const std::string& name) const = 0;

  /**
   * @brief Get all attached bodies
   * @return A map of attached bodies
   */
  virtual const AttachedBodyInfoMap& getAttachedBodies() const = 0;

  /**
   * @brief Attached an attachable object to the environment
   * @param attached_body Information of attaching creating the attached body
   */
  virtual void attachBody(const AttachedBodyInfo& attached_body_info) = 0;

  /**
   * @brief Detach an attachable object from the environment
   * @param name The name given to the Attached Body when attached
   */
  virtual void detachBody(const std::string& name) = 0;

  /** @brief This will detach all bodies */
  virtual void clearAttachedBodies() = 0;

  /** @brief Get a map of object names to colors */
  virtual ObjectColorMapConstPtr getKnownObjectColors() const = 0;

  /** @brief Clear all known object colors */
  virtual void clearKnownObjectColors() = 0;

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
   * @brief Get the current state of the manipulator
   *
   * Order should be the same as BasicKin.getJointNames()
   *
   * @return A vector of joint values
   */
  virtual Eigen::VectorXd getCurrentJointValues(const std::string& manipulator_name) const = 0;

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
  virtual IsContactAllowedFn getIsContactAllowedFn() const = 0;

  /** @brief Set the active function for determining if two links are allowed to be in collision */
  virtual void setIsContactAllowedFn(IsContactAllowedFn fn) = 0;

  /** @brief Get a copy of the environments discrete contact manager */
  virtual DiscreteContactManagerBasePtr getDiscreteContactManager() const = 0;

  /** @brief Get a copy of the environments continuous contact manager */
  virtual ContinuousContactManagerBasePtr getContinuousContactManager() const = 0;

};  // class BasicEnvBase

typedef std::shared_ptr<BasicEnv> BasicEnvPtr;
typedef std::shared_ptr<const BasicEnv> BasicEnvConstPtr;

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
inline bool continuousCollisionCheckTrajectory(ContinuousContactManagerBase& manager,
                                               const BasicEnv& env,
                                               const BasicKin& kin,
                                               const Eigen::Ref<const TrajArray>& traj,
                                               std::vector<ContactResultMap>& contacts,
                                               bool first_only = true)
{
  bool found = false;
  const std::vector<std::string>& joint_names = kin.getJointNames();
  const std::vector<std::string>& link_names = kin.getLinkNames();

  contacts.reserve(static_cast<size_t>(traj.rows() - 1));
  for (int iStep = 0; iStep < traj.rows() - 1; ++iStep)
  {
    ContactResultMap collisions;

    EnvStatePtr state0 = env.getState(joint_names, traj.row(iStep));
    EnvStatePtr state1 = env.getState(joint_names, traj.row(iStep + 1));

    for (const auto& link_name : link_names)
      manager.setCollisionObjectsTransform(link_name, state0->transforms[link_name], state1->transforms[link_name]);

    manager.contactTest(collisions, ContactTestTypes::FIRST);

    if (collisions.size() > 0)
      found = true;

    contacts.push_back(collisions);

    if (found && first_only)
      break;
  }

  return found;
}

}  // namespace tesseract

#endif  // TESSERACT_CORE_BASIC_ENV_H
