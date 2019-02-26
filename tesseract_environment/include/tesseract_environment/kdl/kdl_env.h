/**
 * @file kdl_env.h
 * @brief Tesseract environment kdl implementation.
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
#ifndef TESSERACT_ENVIRONMENT_KDL_ENV_H
#define TESSERACT_ENVIRONMENT_KDL_ENV_H

#include <tesseract_environment/core/macros.h>
TESSERACT_ENVIRONMENT_IGNORE_WARNINGS_PUSH
#include <kdl/tree.hpp>
#include <kdl_parser/kdl_parser.hpp>
#include <kdl/jntarray.hpp>
#include <sensor_msgs/JointState.h>
#include <ros/publisher.h>
#include <urdf/model.h>
#include <pluginlib/class_loader.hpp>
TESSERACT_ENVIRONMENT_IGNORE_WARNINGS_POP

#include <tesseract_collision/core/discrete_contact_manager.h>
#include <tesseract_collision/core/continuous_contact_manager.h>
#include <tesseract_environment/core/environment.h>

namespace tesseract_environment
{

class KDLEnv : public Environment
{
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  KDLEnv() : Environment(), initialized_(false), allowed_collision_matrix_(new AllowedCollisionMatrix())
  {
    is_contact_allowed_fn_ = std::bind(&tesseract_environment::KDLEnv::defaultIsContactAllowedFn,
                                       this,
                                       std::placeholders::_1,
                                       std::placeholders::_2);
  }

  bool init(urdf::ModelInterfaceConstSharedPtr urdf_model);

  void setName(const std::string& name) override { name_ = name; }
  const std::string& getName() const override { return name_; }
  bool checkInitialized() const { return initialized_; }
  EnvStateConstPtr getState() const override { return current_state_; }
  void setState(const std::unordered_map<std::string, double>& joints) override;
  void setState(const std::vector<std::string>& joint_names, const std::vector<double>& joint_values) override;
  void setState(const std::vector<std::string>& joint_names,
                const Eigen::Ref<const Eigen::VectorXd>& joint_values) override;

  EnvStatePtr getState(const std::unordered_map<std::string, double>& joints) const override;
  EnvStatePtr getState(const std::vector<std::string>& joint_names,
                       const std::vector<double>& joint_values) const override;
  EnvStatePtr getState(const std::vector<std::string>& joint_names,
                       const Eigen::Ref<const Eigen::VectorXd>& joint_values) const override;

  std::vector<std::string> getJointNames() const override { return joint_names_; }
  Eigen::VectorXd getCurrentJointValues() const override;

  Eigen::VectorXd getCurrentJointValues(const std::string& manipulator_name) const override;

  const std::string& getRootLinkName() const override { return kdl_tree_->getRootSegment()->second.segment.getName(); }
  std::vector<std::string> getLinkNames() const override { return link_names_; }
  std::vector<std::string> getActiveLinkNames() const override { return active_link_names_; }
  VectorIsometry3d getLinkTransforms() const override;

  const Eigen::Isometry3d& getLinkTransform(const std::string& link_name) const override;

  bool hasManipulator(const std::string& manipulator_name) const override;

  tesseract_kinematics::ForwardKinematicsConstPtr getManipulator(const std::string& manipulator_name) const override;

  bool addManipulator(const std::string& base_link,
                      const std::string& tip_link,
                      const std::string& manipulator_name) override;

  bool addManipulator(const std::vector<std::string>& joint_names, const std::string& manipulator_name) override;

  ObjectColorMapConstPtr getKnownObjectColors() const override { return object_colors_; }
  void clearKnownObjectColors() override { object_colors_->clear(); }
  void addAttachableObject(const AttachableObjectConstPtr attachable_object) override;

  void removeAttachableObject(const std::string& name) override;

  const AttachableObjectConstPtrMap& getAttachableObjects() const override { return attachable_objects_; }
  void clearAttachableObjects() override;

  const AttachedBodyInfo& getAttachedBody(const std::string& name) const override;

  const AttachedBodyInfoMap& getAttachedBodies() const override { return attached_bodies_; }
  void attachBody(const AttachedBodyInfo& attached_body_info) override;

  void detachBody(const std::string& name) override;

  void clearAttachedBodies() override;

  urdf::ModelInterfaceConstSharedPtr getURDF() const { return urdf_model_; }
  AllowedCollisionMatrixConstPtr getAllowedCollisionMatrix() const override { return allowed_collision_matrix_; }
  AllowedCollisionMatrixPtr getAllowedCollisionMatrixNonConst() override { return allowed_collision_matrix_; }
  tesseract_collision::IsContactAllowedFn getIsContactAllowedFn() const override { return is_contact_allowed_fn_; }
  void setIsContactAllowedFn(tesseract_collision::IsContactAllowedFn fn) override { is_contact_allowed_fn_ = fn; }

  bool setDiscreteContactManager(tesseract_collision::DiscreteContactManagerConstPtr manager) override;
  tesseract_collision::DiscreteContactManagerPtr getDiscreteContactManager() const override { return discrete_manager_->clone(); }

  bool setContinuousContactManager(tesseract_collision::ContinuousContactManagerConstPtr manager) override;
  tesseract_collision::ContinuousContactManagerPtr getContinuousContactManager() const override { return continuous_manager_->clone(); }

private:
  bool initialized_;                                           /**< Identifies if the object has been initialized */
  std::string name_;                                           /**< Name of the environment (may be empty) */
  urdf::ModelInterfaceConstSharedPtr urdf_model_;              /**< URDF MODEL */
  std::shared_ptr<const KDL::Tree> kdl_tree_;                  /**< KDL tree object */
  EnvStatePtr current_state_;                                  /**< Current state of the robot */
  std::unordered_map<std::string, unsigned int> joint_to_qnr_; /**< Map between joint name and kdl q index */
  KDL::JntArray kdl_jnt_array_;                                /**< The kdl joint array */
  AttachedBodyInfoMap attached_bodies_;                        /**< A map of attached bodies */
  AttachableObjectConstPtrMap
      attachable_objects_; /**< A map of objects that can be attached/detached from environment */
  std::unordered_map<std::string, tesseract_kinematics::ForwardKinematicsPtr> manipulators_; /**< A map of manipulator names to kinematics object */
  std::vector<std::string> link_names_;                       /**< A vector of link names */
  std::vector<std::string> joint_names_;                      /**< A vector of joint names */
  std::vector<std::string> active_link_names_;                /**< A vector of active link names */
  ObjectColorMapPtr object_colors_;                           /**< A map of objects to color */
  AllowedCollisionMatrixPtr
      allowed_collision_matrix_; /**< The allowed collision matrix used during collision checking */
  tesseract_collision::IsContactAllowedFn is_contact_allowed_fn_;       /**< The function used to determine if two objects are allowed in collision */
  tesseract_collision::DiscreteContactManagerPtr discrete_manager_;     /**< The discrete contact manager object */
  tesseract_collision::ContinuousContactManagerPtr continuous_manager_; /**< The continuous contact manager object */

  bool defaultIsContactAllowedFn(const std::string& link_name1, const std::string& link_name2) const;

  void calculateTransforms(TransformMap& transforms,
                           const KDL::JntArray& q_in,
                           const KDL::SegmentMap::const_iterator& it,
                           const Eigen::Isometry3d& parent_frame) const;

  void calculateTransformsHelper(TransformMap& transforms,
                                 const KDL::JntArray& q_in,
                                 const KDL::SegmentMap::const_iterator& it,
                                 const Eigen::Isometry3d& parent_frame) const;

  bool setJointValuesHelper(KDL::JntArray& q, const std::string& joint_name, const double& joint_value) const;

  std::string getManipulatorName(const std::vector<std::string>& joint_names) const;
};
typedef std::shared_ptr<KDLEnv> KDLEnvPtr;
typedef std::shared_ptr<const KDLEnv> KDLEnvConstPtr;
}

#endif  // TESSERACT_ENVIRONMENT_KDL_ENV_H
