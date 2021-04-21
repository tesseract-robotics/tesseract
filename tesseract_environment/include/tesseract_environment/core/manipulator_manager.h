/**
 * @file manipulator_manager.h
 * @brief This managers everything about all manipulator, like forward kinematics, inverse kinematics, tcp's, etc.
 *
 * @author Levi Armstrong
 * @date Sep 7, 2020
 * @version TODO
 * @bug No known bugs
 *
 * @copyright Copyright (c) 2020, Southwest Research Institute
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
#ifndef TESSERACT_MANIPULATOR_MANAGER_H
#define TESSERACT_MANIPULATOR_MANAGER_H

#include <tesseract_common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <memory>
#include <Eigen/Geometry>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract_srdf/srdf_model.h>
#include <tesseract_kinematics/core/forward_kinematics_factory.h>
#include <tesseract_kinematics/core/inverse_kinematics_factory.h>
#include <tesseract_environment/core/commands.h>

#ifdef SWIG
%shared_ptr(tesseract_environment::ManipulatorManager)
#endif  // SWIG

namespace tesseract_environment
{
class ManipulatorManager
{
public:
  // LCOV_EXCL_START
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  // LCOV_EXCL_STOP

  using Ptr = std::shared_ptr<ManipulatorManager>;
  using ConstPtr = std::shared_ptr<const ManipulatorManager>;

  ManipulatorManager() = default;
  virtual ~ManipulatorManager() = default;
  ManipulatorManager(const ManipulatorManager&) = default;
  ManipulatorManager& operator=(const ManipulatorManager&) = default;
  ManipulatorManager(ManipulatorManager&&) = default;
  ManipulatorManager& operator=(ManipulatorManager&&) = default;

  /**
   * @brief Initialize the manipulator manager
   * @param scene_graph The scene graph associated with the manipulator manager
   * @param kinematics_information The kinematic information used to initialize the manager
   * @return True if successful, otherwise false.
   */
  bool init(tesseract_scene_graph::SceneGraph::ConstPtr scene_graph,
            tesseract_srdf::KinematicsInformation kinematics_information);

  /**
   * @brief Check if manipulator manager has been initialized
   * @return True if successful, otherwise false
   */
  bool isInitialized() const;

  /**
   * @brief This will clone the manager and assign the new environment object
   * @param environment The SceneGraph the clone is associated with.
   */
  ManipulatorManager::Ptr clone(tesseract_scene_graph::SceneGraph::ConstPtr scene_graph) const;

  /** @brief Kinematics Information */
  bool addKinematicsInformation(const tesseract_srdf::KinematicsInformation& kinematics_information);

  /** @brief Get the kinematics information */
  const tesseract_srdf::KinematicsInformation& getKinematicsInformation() const;

  /** @brief Get Group Names */
  const tesseract_srdf::GroupNames& getGroupNames() const;
  bool hasGroup(const std::string& group_name) const;

  bool addChainGroup(const std::string& group_name, const tesseract_srdf::ChainGroup& chain_group);
  void removeChainGroup(const std::string& group_name);
  bool hasChainGroup(const std::string& group_name) const;
  const tesseract_srdf::ChainGroup& getChainGroup(const std::string& group_name) const;
  const tesseract_srdf::ChainGroups& getChainGroups() const;

  bool addJointGroup(const std::string& group_name, const tesseract_srdf::JointGroup& joint_group);
  void removeJointGroup(const std::string& group_name);
  bool hasJointGroup(const std::string& group_name) const;
  const tesseract_srdf::JointGroup& getJointGroup(const std::string& group_name) const;
  const tesseract_srdf::JointGroups& getJointGroups() const;

  bool addLinkGroup(const std::string& group_name, const tesseract_srdf::LinkGroup& link_group);
  void removeLinkGroup(const std::string& group_name);
  bool hasLinkGroup(const std::string& group_name) const;
  const tesseract_srdf::LinkGroup& getLinkGroup(const std::string& group_name) const;
  const tesseract_srdf::LinkGroups& getLinkGroups() const;

  bool addROPKinematicsSolver(const std::string& group_name, const tesseract_srdf::ROPKinematicParameters& rop_group);
  void removeROPKinematicsSolver(const std::string& group_name);
  bool hasROPKinematicsSolver(const std::string& group_name) const;
  const tesseract_srdf::ROPKinematicParameters& getROPKinematicsSolver(const std::string& group_name) const;
  const tesseract_srdf::GroupROPKinematics& getROPKinematicsSolvers() const;

  bool addREPKinematicsSolver(const std::string& group_name, const tesseract_srdf::REPKinematicParameters& rep_group);
  void removeREPKinematicsSolver(const std::string& group_name);
  bool hasREPKinematicsSolver(const std::string& group_name) const;
  const tesseract_srdf::REPKinematicParameters& getREPKinematicsSolver(const std::string& group_name) const;
  const tesseract_srdf::GroupREPKinematics& getREPKinematicsSolvers() const;

  bool addOPWKinematicsSolver(const std::string& group_name, const tesseract_srdf::OPWKinematicParameters& opw_params);
  void removeOPWKinematicsSovler(const std::string& group_name);
  bool hasOPWKinematicsSolver(const std::string& group_name) const;
  const tesseract_srdf::OPWKinematicParameters& getOPWKinematicsSolver(const std::string& group_name) const;
  const tesseract_srdf::GroupOPWKinematics& getOPWKinematicsSolvers() const;

  bool addGroupJointState(const std::string& group_name,
                          const std::string& state_name,
                          const tesseract_srdf::GroupsJointState& joint_state);
  void removeGroupJointState(const std::string& group_name, const std::string& state_name);
  bool hasGroupJointState(const std::string& group_name, const std::string& state_name) const;
  const tesseract_srdf::GroupsJointState& getGroupsJointState(const std::string& group_name,
                                                              const std::string& state_name) const;
  const tesseract_srdf::GroupsJointStates& getGroupsJointStates(const std::string& group_name) const;
  const tesseract_srdf::GroupJointStates& getGroupJointStates() const;

  bool addGroupTCP(const std::string& group_name, const std::string& tcp_name, const Eigen::Isometry3d& tcp);
  void removeGroupTCP(const std::string& group_name, const std::string& tcp_name);
  bool hasGroupTCP(const std::string& group_name, const std::string& tcp_name) const;
  const Eigen::Isometry3d& getGroupsTCP(const std::string& group_name, const std::string& tcp_name) const;
  const tesseract_srdf::GroupsTCPs& getGroupsTCPs(const std::string& group_name) const;
  const tesseract_srdf::GroupTCPs& getGroupTCPs() const;

  /**
   * @brief Register a forward kinematics factory
   * @param factory The factory to register
   * @return False if factory already exists, otherwise true.
   */
  bool registerFwdKinematicsFactory(tesseract_kinematics::ForwardKinematicsFactory::ConstPtr factory);

  /**
   * @brief Removes a registered forward kinematics factory
   * @param name The name of the factory to remove
   */
  void removeFwdKinematicsFactory(const std::string& name);

  /**
   * @brief Get a list of all available forward kinematics solvers
   * @return Vector of names
   */
  std::vector<std::string> getAvailableFwdKinematicsSolvers() const;

  /**
   * @brief Get a list of forward kinematics solver for a specific type {CHAIN, TREE, GRAPH}
   * @param type The type of solver {CHAIN, TREE, GRAPH}
   * @return Vector of names
   */
  std::vector<std::string>
  getAvailableFwdKinematicsSolvers(tesseract_kinematics::ForwardKinematicsFactoryType type) const;

  /**
   * @brief This will return the forward kinematics solver factory
   * @param name The name of the solver
   * @return If not found it returns a nullptr, otherwise a new instance of the solver.
   */
  tesseract_kinematics::ForwardKinematicsFactory::ConstPtr getFwdKinematicFactory(const std::string& name) const;

  /**
   * @brief Add a manipulator forward kinematics solver
   * @param manipulator The manipulator name
   * @param solver The solver
   * @return
   */
  bool addFwdKinematicSolver(const tesseract_kinematics::ForwardKinematics::Ptr& solver);

  /**
   * @brief Remove a forward kinematic solver for a given manipulator
   * @param manipulator The name of the manipulator
   * @param name The name of the solver
   */
  void removeFwdKinematicSolver(const std::string& manipulator, const std::string& name);

  /**
   * @brief Remove all forward kinematic solver for a given manipulator
   * @param manipulator The name of the manipulator
   * @param name The name of the solver
   */
  void removeFwdKinematicSolver(const std::string& manipulator);

  /**
   * @brief Get a list of all available forward kinematics manipulators
   * @return Vector of names
   */
  std::vector<std::string> getAvailableFwdKinematicsManipulators() const;

  /**
   * @brief Set default forward kinematic solver for manipulator
   * @param manipulator The name of the manipulator
   * @param name The name of the solver
   * @return True if manipulator solver pair exist, otherwise false
   */
  bool setDefaultFwdKinematicSolver(const std::string& manipulator, const std::string& name);

  /**
   * @brief Get forward kinematic solver for manipulator
   * @param manipulator The name of the manipulator
   * @param name The name of the solver
   * @return If not found returns a nullptr, otherwise a instance of the solver.
   */
  tesseract_kinematics::ForwardKinematics::Ptr getFwdKinematicSolver(const std::string& manipulator,
                                                                     const std::string& name) const;

  /**
   * @brief Get default forward kinematic solver for manipulator
   * @param manipulator The name of the manipulator
   * @return If not found returns a nullptr, otherwise a instance of the solver.
   */
  tesseract_kinematics::ForwardKinematics::Ptr getFwdKinematicSolver(const std::string& manipulator) const;

  /**
   * @brief Register a inverse kinematics factory
   * @param factory The factory to register
   * @return False if factory already exists, otherwise true.
   */
  bool registerInvKinematicsFactory(tesseract_kinematics::InverseKinematicsFactory::ConstPtr factory);

  /**
   * @brief Removes a registered inverse kinematics factory
   * @param name The name of the factory to remove
   */
  void removeInvKinematicsFactory(const std::string& name);

  /**
   * @brief Get a list of all available inverse kinematics solvers
   * @return Vector of names
   */
  std::vector<std::string> getAvailableInvKinematicsSolvers() const;

  /**
   * @brief Get a list of inverse kinematics solver for a specific type {CHAIN, TREE, GRAPH}
   * @param type The type of solver {CHAIN, TREE, GRAPH}
   * @return Vector of names
   */
  std::vector<std::string>
  getAvailableInvKinematicsSolvers(tesseract_kinematics::InverseKinematicsFactoryType type) const;

  /**
   * @brief This will return the inverse kinematics solver factory
   * @param name The name of the solver
   * @return If not found it returns a nullptr, otherwise a new instance of the solver.
   */
  tesseract_kinematics::InverseKinematicsFactory::ConstPtr getInvKinematicFactory(const std::string& name) const;

  /**
   * @brief Add a manipulator inverse kinematics solver
   * @param manipulator The manipulator name
   * @param solver The solver
   * @return
   */
  bool addInvKinematicSolver(const tesseract_kinematics::InverseKinematics::Ptr& solver);

  /**
   * @brief Remove a inverse kinematic solver for a given manipulator
   * @param manipulator The name of the manipulator
   * @param name The name of the solver
   */
  void removeInvKinematicSolver(const std::string& manipulator, const std::string& name);

  /**
   * @brief Remove all forward kinematic solver for a given manipulator
   * @param manipulator The name of the manipulator
   * @param name The name of the solver
   */
  void removeInvKinematicSolver(const std::string& manipulator);

  /**
   * @brief Get a list of all available forward kinematics manipulators
   * @return Vector of names
   */
  std::vector<std::string> getAvailableInvKinematicsManipulators() const;

  /**
   * @brief Set default inverse kinematic solver for manipulator
   * @param manipulator The name of the manipulator
   * @param name The name of the solver
   * @return True if manipulator solver pair exist, otherwise false
   */
  bool setDefaultInvKinematicSolver(const std::string& manipulator, const std::string& name);

  /**
   * @brief Get inverse kinematic solver for manipulator
   * @param manipulator The name of the manipulator
   * @param name The name of the solver
   * @return If not found returns a nullptr, otherwise a instance of the solver.
   */
  tesseract_kinematics::InverseKinematics::Ptr getInvKinematicSolver(const std::string& manipulator,
                                                                     const std::string& name) const;

  /**
   * @brief Get default inverse kinematic solver for manipulator
   * @param manipulator The name of the manipulator
   * @return If not found returns a nullptr, otherwise a instance of the solver.
   */
  tesseract_kinematics::InverseKinematics::Ptr getInvKinematicSolver(const std::string& manipulator) const;

private:
  bool initialized_{ false };
  tesseract_srdf::KinematicsInformation kinematics_information_;
  tesseract_scene_graph::SceneGraph::ConstPtr scene_graph_;
  tesseract_kinematics::ForwardKinematicsFactory::ConstPtr fwd_kin_chain_default_factory_;
  tesseract_kinematics::ForwardKinematicsFactory::ConstPtr fwd_kin_tree_default_factory_;
  tesseract_kinematics::InverseKinematicsFactory::ConstPtr inv_kin_chain_default_factory_;
  std::unordered_map<std::string, tesseract_kinematics::ForwardKinematicsFactory::ConstPtr> fwd_kin_factories_;
  std::map<std::pair<std::string, std::string>, tesseract_kinematics::ForwardKinematics::Ptr> fwd_kin_manipulators_;
  std::unordered_map<std::string, tesseract_kinematics::ForwardKinematics::Ptr> fwd_kin_manipulators_default_;
  std::unordered_map<std::string, tesseract_kinematics::InverseKinematicsFactory::ConstPtr> inv_kin_factories_;
  std::map<std::pair<std::string, std::string>, tesseract_kinematics::InverseKinematics::Ptr> inv_kin_manipulators_;
  std::unordered_map<std::string, tesseract_kinematics::InverseKinematics::Ptr> inv_kin_manipulators_default_;
  int revision_{ 0 };

  bool registerDefaultChainSolver(const std::string& group_name, const tesseract_srdf::ChainGroup& chain_group);
  bool registerDefaultJointSolver(const std::string& group_name, const tesseract_srdf::JointGroup& joint_group);
  bool registerDefaultLinkSolver(const std::string& group_name, const tesseract_srdf::LinkGroup& joint_group);
  bool registerOPWSolver(const std::string& group_name, const tesseract_srdf::OPWKinematicParameters& opw_params);
  bool registerROPSolver(const std::string& group_name, const tesseract_srdf::ROPKinematicParameters& rop_group);
  bool registerREPSolver(const std::string& group_name, const tesseract_srdf::REPKinematicParameters& rep_group);

  /** @brief Apply environment command to update kinematics if needed */
  void onEnvironmentChanged(const Commands& commands);

  friend class Environment;
};
}  // namespace tesseract_environment

#endif  // TESSERACT_MANIPULATOR_MANAGER_H
