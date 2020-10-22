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

#include <tesseract/manipulator_manager.h>
#include <tesseract_kinematics/kdl/kdl_fwd_kin_chain_factory.h>
#include <tesseract_kinematics/kdl/kdl_fwd_kin_tree_factory.h>
#include <tesseract_kinematics/kdl/kdl_inv_kin_chain_lma_factory.h>
#include <tesseract_kinematics/kdl/kdl_inv_kin_chain_nr_factory.h>
#include <tesseract_kinematics/opw/opw_inv_kin.h>
#include <tesseract_kinematics/core/rop_inverse_kinematics.h>
#include <tesseract_kinematics/core/rep_inverse_kinematics.h>

namespace tesseract
{
bool ManipulatorManager::init(tesseract_environment::Environment::ConstPtr environment,
                              tesseract_scene_graph::SRDFModel::Ptr srdf_model)
{
  env_ = std::move(environment);
  scene_graph_ = std::move(env_->getSceneGraph());
  srdf_model_ = std::move(srdf_model);

  if (scene_graph_ == nullptr || srdf_model_ == nullptr || env_ == nullptr)
    return false;

  fwd_kin_chain_default_factory_ = std::make_shared<tesseract_kinematics::KDLFwdKinChainFactory>();
  registerFwdKinematicsFactory(fwd_kin_chain_default_factory_);

  fwd_kin_tree_default_factory_ = std::make_shared<tesseract_kinematics::KDLFwdKinTreeFactory>();
  registerFwdKinematicsFactory(fwd_kin_tree_default_factory_);

  inv_kin_chain_default_factory_ = std::make_shared<tesseract_kinematics::KDLInvKinChainLMAFactory>();
  registerInvKinematicsFactory(inv_kin_chain_default_factory_);

  bool success = true;
  for (const auto& group : srdf_model_->getChainGroups())
    success &= registerDefaultChainSolver(group.first, group.second);

  for (const auto& group : srdf_model_->getJointGroups())
    success &= registerDefaultJointSolver(group.first, group.second);

  //  for (const auto& group : srdf_model_->getLinkGroups())
  //    success &= registerDefaultLinkSolver(group.first, group.second);

  for (const auto& group : srdf_model_->getGroupOPWKinematics())
    success &= registerOPWSolver(group.first, group.second);

  for (const auto& group : srdf_model_->getGroupROPKinematics())
    success &= registerROPSolver(group.first, group.second);

  for (const auto& group : srdf_model_->getGroupREPKinematics())
    success &= registerREPSolver(group.first, group.second);

  return success;
}

bool ManipulatorManager::update()
{
  bool success = true;
  for (auto& fk : fwd_kin_manipulators_)
    success &= fk.second->update();

  for (auto& ik : inv_kin_manipulators_)
    success &= ik.second->update();

  return success;
}

ManipulatorManager::Ptr ManipulatorManager::clone(tesseract_environment::Environment::ConstPtr environment) const
{
  auto cloned_manager = std::make_shared<ManipulatorManager>(*this);
  cloned_manager->env_ = std::move(environment);
  cloned_manager->scene_graph_ = std::move(cloned_manager->env_->getSceneGraph());
  cloned_manager->srdf_model_ = std::make_shared<tesseract_scene_graph::SRDFModel>(*srdf_model_);
  return cloned_manager;
}

tesseract_scene_graph::SRDFModel::ConstPtr ManipulatorManager::getSRDFModel() const { return srdf_model_; }

const tesseract_scene_graph::GroupNames& ManipulatorManager::getGroupNames() const
{
  return srdf_model_->getGroupNames();
}

bool ManipulatorManager::hasGroup(const std::string& group_name) const
{
  const tesseract_scene_graph::GroupNames& group_names = srdf_model_->getGroupNames();
  return std::find(group_names.begin(), group_names.end(), group_name) != group_names.end();
}

bool ManipulatorManager::addChainGroup(const std::string& group_name,
                                       const tesseract_scene_graph::ChainGroup& chain_group)
{
  if (hasGroup(group_name))
  {
    CONSOLE_BRIDGE_logError("ManipulatorManager: Group name is already taken!");
    return false;
  }

  srdf_model_->getChainGroups()[group_name] = chain_group;
  srdf_model_->getGroupNames().push_back(group_name);
  return registerDefaultChainSolver(group_name, chain_group);
}

void ManipulatorManager::removeChainGroup(const std::string& group_name)
{
  srdf_model_->getChainGroups().erase(group_name);
  tesseract_scene_graph::GroupNames& group_names = srdf_model_->getGroupNames();
  group_names.erase(std::remove_if(
      group_names.begin(), group_names.end(), [group_name](const std::string& gn) { return gn == group_name; }));
  removeFwdKinematicSolver(group_name);
  removeInvKinematicSolver(group_name);
}

const tesseract_scene_graph::ChainGroup& ManipulatorManager::getChainGroup(const std::string& group_name) const
{
  return srdf_model_->getChainGroups().at(group_name);
}

const tesseract_scene_graph::ChainGroups& ManipulatorManager::getChainGroups() const
{
  return srdf_model_->getChainGroups();
}

bool ManipulatorManager::addJointGroup(const std::string& group_name,
                                       const tesseract_scene_graph::JointGroup& joint_group)
{
  if (hasGroup(group_name))
  {
    CONSOLE_BRIDGE_logError("ManipulatorManager: Group name is already taken!");
    return false;
  }

  srdf_model_->getJointGroups()[group_name] = joint_group;
  srdf_model_->getGroupNames().push_back(group_name);
  return registerDefaultJointSolver(group_name, joint_group);
}

void ManipulatorManager::removeJointGroup(const std::string& group_name)
{
  srdf_model_->getJointGroups().erase(group_name);
  tesseract_scene_graph::GroupNames& group_names = srdf_model_->getGroupNames();
  group_names.erase(std::remove_if(
      group_names.begin(), group_names.end(), [group_name](const std::string& gn) { return gn == group_name; }));
  removeFwdKinematicSolver(group_name);
  removeInvKinematicSolver(group_name);
}

const tesseract_scene_graph::JointGroup& ManipulatorManager::getJointGroup(const std::string& group_name) const
{
  return srdf_model_->getJointGroups().at(group_name);
}

const tesseract_scene_graph::JointGroups& ManipulatorManager::getJointGroups() const
{
  return srdf_model_->getJointGroups();
}

bool ManipulatorManager::addLinkGroup(const std::string& group_name, const tesseract_scene_graph::LinkGroup& link_group)
{
  if (hasGroup(group_name))
  {
    CONSOLE_BRIDGE_logError("ManipulatorManager: Group name is already taken!");
    return false;
  }

  srdf_model_->getLinkGroups()[group_name] = link_group;
  srdf_model_->getGroupNames().push_back(group_name);
  return registerDefaultLinkSolver(group_name, link_group);
}

void ManipulatorManager::removeLinkGroup(const std::string& group_name)
{
  srdf_model_->getLinkGroups().erase(group_name);
  tesseract_scene_graph::GroupNames& group_names = srdf_model_->getGroupNames();
  group_names.erase(std::remove_if(
      group_names.begin(), group_names.end(), [group_name](const std::string& gn) { return gn == group_name; }));
  removeFwdKinematicSolver(group_name);
  removeInvKinematicSolver(group_name);
}

const tesseract_scene_graph::LinkGroup& ManipulatorManager::getLinkGroup(const std::string& group_name) const
{
  return srdf_model_->getLinkGroups().at(group_name);
}

const tesseract_scene_graph::LinkGroups& ManipulatorManager::getLinkGroups() const
{
  return srdf_model_->getLinkGroups();
}

bool ManipulatorManager::addROPKinematicsSolver(const std::string& group_name,
                                                const tesseract_scene_graph::ROPKinematicParameters& rop_group)
{
  if (hasGroup(group_name))
  {
    CONSOLE_BRIDGE_logError("ManipulatorManager: Group name is already taken!");
    return false;
  }

  srdf_model_->getGroupROPKinematics()[group_name] = rop_group;
  srdf_model_->getGroupNames().push_back(group_name);
  return registerROPSolver(group_name, rop_group);
}

void ManipulatorManager::removeROPKinematicsSolver(const std::string& group_name)
{
  srdf_model_->getGroupROPKinematics().erase(group_name);
  tesseract_scene_graph::GroupNames& group_names = srdf_model_->getGroupNames();
  group_names.erase(std::remove_if(
      group_names.begin(), group_names.end(), [group_name](const std::string& gn) { return gn == group_name; }));
  removeFwdKinematicSolver(group_name);
  removeInvKinematicSolver(group_name);
}

const tesseract_scene_graph::ROPKinematicParameters&
ManipulatorManager::getROPKinematicsSolver(const std::string& group_name) const
{
  return srdf_model_->getGroupROPKinematics().at(group_name);
}

const tesseract_scene_graph::GroupROPKinematics& ManipulatorManager::getROPKinematicsSolvers() const
{
  return srdf_model_->getGroupROPKinematics();
}

bool ManipulatorManager::addREPKinematicsSolver(const std::string& group_name,
                                                const tesseract_scene_graph::REPKinematicParameters& rep_group)
{
  if (hasGroup(group_name))
  {
    CONSOLE_BRIDGE_logError("ManipulatorManager: Group name is already taken!");
    return false;
  }

  srdf_model_->getGroupREPKinematics()[group_name] = rep_group;
  srdf_model_->getGroupNames().push_back(group_name);
  return registerREPSolver(group_name, rep_group);
}

void ManipulatorManager::removeREPKinematicsSolver(const std::string& group_name)
{
  srdf_model_->getGroupREPKinematics().erase(group_name);
  tesseract_scene_graph::GroupNames& group_names = srdf_model_->getGroupNames();
  group_names.erase(std::remove_if(
      group_names.begin(), group_names.end(), [group_name](const std::string& gn) { return gn == group_name; }));
  removeFwdKinematicSolver(group_name);
  removeInvKinematicSolver(group_name);
}

const tesseract_scene_graph::REPKinematicParameters&
ManipulatorManager::getREPKinematicsSolver(const std::string& group_name) const
{
  return srdf_model_->getGroupREPKinematics().at(group_name);
}

const tesseract_scene_graph::GroupREPKinematics& ManipulatorManager::getREPKinematicsSolvers() const
{
  return srdf_model_->getGroupREPKinematics();
}

bool ManipulatorManager::addOPWKinematicsSolver(const std::string& group_name,
                                                const tesseract_scene_graph::OPWKinematicParameters& opw_params)
{
  if (!hasGroup(group_name))
  {
    CONSOLE_BRIDGE_logError("ManipulatorManager: Tried to add OPW Kinematics Solver for group that does not exist!");
    return false;
  }

  srdf_model_->getGroupOPWKinematics()[group_name] = opw_params;
  return registerOPWSolver(group_name, opw_params);
}

void ManipulatorManager::removeOPWKinematicsSovler(const std::string& group_name)
{
  srdf_model_->getGroupOPWKinematics().erase(group_name);
}

const tesseract_scene_graph::OPWKinematicParameters&
ManipulatorManager::getOPWKinematicsSolver(const std::string& group_name) const
{
  return srdf_model_->getGroupOPWKinematics().at(group_name);
}

const tesseract_scene_graph::GroupOPWKinematics& ManipulatorManager::getOPWKinematicsSolvers() const
{
  return srdf_model_->getGroupOPWKinematics();
}

bool ManipulatorManager::addGroupJointState(const std::string& group_name,
                                            const std::string& state_name,
                                            const tesseract_scene_graph::GroupsJointState& joint_state)
{
  if (!hasGroup(group_name))
  {
    CONSOLE_BRIDGE_logError("ManipulatorManager: Tried to add Group State for group that does not exist!");
    return false;
  }

  srdf_model_->getGroupStates()[group_name][state_name] = joint_state;
  // @TODO should check if within limits
  return true;
}

void ManipulatorManager::removeGroupJointState(const std::string& group_name, const std::string& state_name)
{
  srdf_model_->getGroupStates()[group_name].erase(state_name);

  if (srdf_model_->getGroupStates()[group_name].empty())
    srdf_model_->getGroupStates().erase(group_name);
}

const tesseract_scene_graph::GroupsJointState&
ManipulatorManager::getGroupsJointState(const std::string& group_name, const std::string& state_name) const
{
  return srdf_model_->getGroupStates().at(group_name).at(state_name);
}

const tesseract_scene_graph::GroupsJointStates&
ManipulatorManager::getGroupsJointStates(const std::string& group_name) const
{
  return srdf_model_->getGroupStates().at(group_name);
}

const tesseract_scene_graph::GroupJointStates& ManipulatorManager::getGroupJointStates() const
{
  return srdf_model_->getGroupStates();
}

bool ManipulatorManager::addGroupTCP(const std::string& group_name,
                                     const std::string& tcp_name,
                                     const Eigen::Isometry3d& tcp)
{
  if (!hasGroup(group_name))
  {
    CONSOLE_BRIDGE_logError("ManipulatorManager: Tried to add Group TCP for group that does not exist!");
    return false;
  }

  srdf_model_->getGroupTCPs()[group_name][tcp_name] = tcp;
  return true;
}

void ManipulatorManager::removeGroupTCP(const std::string& group_name, const std::string& tcp_name)
{
  srdf_model_->getGroupTCPs().at(group_name).at(tcp_name);

  if (srdf_model_->getGroupTCPs()[group_name].empty())
    srdf_model_->getGroupTCPs().erase(group_name);
}

const Eigen::Isometry3d& ManipulatorManager::getGroupsTCP(const std::string& group_name,
                                                          const std::string& tcp_name) const
{
  return srdf_model_->getGroupTCPs().at(group_name).at(tcp_name);
}

const tesseract_scene_graph::GroupsTCPs& ManipulatorManager::getGroupsTCPs(const std::string& group_name) const
{
  return srdf_model_->getGroupTCPs().at(group_name);
}

const tesseract_scene_graph::GroupTCPs& ManipulatorManager::getGroupTCPs() const { return srdf_model_->getGroupTCPs(); }

bool ManipulatorManager::hasGroupTCP(const std::string& group_name, const std::string& tcp_name) const
{
  auto group_it = srdf_model_->getGroupTCPs().find(group_name);
  if (group_it == srdf_model_->getGroupTCPs().end())
    return false;

  auto tcp_it = group_it->second.find(tcp_name);
  if (tcp_it == group_it->second.end())
    return false;

  return true;
}

// This is exposed for the SRDF editor should not use in normal applications
void ManipulatorManager::addAllowedCollision(const std::string& link_1,
                                             const std::string& link_2,
                                             const std::string& reason)
{
  srdf_model_->getAllowedCollisionMatrix().addAllowedCollision(link_1, link_2, reason);
}

void ManipulatorManager::removeAllowedCollision(const std::string& link_1, const std::string& link_2)
{
  srdf_model_->getAllowedCollisionMatrix().removeAllowedCollision(link_1, link_2);
}

const tesseract_scene_graph::AllowedCollisionMatrix& ManipulatorManager::getAllowedCollisionMatrix() const
{
  return srdf_model_->getAllowedCollisionMatrix();
}

void ManipulatorManager::clearAllowedCollisions() { srdf_model_->getAllowedCollisionMatrix().clearAllowedCollisions(); }

bool ManipulatorManager::registerFwdKinematicsFactory(tesseract_kinematics::ForwardKinematicsFactory::ConstPtr factory)
{
  std::string name = factory->getName();
  if (fwd_kin_factories_.find(name) == fwd_kin_factories_.end())
  {
    fwd_kin_factories_[name] = std::move(factory);
    return true;
  }
  return false;
}

void ManipulatorManager::removeFwdKinematicsFactory(const std::string& name) { fwd_kin_factories_.erase(name); }

std::vector<std::string> ManipulatorManager::getAvailableFwdKinematicsSolvers() const
{
  std::vector<std::string> names;
  names.reserve(fwd_kin_factories_.size());
  for (const auto& factory : fwd_kin_factories_)
    names.push_back(factory.first);

  return names;
}

std::vector<std::string>
ManipulatorManager::getAvailableFwdKinematicsSolvers(tesseract_kinematics::ForwardKinematicsFactoryType type) const
{
  std::vector<std::string> names;
  names.reserve(fwd_kin_factories_.size());
  for (const auto& factory : fwd_kin_factories_)
    if (factory.second->getType() == type)
      names.push_back(factory.first);

  return names;
}

tesseract_kinematics::ForwardKinematicsFactory::ConstPtr
ManipulatorManager::getFwdKinematicFactory(const std::string& name) const
{
  auto it = fwd_kin_factories_.find(name);
  if (it != fwd_kin_factories_.end())
    return it->second;

  return nullptr;
}

bool ManipulatorManager::addFwdKinematicSolver(const tesseract_kinematics::ForwardKinematics::ConstPtr& solver)
{
  auto it = fwd_kin_manipulators_.find(std::make_pair(solver->getName(), solver->getSolverName()));
  if (it != fwd_kin_manipulators_.end())
    return false;

  fwd_kin_manipulators_[std::make_pair(solver->getName(), solver->getSolverName())] = solver;

  // If default solver does not exist for this manipulator set this solver as the default.
  auto it2 = fwd_kin_manipulators_default_.find(solver->getName());
  if (it2 == fwd_kin_manipulators_default_.end())
    fwd_kin_manipulators_default_[solver->getName()] = solver;

  return true;
}

void ManipulatorManager::removeFwdKinematicSolver(const std::string& manipulator, const std::string& name)
{
  fwd_kin_manipulators_.erase(std::make_pair(manipulator, name));
}

void ManipulatorManager::removeFwdKinematicSolver(const std::string& manipulator)
{
  auto it = fwd_kin_manipulators_.begin();
  while (it != fwd_kin_manipulators_.end())
  {
    if (it->first.first == manipulator)
      it = fwd_kin_manipulators_.erase(it);
    else
      ++it;
  }

  fwd_kin_manipulators_default_.erase(manipulator);
}

std::vector<std::string> ManipulatorManager::getAvailableFwdKinematicsManipulators() const
{
  std::vector<std::string> names;
  names.reserve(fwd_kin_manipulators_default_.size());
  for (const auto& manip : fwd_kin_manipulators_default_)
    names.push_back(manip.first);

  return names;
}

bool ManipulatorManager::setDefaultFwdKinematicSolver(const std::string& manipulator, const std::string& name)
{
  auto it = fwd_kin_manipulators_.find(std::make_pair(manipulator, name));
  if (it == fwd_kin_manipulators_.end())
    return false;

  fwd_kin_manipulators_default_[manipulator] = it->second;

  return true;
}

tesseract_kinematics::ForwardKinematics::Ptr ManipulatorManager::getFwdKinematicSolver(const std::string& manipulator,
                                                                                       const std::string& name) const
{
  auto it = fwd_kin_manipulators_.find(std::make_pair(manipulator, name));
  if (it != fwd_kin_manipulators_.end())
    return it->second->clone();

  return nullptr;
}

tesseract_kinematics::ForwardKinematics::Ptr
ManipulatorManager::getFwdKinematicSolver(const std::string& manipulator) const
{
  auto it = fwd_kin_manipulators_default_.find(manipulator);
  if (it != fwd_kin_manipulators_default_.end())
    return it->second->clone();

  return nullptr;
}

bool ManipulatorManager::registerInvKinematicsFactory(tesseract_kinematics::InverseKinematicsFactory::ConstPtr factory)
{
  std::string name = factory->getName();
  if (inv_kin_factories_.find(name) == inv_kin_factories_.end())
  {
    inv_kin_factories_[name] = std::move(factory);
    return true;
  }
  return false;
}

void ManipulatorManager::removeInvKinematicsFactory(const std::string& name) { inv_kin_factories_.erase(name); }

std::vector<std::string> ManipulatorManager::getAvailableInvKinematicsSolvers() const
{
  std::vector<std::string> names;
  names.reserve(inv_kin_factories_.size());
  for (const auto& factory : inv_kin_factories_)
    names.push_back(factory.first);

  return names;
}

std::vector<std::string>
ManipulatorManager::getAvailableInvKinematicsSolvers(tesseract_kinematics::InverseKinematicsFactoryType type) const
{
  std::vector<std::string> names;
  names.reserve(inv_kin_factories_.size());
  for (const auto& factory : inv_kin_factories_)
    if (factory.second->getType() == type)
      names.push_back(factory.first);

  return names;
}

tesseract_kinematics::InverseKinematicsFactory::ConstPtr
ManipulatorManager::getInvKinematicFactory(const std::string& name) const
{
  auto it = inv_kin_factories_.find(name);
  if (it != inv_kin_factories_.end())
    return it->second;

  return nullptr;
}

bool ManipulatorManager::addInvKinematicSolver(const tesseract_kinematics::InverseKinematics::ConstPtr& solver)
{
  auto it = inv_kin_manipulators_.find(std::make_pair(solver->getName(), solver->getSolverName()));
  if (it != inv_kin_manipulators_.end())
    return false;

  inv_kin_manipulators_[std::make_pair(solver->getName(), solver->getSolverName())] = solver;

  // If default solver does not exist for this manipulator set this solver as the default.
  auto it2 = inv_kin_manipulators_default_.find(solver->getName());
  if (it2 == inv_kin_manipulators_default_.end())
    inv_kin_manipulators_default_[solver->getName()] = solver;

  return true;
}

void ManipulatorManager::removeInvKinematicSolver(const std::string& manipulator, const std::string& name)
{
  inv_kin_manipulators_.erase(std::make_pair(manipulator, name));
}

void ManipulatorManager::removeInvKinematicSolver(const std::string& manipulator)
{
  auto it = inv_kin_manipulators_.begin();
  while (it != inv_kin_manipulators_.end())
  {
    if (it->first.first == manipulator)
      it = inv_kin_manipulators_.erase(it);
    else
      ++it;
  }

  inv_kin_manipulators_default_.erase(manipulator);
}

std::vector<std::string> ManipulatorManager::getAvailableInvKinematicsManipulators() const
{
  std::vector<std::string> names;
  names.reserve(inv_kin_manipulators_default_.size());
  for (const auto& manip : inv_kin_manipulators_default_)
    names.push_back(manip.first);

  return names;
}

bool ManipulatorManager::setDefaultInvKinematicSolver(const std::string& manipulator, const std::string& name)
{
  auto it = inv_kin_manipulators_.find(std::make_pair(manipulator, name));
  if (it == inv_kin_manipulators_.end())
    return false;

  inv_kin_manipulators_default_[manipulator] = it->second;

  return true;
}

tesseract_kinematics::InverseKinematics::Ptr ManipulatorManager::getInvKinematicSolver(const std::string& manipulator,
                                                                                       const std::string& name) const
{
  auto it = inv_kin_manipulators_.find(std::make_pair(manipulator, name));
  if (it != inv_kin_manipulators_.end())
    return it->second->clone();

  return nullptr;
}

tesseract_kinematics::InverseKinematics::Ptr
ManipulatorManager::getInvKinematicSolver(const std::string& manipulator) const
{
  auto it = inv_kin_manipulators_default_.find(manipulator);
  if (it != inv_kin_manipulators_default_.end())
    return it->second->clone();

  return nullptr;
}

bool ManipulatorManager::registerDefaultChainSolver(const std::string& group_name,
                                                    const tesseract_scene_graph::ChainGroup& chain_group)
{
  if (chain_group.empty())
    return false;

  auto fwd_solver = fwd_kin_chain_default_factory_->create(scene_graph_, chain_group, group_name);
  if (fwd_solver == nullptr)
  {
    CONSOLE_BRIDGE_logError("Failed to create forward kinematic chain solver %s for manipulator %s!",
                            fwd_solver->getSolverName().c_str(),
                            group_name.c_str());
    return false;
  }

  if (!addFwdKinematicSolver(fwd_solver))
  {
    CONSOLE_BRIDGE_logError("Failed to add forward kinematic chain solver %s for manipulator %s to manager!",
                            fwd_solver->getSolverName().c_str(),
                            group_name.c_str());
    return false;
  }

  auto inv_solver = inv_kin_chain_default_factory_->create(scene_graph_, chain_group, group_name);
  if (inv_solver == nullptr)
  {
    CONSOLE_BRIDGE_logError("Failed to create inverse kinematic chain solver %s for manipulator %s!",
                            inv_solver->getSolverName().c_str(),
                            group_name.c_str());
    return false;
  }

  if (!addInvKinematicSolver(inv_solver))
  {
    CONSOLE_BRIDGE_logError("Failed to add inverse kinematic chain solver %s for manipulator %s to manager!",
                            inv_solver->getSolverName().c_str(),
                            group_name.c_str());
    return false;
  }

  return true;
}

bool ManipulatorManager::registerDefaultJointSolver(const std::string& group_name,
                                                    const tesseract_scene_graph::JointGroup& joint_group)
{
  if (joint_group.empty())
    return false;

  auto solver = fwd_kin_tree_default_factory_->create(scene_graph_, joint_group, group_name);
  if (solver == nullptr)
  {
    CONSOLE_BRIDGE_logError("Failed to create inverse kinematic tree solver %s for manipulator %s!",
                            solver->getSolverName().c_str(),
                            group_name.c_str());
    return false;
  }

  if (!addFwdKinematicSolver(solver))
  {
    CONSOLE_BRIDGE_logError("Failed to add inverse kinematic tree solver %s for manipulator %s to manager!",
                            solver->getSolverName().c_str(),
                            group_name.c_str());
    return false;
  }

  CONSOLE_BRIDGE_logWarn("Joint groups are currently not supported by inverse kinematics!");
  return true;
}

bool ManipulatorManager::registerDefaultLinkSolver(const std::string& /*group_name*/,
                                                   const tesseract_scene_graph::LinkGroup& /*joint_group*/)
{
  CONSOLE_BRIDGE_logError("Link groups are currently not supported!");
  return false;
}

bool ManipulatorManager::registerOPWSolver(const std::string& group_name,
                                           const tesseract_scene_graph::OPWKinematicParameters& opw_params)
{
  tesseract_kinematics::ForwardKinematics::Ptr fwd_kin = getFwdKinematicSolver(group_name);
  if (fwd_kin == nullptr)
  {
    CONSOLE_BRIDGE_logError("Failed to add inverse kinematic opw solver for manipulator %s to manager!",
                            group_name.c_str());
    return false;
  }

  opw_kinematics::Parameters<double> params;
  params.a1 = opw_params.a1;
  params.a2 = opw_params.a2;
  params.b = opw_params.b;
  params.c1 = opw_params.c1;
  params.c2 = opw_params.c2;
  params.c3 = opw_params.c3;
  params.c4 = opw_params.c4;
  for (std::size_t i = 0; i < 6; ++i)
  {
    params.offsets[i] = opw_params.offsets[i];
    params.sign_corrections[i] = opw_params.sign_corrections[i];
  }

  auto solver = std::make_shared<tesseract_kinematics::OPWInvKin>();
  solver->init(group_name,
               params,
               fwd_kin->getBaseLinkName(),
               fwd_kin->getTipLinkName(),
               fwd_kin->getJointNames(),
               fwd_kin->getLinkNames(),
               fwd_kin->getActiveLinkNames(),
               fwd_kin->getLimits());

  if (!solver->checkInitialized())
  {
    CONSOLE_BRIDGE_logError("Failed to create inverse kinematic opw solver for manipulator %s!", group_name.c_str());
    return false;
  }

  if (!addInvKinematicSolver(solver))
  {
    CONSOLE_BRIDGE_logError("Failed to add inverse kinematic opw solver for manipulator %s to manager!",
                            group_name.c_str());
    return false;
  }

  // Automatically set OPW Inverse Kinematics as the default for the manipulator
  setDefaultInvKinematicSolver(solver->getName(), solver->getSolverName());

  return true;
}

bool ManipulatorManager::registerROPSolver(const std::string& group_name,
                                           const tesseract_scene_graph::ROPKinematicParameters& rop_group)
{
  tesseract_kinematics::ForwardKinematics::Ptr fwd_kin = getFwdKinematicSolver(group_name);
  if (fwd_kin == nullptr)
  {
    CONSOLE_BRIDGE_logError("Failed to add inverse kinematic ROP solver for manipulator %s to manager!",
                            group_name.c_str());
    return false;
  }

  tesseract_kinematics::InverseKinematics::Ptr manip_ik_solver =
      getInvKinematicSolver(rop_group.manipulator_group, rop_group.manipulator_ik_solver);

  tesseract_kinematics::ForwardKinematics::Ptr positioner_fk_solver;
  if (rop_group.positioner_fk_solver.empty())
    positioner_fk_solver = getFwdKinematicSolver(rop_group.positioner_group);
  else
    positioner_fk_solver = getFwdKinematicSolver(rop_group.positioner_group, rop_group.positioner_fk_solver);

  // Extract Sampling resolutions
  const std::vector<std::string>& positioner_joints = positioner_fk_solver->getJointNames();
  Eigen::VectorXd positioner_sample_resolution(positioner_fk_solver->numJoints());
  for (std::size_t i = 0; i < positioner_joints.size(); ++i)
  {
    auto it = rop_group.positioner_sample_resolution.find(positioner_joints[i]);
    if (it == rop_group.positioner_sample_resolution.end())
    {
      CONSOLE_BRIDGE_logError("ManipulatorManager: Missing sampling resolution for joint: %s!",
                              positioner_joints[i].c_str());
      return false;
    }

    positioner_sample_resolution(static_cast<long>(i)) = it->second;
  }

  auto solver = std::make_shared<tesseract_kinematics::RobotOnPositionerInvKin>();
  if (!solver->init(scene_graph_,
                    manip_ik_solver,
                    rop_group.manipulator_reach,
                    positioner_fk_solver,
                    positioner_sample_resolution,
                    group_name))
    return false;

  if (!addInvKinematicSolver(solver))
  {
    CONSOLE_BRIDGE_logError("Failed to add inverse kinematic ROP solver for manipulator %s to manager!",
                            group_name.c_str());
    return false;
  }

  // Automatically set ROP Inverse Kinematics as the default for the manipulator
  setDefaultInvKinematicSolver(solver->getName(), solver->getSolverName());

  return true;
}

bool ManipulatorManager::registerREPSolver(const std::string& group_name,
                                           const tesseract_scene_graph::REPKinematicParameters& rep_group)
{
  tesseract_kinematics::ForwardKinematics::Ptr fwd_kin = getFwdKinematicSolver(group_name);
  if (fwd_kin == nullptr)
  {
    CONSOLE_BRIDGE_logError("Failed to add inverse kinematic REP solver for manipulator %s to manager!",
                            group_name.c_str());
    return false;
  }

  tesseract_kinematics::InverseKinematics::Ptr manip_ik_solver =
      getInvKinematicSolver(rep_group.manipulator_group, rep_group.manipulator_ik_solver);

  tesseract_kinematics::ForwardKinematics::Ptr positioner_fk_solver;
  if (rep_group.positioner_fk_solver.empty())
    positioner_fk_solver = getFwdKinematicSolver(rep_group.positioner_group);
  else
    positioner_fk_solver = getFwdKinematicSolver(rep_group.positioner_group, rep_group.positioner_fk_solver);

  // Extract Sampling resolutions
  const std::vector<std::string>& positioner_joints = positioner_fk_solver->getJointNames();
  Eigen::VectorXd positioner_sample_resolution(positioner_fk_solver->numJoints());
  for (std::size_t i = 0; i < positioner_joints.size(); ++i)
  {
    auto it = rep_group.positioner_sample_resolution.find(positioner_joints[i]);
    if (it == rep_group.positioner_sample_resolution.end())
    {
      CONSOLE_BRIDGE_logError("ManipulatorManager: Missing sampling resolution for joint: %s!",
                              positioner_joints[i].c_str());
      return false;
    }

    positioner_sample_resolution(static_cast<long>(i)) = it->second;
  }

  auto solver = std::make_shared<tesseract_kinematics::RobotWithExternalPositionerInvKin>();
  if (!solver->init(scene_graph_,
                    manip_ik_solver,
                    rep_group.manipulator_reach,
                    positioner_fk_solver,
                    positioner_sample_resolution,
                    env_->getCurrentState()->link_transforms,
                    group_name))
    return false;

  if (!addInvKinematicSolver(solver))
  {
    CONSOLE_BRIDGE_logError("Failed to add inverse kinematic REP solver for manipulator %s to manager!",
                            group_name.c_str());
    return false;
  }

  // Automatically set ROP Inverse Kinematics as the default for the manipulator
  setDefaultInvKinematicSolver(solver->getName(), solver->getSolverName());

  return true;
}

}  // namespace tesseract
