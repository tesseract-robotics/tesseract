/**
 * @file environment.cpp
 * @brief Tesseract environment interface implementation.
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

#include <tesseract_environment/environment.h>
#include <tesseract_environment/utils.h>
#include <tesseract_collision/core/common.h>
#include <tesseract_srdf/utils.h>
#include <tesseract_state_solver/ofkt/ofkt_state_solver.h>
#include <tesseract_kinematics/core/validate.h>

TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <queue>
#include <boost/serialization/nvp.hpp>
#include <boost/serialization/shared_ptr.hpp>
#include <boost/serialization/binary_object.hpp>
#include <boost/serialization/vector.hpp>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

namespace tesseract_environment
{
bool Environment::initHelper(const Commands& commands)
{
  if (commands.empty())
    return false;

  if (commands.at(0)->getType() != CommandType::ADD_SCENE_GRAPH)
  {
    CONSOLE_BRIDGE_logError("When initializing environment from command history the first command must be type "
                            "ADD_SCENE_GRAPH!");
    return false;
  }

  clear();

  scene_graph_ = std::make_shared<tesseract_scene_graph::SceneGraph>(
      std::static_pointer_cast<const AddSceneGraphCommand>(commands.at(0))->getSceneGraph()->getName());
  scene_graph_const_ = scene_graph_;

  is_contact_allowed_fn_ = [this](const std::string& l1, const std::string& l2) {
    return scene_graph_->isCollisionAllowed(l1, l2);
  };

  if (!applyCommandsHelper(commands))
  {
    CONSOLE_BRIDGE_logError("When initializing environment from command history, it failed to apply a command!");
    return false;
  }

  initialized_ = true;
  init_revision_ = revision_;

  environmentChanged();

  return initialized_;
}

bool Environment::init(const Commands& commands)
{
  bool success{ false };
  {
    std::unique_lock<std::shared_mutex> lock(mutex_);
    success = initHelper(commands);
  }

  // Call the event callbacks
  std::shared_lock<std::shared_mutex> lock(mutex_);
  triggerEnvironmentChangedCallbacks();
  triggerCurrentStateChangedCallbacks();

  return success;
}

bool Environment::init(const tesseract_scene_graph::SceneGraph& scene_graph,
                       const tesseract_srdf::SRDFModel::ConstPtr& srdf_model)
{
  Commands commands = getInitCommands(scene_graph, srdf_model);
  return init(commands);
}

bool Environment::init(const std::string& urdf_string, const tesseract_common::ResourceLocator::ConstPtr& locator)
{
  resource_locator_ = locator;

  // Parse urdf string into Scene Graph
  tesseract_scene_graph::SceneGraph::Ptr scene_graph;
  try
  {
    scene_graph = tesseract_urdf::parseURDFString(urdf_string, *resource_locator_);
  }
  catch (const std::exception& e)
  {
    CONSOLE_BRIDGE_logError("Failed to parse URDF.");
    tesseract_common::printNestedException(e);
    return false;
  }

  Commands commands = getInitCommands(*scene_graph);
  return init(commands);
}

bool Environment::init(const std::string& urdf_string,
                       const std::string& srdf_string,
                       const tesseract_common::ResourceLocator::ConstPtr& locator)
{
  resource_locator_ = locator;

  // Parse urdf string into Scene Graph
  tesseract_scene_graph::SceneGraph::Ptr scene_graph;
  try
  {
    scene_graph = tesseract_urdf::parseURDFString(urdf_string, *resource_locator_);
  }
  catch (const std::exception& e)
  {
    CONSOLE_BRIDGE_logError("Failed to parse URDF.");
    tesseract_common::printNestedException(e);
    return false;
  }

  // Parse srdf string into SRDF Model
  auto srdf = std::make_shared<tesseract_srdf::SRDFModel>();
  try
  {
    srdf->initString(*scene_graph, srdf_string, *resource_locator_);
  }
  catch (const std::exception& e)
  {
    CONSOLE_BRIDGE_logError("Failed to parse SRDF.");
    tesseract_common::printNestedException(e);
    return false;
  }

  Commands commands = getInitCommands(*scene_graph, srdf);
  return init(commands);
}

bool Environment::init(const tesseract_common::fs::path& urdf_path,
                       const tesseract_common::ResourceLocator::ConstPtr& locator)
{
  resource_locator_ = locator;

  // Parse urdf file into Scene Graph
  tesseract_scene_graph::SceneGraph::Ptr scene_graph;
  try
  {
    scene_graph = tesseract_urdf::parseURDFFile(urdf_path.string(), *resource_locator_);
  }
  catch (const std::exception& e)
  {
    CONSOLE_BRIDGE_logError("Failed to parse URDF.");
    tesseract_common::printNestedException(e);
    return false;
  }

  Commands commands = getInitCommands(*scene_graph);
  return init(commands);
}

bool Environment::init(const tesseract_common::fs::path& urdf_path,
                       const tesseract_common::fs::path& srdf_path,
                       const tesseract_common::ResourceLocator::ConstPtr& locator)
{
  resource_locator_ = locator;

  // Parse urdf file into Scene Graph
  tesseract_scene_graph::SceneGraph::Ptr scene_graph;
  try
  {
    scene_graph = tesseract_urdf::parseURDFFile(urdf_path.string(), *resource_locator_);
  }
  catch (const std::exception& e)
  {
    CONSOLE_BRIDGE_logError("Failed to parse URDF.");
    tesseract_common::printNestedException(e);
    return false;
  }

  // Parse srdf file into SRDF Model
  auto srdf = std::make_shared<tesseract_srdf::SRDFModel>();
  try
  {
    srdf->initFile(*scene_graph, srdf_path.string(), *resource_locator_);
  }
  catch (const std::exception& e)
  {
    CONSOLE_BRIDGE_logError("Failed to parse SRDF.");
    tesseract_common::printNestedException(e);
    return false;
  }

  Commands commands = getInitCommands(*scene_graph, srdf);
  return init(commands);
}

bool Environment::reset()
{
  bool success{ false };
  {
    std::unique_lock<std::shared_mutex> lock(mutex_);

    Commands init_command;
    if (commands_.empty() || !initialized_)
      return false;

    init_command.reserve(static_cast<std::size_t>(init_revision_));
    for (std::size_t i = 0; i < static_cast<std::size_t>(init_revision_); ++i)
      init_command.push_back(commands_[i]);

    success = initHelper(init_command);
  }

  // Call the event callbacks
  std::shared_lock<std::shared_mutex> lock(mutex_);
  triggerCurrentStateChangedCallbacks();
  triggerEnvironmentChangedCallbacks();

  return success;
}

void Environment::clear()
{
  initialized_ = false;
  revision_ = 0;
  init_revision_ = 0;
  scene_graph_ = nullptr;
  scene_graph_const_ = nullptr;
  state_solver_ = nullptr;
  commands_.clear();
  kinematics_information_.clear();
  collision_margin_data_ = tesseract_collision::CollisionMarginData();
}

Commands Environment::getInitCommands(const tesseract_scene_graph::SceneGraph& scene_graph,
                                      const tesseract_srdf::SRDFModel::ConstPtr& srdf_model)
{
  Commands commands;

  tesseract_scene_graph::SceneGraph::Ptr local_sg = scene_graph.clone();
  if (local_sg == nullptr)
  {
    CONSOLE_BRIDGE_logError("Null pointer to Scene Graph");
    return Commands();
  }

  if (!local_sg->getLink(local_sg->getRoot()))
  {
    CONSOLE_BRIDGE_logError("The scene graph has an invalid root.");
    return Commands();
  }

  if (srdf_model != nullptr)
    tesseract_srdf::processSRDFAllowedCollisions(*local_sg, *srdf_model);

  commands.push_back(std::make_shared<AddSceneGraphCommand>(*local_sg));

  if (srdf_model)
  {
    commands.push_back(std::make_shared<AddContactManagersPluginInfoCommand>(srdf_model->contact_managers_plugin_info));
    commands.push_back(std::make_shared<AddKinematicsInformationCommand>(srdf_model->kinematics_information));

    // Apply calibration information
    for (const auto& cal : srdf_model->calibration_info.joints)
      commands.push_back(std::make_shared<ChangeJointOriginCommand>(cal.first, cal.second));

    // Check srdf for collision margin data
    if (srdf_model->collision_margin_data)
      commands.push_back(std::make_shared<ChangeCollisionMarginsCommand>(
          *srdf_model->collision_margin_data, tesseract_common::CollisionMarginOverrideType::REPLACE));
  }

  return commands;
}

bool Environment::isInitialized() const
{
  std::shared_lock<std::shared_mutex> lock(mutex_);
  return initialized_;
}

int Environment::getRevision() const
{
  std::shared_lock<std::shared_mutex> lock(mutex_);
  return revision_;
}

Commands Environment::getCommandHistory() const
{
  std::shared_lock<std::shared_mutex> lock(mutex_);
  return commands_;
}

bool Environment::applyCommands(const Commands& commands)
{
  bool success{ false };
  {
    std::unique_lock<std::shared_mutex> lock(mutex_);
    success = applyCommandsHelper(commands);
  }
  std::shared_lock<std::shared_mutex> lock(mutex_);
  triggerEnvironmentChangedCallbacks();
  triggerCurrentStateChangedCallbacks();

  return success;
}

bool Environment::applyCommand(Command::ConstPtr command) { return applyCommands({ std::move(command) }); }

tesseract_scene_graph::SceneGraph::ConstPtr Environment::getSceneGraph() const { return scene_graph_const_; }

std::vector<std::string> Environment::getGroupJointNames(const std::string& group_name) const
{
  std::shared_lock<std::shared_mutex> lock(mutex_);
  auto it =
      std::find(kinematics_information_.group_names.begin(), kinematics_information_.group_names.end(), group_name);

  if (it == kinematics_information_.group_names.end())
    throw std::runtime_error("Environment, Joint group '" + group_name + "' does not exist!");

  std::unique_lock<std::shared_mutex> cache_lock(group_joint_names_cache_mutex_);
  auto cache_it = group_joint_names_cache_.find(group_name);
  if (cache_it != group_joint_names_cache_.end())
  {
    CONSOLE_BRIDGE_logDebug("Environment, getGroupJointNames(%s) cache hit!", group_name.c_str());
    return cache_it->second;
  }

  CONSOLE_BRIDGE_logDebug("Environment, getGroupJointNames(%s) cache miss!", group_name.c_str());
  auto chain_it = kinematics_information_.chain_groups.find(group_name);
  if (chain_it != kinematics_information_.chain_groups.end())
  {
    if (chain_it->second.size() > 1)
      throw std::runtime_error("Environment, Groups with multiple chains is not supported!");

    tesseract_scene_graph::ShortestPath path =
        scene_graph_const_->getShortestPath(chain_it->second.begin()->first, chain_it->second.begin()->second);

    group_joint_names_cache_[group_name] = path.active_joints;
    return path.active_joints;
  }

  auto joint_it = kinematics_information_.joint_groups.find(group_name);
  if (joint_it != kinematics_information_.joint_groups.end())
  {
    group_joint_names_cache_[group_name] = joint_it->second;
    return joint_it->second;
  }

  auto link_it = kinematics_information_.link_groups.find(group_name);
  if (link_it != kinematics_information_.link_groups.end())
    throw std::runtime_error("Environment, Link groups are currently not supported!");

  throw std::runtime_error("Environment, failed to get group '" + group_name + "' joint names!");
}

tesseract_kinematics::JointGroup::UPtr Environment::getJointGroup(const std::string& group_name) const
{
  std::shared_lock<std::shared_mutex> lock(mutex_);

  std::unique_lock<std::shared_mutex> cache_lock(joint_group_cache_mutex_);
  auto it = joint_group_cache_.find(group_name);
  if (it != joint_group_cache_.end())
  {
    CONSOLE_BRIDGE_logDebug("Environment, getJointGroup(%s) cache hit!", group_name.c_str());
    return std::make_unique<tesseract_kinematics::JointGroup>(*it->second);
  }

  CONSOLE_BRIDGE_logDebug("Environment, getJointGroup(%s) cache miss!", group_name.c_str());
  // Store copy in cache and return
  std::vector<std::string> joint_names = getGroupJointNames(group_name);
  tesseract_kinematics::JointGroup::UPtr jg = getJointGroup(group_name, joint_names);
  joint_group_cache_[group_name] = std::make_unique<tesseract_kinematics::JointGroup>(*jg);

  return jg;
}

tesseract_kinematics::JointGroup::UPtr Environment::getJointGroup(const std::string& name,
                                                                  const std::vector<std::string>& joint_names) const
{
  std::shared_lock<std::shared_mutex> lock(mutex_);
  return std::make_unique<tesseract_kinematics::JointGroup>(name, joint_names, *scene_graph_const_, current_state_);
}

tesseract_kinematics::KinematicGroup::UPtr Environment::getKinematicGroup(const std::string& group_name,
                                                                          std::string ik_solver_name) const
{
  std::shared_lock<std::shared_mutex> lock(mutex_);

  std::unique_lock<std::shared_mutex> cache_lock(kinematic_group_cache_mutex_);
  std::pair<std::string, std::string> key = std::make_pair(group_name, ik_solver_name);
  auto it = kinematic_group_cache_.find(key);
  if (it != kinematic_group_cache_.end())
  {
    CONSOLE_BRIDGE_logDebug(
        "Environment, getKinematicGroup(%s, %s) cache hit!", group_name.c_str(), ik_solver_name.c_str());
    return std::make_unique<tesseract_kinematics::KinematicGroup>(*it->second);
  }

  CONSOLE_BRIDGE_logDebug(
      "Environment, getKinematicGroup(%s, %s) cache miss!", group_name.c_str(), ik_solver_name.c_str());
  std::vector<std::string> joint_names = getGroupJointNames(group_name);

  if (ik_solver_name.empty())
    ik_solver_name = kinematics_factory_.getDefaultInvKinPlugin(group_name);

  tesseract_kinematics::InverseKinematics::UPtr inv_kin =
      kinematics_factory_.createInvKin(group_name, ik_solver_name, *scene_graph_const_, current_state_);

  // TODO add error message
  if (inv_kin == nullptr)
    return nullptr;

  // Store copy in cache and return
  auto kg = std::make_unique<tesseract_kinematics::KinematicGroup>(
      group_name, joint_names, std::move(inv_kin), *scene_graph_const_, current_state_);

  kinematic_group_cache_[key] = std::make_unique<tesseract_kinematics::KinematicGroup>(*kg);

#ifndef NDEBUG
  if (!tesseract_kinematics::checkKinematics(*kg))
  {
    CONSOLE_BRIDGE_logError("Check Kinematics failed. This means that inverse kinematics solution for a pose do not "
                            "match forward kinematics solution. Did you change the URDF recently?");
  }
#endif

  return kg;
}

// NOLINTNEXTLINE
Eigen::Isometry3d Environment::findTCPOffset(const tesseract_common::ManipulatorInfo& manip_info) const
{
  std::shared_lock<std::shared_mutex> lock(mutex_);

  // If it is already an Isometry3d return the offset
  if (manip_info.tcp_offset.index() != 0)
    return std::get<1>(manip_info.tcp_offset);

  // Check if the tcp offset name is a link in the scene, if so throw an exception
  const std::string& tcp_offset_name = std::get<0>(manip_info.tcp_offset);
  if (state_solver_->hasLinkName(tcp_offset_name))
    throw std::runtime_error("The tcp offset name '" + tcp_offset_name +
                             "' should not be an existing link in the scene. Assign it as the tcp_frame instead!");

  // Check Manipulator Manager for TCP
  if (kinematics_information_.hasGroupTCP(manip_info.manipulator, tcp_offset_name))
    return kinematics_information_.group_tcps.at(manip_info.manipulator).at(tcp_offset_name);

  // Check callbacks for TCP Offset
  for (const auto& fn : find_tcp_cb_)
  {
    try
    {
      Eigen::Isometry3d tcp = fn(manip_info);
      return tcp;
    }
    catch (...)
    {
      CONSOLE_BRIDGE_logDebug("User Defined Find TCP Callback Failed!");
    }
  }

  throw std::runtime_error("Could not find tcp by name " + tcp_offset_name + "'!");
}

void Environment::addFindTCPOffsetCallback(const FindTCPOffsetCallbackFn& fn)
{
  std::unique_lock<std::shared_mutex> lock(mutex_);
  find_tcp_cb_.push_back(fn);
}

std::vector<FindTCPOffsetCallbackFn> Environment::getFindTCPOffsetCallbacks() const
{
  std::shared_lock<std::shared_mutex> lock(mutex_);
  return find_tcp_cb_;
}

void Environment::addEventCallback(std::size_t hash, const EventCallbackFn& fn)
{
  std::unique_lock<std::shared_mutex> lock(mutex_);
  event_cb_[hash] = fn;
}

void Environment::removeEventCallback(std::size_t hash)
{
  std::unique_lock<std::shared_mutex> lock(mutex_);
  event_cb_.erase(hash);
}

void Environment::clearEventCallbacks()
{
  std::unique_lock<std::shared_mutex> lock(mutex_);
  event_cb_.clear();
}

std::map<std::size_t, EventCallbackFn> Environment::getEventCallbacks() const
{
  std::shared_lock<std::shared_mutex> lock(mutex_);
  return event_cb_;
}

void Environment::setResourceLocator(tesseract_common::ResourceLocator::ConstPtr locator)
{
  std::unique_lock<std::shared_mutex> lock(mutex_);
  resource_locator_ = std::move(locator);
}

tesseract_common::ResourceLocator::ConstPtr Environment::getResourceLocator() const
{
  std::shared_lock<std::shared_mutex> lock(mutex_);
  return resource_locator_;
}

void Environment::setName(const std::string& name)
{
  std::unique_lock<std::shared_mutex> lock(mutex_);
  scene_graph_->setName(name);
}

const std::string& Environment::getName() const
{
  std::shared_lock<std::shared_mutex> lock(mutex_);
  return scene_graph_->getName();
}

void Environment::setState(const std::unordered_map<std::string, double>& joints)
{
  {
    std::unique_lock<std::shared_mutex> lock(mutex_);
    state_solver_->setState(joints);
    currentStateChanged();
  }

  std::shared_lock<std::shared_mutex> lock;
  triggerCurrentStateChangedCallbacks();
}

void Environment::setState(const std::vector<std::string>& joint_names,
                           const Eigen::Ref<const Eigen::VectorXd>& joint_values)
{
  {
    std::unique_lock<std::shared_mutex> lock(mutex_);
    state_solver_->setState(joint_names, joint_values);
    currentStateChanged();
  }

  std::shared_lock<std::shared_mutex> lock;
  triggerCurrentStateChangedCallbacks();
}

tesseract_scene_graph::SceneState Environment::getState(const std::unordered_map<std::string, double>& joints) const
{
  std::shared_lock<std::shared_mutex> lock(mutex_);
  return state_solver_->getState(joints);
}

tesseract_scene_graph::SceneState Environment::getState(const std::vector<std::string>& joint_names,
                                                        const Eigen::Ref<const Eigen::VectorXd>& joint_values) const
{
  std::shared_lock<std::shared_mutex> lock(mutex_);
  return state_solver_->getState(joint_names, joint_values);
}

tesseract_scene_graph::SceneState Environment::getState() const
{
  std::shared_lock<std::shared_mutex> lock(mutex_);
  return current_state_;
}

std::chrono::system_clock::time_point Environment::getTimestamp() const
{
  std::shared_lock<std::shared_mutex> lock(mutex_);
  return timestamp_;
}

std::chrono::system_clock::time_point Environment::getCurrentStateTimestamp() const
{
  std::shared_lock<std::shared_mutex> lock(mutex_);
  return current_state_timestamp_;
}

tesseract_scene_graph::Link::ConstPtr Environment::getLink(const std::string& name) const
{
  std::shared_lock<std::shared_mutex> lock(mutex_);
  tesseract_scene_graph::Link::ConstPtr link = scene_graph_->getLink(name);
  return link;
}

tesseract_scene_graph::JointLimits::ConstPtr Environment::getJointLimits(const std::string& joint_name) const
{
  std::shared_lock<std::shared_mutex> lock(mutex_);
  return scene_graph_->getJointLimits(joint_name);
}

bool Environment::getLinkCollisionEnabled(const std::string& name) const
{
  std::shared_lock<std::shared_mutex> lock(mutex_);
  return scene_graph_->getLinkCollisionEnabled(name);
}

bool Environment::getLinkVisibility(const std::string& name) const
{
  std::shared_lock<std::shared_mutex> lock(mutex_);
  return scene_graph_->getLinkVisibility(name);
}

tesseract_common::AllowedCollisionMatrix::ConstPtr Environment::getAllowedCollisionMatrix() const
{
  std::shared_lock<std::shared_mutex> lock(mutex_);
  return scene_graph_->getAllowedCollisionMatrix();
}

std::vector<std::string> Environment::getJointNames() const
{
  std::shared_lock<std::shared_mutex> lock(mutex_);
  return state_solver_->getJointNames();
}

std::vector<std::string> Environment::getActiveJointNames() const
{
  std::shared_lock<std::shared_mutex> lock(mutex_);
  return state_solver_->getActiveJointNames();
}

tesseract_scene_graph::Joint::ConstPtr Environment::getJoint(const std::string& name) const
{
  std::shared_lock<std::shared_mutex> lock(mutex_);
  return scene_graph_->getJoint(name);
}

Eigen::VectorXd Environment::getCurrentJointValues() const
{
  std::shared_lock<std::shared_mutex> lock(mutex_);
  Eigen::VectorXd jv;
  std::vector<std::string> active_joint_names = state_solver_->getActiveJointNames();
  jv.resize(static_cast<long int>(active_joint_names.size()));
  for (auto j = 0U; j < active_joint_names.size(); ++j)
    jv(j) = current_state_.joints.at(active_joint_names[j]);

  return jv;
}

Eigen::VectorXd Environment::getCurrentJointValues(const std::vector<std::string>& joint_names) const
{
  std::shared_lock<std::shared_mutex> lock(mutex_);
  Eigen::VectorXd jv;
  jv.resize(static_cast<long int>(joint_names.size()));
  for (auto j = 0U; j < joint_names.size(); ++j)
    jv(j) = current_state_.joints.at(joint_names[j]);

  return jv;
}

std::string Environment::getRootLinkName() const
{
  std::shared_lock<std::shared_mutex> lock(mutex_);
  return scene_graph_->getRoot();
}

std::vector<std::string> Environment::getLinkNames() const
{
  std::shared_lock<std::shared_mutex> lock(mutex_);
  return state_solver_->getLinkNames();
}

std::vector<std::string> Environment::getActiveLinkNames() const
{
  std::shared_lock<std::shared_mutex> lock(mutex_);
  return state_solver_->getActiveLinkNames();
}

std::vector<std::string> Environment::getActiveLinkNames(const std::vector<std::string>& joint_names) const
{
  std::shared_lock<std::shared_mutex> lock(mutex_);
  return scene_graph_const_->getJointChildrenNames(joint_names);
}

std::vector<std::string> Environment::getStaticLinkNames() const
{
  std::shared_lock<std::shared_mutex> lock(mutex_);
  return state_solver_->getStaticLinkNames();
}

std::vector<std::string> Environment::getStaticLinkNames(const std::vector<std::string>& joint_names) const
{
  std::shared_lock<std::shared_mutex> lock(mutex_);
  std::vector<std::string> active_link_names = getActiveLinkNames(joint_names);
  std::vector<std::string> full_link_names = state_solver_->getLinkNames();
  std::vector<std::string> static_link_names;
  static_link_names.reserve(full_link_names.size());

  std::sort(active_link_names.begin(), active_link_names.end());
  std::sort(full_link_names.begin(), full_link_names.end());

  std::set_difference(full_link_names.begin(),
                      full_link_names.end(),
                      active_link_names.begin(),
                      active_link_names.end(),
                      std::inserter(static_link_names, static_link_names.begin()));

  return static_link_names;
}

tesseract_common::VectorIsometry3d Environment::getLinkTransforms() const
{
  std::shared_lock<std::shared_mutex> lock(mutex_);
  return state_solver_->getLinkTransforms();
}

Eigen::Isometry3d Environment::getLinkTransform(const std::string& link_name) const
{
  std::shared_lock<std::shared_mutex> lock(mutex_);
  return state_solver_->getLinkTransform(link_name);
}

Eigen::Isometry3d Environment::getRelativeLinkTransform(const std::string& from_link_name,
                                                        const std::string& to_link_name) const
{
  std::shared_lock<std::shared_mutex> lock(mutex_);
  return state_solver_->getRelativeLinkTransform(from_link_name, to_link_name);
}

tesseract_scene_graph::StateSolver::UPtr Environment::getStateSolver() const
{
  std::shared_lock<std::shared_mutex> lock(mutex_);
  return state_solver_->clone();
}

tesseract_srdf::KinematicsInformation Environment::getKinematicsInformation() const
{
  std::shared_lock<std::shared_mutex> lock(mutex_);
  return kinematics_information_;
}

tesseract_srdf::GroupNames Environment::getGroupNames() const
{
  std::shared_lock<std::shared_mutex> lock(mutex_);
  return kinematics_information_.group_names;
}

tesseract_common::ContactManagersPluginInfo Environment::getContactManagersPluginInfo() const
{
  std::shared_lock<std::shared_mutex> lock(mutex_);
  return contact_managers_plugin_info_;
}

bool Environment::setActiveDiscreteContactManager(const std::string& name)
{
  std::unique_lock<std::shared_mutex> lock(mutex_);
  std::unique_lock<std::shared_mutex> discrete_lock(discrete_manager_mutex_);
  return setActiveDiscreteContactManagerHelper(name);
}

tesseract_collision::DiscreteContactManager::UPtr Environment::getDiscreteContactManager(const std::string& name) const
{
  std::shared_lock<std::shared_mutex> lock(mutex_);
  tesseract_collision::DiscreteContactManager::UPtr manager = getDiscreteContactManagerHelper(name);
  if (manager == nullptr)
  {
    CONSOLE_BRIDGE_logError("Discrete manager with %s does not exist in factory!", name.c_str());
    return nullptr;
  }

  return manager;
}

bool Environment::setActiveContinuousContactManager(const std::string& name)
{
  std::unique_lock<std::shared_mutex> lock(mutex_);
  std::unique_lock<std::shared_mutex> continous_lock(continuous_manager_mutex_);
  return setActiveContinuousContactManagerHelper(name);
}

tesseract_collision::DiscreteContactManager::UPtr Environment::getDiscreteContactManager() const
{
  std::shared_lock<std::shared_mutex> lock(mutex_);
  {  // Clone cached manager if exists
    std::shared_lock<std::shared_mutex> discrete_lock(discrete_manager_mutex_);
    if (discrete_manager_)
      return discrete_manager_->clone();
  }

  {  // Try to create the default plugin
    std::unique_lock<std::shared_mutex> discrete_lock(discrete_manager_mutex_);
    const std::string& name = contact_managers_plugin_info_.discrete_plugin_infos.default_plugin;
    discrete_manager_ = getDiscreteContactManagerHelper(name);
    if (discrete_manager_ == nullptr)
    {
      CONSOLE_BRIDGE_logError("Discrete manager with %s does not exist in factory!", name.c_str());
      return nullptr;
    }
  }

  return discrete_manager_->clone();
}

void Environment::clearCachedDiscreteContactManager() const
{
  std::shared_lock<std::shared_mutex> lock;
  std::unique_lock<std::shared_mutex> discrete_lock(discrete_manager_mutex_);
  discrete_manager_ = nullptr;
}

tesseract_collision::ContinuousContactManager::UPtr Environment::getContinuousContactManager() const
{
  std::shared_lock<std::shared_mutex> lock(mutex_);
  {  // Clone cached manager if exists
    std::shared_lock<std::shared_mutex> continuous_lock(continuous_manager_mutex_);
    if (continuous_manager_)
      return continuous_manager_->clone();
  }

  {  // Try to create the default plugin
    std::unique_lock<std::shared_mutex> continuous_lock(continuous_manager_mutex_);
    const std::string& name = contact_managers_plugin_info_.continuous_plugin_infos.default_plugin;
    continuous_manager_ = getContinuousContactManagerHelper(name);
    if (continuous_manager_ == nullptr)
    {
      CONSOLE_BRIDGE_logError("Continuous manager with %s does not exist in factory!", name.c_str());
      return nullptr;
    }
  }

  return continuous_manager_->clone();
}

void Environment::clearCachedContinuousContactManager() const
{
  std::shared_lock<std::shared_mutex> lock(mutex_);
  std::unique_lock<std::shared_mutex> continuous_lock(continuous_manager_mutex_);
  continuous_manager_ = nullptr;
}

tesseract_collision::ContinuousContactManager::UPtr
Environment::getContinuousContactManager(const std::string& name) const
{
  std::shared_lock<std::shared_mutex> lock(mutex_);
  tesseract_collision::ContinuousContactManager::UPtr manager = getContinuousContactManagerHelper(name);
  if (manager == nullptr)
  {
    CONSOLE_BRIDGE_logError("Continuous manager with %s does not exist in factory!", name.c_str());
    return nullptr;
  }

  return manager;
}

tesseract_common::CollisionMarginData Environment::getCollisionMarginData() const
{
  std::shared_lock<std::shared_mutex> lock(mutex_);
  return collision_margin_data_;
}

std::shared_lock<std::shared_mutex> Environment::lockRead() const
{
  return std::shared_lock<std::shared_mutex>(mutex_);
}

bool Environment::operator==(const Environment& rhs) const
{
  std::shared_lock<std::shared_mutex> lock(mutex_);

  // No need to check everything mainly the items serialized
  bool equal = true;
  equal &= initialized_ == rhs.initialized_;
  equal &= revision_ == rhs.revision_;
  equal &= init_revision_ == rhs.init_revision_;
  equal &= commands_.size() == rhs.commands_.size();
  if (!equal)
    return equal;

  for (std::size_t i = 0; i < commands_.size(); ++i)
  {
    equal &= *(commands_[i]) == *(rhs.commands_[i]);
    if (!equal)
      return equal;
  }

  equal &= current_state_ == rhs.current_state_;
  equal &= timestamp_ == rhs.timestamp_;
  equal &= current_state_timestamp_ == rhs.current_state_timestamp_;

  /** @todo uncomment after serialized */
  //  equal &= is_contact_allowed_fn_ == rhs.is_contact_allowed_fn_;
  //  equal &= find_tcp_cb_ == rhs.find_tcp_cb_;

  return equal;
}
bool Environment::operator!=(const Environment& rhs) const { return !operator==(rhs); }

bool Environment::setActiveDiscreteContactManagerHelper(const std::string& name)
{
  tesseract_collision::DiscreteContactManager::UPtr manager = getDiscreteContactManagerHelper(name);
  if (manager == nullptr)
  {
    std::string msg = "\n  Discrete manager with " + name + " does not exist in factory!\n";
    msg += "    Available Managers:\n";
    for (const auto& m : contact_managers_factory_.getDiscreteContactManagerPlugins())
      msg += "      " + m.first + "\n";

    CONSOLE_BRIDGE_logError(msg.c_str());
    return false;
  }

  contact_managers_plugin_info_.discrete_plugin_infos.default_plugin = name;

  // The calling function should be locking discrete_manager_mutex_
  discrete_manager_ = std::move(manager);

  return true;
}

bool Environment::setActiveContinuousContactManagerHelper(const std::string& name)
{
  tesseract_collision::ContinuousContactManager::UPtr manager = getContinuousContactManagerHelper(name);

  if (manager == nullptr)
  {
    std::string msg = "\n  Continuous manager with " + name + " does not exist in factory!\n";
    msg += "    Available Managers:\n";
    for (const auto& m : contact_managers_factory_.getContinuousContactManagerPlugins())
      msg += "      " + m.first + "\n";

    CONSOLE_BRIDGE_logError(msg.c_str());
    return false;
  }

  contact_managers_plugin_info_.continuous_plugin_infos.default_plugin = name;

  // The calling function should be locking continuous_manager_mutex_
  continuous_manager_ = std::move(manager);

  return true;
}

tesseract_collision::DiscreteContactManager::UPtr
Environment::getDiscreteContactManagerHelper(const std::string& name) const
{
  tesseract_collision::DiscreteContactManager::UPtr manager =
      contact_managers_factory_.createDiscreteContactManager(name);
  if (manager == nullptr)
    return nullptr;

  manager->setIsContactAllowedFn(is_contact_allowed_fn_);
  if (scene_graph_ != nullptr)
  {
    for (const auto& link : scene_graph_->getLinks())
    {
      if (!link->collision.empty())
      {
        tesseract_collision::CollisionShapesConst shapes;
        tesseract_common::VectorIsometry3d shape_poses;
        getCollisionObject(shapes, shape_poses, *link);
        manager->addCollisionObject(link->getName(), 0, shapes, shape_poses, true);
      }
    }

    manager->setActiveCollisionObjects(state_solver_->getActiveLinkNames());
  }

  manager->setCollisionMarginData(collision_margin_data_);

  manager->setCollisionObjectsTransform(current_state_.link_transforms);

  return manager;
}

tesseract_collision::ContinuousContactManager::UPtr
Environment::getContinuousContactManagerHelper(const std::string& name) const
{
  tesseract_collision::ContinuousContactManager::UPtr manager =
      contact_managers_factory_.createContinuousContactManager(name);

  if (manager == nullptr)
    return nullptr;

  manager->setIsContactAllowedFn(is_contact_allowed_fn_);
  if (scene_graph_ != nullptr)
  {
    for (const auto& link : scene_graph_->getLinks())
    {
      if (!link->collision.empty())
      {
        tesseract_collision::CollisionShapesConst shapes;
        tesseract_common::VectorIsometry3d shape_poses;
        getCollisionObject(shapes, shape_poses, *link);
        manager->addCollisionObject(link->getName(), 0, shapes, shape_poses, true);
      }
    }

    manager->setActiveCollisionObjects(state_solver_->getActiveLinkNames());
  }

  manager->setCollisionMarginData(collision_margin_data_);

  std::vector<std::string> active_link_names = state_solver_->getActiveLinkNames();
  for (const auto& tf : current_state_.link_transforms)
  {
    if (std::find(active_link_names.begin(), active_link_names.end(), tf.first) != active_link_names.end())
      manager->setCollisionObjectsTransform(tf.first, tf.second, tf.second);
    else
      manager->setCollisionObjectsTransform(tf.first, tf.second);
  }

  return manager;
}

void Environment::getCollisionObject(tesseract_collision::CollisionShapesConst& shapes,
                                     tesseract_common::VectorIsometry3d& shape_poses,
                                     const tesseract_scene_graph::Link& link)
{
  for (const auto& c : link.collision)
  {
    shapes.push_back(c->geometry);
    shape_poses.push_back(c->origin);
  }
}

void Environment::currentStateChanged()
{
  timestamp_ = std::chrono::system_clock::now();
  current_state_timestamp_ = timestamp_;
  current_state_ = state_solver_->getState();

  std::unique_lock<std::shared_mutex> discrete_lock(discrete_manager_mutex_);
  if (discrete_manager_ != nullptr)
    discrete_manager_->setCollisionObjectsTransform(current_state_.link_transforms);

  std::unique_lock<std::shared_mutex> continuous_lock(continuous_manager_mutex_);
  if (continuous_manager_ != nullptr)
  {
    std::vector<std::string> active_link_names = state_solver_->getActiveLinkNames();
    for (const auto& tf : current_state_.link_transforms)
    {
      if (std::find(active_link_names.begin(), active_link_names.end(), tf.first) != active_link_names.end())
        continuous_manager_->setCollisionObjectsTransform(tf.first, tf.second, tf.second);
      else
        continuous_manager_->setCollisionObjectsTransform(tf.first, tf.second);
    }
  }

  {  // Clear JointGroup and KinematicGroup
    std::unique_lock<std::shared_mutex> jg_lock(joint_group_cache_mutex_);
    std::unique_lock<std::shared_mutex> kg_lock(kinematic_group_cache_mutex_);
    joint_group_cache_.clear();
    kinematic_group_cache_.clear();
  }
}

void Environment::environmentChanged()
{
  timestamp_ = std::chrono::system_clock::now();
  std::vector<std::string> active_link_names = state_solver_->getActiveLinkNames();

  {
    std::unique_lock<std::shared_mutex> discrete_lock(discrete_manager_mutex_);
    if (discrete_manager_ != nullptr)
      discrete_manager_->setActiveCollisionObjects(active_link_names);
  }

  {
    std::unique_lock<std::shared_mutex> continuous_lock(continuous_manager_mutex_);
    if (continuous_manager_ != nullptr)
      continuous_manager_->setActiveCollisionObjects(active_link_names);
  }

  {  // Clear JointGroup, KinematicGroup and GroupJointNames cache
    std::unique_lock<std::shared_mutex> jn_lock(group_joint_names_cache_mutex_);
    group_joint_names_cache_.clear();
  }

  currentStateChanged();
}

void Environment::triggerCurrentStateChangedCallbacks()
{
  if (!event_cb_.empty())
  {
    SceneStateChangedEvent event(current_state_);
    for (const auto& cb : event_cb_)
      cb.second(event);
  }
}

void Environment::triggerEnvironmentChangedCallbacks()
{
  if (!event_cb_.empty())
  {
    CommandAppliedEvent event(commands_, revision_);
    for (const auto& cb : event_cb_)
      cb.second(event);
  }
}

bool Environment::removeLinkHelper(const std::string& name)
{
  if (scene_graph_->getLink(name) == nullptr)
  {
    CONSOLE_BRIDGE_logWarn("Tried to remove link (%s) that does not exist", name.c_str());
    return false;
  }
  std::vector<tesseract_scene_graph::Joint::ConstPtr> joints = scene_graph_->getInboundJoints(name);
  assert(joints.size() <= 1);

  // get child link names to remove
  std::vector<std::string> child_link_names = scene_graph_->getLinkChildrenNames(name);

  scene_graph_->removeLink(name, true);

  std::unique_lock<std::shared_mutex> discrete_lock(discrete_manager_mutex_);
  std::unique_lock<std::shared_mutex> continuous_lock(continuous_manager_mutex_);
  if (discrete_manager_ != nullptr)
    discrete_manager_->removeCollisionObject(name);
  if (continuous_manager_ != nullptr)
    continuous_manager_->removeCollisionObject(name);

  for (const auto& link_name : child_link_names)
  {
    if (discrete_manager_ != nullptr)
      discrete_manager_->removeCollisionObject(link_name);
    if (continuous_manager_ != nullptr)
      continuous_manager_->removeCollisionObject(link_name);
  }

  return true;
}

Environment::UPtr Environment::clone() const
{
  auto cloned_env = std::make_unique<Environment>();

  std::shared_lock<std::shared_mutex> lock(mutex_);
  std::shared_lock<std::shared_mutex> jg_lock(joint_group_cache_mutex_);
  std::shared_lock<std::shared_mutex> kg_lock(kinematic_group_cache_mutex_);
  std::shared_lock<std::shared_mutex> jn_lock(group_joint_names_cache_mutex_);
  std::shared_lock<std::shared_mutex> discrete_lock(discrete_manager_mutex_);
  std::shared_lock<std::shared_mutex> continuous_lock(continuous_manager_mutex_);

  if (!initialized_)
    return cloned_env;

  cloned_env->initialized_ = initialized_;
  cloned_env->init_revision_ = revision_;
  cloned_env->revision_ = revision_;
  cloned_env->commands_ = commands_;
  cloned_env->scene_graph_ = scene_graph_->clone();
  cloned_env->scene_graph_const_ = cloned_env->scene_graph_;
  cloned_env->timestamp_ = timestamp_;
  cloned_env->current_state_ = current_state_;
  cloned_env->current_state_timestamp_ = current_state_timestamp_;

  // There is not dynamic pointer cast for std::unique_ptr
  auto cloned_solver = state_solver_->clone();
  auto* p = dynamic_cast<tesseract_scene_graph::MutableStateSolver*>(cloned_solver.get());
  if (p != nullptr)
    (void)cloned_solver.release();

  cloned_env->state_solver_ = std::unique_ptr<tesseract_scene_graph::MutableStateSolver>(p);
  cloned_env->kinematics_information_ = kinematics_information_;
  cloned_env->kinematics_factory_ = kinematics_factory_;
  cloned_env->find_tcp_cb_ = find_tcp_cb_;
  cloned_env->collision_margin_data_ = collision_margin_data_;

  // Copy cache
  cloned_env->joint_group_cache_.reserve(joint_group_cache_.size());
  for (const auto& c : joint_group_cache_)
    cloned_env->joint_group_cache_[c.first] = (std::make_unique<tesseract_kinematics::JointGroup>(*c.second));

  for (const auto& c : kinematic_group_cache_)
    cloned_env->kinematic_group_cache_[c.first] = (std::make_unique<tesseract_kinematics::KinematicGroup>(*c.second));

  cloned_env->group_joint_names_cache_ = group_joint_names_cache_;

  // NOLINTNEXTLINE
  cloned_env->is_contact_allowed_fn_ = std::bind(&tesseract_scene_graph::SceneGraph::isCollisionAllowed,
                                                 cloned_env->scene_graph_,
                                                 std::placeholders::_1,
                                                 std::placeholders::_2);

  if (discrete_manager_)
  {
    cloned_env->discrete_manager_ = discrete_manager_->clone();
    cloned_env->discrete_manager_->setIsContactAllowedFn(cloned_env->is_contact_allowed_fn_);
  }
  if (continuous_manager_)
  {
    cloned_env->continuous_manager_ = continuous_manager_->clone();
    cloned_env->continuous_manager_->setIsContactAllowedFn(cloned_env->is_contact_allowed_fn_);
  }

  cloned_env->contact_managers_plugin_info_ = contact_managers_plugin_info_;
  cloned_env->contact_managers_factory_ = contact_managers_factory_;

  return cloned_env;
}

bool Environment::applyCommandsHelper(const Commands& commands)
{
  bool success = true;
  for (const auto& command : commands)
  {
    if (!command)
    {
      success = false;
      break;
    }

    switch (command->getType())
    {
      case tesseract_environment::CommandType::ADD_LINK:
      {
        auto cmd = std::static_pointer_cast<const AddLinkCommand>(command);
        success &= applyAddCommand(cmd);
        break;
      }
      case tesseract_environment::CommandType::MOVE_LINK:
      {
        auto cmd = std::static_pointer_cast<const MoveLinkCommand>(command);
        success &= applyMoveLinkCommand(cmd);
        break;
      }
      case tesseract_environment::CommandType::MOVE_JOINT:
      {
        auto cmd = std::static_pointer_cast<const MoveJointCommand>(command);
        success &= applyMoveJointCommand(cmd);
        break;
      }
      case tesseract_environment::CommandType::REMOVE_LINK:
      {
        auto cmd = std::static_pointer_cast<const RemoveLinkCommand>(command);
        success &= applyRemoveLinkCommand(cmd);
        break;
      }
      case tesseract_environment::CommandType::REMOVE_JOINT:
      {
        auto cmd = std::static_pointer_cast<const RemoveJointCommand>(command);
        success &= applyRemoveJointCommand(cmd);
        break;
      }
      case tesseract_environment::CommandType::REPLACE_JOINT:
      {
        auto cmd = std::static_pointer_cast<const ReplaceJointCommand>(command);
        success &= applyReplaceJointCommand(cmd);
        break;
      }
      case tesseract_environment::CommandType::CHANGE_LINK_ORIGIN:
      {
        auto cmd = std::static_pointer_cast<const ChangeLinkOriginCommand>(command);
        success &= applyChangeLinkOriginCommand(cmd);
        break;
      }
      case tesseract_environment::CommandType::CHANGE_JOINT_ORIGIN:
      {
        auto cmd = std::static_pointer_cast<const ChangeJointOriginCommand>(command);
        success &= applyChangeJointOriginCommand(cmd);
        break;
      }
      case tesseract_environment::CommandType::CHANGE_LINK_COLLISION_ENABLED:
      {
        auto cmd = std::static_pointer_cast<const ChangeLinkCollisionEnabledCommand>(command);
        success &= applyChangeLinkCollisionEnabledCommand(cmd);
        break;
      }
      case tesseract_environment::CommandType::CHANGE_LINK_VISIBILITY:
      {
        auto cmd = std::static_pointer_cast<const ChangeLinkVisibilityCommand>(command);
        success &= applyChangeLinkVisibilityCommand(cmd);
        break;
      }
      case tesseract_environment::CommandType::ADD_ALLOWED_COLLISION:
      {
        auto cmd = std::static_pointer_cast<const AddAllowedCollisionCommand>(command);
        success &= applyAddAllowedCollisionCommand(cmd);
        break;
      }
      case tesseract_environment::CommandType::REMOVE_ALLOWED_COLLISION:
      {
        auto cmd = std::static_pointer_cast<const RemoveAllowedCollisionCommand>(command);
        success &= applyRemoveAllowedCollisionCommand(cmd);
        break;
      }
      case tesseract_environment::CommandType::REMOVE_ALLOWED_COLLISION_LINK:
      {
        auto cmd = std::static_pointer_cast<const RemoveAllowedCollisionLinkCommand>(command);
        success &= applyRemoveAllowedCollisionLinkCommand(cmd);
        break;
      }
      case tesseract_environment::CommandType::ADD_SCENE_GRAPH:
      {
        auto cmd = std::static_pointer_cast<const AddSceneGraphCommand>(command);
        success &= applyAddSceneGraphCommand(cmd);
        break;
      }
      case tesseract_environment::CommandType::CHANGE_JOINT_POSITION_LIMITS:
      {
        auto cmd = std::static_pointer_cast<const ChangeJointPositionLimitsCommand>(command);
        success &= applyChangeJointPositionLimitsCommand(cmd);
        break;
      }
      case tesseract_environment::CommandType::CHANGE_JOINT_VELOCITY_LIMITS:
      {
        auto cmd = std::static_pointer_cast<const ChangeJointVelocityLimitsCommand>(command);
        success &= applyChangeJointVelocityLimitsCommand(cmd);
        break;
      }
      case tesseract_environment::CommandType::CHANGE_JOINT_ACCELERATION_LIMITS:
      {
        auto cmd = std::static_pointer_cast<const ChangeJointAccelerationLimitsCommand>(command);
        success &= applyChangeJointAccelerationLimitsCommand(cmd);
        break;
      }
      case tesseract_environment::CommandType::ADD_KINEMATICS_INFORMATION:
      {
        auto cmd = std::static_pointer_cast<const AddKinematicsInformationCommand>(command);
        success &= applyAddKinematicsInformationCommand(cmd);
        break;
      }
      case tesseract_environment::CommandType::CHANGE_COLLISION_MARGINS:
      {
        auto cmd = std::static_pointer_cast<const ChangeCollisionMarginsCommand>(command);
        success &= applyChangeCollisionMarginsCommand(cmd);
        break;
      }
      case tesseract_environment::CommandType::ADD_CONTACT_MANAGERS_PLUGIN_INFO:
      {
        auto cmd = std::static_pointer_cast<const AddContactManagersPluginInfoCommand>(command);
        success &= applyAddContactManagersPluginInfoCommand(cmd);
        break;
      }
      case tesseract_environment::CommandType::SET_ACTIVE_CONTINUOUS_CONTACT_MANAGER:
      {
        auto cmd = std::static_pointer_cast<const SetActiveContinuousContactManagerCommand>(command);
        success &= applySetActiveContinuousContactManagerCommand(cmd);
        break;
      }
      case tesseract_environment::CommandType::SET_ACTIVE_DISCRETE_CONTACT_MANAGER:
      {
        auto cmd = std::static_pointer_cast<const SetActiveDiscreteContactManagerCommand>(command);
        success &= applySetActiveDiscreteContactManagerCommand(cmd);
        break;
      }
      // LCOV_EXCL_START
      default:
      {
        CONSOLE_BRIDGE_logError("Unhandled environment command");
        success &= false;
      }
        // LCOV_EXCL_STOP
    }

    if (!success)
      break;
  }

  // Update the solver revision to match environment
  state_solver_->setRevision(revision_);

  // If this is not true then the initHelper function has called applyCommand so do not call.
  if (initialized_)
    environmentChanged();

  return success;
}

//////////////////////////////////////////////////////////////
//////////////// Internal Apply Command //////////////////////
//////////////////////////////////////////////////////////////

bool Environment::applyAddCommand(AddLinkCommand::ConstPtr cmd)
{
  // The command should not allow this to occur but adding an assert to catch if something changes
  assert(!(!cmd->getLink() && !cmd->getJoint()));
  assert(!((cmd->getLink() != nullptr) && (cmd->getJoint() != nullptr) &&
           (cmd->getJoint()->child_link_name != cmd->getLink()->getName())));

  bool link_exists = false;
  bool joint_exists = false;
  std::string link_name, joint_name;

  if (cmd->getLink() != nullptr)
  {
    link_name = cmd->getLink()->getName();
    link_exists = (scene_graph_->getLink(link_name) != nullptr);
  }

  if (cmd->getJoint() != nullptr)
  {
    joint_name = cmd->getJoint()->getName();
    joint_exists = (scene_graph_->getJoint(joint_name) != nullptr);
  }

  if ((link_exists && !cmd->replaceAllowed()))
  {
    CONSOLE_BRIDGE_logWarn("Tried to add link (%s) which already exists. Set replace_allowed to enable replacing.",
                           link_name.c_str());
    return false;
  }

  if ((joint_exists && !cmd->replaceAllowed()))
  {
    CONSOLE_BRIDGE_logWarn("Tried to replace link (%s) and joint (%s) where the joint exist but the link does not. "
                           "This is not supported.",
                           link_name.c_str(),
                           joint_name.c_str());
    return false;
  }

  if (!link_exists && joint_exists)
  {
    CONSOLE_BRIDGE_logWarn("Tried to add link (%s) which already exists with a joint provided which does not exist. "
                           "This is not supported.",
                           link_name.c_str());
    return false;
  }

  if ((link_exists && cmd->getJoint() && !joint_exists))
  {
    CONSOLE_BRIDGE_logWarn("Tried to add link (%s) which already exists with a joint provided which does not exist. "
                           "This is not supported.",
                           link_name.c_str());
    return false;
  }

  if (link_exists && !cmd->getJoint())
  {  // A link is being replaced
    if (!scene_graph_->addLink(*cmd->getLink(), true))
      return false;

    // Solver is not affected by replace links
  }
  else if (link_exists && joint_exists)
  {  // A link and joint pair is being replaced
    tesseract_scene_graph::Link::ConstPtr orig_link = scene_graph_->getLink(link_name);
    tesseract_scene_graph::Joint::ConstPtr orig_joint = scene_graph_->getJoint(joint_name);

    if (orig_joint->child_link_name != orig_link->getName())
    {
      CONSOLE_BRIDGE_logWarn("Tried to replace link (%s) and joint (%s) which are currently not linked. This is not "
                             "supported.",
                             link_name.c_str(),
                             joint_name.c_str());
      return false;
    }

    if (!scene_graph_->addLink(*cmd->getLink(), true))
      return false;

    if (!scene_graph_->removeJoint(joint_name))
    {
      // Replace with original link
      if (!scene_graph_->addLink(*orig_link, true))
        throw std::runtime_error("Environment: Failed to replace link and joint and reset to original state.");

      return false;
    }

    if (!scene_graph_->addJoint(*cmd->getJoint()))
    {
      // Replace with original link
      if (!scene_graph_->addLink(*orig_link, true))
        throw std::runtime_error("Environment: Failed to replace link and joint reset to original state.");

      // Replace with original link
      if (!scene_graph_->addJoint(*orig_joint))
        throw std::runtime_error("Environment: Failed to replace link and joint and reset to original state.");

      return false;
    }

    if (!state_solver_->replaceJoint(*cmd->getJoint()))
      throw std::runtime_error("Environment, failed to replace link and joint in state solver.");
  }
  else if (!link_exists && !cmd->getJoint())
  {  // Add a new link is being added attached to the world
    std::string joint_name = "joint_" + link_name;
    tesseract_scene_graph::Joint joint(joint_name);
    joint.type = tesseract_scene_graph::JointType::FIXED;
    joint.child_link_name = link_name;
    joint.parent_link_name = scene_graph_->getRoot();

    tesseract_scene_graph::Link::ConstPtr link = cmd->getLink();
    cmd = std::make_shared<AddLinkCommand>(*link, joint);

    if (!scene_graph_->addLink(*cmd->getLink(), *cmd->getJoint()))
      return false;

    if (!state_solver_->addLink(*cmd->getLink(), *cmd->getJoint()))
      throw std::runtime_error("Environment, failed to add link and joint in state solver.");
  }
  else
  {  // A new link and joint is being added
    if (!scene_graph_->addLink(*cmd->getLink(), *cmd->getJoint()))
      return false;

    if (!state_solver_->addLink(*cmd->getLink(), *cmd->getJoint()))
      throw std::runtime_error("Environment, failed to add link and joint in state solver.");
  }

  // If Link existed remove it from collision before adding the replacing links geometry
  if (link_exists)
  {
    std::unique_lock<std::shared_mutex> discrete_lock(discrete_manager_mutex_);
    if (discrete_manager_ != nullptr)
      discrete_manager_->removeCollisionObject(link_name);

    std::unique_lock<std::shared_mutex> continuous_lock(continuous_manager_mutex_);
    if (continuous_manager_ != nullptr)
      continuous_manager_->removeCollisionObject(link_name);
  }

  // We have moved the original objects, get a pointer to them from scene_graph
  if (!cmd->getLink()->collision.empty())
  {
    tesseract_collision::CollisionShapesConst shapes;
    tesseract_common::VectorIsometry3d shape_poses;
    getCollisionObject(shapes, shape_poses, *cmd->getLink());

    std::unique_lock<std::shared_mutex> discrete_lock(discrete_manager_mutex_);
    if (discrete_manager_ != nullptr)
      discrete_manager_->addCollisionObject(link_name, 0, shapes, shape_poses, true);

    std::unique_lock<std::shared_mutex> continuous_lock(continuous_manager_mutex_);
    if (continuous_manager_ != nullptr)
      continuous_manager_->addCollisionObject(link_name, 0, shapes, shape_poses, true);
  }

  ++revision_;
  commands_.push_back(cmd);

  return true;
}

bool Environment::applyMoveLinkCommand(const MoveLinkCommand::ConstPtr& cmd)
{
  if (!scene_graph_->moveLink(*cmd->getJoint()))
    return false;

  if (!state_solver_->moveLink(*cmd->getJoint()))
    throw std::runtime_error("Environment, failed to move link in state solver.");

  ++revision_;
  commands_.push_back(cmd);

  return true;
}

bool Environment::applyMoveJointCommand(const MoveJointCommand::ConstPtr& cmd)
{
  if (!scene_graph_->moveJoint(cmd->getJointName(), cmd->getParentLink()))
    return false;

  if (!state_solver_->moveJoint(cmd->getJointName(), cmd->getParentLink()))
    throw std::runtime_error("Environment, failed to move joint in state solver.");

  ++revision_;
  commands_.push_back(cmd);

  return true;
}

bool Environment::applyRemoveLinkCommand(const RemoveLinkCommand::ConstPtr& cmd)
{
  if (!removeLinkHelper(cmd->getLinkName()))
    return false;

  if (!state_solver_->removeLink(cmd->getLinkName()))
    throw std::runtime_error("Environment, failed to remove link in state solver.");

  ++revision_;
  commands_.push_back(cmd);

  return true;
}

bool Environment::applyRemoveJointCommand(const RemoveJointCommand::ConstPtr& cmd)
{
  if (scene_graph_->getJoint(cmd->getJointName()) == nullptr)
  {
    CONSOLE_BRIDGE_logWarn("Tried to remove Joint (%s) that does not exist", cmd->getJointName().c_str());
    return false;
  }

  std::string target_link_name = scene_graph_->getTargetLink(cmd->getJointName())->getName();

  if (!removeLinkHelper(target_link_name))
    return false;

  if (!state_solver_->removeJoint(cmd->getJointName()))
    throw std::runtime_error("Environment, failed to remove joint in state solver.");

  ++revision_;
  commands_.push_back(cmd);

  return true;
}

bool Environment::applyReplaceJointCommand(const ReplaceJointCommand::ConstPtr& cmd)
{
  tesseract_scene_graph::Joint::ConstPtr current_joint = scene_graph_->getJoint(cmd->getJoint()->getName());
  if (current_joint == nullptr)
  {
    CONSOLE_BRIDGE_logWarn("Tried to replace Joint (%s) that does not exist", cmd->getJoint()->getName().c_str());
    return false;
  }

  if (cmd->getJoint()->child_link_name != current_joint->child_link_name)
  {
    CONSOLE_BRIDGE_logWarn("Tried to replace Joint (%s) where the child links are not the same",
                           cmd->getJoint()->getName().c_str());
    return false;
  }

  if (!scene_graph_->removeJoint(cmd->getJoint()->getName()))
    return false;

  if (!scene_graph_->addJoint(*cmd->getJoint()))
  {
    // Add old joint back
    if (!scene_graph_->addJoint(*current_joint))
      throw std::runtime_error("Environment: Failed to add old joint back when replace failed!");

    return false;
  }

  if (!state_solver_->replaceJoint(*cmd->getJoint()))
    throw std::runtime_error("Environment, failed to replace joint in state solver.");

  ++revision_;
  commands_.push_back(cmd);

  return true;
}

bool Environment::applyChangeLinkOriginCommand(const ChangeLinkOriginCommand::ConstPtr& /*cmd*/)  // NOLINT
{
  throw std::runtime_error("Unhandled environment command: CHANGE_LINK_ORIGIN");
}

bool Environment::applyChangeJointOriginCommand(const ChangeJointOriginCommand::ConstPtr& cmd)
{
  if (!scene_graph_->changeJointOrigin(cmd->getJointName(), cmd->getOrigin()))
    return false;

  if (!state_solver_->changeJointOrigin(cmd->getJointName(), cmd->getOrigin()))
    throw std::runtime_error("Environment, failed to change joint origin in state solver.");

  ++revision_;
  commands_.push_back(cmd);

  return true;
}

bool Environment::applyChangeLinkCollisionEnabledCommand(const ChangeLinkCollisionEnabledCommand::ConstPtr& cmd)
{
  std::unique_lock<std::shared_mutex> discrete_lock(discrete_manager_mutex_);
  if (discrete_manager_ != nullptr)
  {
    if (cmd->getEnabled())
      discrete_manager_->enableCollisionObject(cmd->getLinkName());
    else
      discrete_manager_->disableCollisionObject(cmd->getLinkName());
  }

  std::unique_lock<std::shared_mutex> continuous_lock(continuous_manager_mutex_);
  if (continuous_manager_ != nullptr)
  {
    if (cmd->getEnabled())
      continuous_manager_->enableCollisionObject(cmd->getLinkName());
    else
      continuous_manager_->disableCollisionObject(cmd->getLinkName());
  }

  scene_graph_->setLinkCollisionEnabled(cmd->getLinkName(), cmd->getEnabled());

  if (scene_graph_->getLinkCollisionEnabled(cmd->getLinkName()) != cmd->getEnabled())
    return false;

  ++revision_;
  commands_.push_back(cmd);

  return true;
}

bool Environment::applyChangeLinkVisibilityCommand(const ChangeLinkVisibilityCommand::ConstPtr& cmd)
{
  scene_graph_->setLinkVisibility(cmd->getLinkName(), cmd->getEnabled());
  if (scene_graph_->getLinkVisibility(cmd->getLinkName()) != cmd->getEnabled())
    return false;

  ++revision_;
  commands_.push_back(cmd);

  return true;
}

bool Environment::applyAddAllowedCollisionCommand(const AddAllowedCollisionCommand::ConstPtr& cmd)
{
  scene_graph_->addAllowedCollision(cmd->getLinkName1(), cmd->getLinkName2(), cmd->getReason());

  ++revision_;
  commands_.push_back(cmd);

  return true;
}

bool Environment::applyRemoveAllowedCollisionCommand(const RemoveAllowedCollisionCommand::ConstPtr& cmd)
{
  scene_graph_->removeAllowedCollision(cmd->getLinkName1(), cmd->getLinkName2());

  ++revision_;
  commands_.push_back(cmd);

  return true;
}

bool Environment::applyRemoveAllowedCollisionLinkCommand(const RemoveAllowedCollisionLinkCommand::ConstPtr& cmd)
{
  scene_graph_->removeAllowedCollision(cmd->getLinkName());

  ++revision_;
  commands_.push_back(cmd);

  return true;
}

bool Environment::applyAddSceneGraphCommand(AddSceneGraphCommand::ConstPtr cmd)
{
  if (scene_graph_->isEmpty() && cmd->getJoint())
    return false;

  std::vector<tesseract_scene_graph::Link::ConstPtr> pre_links = scene_graph_->getLinks();
  if (scene_graph_->isEmpty())
  {
    if (!scene_graph_->insertSceneGraph(*cmd->getSceneGraph(), cmd->getPrefix()))
      return false;

    state_solver_ = std::make_unique<tesseract_scene_graph::OFKTStateSolver>(*cmd->getSceneGraph(), cmd->getPrefix());
  }
  else if (!cmd->getJoint())
  {
    // Connect root of subgraph to graph
    tesseract_scene_graph::Joint root_joint(cmd->getPrefix() + cmd->getSceneGraph()->getName() + "_joint");
    root_joint.type = tesseract_scene_graph::JointType::FIXED;
    root_joint.parent_link_name = scene_graph_->getRoot();
    root_joint.child_link_name = cmd->getPrefix() + cmd->getSceneGraph()->getRoot();
    root_joint.parent_to_joint_origin_transform = Eigen::Isometry3d::Identity();

    tesseract_scene_graph::SceneGraph::ConstPtr sg = cmd->getSceneGraph();
    std::string prefix = cmd->getPrefix();
    cmd = std::make_shared<AddSceneGraphCommand>(*sg, root_joint, prefix);
    if (!scene_graph_->insertSceneGraph(*cmd->getSceneGraph(), *cmd->getJoint(), cmd->getPrefix()))
      return false;

    if (!state_solver_->insertSceneGraph(*cmd->getSceneGraph(), *cmd->getJoint(), cmd->getPrefix()))
      throw std::runtime_error("Environment, failed to insert scene graph into state solver.");
  }
  else
  {
    if (!scene_graph_->insertSceneGraph(*cmd->getSceneGraph(), *cmd->getJoint(), cmd->getPrefix()))
      return false;

    if (!state_solver_->insertSceneGraph(*cmd->getSceneGraph(), *cmd->getJoint(), cmd->getPrefix()))
      throw std::runtime_error("Environment, failed to insert scene graph into state solver.");
  }

  // Now need to get list of added links to add to the contact manager
  std::vector<tesseract_scene_graph::Link::ConstPtr> post_links = scene_graph_->getLinks();
  assert(post_links.size() > pre_links.size());
  std::sort(pre_links.begin(), pre_links.end());
  std::sort(post_links.begin(), post_links.end());
  std::vector<tesseract_scene_graph::Link::ConstPtr> diff_links;
  std::set_difference(post_links.begin(),
                      post_links.end(),
                      pre_links.begin(),
                      pre_links.end(),
                      std::inserter(diff_links, diff_links.begin()));

  std::unique_lock<std::shared_mutex> discrete_lock(discrete_manager_mutex_);
  std::unique_lock<std::shared_mutex> continuous_lock(continuous_manager_mutex_);
  for (const auto& link : diff_links)
  {
    if (!link->collision.empty())
    {
      tesseract_collision::CollisionShapesConst shapes;
      tesseract_common::VectorIsometry3d shape_poses;
      getCollisionObject(shapes, shape_poses, *link);

      if (discrete_manager_ != nullptr)
        discrete_manager_->addCollisionObject(link->getName(), 0, shapes, shape_poses, true);
      if (continuous_manager_ != nullptr)
        continuous_manager_->addCollisionObject(link->getName(), 0, shapes, shape_poses, true);
    }
  }

  ++revision_;
  commands_.push_back(cmd);

  return true;
}

bool Environment::applyChangeJointPositionLimitsCommand(const ChangeJointPositionLimitsCommand::ConstPtr& cmd)
{
  // First check if all of the joint exist
  for (const auto& jp : cmd->getLimits())
  {
    tesseract_scene_graph::JointLimits::ConstPtr jl = scene_graph_->getJointLimits(jp.first);
    if (jl == nullptr)
      return false;
  }

  for (const auto& jp : cmd->getLimits())
  {
    tesseract_scene_graph::JointLimits jl_copy = *scene_graph_->getJointLimits(jp.first);
    jl_copy.lower = jp.second.first;
    jl_copy.upper = jp.second.second;

    if (!scene_graph_->changeJointLimits(jp.first, jl_copy))
      return false;

    if (!state_solver_->changeJointPositionLimits(jp.first, jp.second.first, jp.second.second))
      throw std::runtime_error("Environment, failed to change joint position limits in state solver.");
  }

  ++revision_;
  commands_.push_back(cmd);

  return true;
}

bool Environment::applyChangeJointVelocityLimitsCommand(const ChangeJointVelocityLimitsCommand::ConstPtr& cmd)
{
  // First check if all of the joint exist
  for (const auto& jp : cmd->getLimits())
  {
    tesseract_scene_graph::JointLimits::ConstPtr jl = scene_graph_->getJointLimits(jp.first);
    if (jl == nullptr)
      return false;
  }

  for (const auto& jp : cmd->getLimits())
  {
    tesseract_scene_graph::JointLimits jl_copy = *scene_graph_->getJointLimits(jp.first);
    jl_copy.velocity = jp.second;

    if (!scene_graph_->changeJointLimits(jp.first, jl_copy))
      return false;

    if (!state_solver_->changeJointVelocityLimits(jp.first, jp.second))
      throw std::runtime_error("Environment, failed to change joint velocity limits in state solver.");
  }

  ++revision_;
  commands_.push_back(cmd);

  return true;
}

bool Environment::applyChangeJointAccelerationLimitsCommand(const ChangeJointAccelerationLimitsCommand::ConstPtr& cmd)
{
  // First check if all of the joint exist
  for (const auto& jp : cmd->getLimits())
  {
    tesseract_scene_graph::JointLimits::ConstPtr jl = scene_graph_->getJointLimits(jp.first);
    if (jl == nullptr)
      return false;
  }

  for (const auto& jp : cmd->getLimits())
  {
    tesseract_scene_graph::JointLimits jl_copy = *scene_graph_->getJointLimits(jp.first);
    jl_copy.acceleration = jp.second;

    if (!scene_graph_->changeJointLimits(jp.first, jl_copy))
      return false;

    if (!state_solver_->changeJointAccelerationLimits(jp.first, jp.second))
      throw std::runtime_error("Environment, failed to change joint acceleration limits in state solver.");
  }

  ++revision_;
  commands_.push_back(cmd);

  return true;
}

bool Environment::applyAddKinematicsInformationCommand(const AddKinematicsInformationCommand::ConstPtr& cmd)
{
  kinematics_information_.insert(cmd->getKinematicsInformation());

  if (!cmd->getKinematicsInformation().kinematics_plugin_info.empty())
  {
    const auto& info = cmd->getKinematicsInformation().kinematics_plugin_info;
    for (const auto& search_path : info.search_paths)
      kinematics_factory_.addSearchPath(search_path);

    for (const auto& search_library : info.search_libraries)
      kinematics_factory_.addSearchLibrary(search_library);

    for (const auto& group : info.fwd_plugin_infos)
    {
      for (const auto& solver : group.second.plugins)
        kinematics_factory_.addFwdKinPlugin(group.first, solver.first, solver.second);

      if (!group.second.default_plugin.empty())
        kinematics_factory_.setDefaultFwdKinPlugin(group.first, group.second.default_plugin);
    }

    for (const auto& group : info.inv_plugin_infos)
    {
      for (const auto& solver : group.second.plugins)
        kinematics_factory_.addInvKinPlugin(group.first, solver.first, solver.second);

      if (!group.second.default_plugin.empty())
        kinematics_factory_.setDefaultInvKinPlugin(group.first, group.second.default_plugin);
    }
  }

  ++revision_;
  commands_.push_back(cmd);

  return true;
}

bool Environment::applyAddContactManagersPluginInfoCommand(const AddContactManagersPluginInfoCommand::ConstPtr& cmd)
{
  const auto& info = cmd->getContactManagersPluginInfo();

  if (!info.empty())
  {
    contact_managers_plugin_info_.insert(info);

    for (const auto& search_path : info.search_paths)
      contact_managers_factory_.addSearchPath(search_path);

    for (const auto& search_library : info.search_libraries)
      contact_managers_factory_.addSearchLibrary(search_library);

    for (const auto& cm : info.discrete_plugin_infos.plugins)
      contact_managers_factory_.addDiscreteContactManagerPlugin(cm.first, cm.second);

    if (!info.discrete_plugin_infos.default_plugin.empty())
      contact_managers_factory_.setDefaultDiscreteContactManagerPlugin(info.discrete_plugin_infos.default_plugin);

    for (const auto& cm : info.continuous_plugin_infos.plugins)
      contact_managers_factory_.addContinuousContactManagerPlugin(cm.first, cm.second);

    if (!info.continuous_plugin_infos.default_plugin.empty())
      contact_managers_factory_.setDefaultContinuousContactManagerPlugin(info.continuous_plugin_infos.default_plugin);
  }

  if (contact_managers_factory_.hasDiscreteContactManagerPlugins())
  {
    std::string discrete_default = contact_managers_factory_.getDefaultDiscreteContactManagerPlugin();
    std::unique_lock<std::shared_mutex> discrete_lock(discrete_manager_mutex_);
    if (discrete_manager_ == nullptr || discrete_manager_->getName() != discrete_default)
      setActiveDiscreteContactManagerHelper(discrete_default);
  }
  else
  {
    CONSOLE_BRIDGE_logDebug("Environment, No discrete contact manager plugins were provided");
  }

  if (contact_managers_factory_.hasContinuousContactManagerPlugins())
  {
    std::string continuous_default = contact_managers_factory_.getDefaultContinuousContactManagerPlugin();
    std::unique_lock<std::shared_mutex> continuous_lock(continuous_manager_mutex_);
    if (continuous_manager_ == nullptr || continuous_manager_->getName() != continuous_default)
      setActiveContinuousContactManagerHelper(continuous_default);
  }
  else
  {
    CONSOLE_BRIDGE_logDebug("Environment, No continuous contact manager plugins were provided");
  }

  ++revision_;
  commands_.push_back(cmd);

  return true;
}

bool Environment::applySetActiveContinuousContactManagerCommand(
    const SetActiveContinuousContactManagerCommand::ConstPtr& cmd)
{
  setActiveContinuousContactManagerHelper(cmd->getName());

  ++revision_;
  commands_.push_back(cmd);

  return true;
}

bool Environment::applySetActiveDiscreteContactManagerCommand(
    const SetActiveDiscreteContactManagerCommand::ConstPtr& cmd)
{
  setActiveDiscreteContactManagerHelper(cmd->getName());

  ++revision_;
  commands_.push_back(cmd);

  return true;
}

bool Environment::applyChangeCollisionMarginsCommand(const ChangeCollisionMarginsCommand::ConstPtr& cmd)
{
  collision_margin_data_.apply(cmd->getCollisionMarginData(), cmd->getCollisionMarginOverrideType());

  std::unique_lock<std::shared_mutex> continuous_lock(continuous_manager_mutex_);
  if (continuous_manager_ != nullptr)
    continuous_manager_->setCollisionMarginData(collision_margin_data_, CollisionMarginOverrideType::REPLACE);

  std::unique_lock<std::shared_mutex> discrete_lock(discrete_manager_mutex_);
  if (discrete_manager_ != nullptr)
    discrete_manager_->setCollisionMarginData(collision_margin_data_, CollisionMarginOverrideType::REPLACE);

  ++revision_;
  commands_.push_back(cmd);

  return true;
}

template <class Archive>
void Environment::save(Archive& ar, const unsigned int /*version*/) const
{
  std::shared_lock<std::shared_mutex> lock(mutex_);

  ar& BOOST_SERIALIZATION_NVP(resource_locator_);
  ar& BOOST_SERIALIZATION_NVP(commands_);
  ar& BOOST_SERIALIZATION_NVP(init_revision_);
  ar& BOOST_SERIALIZATION_NVP(current_state_);
  ar& boost::serialization::make_nvp("timestamp_",
                                     boost::serialization::make_binary_object(&timestamp_, sizeof(timestamp_)));
  ar& boost::serialization::make_nvp(
      "current_state_timestamp_",
      boost::serialization::make_binary_object(&current_state_timestamp_, sizeof(current_state_timestamp_)));
}

template <class Archive>
void Environment::load(Archive& ar, const unsigned int /*version*/)
{
  ar& BOOST_SERIALIZATION_NVP(resource_locator_);

  tesseract_environment::Commands commands;
  ar& boost::serialization::make_nvp("commands_", commands);
  init(commands);

  ar& BOOST_SERIALIZATION_NVP(init_revision_);

  tesseract_scene_graph::SceneState current_state;
  ar& boost::serialization::make_nvp("current_state_", current_state);
  setState(current_state.joints);

  ar& boost::serialization::make_nvp("timestamp_",
                                     boost::serialization::make_binary_object(&timestamp_, sizeof(timestamp_)));
  ar& boost::serialization::make_nvp(
      "current_state_timestamp_",
      boost::serialization::make_binary_object(&current_state_timestamp_, sizeof(current_state_timestamp_)));
}

template <class Archive>
void Environment::serialize(Archive& ar, const unsigned int version)
{
  boost::serialization::split_member(ar, *this, version);
}

}  // namespace tesseract_environment

#include <tesseract_common/serialization.h>
TESSERACT_SERIALIZE_ARCHIVES_INSTANTIATE(tesseract_environment::Environment)
