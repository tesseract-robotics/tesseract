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

#include <tesseract_environment/core/environment.h>
#include <tesseract_environment/core/utils.h>
#include <tesseract_collision/core/common.h>
#include <tesseract_collision/bullet/bullet_cast_bvh_manager.h>
#include <tesseract_collision/bullet/bullet_discrete_bvh_manager.h>
#include <tesseract_collision/fcl/fcl_discrete_managers.h>
#include <tesseract_srdf/utils.h>

TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <queue>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

namespace tesseract_environment
{
Environment::Environment(bool register_default_contact_managers)
  : register_default_contact_managers_(register_default_contact_managers)
{
}

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

  manipulator_manager_ = std::make_shared<ManipulatorManager>();
  manipulator_manager_->init(scene_graph_, tesseract_srdf::KinematicsInformation());

  if (!applyCommandsHelper(commands))
  {
    CONSOLE_BRIDGE_logError("When initializing environment from command history, it failed to apply a command!");
    return false;
  }

  is_contact_allowed_fn_ = std::bind(&tesseract_scene_graph::SceneGraph::isCollisionAllowed,
                                     scene_graph_,
                                     std::placeholders::_1,
                                     std::placeholders::_2);

  manipulator_manager_->revision_ = revision_;

  if (!state_solver_->init(scene_graph_, revision_))
  {
    CONSOLE_BRIDGE_logError("The environment state solver failed to initialize");
    return false;
  }

  initialized_ = true;
  init_revision_ = revision_;

  environmentChanged();

  // Register Default Managers, must be called after initialized after initialized_ is set to true
  if (register_default_contact_managers_)
    registerDefaultContactManagersHelper();

  return initialized_;
}

bool Environment::reset()
{
  std::unique_lock<std::shared_mutex> lock(mutex_);

  Commands init_command;
  if (commands_.empty() || !initialized_)
    return false;

  init_command.reserve(static_cast<std::size_t>(init_revision_));
  for (std::size_t i = 0; i < static_cast<std::size_t>(init_revision_); ++i)
    init_command.push_back(commands_[i]);

  return initHelper(init_command);
}

void Environment::clear()
{
  initialized_ = false;
  revision_ = 0;
  init_revision_ = 0;
  scene_graph_ = nullptr;
  scene_graph_const_ = nullptr;
  commands_.clear();
  link_names_.clear();
  joint_names_.clear();
  active_link_names_.clear();
  active_joint_names_.clear();
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
    commands.push_back(std::make_shared<AddKinematicsInformationCommand>(srdf_model->kinematics_information));

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
  std::unique_lock<std::shared_mutex> lock(mutex_);
  return applyCommandsHelper(commands);
}

bool Environment::applyCommand(const Command::ConstPtr& command) { return applyCommands({ command }); }

const tesseract_scene_graph::SceneGraph::ConstPtr& Environment::getSceneGraph() const { return scene_graph_const_; }

ManipulatorManager::Ptr Environment::getManipulatorManager() { return manipulator_manager_; }

ManipulatorManager::ConstPtr Environment::getManipulatorManager() const { return manipulator_manager_; }

Eigen::Isometry3d Environment::findTCP(const tesseract_common::ManipulatorInfo& manip_info) const
{
  std::shared_lock<std::shared_mutex> lock(mutex_);
  if (manip_info.tcp.empty())
    return Eigen::Isometry3d::Identity();

  auto composite_mi_fwd_kin = manipulator_manager_->getFwdKinematicSolver(manip_info.manipulator);
  if (composite_mi_fwd_kin == nullptr)
    throw std::runtime_error("findTCP: Manipulator '" + manip_info.manipulator + "' does not exist!");

  const std::string& tip_link = composite_mi_fwd_kin->getTipLinkName();
  if (manip_info.tcp.isString())
  {
    // Check Manipulator Manager for TCP
    const std::string& tcp_name = manip_info.tcp.getString();
    if (manipulator_manager_->hasGroupTCP(manip_info.manipulator, tcp_name))
      return manipulator_manager_->getGroupsTCP(manip_info.manipulator, tcp_name);

    // Check Environment for links and calculate TCP
    auto link_it = current_state_->link_transforms.find(tcp_name);
    if (link_it != current_state_->link_transforms.end())
    {
      // If it is external then the tcp is not attached to the robot
      if (manip_info.tcp.isExternal())
        return link_it->second;
      else
        return current_state_->link_transforms.at(tip_link).inverse() * link_it->second;
    }

    // Check callbacks for TCP
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

    throw std::runtime_error("Could not find tcp by name " + tcp_name + "' setting to Identity!");
  }

  if (manip_info.tcp.isTransform())
    return manip_info.tcp.getTransform();

  throw std::runtime_error("Could not find tcp!");
}

void Environment::addFindTCPCallback(FindTCPCallbackFn fn)
{
  std::unique_lock<std::shared_mutex> lock(mutex_);
  find_tcp_cb_.push_back(fn);
}

std::vector<FindTCPCallbackFn> Environment::getFindTCPCallbacks() const
{
  std::shared_lock<std::shared_mutex> lock(mutex_);
  return find_tcp_cb_;
}

void Environment::setResourceLocator(tesseract_scene_graph::ResourceLocator::Ptr locator)
{
  std::unique_lock<std::shared_mutex> lock(mutex_);
  resource_locator_ = locator;
}

tesseract_scene_graph::ResourceLocator::Ptr Environment::getResourceLocator() const
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
  std::unique_lock<std::shared_mutex> lock(mutex_);
  state_solver_->setState(joints);
  currentStateChanged();
}

void Environment::setState(const std::vector<std::string>& joint_names, const std::vector<double>& joint_values)
{
  std::unique_lock<std::shared_mutex> lock(mutex_);
  state_solver_->setState(joint_names, joint_values);
  currentStateChanged();
}

void Environment::setState(const std::vector<std::string>& joint_names,
                           const Eigen::Ref<const Eigen::VectorXd>& joint_values)
{
  std::unique_lock<std::shared_mutex> lock(mutex_);
  state_solver_->setState(joint_names, joint_values);
  currentStateChanged();
}

EnvState::Ptr Environment::getState(const std::unordered_map<std::string, double>& joints) const
{
  std::shared_lock<std::shared_mutex> lock(mutex_);
  EnvState::Ptr state = state_solver_->getState(joints);
  return state;
}

EnvState::Ptr Environment::getState(const std::vector<std::string>& joint_names,
                                    const std::vector<double>& joint_values) const
{
  std::shared_lock<std::shared_mutex> lock(mutex_);
  EnvState::Ptr state = state_solver_->getState(joint_names, joint_values);
  return state;
}

EnvState::Ptr Environment::getState(const std::vector<std::string>& joint_names,
                                    const Eigen::Ref<const Eigen::VectorXd>& joint_values) const
{
  std::shared_lock<std::shared_mutex> lock(mutex_);
  EnvState::Ptr state = state_solver_->getState(joint_names, joint_values);
  return state;
}

EnvState::ConstPtr Environment::getCurrentState() const
{
  std::shared_lock<std::shared_mutex> lock(mutex_);
  return current_state_;
}

std::chrono::high_resolution_clock::duration Environment::getCurrentStateTimestamp() const
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

tesseract_scene_graph::AllowedCollisionMatrix::ConstPtr Environment::getAllowedCollisionMatrix() const
{
  std::shared_lock<std::shared_mutex> lock(mutex_);
  return scene_graph_->getAllowedCollisionMatrix();
}

std::vector<std::string> Environment::getJointNames() const
{
  std::shared_lock<std::shared_mutex> lock(mutex_);
  return joint_names_;
}

std::vector<std::string> Environment::getActiveJointNames() const
{
  std::shared_lock<std::shared_mutex> lock(mutex_);
  return active_joint_names_;
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
  jv.resize(static_cast<long int>(active_joint_names_.size()));
  for (auto j = 0u; j < active_joint_names_.size(); ++j)
  {
    jv(j) = current_state_->joints[active_joint_names_[j]];
  }

  return jv;
}

Eigen::VectorXd Environment::getCurrentJointValues(const std::vector<std::string>& joint_names) const
{
  std::shared_lock<std::shared_mutex> lock(mutex_);
  Eigen::VectorXd jv;
  jv.resize(static_cast<long int>(joint_names.size()));
  for (auto j = 0u; j < joint_names.size(); ++j)
  {
    jv(j) = current_state_->joints[joint_names[j]];
  }

  return jv;
}

const std::string& Environment::getRootLinkName() const
{
  std::shared_lock<std::shared_mutex> lock(mutex_);
  return scene_graph_->getRoot();
}

std::vector<std::string> Environment::getLinkNames() const
{
  std::shared_lock<std::shared_mutex> lock(mutex_);
  return link_names_;
}

std::vector<std::string> Environment::getActiveLinkNames() const
{
  std::shared_lock<std::shared_mutex> lock(mutex_);
  return active_link_names_;
}

tesseract_common::VectorIsometry3d Environment::getLinkTransforms() const
{
  std::shared_lock<std::shared_mutex> lock(mutex_);
  tesseract_common::VectorIsometry3d link_tfs;
  link_tfs.reserve(link_names_.size());
  for (const auto& link_name : link_names_)
    link_tfs.push_back(current_state_->link_transforms[link_name]);

  return link_tfs;
}

Eigen::Isometry3d Environment::getLinkTransform(const std::string& link_name) const
{
  std::shared_lock<std::shared_mutex> lock(mutex_);
  return current_state_->link_transforms[link_name];
}

StateSolver::Ptr Environment::getStateSolver() const
{
  std::shared_lock<std::shared_mutex> lock(mutex_);
  return state_solver_->clone();
}

bool Environment::setActiveDiscreteContactManager(const std::string& name)
{
  std::unique_lock<std::shared_mutex> lock(mutex_);
  return setActiveDiscreteContactManagerHelper(name);
}

tesseract_collision::DiscreteContactManager::Ptr Environment::getDiscreteContactManager(const std::string& name) const
{
  std::shared_lock<std::shared_mutex> lock(mutex_);
  tesseract_collision::DiscreteContactManager::Ptr manager = getDiscreteContactManagerHelper(name);
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
  return setActiveContinuousContactManagerHelper(name);
}

tesseract_collision::DiscreteContactManager::Ptr Environment::getDiscreteContactManager() const
{
  std::shared_lock<std::shared_mutex> lock(mutex_);
  if (!discrete_manager_)
    return nullptr;
  return discrete_manager_->clone();
}

tesseract_collision::ContinuousContactManager::Ptr Environment::getContinuousContactManager() const
{
  std::shared_lock<std::shared_mutex> lock(mutex_);
  if (!continuous_manager_)
    return nullptr;
  return continuous_manager_->clone();
}

tesseract_collision::ContinuousContactManager::Ptr
Environment::getContinuousContactManager(const std::string& name) const
{
  std::shared_lock<std::shared_mutex> lock(mutex_);
  tesseract_collision::ContinuousContactManager::Ptr manager = getContinuousContactManagerHelper(name);
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

bool Environment::registerDiscreteContactManager(
    const std::string& name,
    tesseract_collision::DiscreteContactManagerFactoryCreateMethod create_function)
{
  std::unique_lock<std::shared_mutex> lock(mutex_);
  return discrete_factory_.registar(name, std::move(create_function));
}

bool Environment::registerContinuousContactManager(
    const std::string& name,
    tesseract_collision::ContinuousContactManagerFactoryCreateMethod create_function)
{
  std::unique_lock<std::shared_mutex> lock(mutex_);
  return continuous_factory_.registar(name, std::move(create_function));
}

bool Environment::registerDefaultContactManagers()
{
  std::unique_lock<std::shared_mutex> lock(mutex_);

  if (!initialized_)
    return false;

  return registerDefaultContactManagersHelper();
}

bool Environment::registerDefaultContactManagersHelper()
{
  using namespace tesseract_collision;

  bool success = true;
  // Register contact manager
  success &= discrete_factory_.registar(tesseract_collision_bullet::BulletDiscreteBVHManager::name(),
                                        &tesseract_collision_bullet::BulletDiscreteBVHManager::create);

  success &= discrete_factory_.registar(tesseract_collision_fcl::FCLDiscreteBVHManager::name(),
                                        &tesseract_collision_fcl::FCLDiscreteBVHManager::create);

  success &= continuous_factory_.registar(tesseract_collision_bullet::BulletCastBVHManager::name(),
                                          &tesseract_collision_bullet::BulletCastBVHManager::create);

  // Set Active contact manager
  success &= setActiveDiscreteContactManagerHelper(tesseract_collision_bullet::BulletDiscreteBVHManager::name());
  success &= setActiveContinuousContactManagerHelper(tesseract_collision_bullet::BulletCastBVHManager::name());

  return success;
}

bool Environment::setActiveDiscreteContactManagerHelper(const std::string& name)
{
  tesseract_collision::DiscreteContactManager::Ptr manager = getDiscreteContactManagerHelper(name);
  if (manager == nullptr)
  {
    std::string msg = "\n  Discrete manager with " + name + " does not exist in factory!\n";
    msg += "    Available Managers:\n";
    for (const auto& m : discrete_factory_.getAvailableManagers())
      msg += "      " + m + "\n";

    CONSOLE_BRIDGE_logError(msg.c_str());
    return false;
  }

  discrete_manager_name_ = name;
  discrete_manager_ = std::move(manager);

  // Update the current state information since the contact manager has been created/set
  currentStateChanged();

  return true;
}

bool Environment::setActiveContinuousContactManagerHelper(const std::string& name)
{
  tesseract_collision::ContinuousContactManager::Ptr manager = getContinuousContactManagerHelper(name);

  if (manager == nullptr)
  {
    std::string msg = "\n  Continuous manager with " + name + " does not exist in factory!\n";
    msg += "    Available Managers:\n";
    for (const auto& m : continuous_factory_.getAvailableManagers())
      msg += "      " + m + "\n";

    CONSOLE_BRIDGE_logError(msg.c_str());
    return false;
  }

  continuous_manager_name_ = name;
  continuous_manager_ = std::move(manager);

  // Update the current state information since the contact manager has been created/set
  currentStateChanged();

  return true;
}

tesseract_collision::DiscreteContactManager::Ptr
Environment::getDiscreteContactManagerHelper(const std::string& name) const
{
  tesseract_collision::DiscreteContactManager::Ptr manager = discrete_factory_.create(name);
  if (manager == nullptr)
    return nullptr;

  manager->setIsContactAllowedFn(is_contact_allowed_fn_);
  if (initialized_)
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

    manager->setActiveCollisionObjects(active_link_names_);
  }

  manager->setCollisionMarginData(collision_margin_data_);

  return manager;
}

tesseract_collision::ContinuousContactManager::Ptr
Environment::getContinuousContactManagerHelper(const std::string& name) const
{
  tesseract_collision::ContinuousContactManager::Ptr manager = continuous_factory_.create(name);

  if (manager == nullptr)
    return nullptr;

  manager->setIsContactAllowedFn(is_contact_allowed_fn_);
  if (initialized_)
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

    manager->setActiveCollisionObjects(active_link_names_);
  }

  manager->setCollisionMarginData(collision_margin_data_);

  return manager;
}

void Environment::getCollisionObject(tesseract_collision::CollisionShapesConst& shapes,
                                     tesseract_common::VectorIsometry3d& shape_poses,
                                     const tesseract_scene_graph::Link& link) const
{
  for (const auto& c : link.collision)
  {
    shapes.push_back(c->geometry);
    shape_poses.push_back(c->origin);
  }
}

void Environment::currentStateChanged()
{
  current_state_timestamp_ = std::chrono::high_resolution_clock::now().time_since_epoch();
  current_state_ = std::make_shared<EnvState>(*(state_solver_->getCurrentState()));
  if (discrete_manager_ != nullptr)
    discrete_manager_->setCollisionObjectsTransform(current_state_->link_transforms);
  if (continuous_manager_ != nullptr)
  {
    for (const auto& tf : current_state_->link_transforms)
    {
      if (std::find(active_link_names_.begin(), active_link_names_.end(), tf.first) != active_link_names_.end())
      {
        continuous_manager_->setCollisionObjectsTransform(tf.first, tf.second, tf.second);
      }
      else
      {
        continuous_manager_->setCollisionObjectsTransform(tf.first, tf.second);
      }
    }
  }
}

void Environment::environmentChanged()
{
  // Update link names
  std::vector<tesseract_scene_graph::Link::ConstPtr> links = scene_graph_->getLinks();
  link_names_.clear();
  link_names_.reserve(links.size());
  for (const auto& link : links)
    link_names_.push_back(link->getName());

  // Update joint names and active joint name
  std::vector<tesseract_scene_graph::Joint::ConstPtr> joints = scene_graph_->getJoints();
  active_joint_names_.clear();
  joint_names_.clear();
  joint_names_.reserve(joints.size());
  for (const auto& joint : joints)
  {
    joint_names_.push_back(joint->getName());

    // UNKNOWN, REVOLUTE, CONTINUOUS, PRISMATIC, FLOATING, PLANAR, FIXED
    if (joint->type == tesseract_scene_graph::JointType::REVOLUTE ||
        joint->type == tesseract_scene_graph::JointType::CONTINUOUS ||
        joint->type == tesseract_scene_graph::JointType::PRISMATIC)
      active_joint_names_.push_back(joint->getName());
  }

  // Update active link names
  active_link_names_.clear();
  getActiveLinkNamesRecursive(active_link_names_, scene_graph_, scene_graph_->getRoot(), false);

  if (discrete_manager_ != nullptr)
    discrete_manager_->setActiveCollisionObjects(active_link_names_);
  if (continuous_manager_ != nullptr)
    continuous_manager_->setActiveCollisionObjects(active_link_names_);

  state_solver_->onEnvironmentChanged(commands_);
  manipulator_manager_->onEnvironmentChanged(commands_);

  currentStateChanged();
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

  scene_graph_->removeLink(name);
  if (discrete_manager_ != nullptr)
    discrete_manager_->removeCollisionObject(name);
  if (continuous_manager_ != nullptr)
    continuous_manager_->removeCollisionObject(name);

  for (const auto& link_name : child_link_names)
  {
    scene_graph_->removeLink(link_name);

    if (discrete_manager_ != nullptr)
      discrete_manager_->removeCollisionObject(link_name);
    if (continuous_manager_ != nullptr)
      continuous_manager_->removeCollisionObject(link_name);
  }

  return true;
}

Environment::Ptr Environment::clone() const
{
  auto cloned_env = std::make_shared<Environment>();

  std::shared_lock<std::shared_mutex> lock(mutex_);

  if (!initialized_)
    return cloned_env;

  cloned_env->initialized_ = initialized_;
  cloned_env->init_revision_ = revision_;
  cloned_env->revision_ = revision_;
  cloned_env->commands_ = commands_;
  cloned_env->scene_graph_ = scene_graph_->clone();
  cloned_env->scene_graph_const_ = cloned_env->scene_graph_;
  cloned_env->manipulator_manager_ = manipulator_manager_->clone(cloned_env->getSceneGraph());
  cloned_env->current_state_ = std::make_shared<EnvState>(*current_state_);
  cloned_env->state_solver_ = state_solver_->clone();
  cloned_env->link_names_ = link_names_;
  cloned_env->joint_names_ = joint_names_;
  cloned_env->active_link_names_ = active_link_names_;
  cloned_env->active_joint_names_ = active_joint_names_;
  cloned_env->find_tcp_cb_ = find_tcp_cb_;
  cloned_env->collision_margin_data_ = collision_margin_data_;
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

  cloned_env->discrete_manager_name_ = discrete_manager_name_;
  cloned_env->continuous_manager_name_ = continuous_manager_name_;
  cloned_env->discrete_factory_ = discrete_factory_;
  cloned_env->continuous_factory_ = continuous_factory_;

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

  // If this is not true then the initHelper function has called applyCommand so do not call.
  if (initialized_)
    environmentChanged();

  return success;
}

//////////////////////////////////////////////////////////////
/// External Helper Commands wrapping environment commands ///
//////////////////////////////////////////////////////////////

bool Environment::addKinematicsInformation(const tesseract_srdf::KinematicsInformation& kin_info)
{
  return applyCommand(std::make_shared<AddKinematicsInformationCommand>(kin_info));
}

bool Environment::addLink(const tesseract_scene_graph::Link& link)
{
  return applyCommand(std::make_shared<AddLinkCommand>(link));
}

bool Environment::addLink(const tesseract_scene_graph::Link& link, const tesseract_scene_graph::Joint& joint)
{
  return applyCommand(std::make_shared<AddLinkCommand>(link, joint));
}

bool Environment::moveLink(const tesseract_scene_graph::Joint& joint)
{
  return applyCommand(std::make_shared<MoveLinkCommand>(joint));
}

bool Environment::removeLink(const std::string& name)
{
  return applyCommand(std::make_shared<RemoveLinkCommand>(name));
}

bool Environment::removeJoint(const std::string& name)
{
  return applyCommand(std::make_shared<RemoveJointCommand>(name));
}

bool Environment::moveJoint(const std::string& joint_name, const std::string& parent_link)
{
  return applyCommand(std::make_shared<MoveJointCommand>(joint_name, parent_link));
}

bool Environment::changeJointOrigin(const std::string& joint_name, const Eigen::Isometry3d& new_origin)
{
  return applyCommand(std::make_shared<ChangeJointOriginCommand>(joint_name, new_origin));
}

bool Environment::changeJointPositionLimits(const std::string& joint_name, double lower, double upper)
{
  return applyCommand(std::make_shared<ChangeJointPositionLimitsCommand>(joint_name, lower, upper));
}

bool Environment::changeJointPositionLimits(const std::unordered_map<std::string, std::pair<double, double> >& limits)
{
  return applyCommand(std::make_shared<ChangeJointPositionLimitsCommand>(limits));
}

bool Environment::changeJointVelocityLimits(const std::string& joint_name, double limit)
{
  return applyCommand(std::make_shared<ChangeJointVelocityLimitsCommand>(joint_name, limit));
}

bool Environment::changeJointVelocityLimits(const std::unordered_map<std::string, double>& limits)
{
  return applyCommand(std::make_shared<ChangeJointVelocityLimitsCommand>(limits));
}

bool Environment::changeJointAccelerationLimits(const std::string& joint_name, double limit)
{
  return applyCommand(std::make_shared<ChangeJointAccelerationLimitsCommand>(joint_name, limit));
}

bool Environment::changeJointAccelerationLimits(const std::unordered_map<std::string, double>& limits)
{
  return applyCommand(std::make_shared<ChangeJointAccelerationLimitsCommand>(limits));
}

bool Environment::setLinkCollisionEnabled(const std::string& name, bool enabled)
{
  return applyCommand(std::make_shared<ChangeLinkCollisionEnabledCommand>(name, enabled));
}

bool Environment::setLinkVisibility(const std::string& name, bool visibility)
{
  return applyCommand(std::make_shared<ChangeLinkVisibilityCommand>(name, visibility));
}

bool Environment::addAllowedCollision(const std::string& link_name1,
                                      const std::string& link_name2,
                                      const std::string& reason)
{
  return applyCommand(std::make_shared<AddAllowedCollisionCommand>(link_name1, link_name2, reason));
}

bool Environment::removeAllowedCollision(const std::string& link_name1, const std::string& link_name2)
{
  return applyCommand(std::make_shared<RemoveAllowedCollisionCommand>(link_name1, link_name2));
}

bool Environment::removeAllowedCollision(const std::string& link_name)
{
  return applyCommand(std::make_shared<RemoveAllowedCollisionLinkCommand>(link_name));
}

bool Environment::addSceneGraph(const tesseract_scene_graph::SceneGraph& scene_graph, const std::string& prefix)
{
  return applyCommand(std::make_shared<AddSceneGraphCommand>(scene_graph, prefix));
}

bool Environment::addSceneGraph(const tesseract_scene_graph::SceneGraph& scene_graph,
                                const tesseract_scene_graph::Joint& joint,
                                const std::string& prefix)
{
  return applyCommand(std::make_shared<AddSceneGraphCommand>(scene_graph, joint, prefix));
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
  }
  else
  {  // A new link and joint is being added
    if (!scene_graph_->addLink(*cmd->getLink(), *cmd->getJoint()))
      return false;
  }

  // If Link existed remove it from collision before adding the replacing links geometry
  if (link_exists)
  {
    if (discrete_manager_ != nullptr)
      discrete_manager_->removeCollisionObject(link_name);
    if (continuous_manager_ != nullptr)
      continuous_manager_->removeCollisionObject(link_name);
  }

  // We have moved the original objects, get a pointer to them from scene_graph
  if (!cmd->getLink()->collision.empty())
  {
    tesseract_collision::CollisionShapesConst shapes;
    tesseract_common::VectorIsometry3d shape_poses;
    getCollisionObject(shapes, shape_poses, *cmd->getLink());

    if (discrete_manager_ != nullptr)
      discrete_manager_->addCollisionObject(link_name, 0, shapes, shape_poses, true);
    if (continuous_manager_ != nullptr)
      continuous_manager_->addCollisionObject(link_name, 0, shapes, shape_poses, true);
  }

  ++revision_;
  commands_.push_back(cmd);

  return true;
}

bool Environment::applyMoveLinkCommand(const MoveLinkCommand::ConstPtr& cmd)
{
  std::vector<tesseract_scene_graph::Joint::ConstPtr> joints =
      scene_graph_->getInboundJoints(cmd->getJoint()->child_link_name);
  assert(joints.size() == 1);
  if (!scene_graph_->removeJoint(joints[0]->getName()))
    return false;

  if (!scene_graph_->addJoint(*cmd->getJoint()))
    return false;

  ++revision_;
  commands_.push_back(cmd);

  return true;
}

bool Environment::applyMoveJointCommand(const MoveJointCommand::ConstPtr& cmd)
{
  if (!scene_graph_->moveJoint(cmd->getJointName(), cmd->getParentLink()))
    return false;

  ++revision_;
  commands_.push_back(cmd);

  return true;
}

bool Environment::applyRemoveLinkCommand(const RemoveLinkCommand::ConstPtr& cmd)
{
  if (!removeLinkHelper(cmd->getLinkName()))
    return false;

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

  ++revision_;
  commands_.push_back(cmd);

  return true;
}

bool Environment::applyChangeLinkOriginCommand(const ChangeLinkOriginCommand::ConstPtr& /*cmd*/)
{
  throw std::runtime_error("Unhandled environment command: CHANGE_LINK_ORIGIN");
}

bool Environment::applyChangeJointOriginCommand(const ChangeJointOriginCommand::ConstPtr& cmd)
{
  if (!scene_graph_->changeJointOrigin(cmd->getJointName(), cmd->getOrigin()))
    return false;

  ++revision_;
  commands_.push_back(cmd);

  return true;
}

bool Environment::applyChangeLinkCollisionEnabledCommand(const ChangeLinkCollisionEnabledCommand::ConstPtr& cmd)
{
  if (discrete_manager_ != nullptr)
  {
    if (cmd->getEnabled())
      discrete_manager_->enableCollisionObject(cmd->getLinkName());
    else
      discrete_manager_->disableCollisionObject(cmd->getLinkName());
  }

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
  }
  else
  {
    if (!scene_graph_->insertSceneGraph(*cmd->getSceneGraph(), *cmd->getJoint(), cmd->getPrefix()))
      return false;
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
  }

  ++revision_;
  commands_.push_back(cmd);

  return true;
}

bool Environment::applyAddKinematicsInformationCommand(const AddKinematicsInformationCommand::ConstPtr& cmd)
{
  if (!manipulator_manager_->addKinematicsInformation(cmd->getKinematicsInformation()))
    return false;

  ++revision_;
  commands_.push_back(cmd);

  return true;
}

bool Environment::applyChangeCollisionMarginsCommand(const ChangeCollisionMarginsCommand::ConstPtr& cmd)
{
  collision_margin_data_.apply(cmd->getCollisionMarginData(), cmd->getCollisionMarginOverrideType());

  if (continuous_manager_ != nullptr)
    continuous_manager_->setCollisionMarginData(collision_margin_data_, CollisionMarginOverrideType::REPLACE);

  if (discrete_manager_ != nullptr)
    discrete_manager_->setCollisionMarginData(collision_margin_data_, CollisionMarginOverrideType::REPLACE);

  ++revision_;
  commands_.push_back(cmd);

  return true;
}

}  // namespace tesseract_environment
