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

TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <queue>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

namespace tesseract_environment
{
int Environment::getRevision() const { return revision_; }

const Commands& Environment::getCommandHistory() const { return commands_; }

bool Environment::applyCommands(const Commands& commands)
{
  for (const auto& command : commands)
  {
    if (!command)
      return false;

    if (!applyCommand(*command))
      return false;
  }
  return true;
}

bool Environment::applyCommands(const std::vector<Command>& commands)
{
  for (const auto& command : commands)
    if (!applyCommand(command))
      return false;

  return true;
}

bool Environment::applyCommand(const Command& command)
{
  switch (command.getType())
  {
    case tesseract_environment::CommandType::ADD:
    {
      const auto& cmd = static_cast<const tesseract_environment::AddCommand&>(command);
      // If only a link is provided
      if (cmd.getLink() && !cmd.getJoint())
      {
        if (!addLink(cmd.getLink()->clone()))
          return false;
      }
      else if (cmd.getLink() && cmd.getJoint())
      {
        if (!addLink(cmd.getLink()->clone(), cmd.getJoint()->clone()))
          return false;
      }
      else if (!cmd.getLink() && !cmd.getJoint())
        return false;
      else
        return false;

      break;
    }
    case tesseract_environment::CommandType::MOVE_LINK:
    {
      const auto& cmd = static_cast<const tesseract_environment::MoveLinkCommand&>(command);
      if (!moveLink(cmd.getJoint()->clone()))
        return false;
      break;
    }
    case tesseract_environment::CommandType::MOVE_JOINT:
    {
      const auto& cmd = static_cast<const tesseract_environment::MoveJointCommand&>(command);
      if (!moveJoint(cmd.getJointName(), cmd.getParentLink()))
        return false;
      break;
    }
    case tesseract_environment::CommandType::REMOVE_LINK:
    {
      const auto& cmd = static_cast<const tesseract_environment::RemoveLinkCommand&>(command);
      if (!removeLink(cmd.getLinkName()))
        return false;
      break;
    }
    case tesseract_environment::CommandType::REMOVE_JOINT:
    {
      const auto& cmd = static_cast<const tesseract_environment::RemoveJointCommand&>(command);
      if (!removeJoint(cmd.getJointName()))
        return false;
      break;
    }
    case tesseract_environment::CommandType::CHANGE_LINK_ORIGIN:
    {
      CONSOLE_BRIDGE_logError("Unhandled environment command: CHANGE_LINK_ORIGIN");
      assert(false);
      return false;
    }
    case tesseract_environment::CommandType::CHANGE_JOINT_ORIGIN:
    {
      const auto& cmd = static_cast<const tesseract_environment::ChangeJointOriginCommand&>(command);
      if (!changeJointOrigin(cmd.getJointName(), cmd.getOrigin()))
        return false;
      break;
    }
    case tesseract_environment::CommandType::CHANGE_LINK_COLLISION_ENABLED:
    {
      const auto& cmd = static_cast<const tesseract_environment::ChangeLinkCollisionEnabledCommand&>(command);
      setLinkCollisionEnabled(cmd.getLinkName(), cmd.getEnabled());
      if (getLinkCollisionEnabled(cmd.getLinkName()) != cmd.getEnabled())
        return false;
      break;
    }
    case tesseract_environment::CommandType::CHANGE_LINK_VISIBILITY:
    {
      const auto& cmd = static_cast<const tesseract_environment::ChangeLinkVisibilityCommand&>(command);
      setLinkVisibility(cmd.getLinkName(), cmd.getEnabled());
      if (getLinkVisibility(cmd.getLinkName()) != cmd.getEnabled())
        return false;
      break;
    }
    case tesseract_environment::CommandType::ADD_ALLOWED_COLLISION:
    {
      const auto& cmd = static_cast<const tesseract_environment::AddAllowedCollisionCommand&>(command);
      addAllowedCollision(cmd.getLinkName1(), cmd.getLinkName2(), cmd.getReason());
      break;
    }
    case tesseract_environment::CommandType::REMOVE_ALLOWED_COLLISION:
    {
      const auto& cmd = static_cast<const tesseract_environment::RemoveAllowedCollisionCommand&>(command);
      removeAllowedCollision(cmd.getLinkName1(), cmd.getLinkName2());
      break;
    }
    case tesseract_environment::CommandType::REMOVE_ALLOWED_COLLISION_LINK:
    {
      const auto& cmd = static_cast<const tesseract_environment::RemoveAllowedCollisionLinkCommand&>(command);
      removeAllowedCollision(cmd.getLinkName());
      break;
    }
    case tesseract_environment::CommandType::ADD_SCENE_GRAPH:
    {
      const auto& cmd = static_cast<const tesseract_environment::AddSceneGraphCommand&>(command);
      if (cmd.getJoint() != nullptr)
      {
        if (!addSceneGraph(*(cmd.getSceneGraph()), cmd.getJoint()->clone(), cmd.getPrefix()))
          return false;
      }
      else
      {
        // This should only occur if the this graph is empty.
        if (!addSceneGraph(*(cmd.getSceneGraph()), cmd.getPrefix()))
          return false;
      }
      break;
    }
    case tesseract_environment::CommandType::CHANGE_JOINT_POSITION_LIMITS:
    {
      const auto& cmd = static_cast<const tesseract_environment::ChangeJointPositionLimitsCommand&>(command);
      if (!changeJointPositionLimits(cmd.getLimits()))
        return false;
      break;
    }
    case tesseract_environment::CommandType::CHANGE_JOINT_VELOCITY_LIMITS:
    {
      const auto& cmd = static_cast<const tesseract_environment::ChangeJointVelocityLimitsCommand&>(command);
      if (!changeJointVelocityLimits(cmd.getLimits()))
        return false;
      break;
    }
    case tesseract_environment::CommandType::CHANGE_JOINT_ACCELERATION_LIMITS:
    {
      const auto& cmd = static_cast<const tesseract_environment::ChangeJointAccelerationLimitsCommand&>(command);
      if (!changeJointAccelerationLimits(cmd.getLimits()))
        return false;
      break;
    }
    case tesseract_environment::CommandType::ADD_KINEMATICS_INFORMATION:
    {
      const auto& cmd = static_cast<const tesseract_environment::AddKinematicsInformationCommand&>(command);
      if (!manipulator_manager_->addKinematicsInformation(cmd.getKinematicsInformation()))
        return false;
      break;
    }
    default:
    {
      CONSOLE_BRIDGE_logError("Unhandled environment command");
      return false;
    }
  }

  return true;
}

bool Environment::checkInitialized() const { return initialized_; }

const tesseract_scene_graph::SceneGraph::ConstPtr& Environment::getSceneGraph() const { return scene_graph_const_; }

ManipulatorManager::Ptr Environment::getManipulatorManager() { return manipulator_manager_; }

ManipulatorManager::ConstPtr Environment::getManipulatorManager() const { return manipulator_manager_; }

bool Environment::addKinematicsInformation(const tesseract_scene_graph::KinematicsInformation& kin_info)
{
  std::lock_guard<std::mutex> lock(mutex_);
  if (!manipulator_manager_->addKinematicsInformation(kin_info))
    return false;

  ++revision_;
  commands_.push_back(std::make_shared<AddKinematicsInformationCommand>(kin_info));
  return true;
}

void Environment::setName(const std::string& name) { scene_graph_->setName(name); }

const std::string& Environment::getName() const { return scene_graph_->getName(); }

void Environment::setState(const std::unordered_map<std::string, double>& joints)
{
  std::lock_guard<std::mutex> lock(mutex_);
  state_solver_->setState(joints);
  currentStateChanged();
}

void Environment::setState(const std::vector<std::string>& joint_names, const std::vector<double>& joint_values)
{
  std::lock_guard<std::mutex> lock(mutex_);
  state_solver_->setState(joint_names, joint_values);
  currentStateChanged();
}

void Environment::setState(const std::vector<std::string>& joint_names,
                           const Eigen::Ref<const Eigen::VectorXd>& joint_values)
{
  std::lock_guard<std::mutex> lock(mutex_);
  state_solver_->setState(joint_names, joint_values);
  currentStateChanged();
}

EnvState::Ptr Environment::getState(const std::unordered_map<std::string, double>& joints) const
{
  std::lock_guard<std::mutex> lock(mutex_);
  EnvState::Ptr state = state_solver_->getState(joints);
  return state;
}

EnvState::Ptr Environment::getState(const std::vector<std::string>& joint_names,
                                    const std::vector<double>& joint_values) const
{
  std::lock_guard<std::mutex> lock(mutex_);
  EnvState::Ptr state = state_solver_->getState(joint_names, joint_values);
  return state;
}

EnvState::Ptr Environment::getState(const std::vector<std::string>& joint_names,
                                    const Eigen::Ref<const Eigen::VectorXd>& joint_values) const
{
  std::lock_guard<std::mutex> lock(mutex_);
  EnvState::Ptr state = state_solver_->getState(joint_names, joint_values);

  return state;
}

EnvState::ConstPtr Environment::getCurrentState() const { return current_state_; }

bool Environment::addLink(tesseract_scene_graph::Link link)
{
  std::string joint_name = "joint_" + link.getName();
  tesseract_scene_graph::Joint joint(joint_name);
  joint.type = tesseract_scene_graph::JointType::FIXED;
  joint.child_link_name = link.getName();
  joint.parent_link_name = getRootLinkName();

  return addLink(std::move(link), std::move(joint));
}

bool Environment::addLink(tesseract_scene_graph::Link link, tesseract_scene_graph::Joint joint)
{
  std::string link_name = link.getName();
  std::string joint_name = joint.getName();

  std::lock_guard<std::mutex> lock(mutex_);
  if (!scene_graph_->addLink(std::move(link), std::move(joint)))
    return false;

  // We have moved the original objects, get a pointer to them from scene_graph
  auto link_ptr = scene_graph_->getLink(link_name);
  auto joint_ptr = scene_graph_->getJoint(joint_name);
  if (!link_ptr->collision.empty())
  {
    tesseract_collision::CollisionShapesConst shapes;
    tesseract_common::VectorIsometry3d shape_poses;
    getCollisionObject(shapes, shape_poses, *link_ptr);

    if (discrete_manager_ != nullptr)
      discrete_manager_->addCollisionObject(link_name, 0, shapes, shape_poses, true);
    if (continuous_manager_ != nullptr)
      continuous_manager_->addCollisionObject(link_name, 0, shapes, shape_poses, true);
  }

  ++revision_;
  commands_.push_back(std::make_shared<AddCommand>(link_ptr, joint_ptr));

  environmentChanged();

  return true;
}

bool Environment::removeLink(const std::string& name)
{
  std::lock_guard<std::mutex> lock(mutex_);
  if (!removeLinkHelper(name))
    return false;

  ++revision_;
  commands_.push_back(std::make_shared<RemoveLinkCommand>(name));

  environmentChanged();

  return true;
}

bool Environment::moveLink(tesseract_scene_graph::Joint joint)
{
  std::lock_guard<std::mutex> lock(mutex_);
  std::vector<tesseract_scene_graph::Joint::ConstPtr> joints = scene_graph_->getInboundJoints(joint.child_link_name);
  assert(joints.size() == 1);
  if (!scene_graph_->removeJoint(joints[0]->getName()))
    return false;

  std::string joint_name = joint.getName();
  if (!scene_graph_->addJoint(std::move(joint)))
    return false;

  ++revision_;
  commands_.push_back(std::make_shared<MoveLinkCommand>(scene_graph_->getJoint(joint_name)));

  environmentChanged();

  return true;
}

tesseract_scene_graph::Link::ConstPtr Environment::getLink(const std::string& name) const
{
  std::lock_guard<std::mutex> lock(mutex_);
  tesseract_scene_graph::Link::ConstPtr link = scene_graph_->getLink(name);
  return link;
}

bool Environment::removeJoint(const std::string& name)
{
  std::lock_guard<std::mutex> lock(mutex_);
  if (scene_graph_->getJoint(name) == nullptr)
  {
    CONSOLE_BRIDGE_logWarn("Tried to remove Joint (%s) that does not exist", name.c_str());
    return false;
  }

  std::string target_link_name = scene_graph_->getTargetLink(name)->getName();

  if (!removeLinkHelper(target_link_name))
    return false;

  ++revision_;
  commands_.push_back(std::make_shared<RemoveJointCommand>(name));

  environmentChanged();

  return true;
}

bool Environment::moveJoint(const std::string& joint_name, const std::string& parent_link)
{
  std::lock_guard<std::mutex> lock(mutex_);
  if (!scene_graph_->moveJoint(joint_name, parent_link))
    return false;

  ++revision_;
  commands_.push_back(std::make_shared<MoveJointCommand>(joint_name, parent_link));

  environmentChanged();

  return true;
}

bool Environment::changeJointOrigin(const std::string& joint_name, const Eigen::Isometry3d& new_origin)
{
  std::lock_guard<std::mutex> lock(mutex_);
  if (!scene_graph_->changeJointOrigin(joint_name, new_origin))
    return false;

  ++revision_;
  commands_.push_back(std::make_shared<ChangeJointOriginCommand>(joint_name, new_origin));

  environmentChanged();

  return true;
}

bool Environment::changeJointPositionLimits(const std::string& joint_name, double lower, double upper)
{
  std::lock_guard<std::mutex> lock(mutex_);
  tesseract_scene_graph::JointLimits::ConstPtr jl = scene_graph_->getJointLimits(joint_name);
  if (jl == nullptr)
    return false;

  tesseract_scene_graph::JointLimits jl_copy = *jl;
  jl_copy.lower = lower;
  jl_copy.upper = upper;

  if (!scene_graph_->changeJointLimits(joint_name, jl_copy))
    return false;

  // TODO: Only change joint limits rather than completely rebuilding kinematics
  if (!manipulator_manager_->update())
    return false;

  ++revision_;
  commands_.push_back(std::make_shared<ChangeJointPositionLimitsCommand>(joint_name, lower, upper));

  environmentChanged();

  return true;
}

bool Environment::changeJointPositionLimits(const std::unordered_map<std::string, std::pair<double, double> >& limits)
{
  std::lock_guard<std::mutex> lock(mutex_);
  for (const auto& l : limits)
  {
    tesseract_scene_graph::JointLimits::ConstPtr jl = scene_graph_->getJointLimits(l.first);
    if (jl == nullptr)
      return false;

    tesseract_scene_graph::JointLimits jl_copy = *jl;
    jl_copy.lower = l.second.first;
    jl_copy.upper = l.second.second;

    if (!scene_graph_->changeJointLimits(l.first, jl_copy))
      return false;
  }

  // TODO: Only change joint limits rather than completely rebuilding kinematics
  if (!manipulator_manager_->update())
    return false;

  ++revision_;
  commands_.push_back(std::make_shared<ChangeJointPositionLimitsCommand>(limits));

  environmentChanged();

  return true;
}

bool Environment::changeJointVelocityLimits(const std::string& joint_name, double limit)
{
  std::lock_guard<std::mutex> lock(mutex_);
  tesseract_scene_graph::JointLimits::ConstPtr jl = scene_graph_->getJointLimits(joint_name);
  if (jl == nullptr)
    return false;

  tesseract_scene_graph::JointLimits jl_copy = *jl;
  jl_copy.velocity = limit;

  if (!scene_graph_->changeJointLimits(joint_name, jl_copy))
    return false;

  // TODO: Only change joint limits rather than completely rebuilding kinematics
  if (!manipulator_manager_->update())
    return false;

  ++revision_;
  commands_.push_back(std::make_shared<ChangeJointVelocityLimitsCommand>(joint_name, limit));

  environmentChanged();

  return true;
}

bool Environment::changeJointVelocityLimits(const std::unordered_map<std::string, double>& limits)
{
  std::lock_guard<std::mutex> lock(mutex_);
  for (const auto& l : limits)
  {
    tesseract_scene_graph::JointLimits::ConstPtr jl = scene_graph_->getJointLimits(l.first);
    if (jl == nullptr)
      return false;

    tesseract_scene_graph::JointLimits jl_copy = *jl;
    jl_copy.velocity = l.second;

    if (!scene_graph_->changeJointLimits(l.first, jl_copy))
      return false;
  }

  // TODO: Only change joint limits rather than completely rebuilding kinematics
  if (!manipulator_manager_->update())
    return false;

  ++revision_;
  commands_.push_back(std::make_shared<ChangeJointVelocityLimitsCommand>(limits));

  environmentChanged();

  return true;
}

bool Environment::changeJointAccelerationLimits(const std::string& joint_name, double limit)
{
  std::lock_guard<std::mutex> lock(mutex_);
  tesseract_scene_graph::JointLimits::ConstPtr jl = scene_graph_->getJointLimits(joint_name);
  if (jl == nullptr)
    return false;

  tesseract_scene_graph::JointLimits jl_copy = *jl;
  jl_copy.acceleration = limit;

  if (!scene_graph_->changeJointLimits(joint_name, jl_copy))
    return false;

  // TODO: Only change joint limits rather than completely rebuilding kinematics
  if (!manipulator_manager_->update())
    return false;

  ++revision_;
  commands_.push_back(std::make_shared<ChangeJointAccelerationLimitsCommand>(joint_name, limit));

  environmentChanged();

  return true;
}

bool Environment::changeJointAccelerationLimits(const std::unordered_map<std::string, double>& limits)
{
  std::lock_guard<std::mutex> lock(mutex_);
  for (const auto& l : limits)
  {
    tesseract_scene_graph::JointLimits::ConstPtr jl = scene_graph_->getJointLimits(l.first);
    if (jl == nullptr)
      return false;

    tesseract_scene_graph::JointLimits jl_copy = *jl;
    jl_copy.acceleration = l.second;

    if (!scene_graph_->changeJointLimits(l.first, jl_copy))
      return false;
  }

  // TODO: Only change joint limits rather than completely rebuilding kinematics
  if (!manipulator_manager_->update())
    return false;

  ++revision_;
  commands_.push_back(std::make_shared<ChangeJointAccelerationLimitsCommand>(limits));

  environmentChanged();

  return true;
}

tesseract_scene_graph::JointLimits::ConstPtr Environment::getJointLimits(const std::string& joint_name) const
{
  return scene_graph_->getJointLimits(joint_name);
}

void Environment::setLinkCollisionEnabled(const std::string& name, bool enabled)
{
  std::lock_guard<std::mutex> lock(mutex_);
  if (discrete_manager_ != nullptr)
  {
    if (enabled)
      discrete_manager_->enableCollisionObject(name);
    else
      discrete_manager_->disableCollisionObject(name);
  }

  if (continuous_manager_ != nullptr)
  {
    if (enabled)
      continuous_manager_->enableCollisionObject(name);
    else
      continuous_manager_->disableCollisionObject(name);
  }

  scene_graph_->setLinkCollisionEnabled(name, enabled);

  ++revision_;
  commands_.push_back(std::make_shared<ChangeLinkCollisionEnabledCommand>(name, enabled));
}

bool Environment::getLinkCollisionEnabled(const std::string& name) const
{
  std::lock_guard<std::mutex> lock(mutex_);
  bool enabled = scene_graph_->getLinkCollisionEnabled(name);

  return enabled;
}

void Environment::setLinkVisibility(const std::string& name, bool visibility)
{
  std::lock_guard<std::mutex> lock(mutex_);
  scene_graph_->setLinkVisibility(name, visibility);

  ++revision_;
  commands_.push_back(std::make_shared<ChangeLinkVisibilityCommand>(name, visibility));
}

bool Environment::getLinkVisibility(const std::string& name) const
{
  std::lock_guard<std::mutex> lock(mutex_);
  bool visible = scene_graph_->getLinkVisibility(name);

  return visible;
}

void Environment::addAllowedCollision(const std::string& link_name1,
                                      const std::string& link_name2,
                                      const std::string& reason)
{
  std::lock_guard<std::mutex> lock(mutex_);
  scene_graph_->addAllowedCollision(link_name1, link_name2, reason);

  ++revision_;
  commands_.push_back(std::make_shared<AddAllowedCollisionCommand>(link_name1, link_name2, reason));
}

void Environment::removeAllowedCollision(const std::string& link_name1, const std::string& link_name2)
{
  std::lock_guard<std::mutex> lock(mutex_);
  scene_graph_->removeAllowedCollision(link_name1, link_name2);

  ++revision_;
  commands_.push_back(std::make_shared<RemoveAllowedCollisionCommand>(link_name1, link_name2));
}

void Environment::removeAllowedCollision(const std::string& link_name)
{
  std::lock_guard<std::mutex> lock(mutex_);
  scene_graph_->removeAllowedCollision(link_name);

  ++revision_;
  commands_.push_back(std::make_shared<RemoveAllowedCollisionLinkCommand>(link_name));
}

tesseract_scene_graph::AllowedCollisionMatrix::ConstPtr Environment::getAllowedCollisionMatrix() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  tesseract_scene_graph::AllowedCollisionMatrix::ConstPtr acm = scene_graph_->getAllowedCollisionMatrix();

  return acm;
}

tesseract_scene_graph::Joint::ConstPtr Environment::getJoint(const std::string& name) const
{
  std::lock_guard<std::mutex> lock(mutex_);
  tesseract_scene_graph::Joint::ConstPtr joint = scene_graph_->getJoint(name);

  return joint;
}

Eigen::VectorXd Environment::getCurrentJointValues() const
{
  std::lock_guard<std::mutex> lock(mutex_);
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
  std::lock_guard<std::mutex> lock(mutex_);
  Eigen::VectorXd jv;
  jv.resize(static_cast<long int>(joint_names.size()));
  for (auto j = 0u; j < joint_names.size(); ++j)
  {
    jv(j) = current_state_->joints[joint_names[j]];
  }

  return jv;
}

tesseract_common::VectorIsometry3d Environment::getLinkTransforms() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  tesseract_common::VectorIsometry3d link_tfs;
  link_tfs.resize(link_names_.size());
  for (const auto& link_name : link_names_)
  {
    link_tfs.push_back(current_state_->link_transforms[link_name]);
  }

  return link_tfs;
}

const Eigen::Isometry3d& Environment::getLinkTransform(const std::string& link_name) const
{
  std::lock_guard<std::mutex> lock(mutex_);
  const Eigen::Isometry3d& tf = current_state_->link_transforms[link_name];

  return tf;
}

StateSolver::Ptr Environment::getStateSolver() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  StateSolver::Ptr state_solver = state_solver_->clone();

  return state_solver;
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

bool Environment::setActiveDiscreteContactManager(const std::string& name)
{
  std::lock_guard<std::mutex> lock(mutex_);
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

tesseract_collision::DiscreteContactManager::Ptr Environment::getDiscreteContactManager(const std::string& name) const
{
  std::lock_guard<std::mutex> lock(mutex_);
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
  std::lock_guard<std::mutex> lock(mutex_);
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

tesseract_collision::DiscreteContactManager::Ptr Environment::getDiscreteContactManager() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  if (!discrete_manager_)
    return nullptr;
  return discrete_manager_->clone();
}

tesseract_collision::ContinuousContactManager::Ptr Environment::getContinuousContactManager() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  if (!continuous_manager_)
    return nullptr;
  return continuous_manager_->clone();
}

tesseract_collision::ContinuousContactManager::Ptr
Environment::getContinuousContactManager(const std::string& name) const
{
  std::lock_guard<std::mutex> lock(mutex_);
  tesseract_collision::ContinuousContactManager::Ptr manager = getContinuousContactManagerHelper(name);
  if (manager == nullptr)
  {
    CONSOLE_BRIDGE_logError("Continuous manager with %s does not exist in factory!", name.c_str());
    return nullptr;
  }

  return manager;
}

bool Environment::registerDiscreteContactManager(
    const std::string& name,
    tesseract_collision::DiscreteContactManagerFactory::CreateMethod create_function)
{
  std::lock_guard<std::mutex> lock(mutex_);
  return discrete_factory_.registar(name, std::move(create_function));
}

bool Environment::registerContinuousContactManager(
    const std::string& name,
    tesseract_collision::ContinuousContactManagerFactory::CreateMethod create_function)
{
  std::lock_guard<std::mutex> lock(mutex_);
  return continuous_factory_.registar(name, std::move(create_function));
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

  return manager;
}

void Environment::currentStateChanged()
{
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

bool Environment::addSceneGraph(const tesseract_scene_graph::SceneGraph& scene_graph, const std::string& prefix)
{
  if (scene_graph_->isEmpty())
  {
    if (!scene_graph_->insertSceneGraph(scene_graph, prefix))
      return false;

    ++revision_;
    commands_.push_back(std::make_shared<AddSceneGraphCommand>(scene_graph, nullptr, prefix));

    environmentChanged();
    return true;
  }

  // Connect root of subgraph to graph
  tesseract_scene_graph::Joint root_joint(prefix + scene_graph.getName() + "_joint");
  root_joint.type = tesseract_scene_graph::JointType::FIXED;
  root_joint.parent_link_name = getRootLinkName();
  root_joint.child_link_name = prefix + scene_graph.getRoot();
  root_joint.parent_to_joint_origin_transform = Eigen::Isometry3d::Identity();

  return addSceneGraph(scene_graph, std::move(root_joint), prefix);
}

bool Environment::addSceneGraph(const tesseract_scene_graph::SceneGraph& scene_graph,
                                tesseract_scene_graph::Joint joint,
                                const std::string& prefix)
{
  std::string joint_name = joint.getName();
  if (!scene_graph_->insertSceneGraph(scene_graph, std::move(joint), prefix))
    return false;

  ++revision_;
  commands_.push_back(std::make_shared<AddSceneGraphCommand>(scene_graph, getJoint(joint_name), prefix));

  environmentChanged();
  return true;
}

Environment::Ptr Environment::clone() const
{
  auto cloned_env = std::make_shared<Environment>();

  std::lock_guard<std::mutex> lock(mutex_);
  cloned_env->initialized_ = initialized_;
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
}  // namespace tesseract_environment
