/**
 * @file environment.cpp
 * @brief Tesseract environment interface implementation.
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

#include <tesseract/environment/environment.h>
#include <tesseract/environment/utils.h>
#include <tesseract/environment/events.h>
#include <tesseract/environment/command.h>
#include <tesseract/environment/commands.h>

#include <tesseract/geometry/utils.h>

#include <tesseract/scene_graph/graph.h>
#include <tesseract/scene_graph/link.h>
#include <tesseract/scene_graph/joint.h>
#include <tesseract/scene_graph/scene_state.h>

#include <tesseract/urdf/urdf_parser.h>

#include <tesseract/srdf/srdf_model.h>
#include <tesseract/srdf/kinematics_information.h>
#include <tesseract/srdf/utils.h>

#include <tesseract/kinematics/joint_group.h>
#include <tesseract/kinematics/kinematic_group.h>
#include <tesseract/kinematics/kinematics_plugin_factory.h>
#include <tesseract/kinematics/inverse_kinematics.h>
#include <tesseract/kinematics/validate.h>

#include <tesseract/collision/common.h>
#include <tesseract/collision/types.h>
#include <tesseract/collision/bullet/convex_hull_utils.h>
#include <tesseract/collision/discrete_contact_manager.h>
#include <tesseract/collision/continuous_contact_manager.h>

#include <tesseract/common/manipulator_info.h>
#include <tesseract/common/resource_locator.h>
#include <tesseract/common/eigen_types.h>
#include <tesseract/common/types.h>

#include <tesseract/state_solver/mutable_state_solver.h>
#include <tesseract/state_solver/ofkt/ofkt_state_solver.h>

#include <tesseract/collision/contact_managers_plugin_factory.h>
#include <tesseract/common/collision_margin_data.h>

#include <console_bridge/console.h>

#include <utility>

namespace tesseract::environment
{
EnvironmentContactAllowedValidator::EnvironmentContactAllowedValidator(
    std::shared_ptr<const tesseract::scene_graph::SceneGraph> scene_graph)
  : scene_graph_(std::move(scene_graph))
{
}

bool EnvironmentContactAllowedValidator::operator()(const tesseract::common::LinkIdPair& pair) const
{
  return scene_graph_->isCollisionAllowed(pair);
}

void getCollisionObject(std::vector<std::shared_ptr<const tesseract::geometry::Geometry>>& shapes,
                        tesseract::common::VectorIsometry3d& shape_poses,
                        const tesseract::scene_graph::Link& link)
{
  for (const auto& c : link.collision)
  {
    shapes.push_back(c->geometry);
    shape_poses.push_back(c->origin);
  }
}

std::vector<std::shared_ptr<const Command>>
getInitCommands(const tesseract::scene_graph::SceneGraph& scene_graph,
                const std::shared_ptr<const tesseract::srdf::SRDFModel>& srdf_model = nullptr)
{
  Commands commands;

  tesseract::scene_graph::SceneGraph::Ptr local_sg = scene_graph.clone();
  if (local_sg == nullptr)
  {
    CONSOLE_BRIDGE_logError("Null pointer to Scene Graph");
    return {};
  }

  if (!local_sg->getLink(local_sg->getRoot()))
  {
    CONSOLE_BRIDGE_logError("The scene graph has an invalid root.");
    return {};
  }

  if (srdf_model != nullptr)
    tesseract::srdf::processSRDFAllowedCollisions(*local_sg, *srdf_model);

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
    {
      commands.push_back(std::make_shared<ChangeCollisionMarginsCommand>(
          srdf_model->collision_margin_data->getDefaultCollisionMargin(),
          srdf_model->collision_margin_data->getCollisionMarginPairData(),
          tesseract::common::CollisionMarginPairOverrideType::REPLACE));
    }
  }

  return commands;
}

struct Environment::Implementation
{
  ~Implementation() = default;

  /**
   * @brief Identifies if the object has been initialized
   * @note This is intentionally not serialized it will auto updated
   */
  bool initialized{ false };

  /**
   * @brief This increments when the scene graph is modified
   * @note This is intentionally not serialized it will auto updated
   */
  int revision{ 0 };

  /** @brief This is the revision number after initialization used when reset is called */
  int init_revision{ 0 };

  /** @brief The history of commands applied to the environment after initialization */
  std::vector<std::shared_ptr<const Command>> commands;

  /**
   * @brief Tesseract Scene Graph
   * @note This is intentionally not serialized it will auto updated
   */
  std::shared_ptr<tesseract::scene_graph::SceneGraph> scene_graph{ nullptr };

  /** @brief Current state of the environment */
  tesseract::scene_graph::SceneState current_state;

  /** @brief Environment timestamp */
  std::chrono::system_clock::time_point timestamp{ std::chrono::system_clock::now() };

  /** @brief Current state timestamp */
  std::chrono::system_clock::time_point current_state_timestamp{ std::chrono::system_clock::now() };

  /**
   * @brief Tesseract State Solver
   * @note This is intentionally not serialized it will auto updated
   */
  std::unique_ptr<tesseract::scene_graph::MutableStateSolver> state_solver{ nullptr };

  /**
   * @brief The validator used to determine if two objects are allowed in collision
   */
  tesseract::common::ContactAllowedValidator::ConstPtr contact_allowed_validator;

  /**
   * @brief A vector of user defined callbacks for locating tool center point
   * @todo This needs to be switched to class so it may be serialized
   */
  std::vector<FindTCPOffsetCallbackFn> find_tcp_cb;

  /**
   * @brief A map of user defined event callback functions
   * @details This should not be cloned or serialized
   */
  std::map<std::size_t, EventCallbackFn> event_cb;

  /** @brief Used when initialized by URDF_STRING, URDF_STRING_SRDF_STRING, URDF_PATH, and URDF_PATH_SRDF_PATH */
  std::shared_ptr<const tesseract::common::ResourceLocator> resource_locator{ nullptr };

  /**
   * @brief The collision margin data
   * @note This is intentionally not serialized it will auto updated
   */
  tesseract::common::CollisionMarginData collision_margin_data;

  /**
   * @brief The kinematics information
   * @note This is intentionally not serialized it will auto updated
   */
  tesseract::srdf::KinematicsInformation kinematics_information;

  /**
   * @brief The kinematics factory
   * @note This is intentionally not serialized it will auto updated
   */
  tesseract::kinematics::KinematicsPluginFactory kinematics_factory;

  /**
   * @brief The contact manager information
   * @note This is intentionally not serialized it will auto updated
   */
  tesseract::common::ContactManagersPluginInfo contact_managers_plugin_info;

  /**
   * @brief The contact managers factory
   * @note This is intentionally not serialized it will auto updated
   */
  tesseract::collision::ContactManagersPluginFactory contact_managers_factory;

  /**
   * @brief The discrete contact manager object
   * @note This is intentionally not serialized it will auto updated
   */
  mutable std::unique_ptr<tesseract::collision::DiscreteContactManager> discrete_manager{ nullptr };
  mutable std::shared_mutex discrete_manager_mutex;

  /**
   * @brief The continuous contact manager object
   * @note This is intentionally not serialized it will auto updated
   */
  mutable std::unique_ptr<tesseract::collision::ContinuousContactManager> continuous_manager{ nullptr };
  mutable std::shared_mutex continuous_manager_mutex;

  /**
   * @brief A cache of group joint names to provide faster access
   * @details This will cleared when environment changes
   * @note This is intentionally not serialized it will auto updated
   */
  mutable std::unordered_map<std::string, std::vector<tesseract::common::JointId>> group_joint_names_cache;
  mutable std::shared_mutex group_joint_names_cache_mutex;

  /**
   * @brief A cache of joint groups to provide faster access
   * @details This will cleared when environment changes
   * @note This is intentionally not serialized it will auto updated
   */
  mutable std::unordered_map<std::string, std::shared_ptr<const tesseract::kinematics::JointGroup>> joint_group_cache;
  mutable std::shared_mutex joint_group_cache_mutex;

  /**
   * @brief A cache of kinematic groups to provide faster access
   * @details This will cleared when environment changes
   * @note This is intentionally not serialized it will auto updated
   */
  mutable std::map<std::pair<std::string, std::string>, std::shared_ptr<const tesseract::kinematics::KinematicGroup>>
      kinematic_group_cache;
  mutable std::shared_mutex kinematic_group_cache_mutex;

  bool operator==(const Implementation& rhs) const;

  bool initHelper(const std::vector<std::shared_ptr<const Command>>& commands);

  void setState(const std::unordered_map<std::string, double>& joints,
                const tesseract::common::JointIdTransformMap& floating_joints = {});

  void setState(const std::vector<std::string>& joint_names,
                const Eigen::Ref<const Eigen::VectorXd>& joint_values,
                const tesseract::common::JointIdTransformMap& floating_joints = {});

  void setState(const tesseract::common::JointIdTransformMap& floating_joints);

  void setState(const tesseract::scene_graph::SceneState::JointValues& joint_values,
                const tesseract::common::JointIdTransformMap& floating_joints = {});

  void setState(const std::vector<tesseract::common::JointId>& joint_ids,
                const Eigen::Ref<const Eigen::VectorXd>& joint_values,
                const tesseract::common::JointIdTransformMap& floating_joints = {});

  Eigen::VectorXd getCurrentJointValues() const;

  Eigen::VectorXd getCurrentJointValues(const std::vector<std::string>& joint_names) const;

  Eigen::VectorXd getCurrentJointValues(const std::vector<tesseract::common::JointId>& joint_ids) const;

  tesseract::common::JointIdTransformMap getCurrentFloatingJointValues() const;

  tesseract::common::JointIdTransformMap
  getCurrentFloatingJointValues(const std::vector<std::string>& joint_names) const;

  tesseract::common::JointIdTransformMap
  getCurrentFloatingJointValues(const std::vector<tesseract::common::JointId>& joint_ids) const;

  std::vector<std::string> getStaticLinkNames(const std::vector<std::string>& joint_names) const;

  std::vector<common::LinkId> getStaticLinkIds(const std::vector<common::JointId>& joint_ids) const;

  void clear();

  bool reset();

  /** This will update the contact managers transforms */
  void currentStateChanged();

  /** This will notify the state solver that the environment has changed */
  void environmentChanged();

  /** @brief @brief Passes a current state changed event to the callbacks */
  void triggerCurrentStateChangedCallbacks();

  /** @brief Passes a environment changed event to the callbacks */
  void triggerEnvironmentChangedCallbacks();

  /** @brief Trigger both Current State and Environment changed callback */
  void triggerCallbacks();

  std::vector<std::string> getGroupJointNames(const std::string& group_name) const;

  std::vector<tesseract::common::JointId> getGroupJointIds(const std::string& group_name) const;

  std::shared_ptr<const tesseract::kinematics::JointGroup> getJointGroup(const std::string& group_name) const;

  std::shared_ptr<const tesseract::kinematics::JointGroup>
  getJointGroup(const std::string& name, const std::vector<tesseract::common::JointId>& joint_ids) const;

  std::shared_ptr<const tesseract::kinematics::KinematicGroup> getKinematicGroup(const std::string& group_name,
                                                                                 std::string ik_solver_name) const;

  Eigen::Isometry3d findTCPOffset(const tesseract::common::ManipulatorInfo& manip_info) const;

  std::unique_ptr<tesseract::collision::DiscreteContactManager> getDiscreteContactManager() const;

  std::unique_ptr<tesseract::collision::ContinuousContactManager> getContinuousContactManager() const;

  void clearCachedDiscreteContactManager() const;

  void clearCachedContinuousContactManager() const;

  bool setActiveDiscreteContactManagerHelper(const std::string& name);

  bool setActiveContinuousContactManagerHelper(const std::string& name);

  std::unique_ptr<tesseract::collision::DiscreteContactManager>
  getDiscreteContactManagerHelper(const std::string& name) const;

  std::unique_ptr<tesseract::collision::ContinuousContactManager>
  getContinuousContactManagerHelper(const std::string& name) const;

  bool setActiveDiscreteContactManager(const std::string& name);

  std::unique_ptr<tesseract::collision::DiscreteContactManager>
  getDiscreteContactManager(const std::string& name) const;

  bool setActiveContinuousContactManager(const std::string& name);

  std::unique_ptr<tesseract::collision::ContinuousContactManager>
  getContinuousContactManager(const std::string& name) const;

  bool removeLinkHelper(const common::LinkId& id);

  /** @brief Apply Command Helper which does not lock */
  bool applyCommandsHelper(const std::vector<std::shared_ptr<const Command>>& commands);

  // Command Helper function
  bool applyAddCommand(const std::shared_ptr<const AddLinkCommand>& cmd);
  bool applyMoveLinkCommand(const std::shared_ptr<const MoveLinkCommand>& cmd);
  bool applyMoveJointCommand(const std::shared_ptr<const MoveJointCommand>& cmd);
  bool applyRemoveLinkCommand(const std::shared_ptr<const RemoveLinkCommand>& cmd);
  bool applyRemoveJointCommand(const std::shared_ptr<const RemoveJointCommand>& cmd);
  bool applyReplaceJointCommand(const std::shared_ptr<const ReplaceJointCommand>& cmd);
  bool applyChangeLinkOriginCommand(const std::shared_ptr<const ChangeLinkOriginCommand>& cmd);
  bool applyChangeJointOriginCommand(const std::shared_ptr<const ChangeJointOriginCommand>& cmd);
  bool applyChangeLinkCollisionEnabledCommand(const std::shared_ptr<const ChangeLinkCollisionEnabledCommand>& cmd);
  bool applyChangeLinkVisibilityCommand(const std::shared_ptr<const ChangeLinkVisibilityCommand>& cmd);
  bool applyModifyAllowedCollisionsCommand(const std::shared_ptr<const ModifyAllowedCollisionsCommand>& cmd);
  bool applyRemoveAllowedCollisionLinkCommand(const std::shared_ptr<const RemoveAllowedCollisionLinkCommand>& cmd);
  bool applyAddSceneGraphCommand(std::shared_ptr<const AddSceneGraphCommand> cmd);
  bool applyChangeJointPositionLimitsCommand(const std::shared_ptr<const ChangeJointPositionLimitsCommand>& cmd);
  bool applyChangeJointVelocityLimitsCommand(const std::shared_ptr<const ChangeJointVelocityLimitsCommand>& cmd);
  bool
  applyChangeJointAccelerationLimitsCommand(const std::shared_ptr<const ChangeJointAccelerationLimitsCommand>& cmd);
  bool applyAddKinematicsInformationCommand(const std::shared_ptr<const AddKinematicsInformationCommand>& cmd);
  bool applyChangeCollisionMarginsCommand(const std::shared_ptr<const ChangeCollisionMarginsCommand>& cmd);
  bool applyAddContactManagersPluginInfoCommand(const std::shared_ptr<const AddContactManagersPluginInfoCommand>& cmd);
  bool applySetActiveContinuousContactManagerCommand(
      const std::shared_ptr<const SetActiveContinuousContactManagerCommand>& cmd);
  bool
  applySetActiveDiscreteContactManagerCommand(const std::shared_ptr<const SetActiveDiscreteContactManagerCommand>& cmd);
  bool applyAddTrajectoryLinkCommand(const std::shared_ptr<const AddTrajectoryLinkCommand>& cmd);

  bool applyAddLinkCommandHelper(const std::shared_ptr<const tesseract::scene_graph::Link>& link,
                                 const std::shared_ptr<const tesseract::scene_graph::Joint>& joint,
                                 bool replace_allowed);

  std::unique_ptr<Implementation> clone() const;
};

bool Environment::Implementation::operator==(const Environment::Implementation& rhs) const
{
  // No need to check everything mainly the items serialized
  bool equal = true;
  equal &= initialized == rhs.initialized;
  equal &= revision == rhs.revision;
  equal &= init_revision == rhs.init_revision;
  equal &= commands.size() == rhs.commands.size();
  if (!equal)
    return equal;

  for (std::size_t i = 0; i < commands.size(); ++i)
  {
    equal &= *(commands[i]) == *(rhs.commands[i]);
    if (!equal)
      return equal;
  }

  equal &= current_state == rhs.current_state;
  equal &= timestamp == rhs.timestamp;
  equal &= current_state_timestamp == rhs.current_state_timestamp;

  // No need to check contact_allowed validator because it is constructed internally from the scene graph and cannot be
  // changed

  /** @todo uncomment after serialized */
  //  equal &= find_tcp_cb == rhs.find_tcp_cb;

  return equal;
}

std::unique_ptr<Environment::Implementation> Environment::Implementation::clone() const
{
  auto cloned_env = std::make_unique<Implementation>();

  std::shared_lock<std::shared_mutex> jg_lock(joint_group_cache_mutex);
  std::shared_lock<std::shared_mutex> kg_lock(kinematic_group_cache_mutex);
  std::shared_lock<std::shared_mutex> jn_lock(group_joint_names_cache_mutex);
  std::shared_lock<std::shared_mutex> discrete_lock(discrete_manager_mutex);
  std::shared_lock<std::shared_mutex> continuous_lock(continuous_manager_mutex);

  if (!initialized)
    return cloned_env;

  cloned_env->initialized = initialized;
  cloned_env->init_revision = init_revision;
  cloned_env->revision = revision;
  cloned_env->commands = commands;
  cloned_env->scene_graph = scene_graph->clone();
  cloned_env->timestamp = timestamp;
  cloned_env->current_state = current_state;
  cloned_env->current_state_timestamp = current_state_timestamp;
  cloned_env->resource_locator = resource_locator;

  // There is not dynamic pointer cast for std::unique_ptr
  auto cloned_solver = state_solver->clone();
  auto* p = dynamic_cast<tesseract::scene_graph::MutableStateSolver*>(cloned_solver.get());
  if (p != nullptr)
    (void)cloned_solver.release();  // NOLINT

  cloned_env->state_solver = std::unique_ptr<tesseract::scene_graph::MutableStateSolver>(p);
  cloned_env->kinematics_information = kinematics_information;
  cloned_env->kinematics_factory = kinematics_factory;
  cloned_env->find_tcp_cb = find_tcp_cb;
  cloned_env->collision_margin_data = collision_margin_data;

  // Copy cache
  cloned_env->joint_group_cache = joint_group_cache;
  cloned_env->kinematic_group_cache = kinematic_group_cache;
  cloned_env->group_joint_names_cache = group_joint_names_cache;

  // NOLINTNEXTLINE
  cloned_env->contact_allowed_validator = std::make_shared<EnvironmentContactAllowedValidator>(cloned_env->scene_graph);

  if (discrete_manager)
  {
    cloned_env->discrete_manager = discrete_manager->clone();
    cloned_env->discrete_manager->setContactAllowedValidator(cloned_env->contact_allowed_validator);
  }
  if (continuous_manager)
  {
    cloned_env->continuous_manager = continuous_manager->clone();
    cloned_env->continuous_manager->setContactAllowedValidator(cloned_env->contact_allowed_validator);
  }

  cloned_env->contact_managers_plugin_info = contact_managers_plugin_info;
  cloned_env->contact_managers_factory = contact_managers_factory;

  return cloned_env;
}

bool Environment::Implementation::initHelper(const std::vector<std::shared_ptr<const Command>>& commands)
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

  scene_graph = std::make_shared<tesseract::scene_graph::SceneGraph>(
      std::static_pointer_cast<const AddSceneGraphCommand>(commands.at(0))->getSceneGraph()->getName());

  contact_allowed_validator = std::make_shared<EnvironmentContactAllowedValidator>(scene_graph);

  if (!applyCommandsHelper(commands))
  {
    CONSOLE_BRIDGE_logError("When initializing environment from command history, it failed to apply a command!");
    return false;
  }

  initialized = true;
  init_revision = revision;

  environmentChanged();

  return initialized;
}

void Environment::Implementation::setState(const std::unordered_map<std::string, double>& joints,
                                           const tesseract::common::JointIdTransformMap& floating_joints)
{
  tesseract::scene_graph::SceneState::JointValues id_map;
  for (const auto& [name, val] : joints)
    id_map[tesseract::common::JointId(name)] = val;
  state_solver->setState(id_map, floating_joints);
  currentStateChanged();
}

void Environment::Implementation::setState(const std::vector<std::string>& joint_names,
                                           const Eigen::Ref<const Eigen::VectorXd>& joint_values,
                                           const tesseract::common::JointIdTransformMap& floating_joints)
{
  state_solver->setState(
      tesseract::common::toIds<tesseract::common::JointId>(joint_names), joint_values, floating_joints);
  currentStateChanged();
}

void Environment::Implementation::setState(const tesseract::common::JointIdTransformMap& floating_joints)
{
  state_solver->setState(floating_joints);
  currentStateChanged();
}

void Environment::Implementation::setState(const tesseract::scene_graph::SceneState::JointValues& joint_values,
                                           const tesseract::common::JointIdTransformMap& floating_joints)
{
  state_solver->setState(joint_values, floating_joints);
  currentStateChanged();
}

void Environment::Implementation::setState(const std::vector<tesseract::common::JointId>& joint_ids,
                                           const Eigen::Ref<const Eigen::VectorXd>& joint_values,
                                           const tesseract::common::JointIdTransformMap& floating_joints)
{
  state_solver->setState(joint_ids, joint_values, floating_joints);
  currentStateChanged();
}

Eigen::VectorXd Environment::Implementation::getCurrentJointValues() const
{
  return current_state.getJointValues(state_solver->getActiveJointIds());
}

Eigen::VectorXd Environment::Implementation::getCurrentJointValues(const std::vector<std::string>& joint_names) const
{
  return current_state.getJointValues(common::toIds<common::JointId>(joint_names));
}

Eigen::VectorXd
Environment::Implementation::getCurrentJointValues(const std::vector<tesseract::common::JointId>& joint_ids) const
{
  return current_state.getJointValues(joint_ids);
}

tesseract::common::JointIdTransformMap Environment::Implementation::getCurrentFloatingJointValues() const
{
  return current_state.getFloatingJointValues(state_solver->getFloatingJointIds());
}

tesseract::common::JointIdTransformMap
Environment::Implementation::getCurrentFloatingJointValues(const std::vector<std::string>& joint_names) const
{
  return current_state.getFloatingJointValues(common::toIds<common::JointId>(joint_names));
}

tesseract::common::JointIdTransformMap Environment::Implementation::getCurrentFloatingJointValues(
    const std::vector<tesseract::common::JointId>& joint_ids) const
{
  return current_state.getFloatingJointValues(joint_ids);
}

std::vector<std::string>
Environment::Implementation::getStaticLinkNames(const std::vector<std::string>& joint_names) const
{
  return tesseract::common::toNames(
      getStaticLinkIds(tesseract::common::toIds<tesseract::common::JointId>(joint_names)));
}

std::vector<common::LinkId>
Environment::Implementation::getStaticLinkIds(const std::vector<common::JointId>& joint_ids) const
{
  std::vector<common::LinkId> active_link_ids = scene_graph->getJointChildrenIds(joint_ids);
  std::vector<common::LinkId> full_link_ids = state_solver->getLinkIds();
  std::vector<common::LinkId> static_link_ids;
  static_link_ids.reserve(full_link_ids.size());

  std::sort(active_link_ids.begin(), active_link_ids.end());
  std::sort(full_link_ids.begin(), full_link_ids.end());

  std::set_difference(full_link_ids.begin(),
                      full_link_ids.end(),
                      active_link_ids.begin(),
                      active_link_ids.end(),
                      std::inserter(static_link_ids, static_link_ids.begin()));

  return static_link_ids;
}

void Environment::Implementation::clear()
{
  initialized = false;
  revision = 0;
  init_revision = 0;
  scene_graph = nullptr;
  state_solver = nullptr;
  current_state = tesseract::scene_graph::SceneState();
  commands.clear();
  contact_allowed_validator = nullptr;
  collision_margin_data = tesseract::collision::CollisionMarginData();
  kinematics_information.clear();
  contact_managers_plugin_info.clear();

  {
    std::unique_lock<std::shared_mutex> lock(discrete_manager_mutex);
    discrete_manager = nullptr;
  }

  {
    std::unique_lock<std::shared_mutex> lock(continuous_manager_mutex);
    continuous_manager = nullptr;
  }

  {
    std::unique_lock<std::shared_mutex> lock(group_joint_names_cache_mutex);
    group_joint_names_cache.clear();
  }

  {
    std::unique_lock<std::shared_mutex> lock(joint_group_cache_mutex);
    joint_group_cache.clear();
  }

  {
    std::unique_lock<std::shared_mutex> lock(kinematic_group_cache_mutex);
    kinematic_group_cache.clear();
  }

  // Must clear cache before deleting plugin factories
  kinematics_factory = tesseract::kinematics::KinematicsPluginFactory();
  contact_managers_factory = tesseract::collision::ContactManagersPluginFactory();
}

bool Environment::Implementation::reset()
{
  Commands init_command;
  if (commands.empty() || !initialized)
    return false;

  init_command.reserve(static_cast<std::size_t>(init_revision));
  for (std::size_t i = 0; i < static_cast<std::size_t>(init_revision); ++i)
    init_command.push_back(commands[i]);

  return initHelper(init_command);
}

void Environment::Implementation::currentStateChanged()
{
  timestamp = std::chrono::system_clock::now();
  current_state_timestamp = timestamp;
  current_state = state_solver->getState();

  std::unique_lock<std::shared_mutex> discrete_lock(discrete_manager_mutex);
  if (discrete_manager != nullptr)
    discrete_manager->setCollisionObjectsTransform(current_state.link_transforms);

  std::unique_lock<std::shared_mutex> continuous_lock(continuous_manager_mutex);
  if (continuous_manager != nullptr)
  {
    const std::vector<tesseract::common::LinkId> active_link_ids = state_solver->getActiveLinkIds();
    for (const auto& [id, tf] : current_state.link_transforms)
    {
      if (std::find(active_link_ids.begin(), active_link_ids.end(), id) != active_link_ids.end())
        continuous_manager->setCollisionObjectsTransform(id, tf, tf);
      else
        continuous_manager->setCollisionObjectsTransform(id, tf);
    }
  }

  {  // Clear JointGroup and KinematicGroup
    std::unique_lock<std::shared_mutex> jg_lock(joint_group_cache_mutex);
    std::unique_lock<std::shared_mutex> kg_lock(kinematic_group_cache_mutex);
    joint_group_cache.clear();
    kinematic_group_cache.clear();
  }
}

void Environment::Implementation::environmentChanged()
{
  timestamp = std::chrono::system_clock::now();
  std::vector<tesseract::common::LinkId> active_link_ids = state_solver->getActiveLinkIds();

  {
    std::unique_lock<std::shared_mutex> discrete_lock(discrete_manager_mutex);
    if (discrete_manager != nullptr)
      discrete_manager->setActiveCollisionObjects(active_link_ids);
  }

  {
    std::unique_lock<std::shared_mutex> continuous_lock(continuous_manager_mutex);
    if (continuous_manager != nullptr)
      continuous_manager->setActiveCollisionObjects(active_link_ids);
  }

  {  // Clear JointGroup, KinematicGroup and GroupJointNames cache
    std::unique_lock<std::shared_mutex> jn_lock(group_joint_names_cache_mutex);
    group_joint_names_cache.clear();
  }

  currentStateChanged();
}

void Environment::Implementation::triggerCurrentStateChangedCallbacks()
{
  if (!event_cb.empty())
  {
    SceneStateChangedEvent event(current_state);
    for (const auto& cb : event_cb)
      cb.second(event);
  }
}

void Environment::Implementation::triggerEnvironmentChangedCallbacks()
{
  if (!event_cb.empty())
  {
    CommandAppliedEvent event(commands, revision);
    for (const auto& cb : event_cb)
      cb.second(event);
  }
}

void Environment::Implementation::triggerCallbacks()
{
  triggerEnvironmentChangedCallbacks();
  triggerCurrentStateChangedCallbacks();
}

std::vector<std::string> Environment::Implementation::getGroupJointNames(const std::string& group_name) const
{
  return tesseract::common::toNames(getGroupJointIds(group_name));
}

std::vector<tesseract::common::JointId>
Environment::Implementation::getGroupJointIds(const std::string& group_name) const
{
  if (kinematics_information.group_names.find(group_name) == kinematics_information.group_names.end())
    throw std::runtime_error("Environment, Joint group '" + group_name + "' does not exist!");

  std::unique_lock<std::shared_mutex> cache_lock(group_joint_names_cache_mutex);
  auto cache_it = group_joint_names_cache.find(group_name);
  if (cache_it != group_joint_names_cache.end())
    return cache_it->second;

  auto chain_it = kinematics_information.chain_groups.find(group_name);
  if (chain_it != kinematics_information.chain_groups.end())
  {
    if (chain_it->second.size() > 1)
      throw std::runtime_error("Environment, Groups with multiple chains is not supported!");

    tesseract::scene_graph::ShortestPath path =
        scene_graph->getShortestPath(chain_it->second.begin()->first, chain_it->second.begin()->second);

    group_joint_names_cache[group_name] = std::move(path.active_joints);
    return group_joint_names_cache[group_name];
  }

  auto joint_it = kinematics_information.joint_groups.find(group_name);
  if (joint_it != kinematics_information.joint_groups.end())
  {
    group_joint_names_cache[group_name] = joint_it->second;
    return joint_it->second;
  }

  auto link_it = kinematics_information.link_groups.find(group_name);
  if (link_it != kinematics_information.link_groups.end())
    throw std::runtime_error("Environment, Link groups are currently not supported!");

  throw std::runtime_error("Environment, failed to get group '" + group_name + "' joint names!");
}

std::shared_ptr<const tesseract::kinematics::JointGroup>
Environment::Implementation::getJointGroup(const std::string& group_name) const
{
  std::unique_lock<std::shared_mutex> cache_lock(joint_group_cache_mutex);
  auto it = joint_group_cache.find(group_name);
  if (it != joint_group_cache.end())
    return it->second;

  // Store copy in cache and return
  std::vector<tesseract::common::JointId> joint_ids = getGroupJointIds(group_name);
  tesseract::kinematics::JointGroup::ConstPtr jg = getJointGroup(group_name, joint_ids);
  joint_group_cache[group_name] = jg;

  return jg;
}

std::shared_ptr<const tesseract::kinematics::JointGroup>
Environment::Implementation::getJointGroup(const std::string& name,
                                           const std::vector<tesseract::common::JointId>& joint_ids) const
{
  return std::make_shared<tesseract::kinematics::JointGroup>(name, joint_ids, *scene_graph, current_state);
}

std::shared_ptr<const tesseract::kinematics::KinematicGroup>
Environment::Implementation::getKinematicGroup(const std::string& group_name, std::string ik_solver_name) const
{
  std::unique_lock<std::shared_mutex> cache_lock(kinematic_group_cache_mutex);
  std::pair<std::string, std::string> key = std::make_pair(group_name, ik_solver_name);
  auto it = kinematic_group_cache.find(key);
  if (it != kinematic_group_cache.end())
    return it->second;

  std::vector<tesseract::common::JointId> joint_ids = getGroupJointIds(group_name);

  if (ik_solver_name.empty())
    ik_solver_name = kinematics_factory.getDefaultInvKinPlugin(group_name);

  tesseract::kinematics::InverseKinematics::UPtr inv_kin =
      kinematics_factory.createInvKin(group_name, ik_solver_name, *scene_graph, current_state);

  // TODO add error message
  if (inv_kin == nullptr)
    return nullptr;

  // Store copy in cache and return
  auto kg = std::make_shared<tesseract::kinematics::KinematicGroup>(
      group_name, joint_ids, std::move(inv_kin), *scene_graph, current_state);

  kinematic_group_cache[key] = kg;

#if !defined(NDEBUG) && TESSERACT_ENABLE_TESTING
  if (!tesseract_kinematics::checkKinematics(*kg))
  {
    CONSOLE_BRIDGE_logError("Check Kinematics failed. This means that inverse kinematics solution for a pose do not "
                            "match forward kinematics solution. Did you change the URDF recently?");
  }
#endif

  return kg;
}

Eigen::Isometry3d Environment::Implementation::findTCPOffset(const tesseract::common::ManipulatorInfo& manip_info) const
{
  // If it is already an Isometry3d return the offset
  if (manip_info.tcp_offset.index() != 0)
    return std::get<1>(manip_info.tcp_offset);

  // Check if the tcp offset name is a link in the scene, if so throw an exception
  const common::LinkId& tcp_offset = std::get<0>(manip_info.tcp_offset);
  if (state_solver->hasLinkId(tcp_offset))
    throw std::runtime_error("The tcp offset name '" + tcp_offset.name() +
                             "' should not be an existing link in the scene. Assign it as the tcp_frame instead!");

  // Check Manipulator Manager for TCP
  if (kinematics_information.hasGroupTCP(manip_info.manipulator, tcp_offset))
    return kinematics_information.group_tcps.at(manip_info.manipulator).at(tcp_offset);

  // Check callbacks for TCP Offset
  for (const auto& fn : find_tcp_cb)
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

  throw std::runtime_error("Could not find tcp by name " + tcp_offset.name() + "'!");
}

std::unique_ptr<tesseract::collision::DiscreteContactManager>
Environment::Implementation::getDiscreteContactManager() const
{
  {  // Clone cached manager if exists
    std::shared_lock<std::shared_mutex> discrete_lock(discrete_manager_mutex);
    if (discrete_manager)
      return discrete_manager->clone();
  }

  {  // Try to create the default plugin
    std::unique_lock<std::shared_mutex> discrete_lock(discrete_manager_mutex);
    const std::string& name = contact_managers_plugin_info.discrete_plugin_infos.default_plugin;
    discrete_manager = getDiscreteContactManagerHelper(name);
    if (discrete_manager == nullptr)
    {
      CONSOLE_BRIDGE_logError("Discrete manager with %s does not exist in factory!", name.c_str());
      return nullptr;
    }
  }

  return discrete_manager->clone();
}

std::unique_ptr<tesseract::collision::ContinuousContactManager>
Environment::Implementation::getContinuousContactManager() const
{
  {  // Clone cached manager if exists
    std::shared_lock<std::shared_mutex> continuous_lock(continuous_manager_mutex);
    if (continuous_manager)
      return continuous_manager->clone();
  }

  {  // Try to create the default plugin
    std::unique_lock<std::shared_mutex> continuous_lock(continuous_manager_mutex);
    const std::string& name = contact_managers_plugin_info.continuous_plugin_infos.default_plugin;
    continuous_manager = getContinuousContactManagerHelper(name);
    if (continuous_manager == nullptr)
    {
      CONSOLE_BRIDGE_logError("Continuous manager with %s does not exist in factory!", name.c_str());
      return nullptr;
    }
  }

  return continuous_manager->clone();
}

void Environment::Implementation::clearCachedDiscreteContactManager() const
{
  std::unique_lock<std::shared_mutex> discrete_lock(discrete_manager_mutex);
  discrete_manager = nullptr;
}

void Environment::Implementation::clearCachedContinuousContactManager() const
{
  std::unique_lock<std::shared_mutex> continuous_lock(continuous_manager_mutex);
  continuous_manager = nullptr;
}

bool Environment::Implementation::setActiveDiscreteContactManagerHelper(const std::string& name)
{
  tesseract::collision::DiscreteContactManager::UPtr manager = getDiscreteContactManagerHelper(name);
  if (manager == nullptr)
  {
    std::string msg = "\n  Discrete manager with " + name + " does not exist in factory!\n";
    msg += "    Available Managers:\n";
    for (const auto& m : contact_managers_factory.getDiscreteContactManagerPlugins())
      msg += "      " + m.first + "\n";

    CONSOLE_BRIDGE_logError(msg.c_str());
    return false;
  }

  contact_managers_plugin_info.discrete_plugin_infos.default_plugin = name;

  // The calling function should be locking discrete_manager_mutex_
  discrete_manager = std::move(manager);

  return true;
}

bool Environment::Implementation::setActiveContinuousContactManagerHelper(const std::string& name)
{
  tesseract::collision::ContinuousContactManager::UPtr manager = getContinuousContactManagerHelper(name);

  if (manager == nullptr)
  {
    std::string msg = "\n  Continuous manager with " + name + " does not exist in factory!\n";
    msg += "    Available Managers:\n";
    for (const auto& m : contact_managers_factory.getContinuousContactManagerPlugins())
      msg += "      " + m.first + "\n";

    CONSOLE_BRIDGE_logError(msg.c_str());
    return false;
  }

  contact_managers_plugin_info.continuous_plugin_infos.default_plugin = name;

  // The calling function should be locking continuous_manager_mutex_
  continuous_manager = std::move(manager);

  return true;
}

std::unique_ptr<tesseract::collision::DiscreteContactManager>
Environment::Implementation::getDiscreteContactManagerHelper(const std::string& name) const
{
  tesseract::collision::DiscreteContactManager::UPtr manager =
      contact_managers_factory.createDiscreteContactManager(name);
  if (manager == nullptr)
    return nullptr;

  manager->setContactAllowedValidator(contact_allowed_validator);
  if (scene_graph != nullptr)
  {
    for (const auto& link : scene_graph->getLinks())
    {
      if (!link->collision.empty())
      {
        tesseract::collision::CollisionShapesConst shapes;
        tesseract::common::VectorIsometry3d shape_poses;
        getCollisionObject(shapes, shape_poses, *link);
        manager->addCollisionObject(link->getId(), 0, shapes, shape_poses, true);
      }
    }

    manager->setActiveCollisionObjects(state_solver->getActiveLinkIds());
  }

  manager->setCollisionMarginData(collision_margin_data);

  manager->setCollisionObjectsTransform(current_state.link_transforms);

  return manager;
}

std::unique_ptr<tesseract::collision::ContinuousContactManager>
Environment::Implementation::getContinuousContactManagerHelper(const std::string& name) const
{
  tesseract::collision::ContinuousContactManager::UPtr manager =
      contact_managers_factory.createContinuousContactManager(name);

  if (manager == nullptr)
    return nullptr;

  manager->setContactAllowedValidator(contact_allowed_validator);
  if (scene_graph != nullptr)
  {
    for (const auto& link : scene_graph->getLinks())
    {
      if (!link->collision.empty())
      {
        tesseract::collision::CollisionShapesConst shapes;
        tesseract::common::VectorIsometry3d shape_poses;
        getCollisionObject(shapes, shape_poses, *link);
        manager->addCollisionObject(link->getId(), 0, shapes, shape_poses, true);
      }
    }

    manager->setActiveCollisionObjects(state_solver->getActiveLinkIds());
  }

  manager->setCollisionMarginData(collision_margin_data);

  const std::vector<tesseract::common::LinkId> active_link_ids = state_solver->getActiveLinkIds();
  for (const auto& [id, tf] : current_state.link_transforms)
  {
    if (std::find(active_link_ids.begin(), active_link_ids.end(), id) != active_link_ids.end())
      manager->setCollisionObjectsTransform(id, tf, tf);
    else
      manager->setCollisionObjectsTransform(id, tf);
  }

  return manager;
}

bool Environment::Implementation::setActiveDiscreteContactManager(const std::string& name)
{
  std::unique_lock<std::shared_mutex> discrete_lock(discrete_manager_mutex);
  return setActiveDiscreteContactManagerHelper(name);
}

std::unique_ptr<tesseract::collision::DiscreteContactManager>
Environment::Implementation::getDiscreteContactManager(const std::string& name) const
{
  tesseract::collision::DiscreteContactManager::UPtr manager = getDiscreteContactManagerHelper(name);
  if (manager == nullptr)
  {
    CONSOLE_BRIDGE_logError("Discrete manager with %s does not exist in factory!", name.c_str());
    return nullptr;
  }

  return manager;
}

bool Environment::Implementation::setActiveContinuousContactManager(const std::string& name)
{
  std::unique_lock<std::shared_mutex> continous_lock(continuous_manager_mutex);
  return setActiveContinuousContactManagerHelper(name);
}

std::unique_ptr<tesseract::collision::ContinuousContactManager>
Environment::Implementation::getContinuousContactManager(const std::string& name) const
{
  tesseract::collision::ContinuousContactManager::UPtr manager = getContinuousContactManagerHelper(name);
  if (manager == nullptr)
  {
    CONSOLE_BRIDGE_logError("Continuous manager with %s does not exist in factory!", name.c_str());
    return nullptr;
  }

  return manager;
}

bool Environment::Implementation::removeLinkHelper(const common::LinkId& id)
{
  if (scene_graph->getLink(id) == nullptr)
  {
    CONSOLE_BRIDGE_logWarn("Tried to remove link (%s) that does not exist", id.name().c_str());
    return false;
  }
  std::vector<tesseract::scene_graph::Joint::ConstPtr> joints = scene_graph->getInboundJoints(id);
  assert(joints.size() <= 1);

  // get child link names to remove
  std::vector<common::LinkId> child_link_ids = scene_graph->getLinkChildrenIds(id);

  scene_graph->removeLink(id, true);

  std::unique_lock<std::shared_mutex> discrete_lock(discrete_manager_mutex);
  std::unique_lock<std::shared_mutex> continuous_lock(continuous_manager_mutex);
  if (discrete_manager != nullptr)
    discrete_manager->removeCollisionObject(id);
  if (continuous_manager != nullptr)
    continuous_manager->removeCollisionObject(id);

  for (const auto& link_id : child_link_ids)
  {
    if (discrete_manager != nullptr)
      discrete_manager->removeCollisionObject(link_id);
    if (continuous_manager != nullptr)
      continuous_manager->removeCollisionObject(link_id);
  }

  return true;
}

bool Environment::Implementation::applyCommandsHelper(const std::vector<std::shared_ptr<const Command>>& commands)
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
      case tesseract::environment::CommandType::ADD_LINK:
      {
        auto cmd = std::static_pointer_cast<const AddLinkCommand>(command);
        success &= applyAddCommand(cmd);
        break;
      }
      case tesseract::environment::CommandType::MOVE_LINK:
      {
        auto cmd = std::static_pointer_cast<const MoveLinkCommand>(command);
        success &= applyMoveLinkCommand(cmd);
        break;
      }
      case tesseract::environment::CommandType::MOVE_JOINT:
      {
        auto cmd = std::static_pointer_cast<const MoveJointCommand>(command);
        success &= applyMoveJointCommand(cmd);
        break;
      }
      case tesseract::environment::CommandType::REMOVE_LINK:
      {
        auto cmd = std::static_pointer_cast<const RemoveLinkCommand>(command);
        success &= applyRemoveLinkCommand(cmd);
        break;
      }
      case tesseract::environment::CommandType::REMOVE_JOINT:
      {
        auto cmd = std::static_pointer_cast<const RemoveJointCommand>(command);
        success &= applyRemoveJointCommand(cmd);
        break;
      }
      case tesseract::environment::CommandType::REPLACE_JOINT:
      {
        auto cmd = std::static_pointer_cast<const ReplaceJointCommand>(command);
        success &= applyReplaceJointCommand(cmd);
        break;
      }
      case tesseract::environment::CommandType::CHANGE_LINK_ORIGIN:
      {
        auto cmd = std::static_pointer_cast<const ChangeLinkOriginCommand>(command);
        success &= applyChangeLinkOriginCommand(cmd);
        break;
      }
      case tesseract::environment::CommandType::CHANGE_JOINT_ORIGIN:
      {
        auto cmd = std::static_pointer_cast<const ChangeJointOriginCommand>(command);
        success &= applyChangeJointOriginCommand(cmd);
        break;
      }
      case tesseract::environment::CommandType::CHANGE_LINK_COLLISION_ENABLED:
      {
        auto cmd = std::static_pointer_cast<const ChangeLinkCollisionEnabledCommand>(command);
        success &= applyChangeLinkCollisionEnabledCommand(cmd);
        break;
      }
      case tesseract::environment::CommandType::CHANGE_LINK_VISIBILITY:
      {
        auto cmd = std::static_pointer_cast<const ChangeLinkVisibilityCommand>(command);
        success &= applyChangeLinkVisibilityCommand(cmd);
        break;
      }
      case tesseract::environment::CommandType::MODIFY_ALLOWED_COLLISIONS:
      {
        auto cmd = std::static_pointer_cast<const ModifyAllowedCollisionsCommand>(command);
        success &= applyModifyAllowedCollisionsCommand(cmd);
        break;
      }
      case tesseract::environment::CommandType::REMOVE_ALLOWED_COLLISION_LINK:
      {
        auto cmd = std::static_pointer_cast<const RemoveAllowedCollisionLinkCommand>(command);
        success &= applyRemoveAllowedCollisionLinkCommand(cmd);
        break;
      }
      case tesseract::environment::CommandType::ADD_SCENE_GRAPH:
      {
        auto cmd = std::static_pointer_cast<const AddSceneGraphCommand>(command);
        success &= applyAddSceneGraphCommand(cmd);
        break;
      }
      case tesseract::environment::CommandType::CHANGE_JOINT_POSITION_LIMITS:
      {
        auto cmd = std::static_pointer_cast<const ChangeJointPositionLimitsCommand>(command);
        success &= applyChangeJointPositionLimitsCommand(cmd);
        break;
      }
      case tesseract::environment::CommandType::CHANGE_JOINT_VELOCITY_LIMITS:
      {
        auto cmd = std::static_pointer_cast<const ChangeJointVelocityLimitsCommand>(command);
        success &= applyChangeJointVelocityLimitsCommand(cmd);
        break;
      }
      case tesseract::environment::CommandType::CHANGE_JOINT_ACCELERATION_LIMITS:
      {
        auto cmd = std::static_pointer_cast<const ChangeJointAccelerationLimitsCommand>(command);
        success &= applyChangeJointAccelerationLimitsCommand(cmd);
        break;
      }
      case tesseract::environment::CommandType::ADD_KINEMATICS_INFORMATION:
      {
        auto cmd = std::static_pointer_cast<const AddKinematicsInformationCommand>(command);
        success &= applyAddKinematicsInformationCommand(cmd);
        break;
      }
      case tesseract::environment::CommandType::CHANGE_COLLISION_MARGINS:
      {
        auto cmd = std::static_pointer_cast<const ChangeCollisionMarginsCommand>(command);
        success &= applyChangeCollisionMarginsCommand(cmd);
        break;
      }
      case tesseract::environment::CommandType::ADD_CONTACT_MANAGERS_PLUGIN_INFO:
      {
        auto cmd = std::static_pointer_cast<const AddContactManagersPluginInfoCommand>(command);
        success &= applyAddContactManagersPluginInfoCommand(cmd);
        break;
      }
      case tesseract::environment::CommandType::SET_ACTIVE_CONTINUOUS_CONTACT_MANAGER:
      {
        auto cmd = std::static_pointer_cast<const SetActiveContinuousContactManagerCommand>(command);
        success &= applySetActiveContinuousContactManagerCommand(cmd);
        break;
      }
      case tesseract::environment::CommandType::SET_ACTIVE_DISCRETE_CONTACT_MANAGER:
      {
        auto cmd = std::static_pointer_cast<const SetActiveDiscreteContactManagerCommand>(command);
        success &= applySetActiveDiscreteContactManagerCommand(cmd);
        break;
      }
      case tesseract::environment::CommandType::ADD_TRAJECTORY_LINK:
      {
        auto cmd = std::static_pointer_cast<const AddTrajectoryLinkCommand>(command);
        success &= applyAddTrajectoryLinkCommand(cmd);
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
  state_solver->setRevision(revision);

  // If this is not true then the initHelper function has called applyCommand so do not call.
  if (initialized)
    environmentChanged();

  return success;
}

//////////////////////////////////////////////////////////////
//////////////// Internal Apply Command //////////////////////
//////////////////////////////////////////////////////////////

bool Environment::Implementation::applyAddCommand(const AddLinkCommand::ConstPtr& cmd)
{
  // The command should not allow this to occur but adding an assert to catch if something changes
  assert(!(!cmd->getLink() && !cmd->getJoint()));  // NOLINT
  assert(!((cmd->getLink() != nullptr) && (cmd->getJoint() != nullptr) &&
           (cmd->getJoint()->child_link_id != cmd->getLink()->getId())));

  if (!applyAddLinkCommandHelper(cmd->getLink(), cmd->getJoint(), cmd->replaceAllowed()))
    return false;

  ++revision;
  commands.push_back(cmd);

  return true;
}

bool Environment::Implementation::applyAddTrajectoryLinkCommand(const AddTrajectoryLinkCommand::ConstPtr& cmd)
{
  const tesseract::common::JointTrajectory& traj = cmd->getTrajectory();
  if (traj.empty())
    return false;

  if (!cmd->getLinkId().isValid())
  {
    CONSOLE_BRIDGE_logWarn("Tried to add trajectory link with empty link name.");
    return false;
  }

  if (!cmd->getParentLinkId().isValid())
  {
    CONSOLE_BRIDGE_logWarn("Tried to add trajectory link with empty parent link name.");
    return false;
  }

  const bool parent_link_exists = (scene_graph->getLink(cmd->getParentLinkId()) != nullptr);
  if (!parent_link_exists)
  {
    CONSOLE_BRIDGE_logWarn("Tried to add trajectory link (%s) with parent link (%s) which does not exists.",
                           cmd->getLinkId().name().c_str(),
                           cmd->getParentLinkId().name().c_str());
    return false;
  }

  auto state_solver_clone = state_solver->clone();

  using tesseract::common::JointId;
  using tesseract::common::LinkId;

  auto traj_link = std::make_shared<tesseract::scene_graph::Link>(cmd->getLinkId());
  const LinkId& parent_link_id = cmd->getParentLinkId();
  std::vector<JointId> joint_ids;
  std::vector<LinkId> active_link_ids;
  std::unordered_map<LinkId, std::vector<tesseract::scene_graph::Collision::Ptr>> link_collision_geom;
  std::vector<std::vector<tesseract::scene_graph::Collision::Ptr>> per_state_collision_geom;
  per_state_collision_geom.reserve(traj.size());
  for (const auto& state : traj)
  {
    if (state.joint_ids.empty())
    {
      CONSOLE_BRIDGE_logWarn("Tried to add trajectory link (%s) with empty joint names.",
                             cmd->getLinkId().name().c_str());
      return false;
    }

    if (state.position.rows() == 0)
    {
      CONSOLE_BRIDGE_logWarn("Tried to add trajectory link (%s) with empty position.", cmd->getLinkId().name().c_str());
      return false;
    }

    if (static_cast<Eigen::Index>(state.joint_ids.size()) != state.position.size())
    {
      CONSOLE_BRIDGE_logWarn("Tried to add trajectory link (%s) where joint names and position are different sizes.",
                             cmd->getLinkId().name().c_str());
      return false;
    }

    tesseract::scene_graph::SceneState scene_state = state_solver_clone->getState(state.joint_ids, state.position);
    if (joint_ids.empty() || !tesseract::common::isIdentical(state.joint_ids, joint_ids, false))
    {
      joint_ids = state.joint_ids;
      active_link_ids = scene_graph->getJointChildrenIds(joint_ids);

      if (std::find(active_link_ids.begin(), active_link_ids.end(), parent_link_id) != active_link_ids.end())
      {
        CONSOLE_BRIDGE_logWarn("Tried to add trajectory link (%s) where parent link is an active link.",
                               cmd->getLinkId().name().c_str());
        return false;
      }
    }

    std::vector<tesseract::scene_graph::Collision::Ptr> state_collision_geom;
    Eigen::Isometry3d parent_link_tf_inv = scene_state.link_transforms.at(parent_link_id).inverse();
    for (const auto& link_id : active_link_ids)
    {
      Eigen::Isometry3d link_transform = parent_link_tf_inv * scene_state.link_transforms.at(link_id);
      auto link = scene_graph->getLink(link_id);
      assert(link != nullptr);

      auto clone = link->clone(common::LinkId(link_id.name() + "_clone"));

      for (auto& vis_clone : clone.visual)
      {
        vis_clone->origin = link_transform * vis_clone->origin;
        traj_link->visual.push_back(vis_clone);
      }

      for (auto& col_clone : clone.collision)
      {
        col_clone->origin = link_transform * col_clone->origin;
        link_collision_geom[link_id].push_back(col_clone);
        state_collision_geom.push_back(col_clone);
      }
    }
    per_state_collision_geom.push_back(state_collision_geom);
  }

  // Utility function to create convex hull
  auto createCollision = [](const tesseract::common::VectorVector3d& vertices) {
    std::shared_ptr<tesseract::common::VectorVector3d> ch_vertices =
        std::make_shared<tesseract::common::VectorVector3d>();
    std::shared_ptr<Eigen::VectorXi> ch_faces = std::make_shared<Eigen::VectorXi>();
    int ch_num_faces = tesseract::collision::createConvexHull(*ch_vertices, *ch_faces, vertices);
    auto convex_mesh = std::make_shared<tesseract::geometry::ConvexMesh>(
        ch_vertices, ch_faces, ch_num_faces, nullptr, Eigen::Vector3d(1, 1, 1));

    auto col_obj = std::make_shared<tesseract::scene_graph::Collision>();
    col_obj->geometry = std::make_shared<tesseract::geometry::ConvexMesh>(
        ch_vertices, ch_faces, ch_num_faces, nullptr, Eigen::Vector3d(1, 1, 1));
    col_obj->origin = Eigen::Isometry3d::Identity();
    return col_obj;
  };

  switch (cmd->getMethod())
  {
    case AddTrajectoryLinkCommand::Method::PER_STATE_OBJECTS:
    {
      for (const auto& pair : link_collision_geom)
        traj_link->collision.insert(traj_link->collision.end(), pair.second.begin(), pair.second.end());
      break;
    }
    case AddTrajectoryLinkCommand::Method::PER_STATE_CONVEX_HULL:
    {
      for (const auto& state_collision_geom : per_state_collision_geom)
      {
        tesseract::common::VectorVector3d vertices;
        for (const auto& col_obj : state_collision_geom)
        {
          tesseract::common::VectorVector3d cov =
              tesseract::geometry::extractVertices(*col_obj->geometry, col_obj->origin);
          if (cov.empty())
            return false;
          vertices.insert(vertices.end(), cov.begin(), cov.end());
        }

        traj_link->collision.push_back(createCollision(vertices));
      }
      break;
    }
    case AddTrajectoryLinkCommand::Method::GLOBAL_CONVEX_HULL:
    {
      tesseract::common::VectorVector3d vertices;
      for (const auto& pair : link_collision_geom)
      {
        for (const auto& col_obj : pair.second)
        {
          tesseract::common::VectorVector3d cov =
              tesseract::geometry::extractVertices(*col_obj->geometry, col_obj->origin);
          if (cov.empty())
            return false;
          vertices.insert(vertices.end(), cov.begin(), cov.end());
        }
      }

      traj_link->collision.push_back(createCollision(vertices));
      break;
    }
    case AddTrajectoryLinkCommand::Method::GLOBAL_PER_LINK_CONVEX_HULL:
    {
      for (const auto& pair : link_collision_geom)
      {
        tesseract::common::VectorVector3d vertices;
        for (const auto& col_obj : pair.second)
        {
          tesseract::common::VectorVector3d cov =
              tesseract::geometry::extractVertices(*col_obj->geometry, col_obj->origin);
          if (cov.empty())
            return false;
          vertices.insert(vertices.end(), cov.begin(), cov.end());
        }

        traj_link->collision.push_back(createCollision(vertices));
      }

      break;
    }
    default:
      throw std::runtime_error("Environment, unhandled AddTrajectoryLinkCommand::Method type!");
  }

  auto traj_joint = std::make_shared<tesseract::scene_graph::Joint>(JointId("joint_" + cmd->getLinkId().name()));
  traj_joint->type = tesseract::scene_graph::JointType::FIXED;
  traj_joint->parent_link_id = cmd->getParentLinkId();
  traj_joint->child_link_id = cmd->getLinkId();
  traj_joint->parent_to_joint_origin_transform = Eigen::Isometry3d::Identity();

  if (!applyAddLinkCommandHelper(traj_link, traj_joint, cmd->replaceAllowed()))
    return false;

  ++revision;
  commands.push_back(cmd);

  return true;
}

bool Environment::Implementation::applyAddLinkCommandHelper(
    const std::shared_ptr<const tesseract::scene_graph::Link>& link,
    const std::shared_ptr<const tesseract::scene_graph::Joint>& joint,
    bool replace_allowed)
{
  bool link_exists = false;
  bool joint_exists = false;
  common::LinkId link_id;
  common::JointId joint_id;

  if (link != nullptr)
  {
    link_id = link->getId();
    link_exists = (scene_graph->getLink(link_id) != nullptr);
  }

  if (joint != nullptr)
  {
    joint_id = joint->getId();
    joint_exists = (scene_graph->getJoint(joint_id) != nullptr);
  }

  if (link_exists && !replace_allowed)
  {
    CONSOLE_BRIDGE_logWarn("Tried to add link (%s) which already exists. Set replace_allowed to enable replacing.",
                           link_id.name().c_str());
    return false;
  }

  if (joint_exists && !replace_allowed)
  {
    CONSOLE_BRIDGE_logWarn("Tried to replace link (%s) and joint (%s) where the joint exist but the link does not. "
                           "This is not supported.",
                           link_id.name().c_str(),
                           joint_id.name().c_str());
    return false;
  }

  if (!link_exists && joint_exists)
  {
    CONSOLE_BRIDGE_logWarn("Tried to add link (%s) which does not exists with a joint provided which already exists. "
                           "This is not supported.",
                           link_id.name().c_str());
    return false;
  }

  if (link_exists && joint && !joint_exists)
  {
    CONSOLE_BRIDGE_logWarn("Tried to add link (%s) which already exists with a joint provided which does not exist. "
                           "This is not supported.",
                           link_id.name().c_str());
    return false;
  }

  if (link_exists && !joint)
  {  // A link is being replaced
    if (!scene_graph->addLink(*link, true))
      return false;

    // Solver is not affected by replace links
  }
  else if (link_exists && joint_exists)
  {  // A link and joint pair is being replaced
    tesseract::scene_graph::Link::ConstPtr orig_link = scene_graph->getLink(link_id);
    tesseract::scene_graph::Joint::ConstPtr orig_joint = scene_graph->getJoint(joint_id);

    if (orig_joint->child_link_id != orig_link->getId())
    {
      CONSOLE_BRIDGE_logWarn("Tried to replace link (%s) and joint (%s) which are currently not linked. This is not "
                             "supported.",
                             link_id.name().c_str(),
                             joint_id.name().c_str());
      return false;
    }

    if (!scene_graph->addLink(*link, true))
      return false;

    if (!scene_graph->removeJoint(joint_id))
    {
      // Replace with original link
      if (!scene_graph->addLink(*orig_link, true))
        throw std::runtime_error("Environment: Failed to replace link and joint and reset to original state.");

      return false;
    }

    if (!scene_graph->addJoint(*joint))
    {
      // Replace with original link
      if (!scene_graph->addLink(*orig_link, true))
        throw std::runtime_error("Environment: Failed to replace link and joint reset to original state.");

      // Replace with original link
      if (!scene_graph->addJoint(*orig_joint))
        throw std::runtime_error("Environment: Failed to replace link and joint and reset to original state.");

      return false;
    }

    if (!state_solver->replaceJoint(*joint))
      throw std::runtime_error("Environment, failed to replace link and joint in state solver.");
  }
  else if (!link_exists && !joint)
  {  // Add a new link is being added attached to the world
    auto joint_id = common::JointId("joint_" + link_id.name());
    tesseract::scene_graph::Joint joint(joint_id);
    joint.type = tesseract::scene_graph::JointType::FIXED;
    joint.child_link_id = link_id;
    joint.parent_link_id = scene_graph->getRoot();

    if (!scene_graph->addLink(*link, joint))
      return false;

    if (!state_solver->addLink(*link, joint))
      throw std::runtime_error("Environment, failed to add link and joint in state solver.");
  }
  else
  {  // A new link and joint is being added
    if (!scene_graph->addLink(*link, *joint))
      return false;

    if (!state_solver->addLink(*link, *joint))
      throw std::runtime_error("Environment, failed to add link and joint in state solver.");
  }

  // If Link existed remove it from collision before adding the replacing links geometry
  if (link_exists)
  {
    std::unique_lock<std::shared_mutex> discrete_lock(discrete_manager_mutex);
    if (discrete_manager != nullptr)
      discrete_manager->removeCollisionObject(link_id);

    std::unique_lock<std::shared_mutex> continuous_lock(continuous_manager_mutex);
    if (continuous_manager != nullptr)
      continuous_manager->removeCollisionObject(link_id);
  }

  // We have moved the original objects, get a pointer to them from scene_graph
  if (!link->collision.empty())
  {
    tesseract::collision::CollisionShapesConst shapes;
    tesseract::common::VectorIsometry3d shape_poses;
    getCollisionObject(shapes, shape_poses, *link);

    std::unique_lock<std::shared_mutex> discrete_lock(discrete_manager_mutex);
    if (discrete_manager != nullptr)
      discrete_manager->addCollisionObject(link_id, 0, shapes, shape_poses, true);

    std::unique_lock<std::shared_mutex> continuous_lock(continuous_manager_mutex);
    if (continuous_manager != nullptr)
      continuous_manager->addCollisionObject(link_id, 0, shapes, shape_poses, true);
  }

  return true;
}

bool Environment::Implementation::applyMoveLinkCommand(const std::shared_ptr<const MoveLinkCommand>& cmd)
{
  if (!scene_graph->moveLink(*cmd->getJoint()))
    return false;

  if (!state_solver->moveLink(*cmd->getJoint()))
    throw std::runtime_error("Environment, failed to move link in state solver.");

  ++revision;
  commands.push_back(cmd);

  return true;
}

bool Environment::Implementation::applyMoveJointCommand(const std::shared_ptr<const MoveJointCommand>& cmd)
{
  if (!scene_graph->moveJoint(cmd->getJointId(), cmd->getParentLink()))
    return false;

  if (!state_solver->moveJoint(cmd->getJointId(), cmd->getParentLink()))
    throw std::runtime_error("Environment, failed to move joint in state solver.");

  ++revision;
  commands.push_back(cmd);

  return true;
}

bool Environment::Implementation::applyRemoveLinkCommand(const std::shared_ptr<const RemoveLinkCommand>& cmd)
{
  if (!removeLinkHelper(cmd->getLinkId()))
    return false;

  if (!state_solver->removeLink(cmd->getLinkId()))
    throw std::runtime_error("Environment, failed to remove link in state solver.");

  ++revision;
  commands.push_back(cmd);

  return true;
}

bool Environment::Implementation::applyRemoveJointCommand(const std::shared_ptr<const RemoveJointCommand>& cmd)
{
  if (scene_graph->getJoint(cmd->getJointId()) == nullptr)
  {
    CONSOLE_BRIDGE_logWarn("Tried to remove Joint (%s) that does not exist", cmd->getJointId().name().c_str());
    return false;
  }

  common::LinkId target_link_id = scene_graph->getTargetLink(cmd->getJointId())->getId();

  if (!removeLinkHelper(target_link_id))
    return false;

  if (!state_solver->removeJoint(cmd->getJointId()))
    throw std::runtime_error("Environment, failed to remove joint in state solver.");

  ++revision;
  commands.push_back(cmd);

  return true;
}

bool Environment::Implementation::applyReplaceJointCommand(const std::shared_ptr<const ReplaceJointCommand>& cmd)
{
  tesseract::scene_graph::Joint::ConstPtr current_joint = scene_graph->getJoint(cmd->getJoint()->getId());
  if (current_joint == nullptr)
  {
    CONSOLE_BRIDGE_logWarn("Tried to replace Joint (%s) that does not exist", cmd->getJoint()->getName().c_str());
    return false;
  }

  if (cmd->getJoint()->child_link_id != current_joint->child_link_id)
  {
    CONSOLE_BRIDGE_logWarn("Tried to replace Joint (%s) where the child links are not the same",
                           cmd->getJoint()->getName().c_str());
    return false;
  }

  if (!scene_graph->removeJoint(cmd->getJoint()->getId()))
    return false;

  if (!scene_graph->addJoint(*cmd->getJoint()))
  {
    // Add old joint back
    if (!scene_graph->addJoint(*current_joint))
      throw std::runtime_error("Environment: Failed to add old joint back when replace failed!");

    return false;
  }

  if (!state_solver->replaceJoint(*cmd->getJoint()))
    throw std::runtime_error("Environment, failed to replace joint in state solver.");

  ++revision;
  commands.push_back(cmd);

  return true;
}

// NOLINTNEXTLINE(readability-convert-member-functions-to-static)
bool Environment::Implementation::applyChangeLinkOriginCommand(
    const std::shared_ptr<const ChangeLinkOriginCommand>& /*cmd*/)
{
  throw std::runtime_error("Unhandled environment command: CHANGE_LINK_ORIGIN");
}

bool Environment::Implementation::applyChangeJointOriginCommand(const ChangeJointOriginCommand::ConstPtr& cmd)
{
  if (!scene_graph->changeJointOrigin(cmd->getJointId(), cmd->getOrigin()))
    return false;

  if (!state_solver->changeJointOrigin(cmd->getJointId(), cmd->getOrigin()))
    throw std::runtime_error("Environment, failed to change joint origin in state solver.");

  ++revision;
  commands.push_back(cmd);

  return true;
}

bool Environment::Implementation::applyChangeLinkCollisionEnabledCommand(
    const std::shared_ptr<const ChangeLinkCollisionEnabledCommand>& cmd)
{
  std::unique_lock<std::shared_mutex> discrete_lock(discrete_manager_mutex);
  if (discrete_manager != nullptr)
  {
    if (cmd->getEnabled())
      discrete_manager->enableCollisionObject(cmd->getLinkId());
    else
      discrete_manager->disableCollisionObject(cmd->getLinkId());
  }

  std::unique_lock<std::shared_mutex> continuous_lock(continuous_manager_mutex);
  if (continuous_manager != nullptr)
  {
    if (cmd->getEnabled())
      continuous_manager->enableCollisionObject(cmd->getLinkId());
    else
      continuous_manager->disableCollisionObject(cmd->getLinkId());
  }

  scene_graph->setLinkCollisionEnabled(cmd->getLinkId(), cmd->getEnabled());

  if (scene_graph->getLinkCollisionEnabled(cmd->getLinkId()) != cmd->getEnabled())
    return false;

  ++revision;
  commands.push_back(cmd);

  return true;
}

bool Environment::Implementation::applyChangeLinkVisibilityCommand(
    const std::shared_ptr<const ChangeLinkVisibilityCommand>& cmd)
{
  scene_graph->setLinkVisibility(cmd->getLinkId(), cmd->getEnabled());
  if (scene_graph->getLinkVisibility(cmd->getLinkId()) != cmd->getEnabled())
    return false;

  ++revision;
  commands.push_back(cmd);

  return true;
}

bool Environment::Implementation::applyModifyAllowedCollisionsCommand(
    const std::shared_ptr<const ModifyAllowedCollisionsCommand>& cmd)
{
  switch (cmd->getModifyType())
  {
    case ModifyAllowedCollisionsType::REMOVE:
    {
      for (const auto& [key, entry] : cmd->getAllowedCollisionMatrix().getAllAllowedCollisions())
        scene_graph->removeAllowedCollision(key);

      break;
    }
    case ModifyAllowedCollisionsType::REPLACE:
    {
      scene_graph->clearAllowedCollisions();
      for (const auto& [key, entry] : cmd->getAllowedCollisionMatrix().getAllAllowedCollisions())
        scene_graph->addAllowedCollision(key, entry);
      break;
    }
    case ModifyAllowedCollisionsType::ADD:
    {
      for (const auto& [key, entry] : cmd->getAllowedCollisionMatrix().getAllAllowedCollisions())
        scene_graph->addAllowedCollision(key, entry);

      break;
    }
  }

  ++revision;
  commands.push_back(cmd);

  return true;
}

bool Environment::Implementation::applyRemoveAllowedCollisionLinkCommand(
    const std::shared_ptr<const RemoveAllowedCollisionLinkCommand>& cmd)
{
  scene_graph->removeAllowedCollision(cmd->getLinkId());

  ++revision;
  commands.push_back(cmd);

  return true;
}

bool Environment::Implementation::applyAddSceneGraphCommand(std::shared_ptr<const AddSceneGraphCommand> cmd)
{
  if (scene_graph->isEmpty() && cmd->getJoint())
    return false;

  std::vector<tesseract::scene_graph::Link::ConstPtr> pre_links = scene_graph->getLinks();
  if (scene_graph->isEmpty())
  {
    if (!scene_graph->insertSceneGraph(*cmd->getSceneGraph(), cmd->getPrefix()))
      return false;

    state_solver = std::make_unique<tesseract::scene_graph::OFKTStateSolver>(*cmd->getSceneGraph(), cmd->getPrefix());
  }
  else if (!cmd->getJoint())
  {
    // Connect root of subgraph to graph
    tesseract::scene_graph::Joint root_joint(
        common::JointId(cmd->getPrefix() + cmd->getSceneGraph()->getName() + "_joint"));
    root_joint.type = tesseract::scene_graph::JointType::FIXED;
    root_joint.parent_link_id = scene_graph->getRoot();
    root_joint.child_link_id = common::LinkId(cmd->getPrefix() + cmd->getSceneGraph()->getRoot().name());
    root_joint.parent_to_joint_origin_transform = Eigen::Isometry3d::Identity();

    tesseract::scene_graph::SceneGraph::ConstPtr sg = cmd->getSceneGraph();
    std::string prefix = cmd->getPrefix();
    cmd = std::make_shared<AddSceneGraphCommand>(*sg, root_joint, prefix);
    if (!scene_graph->insertSceneGraph(*cmd->getSceneGraph(), *cmd->getJoint(), cmd->getPrefix()))
      return false;

    if (!state_solver->insertSceneGraph(*cmd->getSceneGraph(), *cmd->getJoint(), cmd->getPrefix()))
      throw std::runtime_error("Environment, failed to insert scene graph into state solver.");
  }
  else
  {
    if (!scene_graph->insertSceneGraph(*cmd->getSceneGraph(), *cmd->getJoint(), cmd->getPrefix()))
      return false;

    if (!state_solver->insertSceneGraph(*cmd->getSceneGraph(), *cmd->getJoint(), cmd->getPrefix()))
      throw std::runtime_error("Environment, failed to insert scene graph into state solver.");
  }

  // Now need to get list of added links to add to the contact manager
  std::vector<tesseract::scene_graph::Link::ConstPtr> post_links = scene_graph->getLinks();
  assert(post_links.size() > pre_links.size());
  std::sort(pre_links.begin(), pre_links.end());
  std::sort(post_links.begin(), post_links.end());
  std::vector<tesseract::scene_graph::Link::ConstPtr> diff_links;
  std::set_difference(post_links.begin(),
                      post_links.end(),
                      pre_links.begin(),
                      pre_links.end(),
                      std::inserter(diff_links, diff_links.begin()));

  std::unique_lock<std::shared_mutex> discrete_lock(discrete_manager_mutex);
  std::unique_lock<std::shared_mutex> continuous_lock(continuous_manager_mutex);
  for (const auto& link : diff_links)
  {
    if (!link->collision.empty())
    {
      tesseract::collision::CollisionShapesConst shapes;
      tesseract::common::VectorIsometry3d shape_poses;
      getCollisionObject(shapes, shape_poses, *link);

      if (discrete_manager != nullptr)
        discrete_manager->addCollisionObject(link->getId(), 0, shapes, shape_poses, true);
      if (continuous_manager != nullptr)
        continuous_manager->addCollisionObject(link->getId(), 0, shapes, shape_poses, true);
    }
  }

  ++revision;
  commands.push_back(cmd);

  return true;
}

bool Environment::Implementation::applyChangeJointPositionLimitsCommand(
    const std::shared_ptr<const ChangeJointPositionLimitsCommand>& cmd)
{
  // First check if all of the joint exist
  for (const auto& jp : cmd->getLimits())
  {
    tesseract::scene_graph::JointLimits::ConstPtr jl = scene_graph->getJointLimits(jp.first);
    if (jl == nullptr)
      return false;
  }

  for (const auto& jp : cmd->getLimits())
  {
    tesseract::scene_graph::JointLimits jl_copy = *scene_graph->getJointLimits(jp.first);
    jl_copy.lower = jp.second.first;
    jl_copy.upper = jp.second.second;

    if (!scene_graph->changeJointLimits(jp.first, jl_copy))
      return false;

    if (!state_solver->changeJointPositionLimits(jp.first, jp.second.first, jp.second.second))
      throw std::runtime_error("Environment, failed to change joint position limits in state solver.");
  }

  ++revision;
  commands.push_back(cmd);

  return true;
}

bool Environment::Implementation::applyChangeJointVelocityLimitsCommand(
    const std::shared_ptr<const ChangeJointVelocityLimitsCommand>& cmd)
{
  // First check if all of the joint exist
  for (const auto& jp : cmd->getLimits())
  {
    tesseract::scene_graph::JointLimits::ConstPtr jl = scene_graph->getJointLimits(jp.first);
    if (jl == nullptr)
      return false;
  }

  for (const auto& jp : cmd->getLimits())
  {
    tesseract::scene_graph::JointLimits jl_copy = *scene_graph->getJointLimits(jp.first);
    jl_copy.velocity = jp.second;

    if (!scene_graph->changeJointLimits(jp.first, jl_copy))
      return false;

    if (!state_solver->changeJointVelocityLimits(jp.first, jp.second))
      throw std::runtime_error("Environment, failed to change joint velocity limits in state solver.");
  }

  ++revision;
  commands.push_back(cmd);

  return true;
}

bool Environment::Implementation::applyChangeJointAccelerationLimitsCommand(
    const std::shared_ptr<const ChangeJointAccelerationLimitsCommand>& cmd)
{
  // First check if all of the joint exist
  for (const auto& jp : cmd->getLimits())
  {
    tesseract::scene_graph::JointLimits::ConstPtr jl = scene_graph->getJointLimits(jp.first);
    if (jl == nullptr)
      return false;
  }

  for (const auto& jp : cmd->getLimits())
  {
    tesseract::scene_graph::JointLimits jl_copy = *scene_graph->getJointLimits(jp.first);
    jl_copy.acceleration = jp.second;

    if (!scene_graph->changeJointLimits(jp.first, jl_copy))
      return false;

    if (!state_solver->changeJointAccelerationLimits(jp.first, jp.second))
      throw std::runtime_error("Environment, failed to change joint acceleration limits in state solver.");
  }

  ++revision;
  commands.push_back(cmd);

  return true;
}

bool Environment::Implementation::applyAddKinematicsInformationCommand(
    const std::shared_ptr<const AddKinematicsInformationCommand>& cmd)
{
  kinematics_information.insert(cmd->getKinematicsInformation());

  if (!cmd->getKinematicsInformation().kinematics_plugin_info.empty())
  {
    const auto& info = cmd->getKinematicsInformation().kinematics_plugin_info;
    for (const auto& search_path : info.search_paths)
      kinematics_factory.addSearchPath(search_path);

    for (const auto& search_library : info.search_libraries)
      kinematics_factory.addSearchLibrary(search_library);

    for (const auto& group : info.fwd_plugin_infos)
    {
      for (const auto& solver : group.second.plugins)
        kinematics_factory.addFwdKinPlugin(group.first, solver.first, solver.second);

      if (!group.second.default_plugin.empty())
        kinematics_factory.setDefaultFwdKinPlugin(group.first, group.second.default_plugin);
    }

    for (const auto& group : info.inv_plugin_infos)
    {
      for (const auto& solver : group.second.plugins)
        kinematics_factory.addInvKinPlugin(group.first, solver.first, solver.second);

      if (!group.second.default_plugin.empty())
        kinematics_factory.setDefaultInvKinPlugin(group.first, group.second.default_plugin);
    }
  }

  ++revision;
  commands.push_back(cmd);

  return true;
}

bool Environment::Implementation::applyAddContactManagersPluginInfoCommand(
    const std::shared_ptr<const AddContactManagersPluginInfoCommand>& cmd)
{
  const auto& info = cmd->getContactManagersPluginInfo();

  if (!info.empty())
  {
    contact_managers_plugin_info.insert(info);

    for (const auto& search_path : info.search_paths)
      contact_managers_factory.addSearchPath(search_path);

    for (const auto& search_library : info.search_libraries)
      contact_managers_factory.addSearchLibrary(search_library);

    for (const auto& cm : info.discrete_plugin_infos.plugins)
      contact_managers_factory.addDiscreteContactManagerPlugin(cm.first, cm.second);

    if (!info.discrete_plugin_infos.default_plugin.empty())
      contact_managers_factory.setDefaultDiscreteContactManagerPlugin(info.discrete_plugin_infos.default_plugin);

    for (const auto& cm : info.continuous_plugin_infos.plugins)
      contact_managers_factory.addContinuousContactManagerPlugin(cm.first, cm.second);

    if (!info.continuous_plugin_infos.default_plugin.empty())
      contact_managers_factory.setDefaultContinuousContactManagerPlugin(info.continuous_plugin_infos.default_plugin);
  }

  if (contact_managers_factory.hasDiscreteContactManagerPlugins())
  {
    std::string discrete_default = contact_managers_factory.getDefaultDiscreteContactManagerPlugin();
    std::unique_lock<std::shared_mutex> discrete_lock(discrete_manager_mutex);
    if (discrete_manager == nullptr || discrete_manager->getName() != discrete_default)
      setActiveDiscreteContactManagerHelper(discrete_default);
  }
  else
  {
    CONSOLE_BRIDGE_logDebug("Environment, No discrete contact manager plugins were provided");
  }

  if (contact_managers_factory.hasContinuousContactManagerPlugins())
  {
    std::string continuous_default = contact_managers_factory.getDefaultContinuousContactManagerPlugin();
    std::unique_lock<std::shared_mutex> continuous_lock(continuous_manager_mutex);
    if (continuous_manager == nullptr || continuous_manager->getName() != continuous_default)
      setActiveContinuousContactManagerHelper(continuous_default);
  }
  else
  {
    CONSOLE_BRIDGE_logDebug("Environment, No continuous contact manager plugins were provided");
  }

  ++revision;
  commands.push_back(cmd);

  return true;
}

bool Environment::Implementation::applySetActiveContinuousContactManagerCommand(
    const std::shared_ptr<const SetActiveContinuousContactManagerCommand>& cmd)
{
  setActiveContinuousContactManager(cmd->getName());

  ++revision;
  commands.push_back(cmd);

  return true;
}

bool Environment::Implementation::applySetActiveDiscreteContactManagerCommand(
    const std::shared_ptr<const SetActiveDiscreteContactManagerCommand>& cmd)
{
  setActiveDiscreteContactManager(cmd->getName());

  ++revision;
  commands.push_back(cmd);

  return true;
}

bool Environment::Implementation::applyChangeCollisionMarginsCommand(
    const std::shared_ptr<const ChangeCollisionMarginsCommand>& cmd)
{
  std::optional<double> default_margin = cmd->getDefaultCollisionMargin();
  const tesseract::common::CollisionMarginPairData& pair_margins = cmd->getCollisionMarginPairData();

  if (default_margin.has_value())
    collision_margin_data.setDefaultCollisionMargin(default_margin.value());

  if (!pair_margins.empty())
    collision_margin_data.apply(pair_margins, cmd->getCollisionMarginPairOverrideType());

  std::unique_lock<std::shared_mutex> continuous_lock(continuous_manager_mutex);
  if (continuous_manager != nullptr)
    continuous_manager->setCollisionMarginData(collision_margin_data);

  std::unique_lock<std::shared_mutex> discrete_lock(discrete_manager_mutex);
  if (discrete_manager != nullptr)
    discrete_manager->setCollisionMarginData(collision_margin_data);

  ++revision;
  commands.push_back(cmd);

  return true;
}

Environment::Environment() : impl_(std::make_unique<Implementation>()) {}
Environment::Environment(std::unique_ptr<Implementation> impl) : impl_(std::move(impl)) {}
Environment::~Environment() = default;

bool Environment::init(const std::vector<std::shared_ptr<const Command>>& commands)
{
  bool success{ false };
  {
    std::unique_lock<std::shared_mutex> lock(mutex_);
    success = impl_->initHelper(commands);
  }

  // Call the event callbacks
  std::shared_lock<std::shared_mutex> lock(mutex_);
  impl_->triggerCallbacks();

  return success;
}

bool Environment::init(const tesseract::scene_graph::SceneGraph& scene_graph,
                       const std::shared_ptr<const tesseract::srdf::SRDFModel>& srdf_model)
{
  Commands commands = getInitCommands(scene_graph, srdf_model);
  return init(commands);
}

bool Environment::init(const std::string& urdf_string,
                       const std::shared_ptr<const tesseract::common::ResourceLocator>& locator)
{
  {
    std::unique_lock<std::shared_mutex> lock(mutex_);
    impl_->resource_locator = locator;
  }

  // Parse urdf string into Scene Graph
  tesseract::scene_graph::SceneGraph::Ptr scene_graph;
  try
  {
    scene_graph = tesseract::urdf::parseURDFString(urdf_string, *locator);
  }
  catch (const std::exception& e)
  {
    CONSOLE_BRIDGE_logError("Failed to parse URDF.");
    tesseract::common::printNestedException(e);
    return false;
  }

  Commands commands = getInitCommands(*scene_graph);
  return init(commands);
}

bool Environment::init(const std::string& urdf_string,
                       const std::string& srdf_string,
                       const std::shared_ptr<const tesseract::common::ResourceLocator>& locator)
{
  {
    std::unique_lock<std::shared_mutex> lock(mutex_);
    impl_->resource_locator = locator;
  }

  // Parse urdf string into Scene Graph
  tesseract::scene_graph::SceneGraph::Ptr scene_graph;
  try
  {
    scene_graph = tesseract::urdf::parseURDFString(urdf_string, *locator);
  }
  catch (const std::exception& e)
  {
    CONSOLE_BRIDGE_logError("Failed to parse URDF.");
    tesseract::common::printNestedException(e);
    return false;
  }

  // Parse srdf string into SRDF Model
  auto srdf = std::make_shared<tesseract::srdf::SRDFModel>();
  try
  {
    srdf->initString(*scene_graph, srdf_string, *locator);
  }
  catch (const std::exception& e)
  {
    CONSOLE_BRIDGE_logError("Failed to parse SRDF.");
    tesseract::common::printNestedException(e);
    return false;
  }

  Commands commands = getInitCommands(*scene_graph, srdf);
  return init(commands);
}

bool Environment::init(const std::filesystem::path& urdf_path,
                       const std::shared_ptr<const tesseract::common::ResourceLocator>& locator)
{
  {
    std::unique_lock<std::shared_mutex> lock(mutex_);
    impl_->resource_locator = locator;
  }

  // Parse urdf file into Scene Graph
  tesseract::scene_graph::SceneGraph::Ptr scene_graph;
  try
  {
    scene_graph = tesseract::urdf::parseURDFFile(urdf_path.string(), *locator);
  }
  catch (const std::exception& e)
  {
    CONSOLE_BRIDGE_logError("Failed to parse URDF.");
    tesseract::common::printNestedException(e);
    return false;
  }

  Commands commands = getInitCommands(*scene_graph);
  return init(commands);
}

bool Environment::init(const std::filesystem::path& urdf_path,
                       const std::filesystem::path& srdf_path,
                       const std::shared_ptr<const tesseract::common::ResourceLocator>& locator)
{
  {
    std::unique_lock<std::shared_mutex> lock(mutex_);
    impl_->resource_locator = locator;
  }

  // Parse urdf file into Scene Graph
  tesseract::scene_graph::SceneGraph::Ptr scene_graph;
  try
  {
    scene_graph = tesseract::urdf::parseURDFFile(urdf_path.string(), *locator);
  }
  catch (const std::exception& e)
  {
    CONSOLE_BRIDGE_logError("Failed to parse URDF.");
    tesseract::common::printNestedException(e);
    return false;
  }

  // Parse srdf file into SRDF Model
  auto srdf = std::make_shared<tesseract::srdf::SRDFModel>();
  try
  {
    srdf->initFile(*scene_graph, srdf_path.string(), *locator);
  }
  catch (const std::exception& e)
  {
    CONSOLE_BRIDGE_logError("Failed to parse SRDF.");
    tesseract::common::printNestedException(e);
    return false;
  }

  Commands commands = getInitCommands(*scene_graph, srdf);
  return init(commands);  // NOLINT
}

void Environment::init(const std::vector<std::shared_ptr<const Command>>& commands,
                       int init_revision,
                       const std::chrono::system_clock::time_point& timestamp,
                       const tesseract::scene_graph::SceneState& current_state,
                       const std::chrono::system_clock::time_point& current_state_timestamp,
                       const std::shared_ptr<const tesseract::common::ResourceLocator>& resource_locator)
{
  std::unique_lock<std::shared_mutex> lock(mutex_);
  impl_->resource_locator = resource_locator;
  impl_->initHelper(commands);
  impl_->init_revision = init_revision;
  impl_->setState(current_state.joints, current_state.floating_joints);
  impl_->timestamp = timestamp;
  impl_->current_state_timestamp = current_state_timestamp;
}

bool Environment::reset()
{
  bool success{ false };
  {
    std::unique_lock<std::shared_mutex> lock(mutex_);
    success = impl_->reset();
  }

  // Call the event callbacks
  std::shared_lock<std::shared_mutex> lock(mutex_);
  impl_->triggerCallbacks();

  return success;
}

void Environment::clear()
{
  std::unique_lock<std::shared_mutex> lock(mutex_);
  impl_->clear();
}

bool Environment::isInitialized() const
{
  std::shared_lock<std::shared_mutex> lock(mutex_);
  return std::as_const<Implementation>(*impl_).initialized;
}

int Environment::getRevision() const
{
  std::shared_lock<std::shared_mutex> lock(mutex_);
  return std::as_const<Implementation>(*impl_).revision;
}

int Environment::getInitRevision() const
{
  std::shared_lock<std::shared_mutex> lock(mutex_);
  return std::as_const<Implementation>(*impl_).init_revision;
}

std::vector<std::shared_ptr<const Command>> Environment::getCommandHistory() const
{
  std::shared_lock<std::shared_mutex> lock(mutex_);
  return std::as_const<Implementation>(*impl_).commands;
}

bool Environment::applyCommands(const std::vector<std::shared_ptr<const Command>>& commands)
{
  bool success{ false };
  {
    std::unique_lock<std::shared_mutex> lock(mutex_);
    success = impl_->applyCommandsHelper(commands);
  }
  std::shared_lock<std::shared_mutex> lock(mutex_);
  impl_->triggerCallbacks();

  return success;
}

bool Environment::applyCommand(std::shared_ptr<const Command> command) { return applyCommands({ std::move(command) }); }

std::shared_ptr<const tesseract::scene_graph::SceneGraph> Environment::getSceneGraph() const
{
  return std::as_const<Implementation>(*impl_).scene_graph;
}

std::vector<std::string> Environment::getGroupJointNames(const std::string& group_name) const
{
  std::shared_lock<std::shared_mutex> lock(mutex_);
  return std::as_const<Implementation>(*impl_).getGroupJointNames(group_name);
}

std::vector<tesseract::common::JointId> Environment::getGroupJointIds(const std::string& group_name) const
{
  std::shared_lock<std::shared_mutex> lock(mutex_);
  return std::as_const<Implementation>(*impl_).getGroupJointIds(group_name);
}

std::shared_ptr<const tesseract::kinematics::JointGroup> Environment::getJointGroup(const std::string& group_name) const
{
  std::shared_lock<std::shared_mutex> lock(mutex_);
  return std::as_const<Implementation>(*impl_).getJointGroup(group_name);
}

std::shared_ptr<const tesseract::kinematics::JointGroup>
Environment::getJointGroup(const std::string& name, const std::vector<tesseract::common::JointId>& joint_ids) const
{
  std::shared_lock<std::shared_mutex> lock(mutex_);
  return std::as_const<Implementation>(*impl_).getJointGroup(name, joint_ids);
}

std::shared_ptr<const tesseract::kinematics::KinematicGroup>
Environment::getKinematicGroup(const std::string& group_name, const std::string& ik_solver_name) const
{
  std::shared_lock<std::shared_mutex> lock(mutex_);
  return std::as_const<Implementation>(*impl_).getKinematicGroup(group_name, ik_solver_name);
}

// NOLINTNEXTLINE
Eigen::Isometry3d Environment::findTCPOffset(const tesseract::common::ManipulatorInfo& manip_info) const
{
  std::shared_lock<std::shared_mutex> lock(mutex_);
  return std::as_const<Implementation>(*impl_).findTCPOffset(manip_info);
}

void Environment::addFindTCPOffsetCallback(const FindTCPOffsetCallbackFn& fn)
{
  std::unique_lock<std::shared_mutex> lock(mutex_);
  impl_->find_tcp_cb.push_back(fn);
}

std::vector<FindTCPOffsetCallbackFn> Environment::getFindTCPOffsetCallbacks() const
{
  std::shared_lock<std::shared_mutex> lock(mutex_);
  return std::as_const<Implementation>(*impl_).find_tcp_cb;
}

void Environment::addEventCallback(std::size_t hash, const EventCallbackFn& fn)
{
  std::unique_lock<std::shared_mutex> lock(mutex_);
  impl_->event_cb[hash] = fn;
}

void Environment::removeEventCallback(std::size_t hash)
{
  std::unique_lock<std::shared_mutex> lock(mutex_);
  impl_->event_cb.erase(hash);
}

void Environment::clearEventCallbacks()
{
  std::unique_lock<std::shared_mutex> lock(mutex_);
  impl_->event_cb.clear();
}

std::map<std::size_t, EventCallbackFn> Environment::getEventCallbacks() const
{
  std::shared_lock<std::shared_mutex> lock(mutex_);
  return std::as_const<Implementation>(*impl_).event_cb;
}

void Environment::setResourceLocator(std::shared_ptr<const tesseract::common::ResourceLocator> locator)
{
  std::unique_lock<std::shared_mutex> lock(mutex_);
  impl_->resource_locator = std::move(locator);
}

std::shared_ptr<const tesseract::common::ResourceLocator> Environment::getResourceLocator() const
{
  std::shared_lock<std::shared_mutex> lock(mutex_);
  return std::as_const<Implementation>(*impl_).resource_locator;
}

void Environment::setName(const std::string& name)
{
  std::unique_lock<std::shared_mutex> lock(mutex_);
  impl_->scene_graph->setName(name);
}

const std::string& Environment::getName() const
{
  std::shared_lock<std::shared_mutex> lock(mutex_);
  return std::as_const<Implementation>(*impl_).scene_graph->getName();
}

void Environment::setState(const std::unordered_map<std::string, double>& joints,
                           const tesseract::common::JointIdTransformMap& floating_joints)
{
  {
    std::unique_lock<std::shared_mutex> lock(mutex_);
    impl_->setState(joints, floating_joints);
  }

  std::shared_lock<std::shared_mutex> lock(mutex_);
  impl_->triggerCurrentStateChangedCallbacks();
}

void Environment::setState(const std::vector<std::string>& joint_names,
                           const Eigen::Ref<const Eigen::VectorXd>& joint_values,
                           const tesseract::common::JointIdTransformMap& floating_joints)
{
  {
    std::unique_lock<std::shared_mutex> lock(mutex_);
    impl_->setState(joint_names, joint_values, floating_joints);
  }

  std::shared_lock<std::shared_mutex> lock(mutex_);
  impl_->triggerCurrentStateChangedCallbacks();
}

void Environment::setState(const tesseract::common::JointIdTransformMap& floating_joints)
{
  {
    std::unique_lock<std::shared_mutex> lock(mutex_);
    impl_->setState(floating_joints);
  }

  std::shared_lock<std::shared_mutex> lock(mutex_);
  impl_->triggerCurrentStateChangedCallbacks();
}

void Environment::setState(const tesseract::scene_graph::SceneState::JointValues& joint_values,
                           const tesseract::common::JointIdTransformMap& floating_joints)
{
  {
    std::unique_lock<std::shared_mutex> lock(mutex_);
    impl_->setState(joint_values, floating_joints);
  }

  std::shared_lock<std::shared_mutex> lock(mutex_);
  impl_->triggerCurrentStateChangedCallbacks();
}

void Environment::setState(const std::vector<tesseract::common::JointId>& joint_ids,
                           const Eigen::Ref<const Eigen::VectorXd>& joint_values,
                           const tesseract::common::JointIdTransformMap& floating_joints)
{
  {
    std::unique_lock<std::shared_mutex> lock(mutex_);
    impl_->setState(joint_ids, joint_values, floating_joints);
  }

  std::shared_lock<std::shared_mutex> lock(mutex_);
  impl_->triggerCurrentStateChangedCallbacks();
}

tesseract::scene_graph::SceneState
Environment::getState(const std::unordered_map<std::string, double>& joints,
                      const tesseract::common::JointIdTransformMap& floating_joints) const
{
  tesseract::scene_graph::SceneState::JointValues id_map;
  for (const auto& [name, val] : joints)
    id_map[tesseract::common::JointId(name)] = val;
  std::shared_lock<std::shared_mutex> lock(mutex_);
  return std::as_const<Implementation>(*impl_).state_solver->getState(id_map, floating_joints);
}

tesseract::scene_graph::SceneState
Environment::getState(const std::vector<std::string>& joint_names,
                      const Eigen::Ref<const Eigen::VectorXd>& joint_values,
                      const tesseract::common::JointIdTransformMap& floating_joints) const
{
  std::shared_lock<std::shared_mutex> lock(mutex_);
  return std::as_const<Implementation>(*impl_).state_solver->getState(
      tesseract::common::toIds<tesseract::common::JointId>(joint_names), joint_values, floating_joints);
}

tesseract::scene_graph::SceneState
Environment::getState(const tesseract::common::JointIdTransformMap& floating_joints) const
{
  std::shared_lock<std::shared_mutex> lock(mutex_);
  return std::as_const<Implementation>(*impl_).state_solver->getState(floating_joints);
}

tesseract::scene_graph::SceneState
Environment::getState(const tesseract::scene_graph::SceneState::JointValues& joint_values,
                      const tesseract::common::JointIdTransformMap& floating_joints) const
{
  std::shared_lock<std::shared_mutex> lock(mutex_);
  return std::as_const<Implementation>(*impl_).state_solver->getState(joint_values, floating_joints);
}

tesseract::scene_graph::SceneState
Environment::getState(const std::vector<tesseract::common::JointId>& joint_ids,
                      const Eigen::Ref<const Eigen::VectorXd>& joint_values,
                      const tesseract::common::JointIdTransformMap& floating_joints) const
{
  std::shared_lock<std::shared_mutex> lock(mutex_);
  return std::as_const<Implementation>(*impl_).state_solver->getState(joint_ids, joint_values, floating_joints);
}

tesseract::scene_graph::SceneState Environment::getState() const
{
  std::shared_lock<std::shared_mutex> lock(mutex_);
  return std::as_const<Implementation>(*impl_).current_state;
}

void Environment::getLinkTransforms(tesseract::common::LinkIdTransformMap& link_transforms,
                                    const std::vector<std::string>& joint_names,
                                    const Eigen::Ref<const Eigen::VectorXd>& joint_values,
                                    const tesseract::common::JointIdTransformMap& floating_joints) const
{
  std::shared_lock<std::shared_mutex> lock(mutex_);
  std::as_const<Implementation>(*impl_).state_solver->getLinkTransforms(
      link_transforms,
      tesseract::common::toIds<tesseract::common::JointId>(joint_names),
      joint_values,
      floating_joints);
}

void Environment::getLinkTransforms(tesseract::common::LinkIdTransformMap& link_transforms,
                                    const std::vector<tesseract::common::JointId>& joint_ids,
                                    const Eigen::Ref<const Eigen::VectorXd>& joint_values,
                                    const tesseract::common::JointIdTransformMap& floating_joints) const
{
  std::shared_lock<std::shared_mutex> lock(mutex_);
  std::as_const<Implementation>(*impl_).state_solver->getLinkTransforms(
      link_transforms, joint_ids, joint_values, floating_joints);
}

void Environment::getLinkTransforms(tesseract::common::LinkIdTransformMap& link_transforms,
                                    const std::vector<std::string>& joint_names,
                                    const Eigen::Ref<const Eigen::VectorXd>& joint_values) const
{
  std::shared_lock<std::shared_mutex> lock(mutex_);
  std::as_const<Implementation>(*impl_).state_solver->getLinkTransforms(
      link_transforms, tesseract::common::toIds<tesseract::common::JointId>(joint_names), joint_values);
}

void Environment::getLinkTransforms(tesseract::common::LinkIdTransformMap& link_transforms,
                                    const std::vector<tesseract::common::JointId>& joint_ids,
                                    const Eigen::Ref<const Eigen::VectorXd>& joint_values) const
{
  std::shared_lock<std::shared_mutex> lock(mutex_);
  std::as_const<Implementation>(*impl_).state_solver->getLinkTransforms(link_transforms, joint_ids, joint_values);
}

std::chrono::system_clock::time_point Environment::getTimestamp() const
{
  std::shared_lock<std::shared_mutex> lock(mutex_);
  return std::as_const<Implementation>(*impl_).timestamp;
}

std::chrono::system_clock::time_point Environment::getCurrentStateTimestamp() const
{
  std::shared_lock<std::shared_mutex> lock(mutex_);
  return std::as_const<Implementation>(*impl_).current_state_timestamp;
}

std::shared_ptr<const tesseract::scene_graph::Link> Environment::getLink(const common::LinkId& id) const
{
  std::shared_lock<std::shared_mutex> lock(mutex_);
  tesseract::scene_graph::Link::ConstPtr link = std::as_const<Implementation>(*impl_).scene_graph->getLink(id);
  return link;
}

std::shared_ptr<const tesseract::scene_graph::JointLimits>
Environment::getJointLimits(const common::JointId& joint_id) const
{
  std::shared_lock<std::shared_mutex> lock(mutex_);
  return std::as_const<Implementation>(*impl_).scene_graph->getJointLimits(joint_id);
}

bool Environment::getLinkCollisionEnabled(const common::LinkId& id) const
{
  std::shared_lock<std::shared_mutex> lock(mutex_);
  return std::as_const<Implementation>(*impl_).scene_graph->getLinkCollisionEnabled(id);
}

bool Environment::getLinkVisibility(const common::LinkId& id) const
{
  std::shared_lock<std::shared_mutex> lock(mutex_);
  return std::as_const<Implementation>(*impl_).scene_graph->getLinkVisibility(id);
}

std::shared_ptr<const tesseract::common::AllowedCollisionMatrix> Environment::getAllowedCollisionMatrix() const
{
  std::shared_lock<std::shared_mutex> lock(mutex_);
  return std::as_const<Implementation>(*impl_).scene_graph->getAllowedCollisionMatrix();
}

std::vector<std::string> Environment::getJointNames() const
{
  std::shared_lock<std::shared_mutex> lock(mutex_);
  return common::toNames(std::as_const<Implementation>(*impl_).state_solver->getJointIds());
}

std::vector<std::string> Environment::getActiveJointNames() const
{
  std::shared_lock<std::shared_mutex> lock(mutex_);
  return common::toNames(std::as_const<Implementation>(*impl_).state_solver->getActiveJointIds());
}

std::vector<tesseract::common::JointId> Environment::getJointIds() const
{
  std::shared_lock<std::shared_mutex> lock(mutex_);
  return std::as_const<Implementation>(*impl_).state_solver->getJointIds();
}

std::vector<tesseract::common::JointId> Environment::getActiveJointIds() const
{
  std::shared_lock<std::shared_mutex> lock(mutex_);
  return std::as_const<Implementation>(*impl_).state_solver->getActiveJointIds();
}

std::shared_ptr<const tesseract::scene_graph::Joint> Environment::getJoint(const common::JointId& id) const
{
  std::shared_lock<std::shared_mutex> lock(mutex_);
  return std::as_const<Implementation>(*impl_).scene_graph->getJoint(id);
}

Eigen::VectorXd Environment::getCurrentJointValues() const
{
  std::shared_lock<std::shared_mutex> lock(mutex_);
  return std::as_const<Implementation>(*impl_).getCurrentJointValues();
}

Eigen::VectorXd Environment::getCurrentJointValues(const std::vector<std::string>& joint_names) const
{
  std::shared_lock<std::shared_mutex> lock(mutex_);
  return std::as_const<Implementation>(*impl_).getCurrentJointValues(joint_names);
}

Eigen::VectorXd Environment::getCurrentJointValues(const std::vector<tesseract::common::JointId>& joint_ids) const
{
  std::shared_lock<std::shared_mutex> lock(mutex_);
  return std::as_const<Implementation>(*impl_).getCurrentJointValues(joint_ids);
}

tesseract::common::JointIdTransformMap Environment::getCurrentFloatingJointValues() const
{
  std::shared_lock<std::shared_mutex> lock(mutex_);
  return std::as_const<Implementation>(*impl_).getCurrentFloatingJointValues();
}

tesseract::common::JointIdTransformMap
Environment::getCurrentFloatingJointValues(const std::vector<std::string>& joint_names) const
{
  std::shared_lock<std::shared_mutex> lock(mutex_);
  return std::as_const<Implementation>(*impl_).getCurrentFloatingJointValues(joint_names);
}

tesseract::common::JointIdTransformMap
Environment::getCurrentFloatingJointValues(const std::vector<tesseract::common::JointId>& joint_ids) const
{
  std::shared_lock<std::shared_mutex> lock(mutex_);
  return std::as_const<Implementation>(*impl_).getCurrentFloatingJointValues(joint_ids);
}

tesseract::common::LinkId Environment::getRootLinkId() const
{
  std::shared_lock<std::shared_mutex> lock(mutex_);
  return std::as_const<Implementation>(*impl_).scene_graph->getRoot();
}

std::string Environment::getRootLinkName() const
{
  std::shared_lock<std::shared_mutex> lock(mutex_);
  return std::as_const<Implementation>(*impl_).scene_graph->getRoot().name();
}

std::vector<std::string> Environment::getLinkNames() const
{
  std::shared_lock<std::shared_mutex> lock(mutex_);
  return common::toNames(std::as_const<Implementation>(*impl_).state_solver->getLinkIds());
}

std::vector<std::string> Environment::getActiveLinkNames() const
{
  std::shared_lock<std::shared_mutex> lock(mutex_);
  return common::toNames(std::as_const<Implementation>(*impl_).state_solver->getActiveLinkIds());
}

std::vector<tesseract::common::LinkId> Environment::getLinkIds() const
{
  std::shared_lock<std::shared_mutex> lock(mutex_);
  return std::as_const<Implementation>(*impl_).state_solver->getLinkIds();
}

std::vector<tesseract::common::LinkId> Environment::getActiveLinkIds() const
{
  std::shared_lock<std::shared_mutex> lock(mutex_);
  return std::as_const<Implementation>(*impl_).state_solver->getActiveLinkIds();
}

std::vector<tesseract::common::LinkId>
Environment::getActiveLinkIds(const std::vector<tesseract::common::JointId>& joint_ids) const
{
  std::shared_lock<std::shared_mutex> lock(mutex_);
  return std::as_const<Implementation>(*impl_).scene_graph->getJointChildrenIds(joint_ids);
}

std::vector<std::string> Environment::getActiveLinkNames(const std::vector<std::string>& joint_names) const
{
  std::shared_lock<std::shared_mutex> lock(mutex_);
  return common::toNames(std::as_const<Implementation>(*impl_).scene_graph->getJointChildrenIds(
      tesseract::common::toIds<common::JointId>(joint_names)));
}

std::vector<std::string> Environment::getStaticLinkNames() const
{
  std::shared_lock<std::shared_mutex> lock(mutex_);
  return common::toNames(std::as_const<Implementation>(*impl_).state_solver->getStaticLinkIds());
}

std::vector<tesseract::common::LinkId> Environment::getStaticLinkIds() const
{
  std::shared_lock<std::shared_mutex> lock(mutex_);
  return std::as_const<Implementation>(*impl_).state_solver->getStaticLinkIds();
}

std::vector<std::string> Environment::getStaticLinkNames(const std::vector<std::string>& joint_names) const
{
  std::shared_lock<std::shared_mutex> lock(mutex_);
  return std::as_const<Implementation>(*impl_).getStaticLinkNames(joint_names);
}

std::vector<tesseract::common::LinkId>
Environment::getStaticLinkIds(const std::vector<tesseract::common::JointId>& joint_ids) const
{
  std::shared_lock<std::shared_mutex> lock(mutex_);
  return std::as_const<Implementation>(*impl_).getStaticLinkIds(joint_ids);
}

tesseract::common::VectorIsometry3d Environment::getLinkTransforms() const
{
  std::shared_lock<std::shared_mutex> lock(mutex_);
  return std::as_const<Implementation>(*impl_).state_solver->getLinkTransforms();
}

Eigen::Isometry3d Environment::getLinkTransform(const tesseract::common::LinkId& link_id) const
{
  std::shared_lock<std::shared_mutex> lock(mutex_);
  return std::as_const<Implementation>(*impl_).state_solver->getLinkTransform(link_id);
}

Eigen::Isometry3d Environment::getRelativeLinkTransform(const tesseract::common::LinkId& from_link_id,
                                                        const tesseract::common::LinkId& to_link_id) const
{
  std::shared_lock<std::shared_mutex> lock(mutex_);
  return std::as_const<Implementation>(*impl_).state_solver->getRelativeLinkTransform(from_link_id, to_link_id);
}

std::unique_ptr<tesseract::scene_graph::StateSolver> Environment::getStateSolver() const
{
  std::shared_lock<std::shared_mutex> lock(mutex_);
  return std::as_const<Implementation>(*impl_).state_solver->clone();
}

tesseract::srdf::KinematicsInformation Environment::getKinematicsInformation() const
{
  std::shared_lock<std::shared_mutex> lock(mutex_);
  return std::as_const<Implementation>(*impl_).kinematics_information;
}

std::set<std::string> Environment::getGroupNames() const
{
  std::shared_lock<std::shared_mutex> lock(mutex_);
  return std::as_const<Implementation>(*impl_).kinematics_information.group_names;
}

tesseract::common::ContactManagersPluginInfo Environment::getContactManagersPluginInfo() const
{
  std::shared_lock<std::shared_mutex> lock(mutex_);
  return std::as_const<Implementation>(*impl_).contact_managers_plugin_info;
}

bool Environment::setActiveDiscreteContactManager(const std::string& name)
{
  std::unique_lock<std::shared_mutex> lock(mutex_);
  return impl_->setActiveDiscreteContactManager(name);
}

std::unique_ptr<tesseract::collision::DiscreteContactManager>
Environment::getDiscreteContactManager(const std::string& name) const
{
  std::shared_lock<std::shared_mutex> lock(mutex_);
  return std::as_const<Implementation>(*impl_).getDiscreteContactManager(name);
}

bool Environment::setActiveContinuousContactManager(const std::string& name)
{
  std::unique_lock<std::shared_mutex> lock(mutex_);
  return impl_->setActiveContinuousContactManager(name);
}

std::unique_ptr<tesseract::collision::ContinuousContactManager>
Environment::getContinuousContactManager(const std::string& name) const
{
  std::shared_lock<std::shared_mutex> lock(mutex_);
  return std::as_const<Implementation>(*impl_).getContinuousContactManager(name);
}

std::unique_ptr<tesseract::collision::DiscreteContactManager> Environment::getDiscreteContactManager() const
{
  std::shared_lock<std::shared_mutex> lock(mutex_);
  return std::as_const<Implementation>(*impl_).getDiscreteContactManager();
}

void Environment::clearCachedDiscreteContactManager() const
{
  std::shared_lock<std::shared_mutex> lock(mutex_);
  std::as_const<Implementation>(*impl_).clearCachedDiscreteContactManager();
}

std::unique_ptr<tesseract::collision::ContinuousContactManager> Environment::getContinuousContactManager() const
{
  std::shared_lock<std::shared_mutex> lock(mutex_);
  return std::as_const<Implementation>(*impl_).getContinuousContactManager();
}

void Environment::clearCachedContinuousContactManager() const
{
  std::shared_lock<std::shared_mutex> lock(mutex_);
  std::as_const<Implementation>(*impl_).clearCachedContinuousContactManager();
}

tesseract::common::CollisionMarginData Environment::getCollisionMarginData() const
{
  std::shared_lock<std::shared_mutex> lock(mutex_);
  return std::as_const<Implementation>(*impl_).collision_margin_data;
}

std::shared_lock<std::shared_mutex> Environment::lockRead() const
{
  return std::shared_lock<std::shared_mutex>(mutex_);
}

bool Environment::operator==(const Environment& rhs) const
{
  std::shared_lock<std::shared_mutex> lock(mutex_);
  return std::as_const<Implementation>(*impl_) == std::as_const<Implementation>(*rhs.impl_);
}

bool Environment::operator!=(const Environment& rhs) const { return !operator==(rhs); }

Environment::UPtr Environment::clone() const
{
  std::shared_lock<std::shared_mutex> lock(mutex_);
  return std::make_unique<Environment>(std::as_const<Implementation>(*impl_).clone());
}

}  // namespace tesseract::environment
