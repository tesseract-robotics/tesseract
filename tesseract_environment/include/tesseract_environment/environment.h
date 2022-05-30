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
#include <tesseract_common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <vector>
#include <string>
#include <shared_mutex>
#include <chrono>
#include <console_bridge/console.h>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract_environment/commands.h>
#include <tesseract_environment/events.h>
#include <tesseract_collision/core/discrete_contact_manager.h>
#include <tesseract_collision/core/continuous_contact_manager.h>
#include <tesseract_collision/core/contact_managers_plugin_factory.h>
#include <tesseract_scene_graph/graph.h>
#include <tesseract_scene_graph/scene_state.h>
#include <tesseract_state_solver/mutable_state_solver.h>
#include <tesseract_urdf/urdf_parser.h>
#include <tesseract_srdf/srdf_model.h>
#include <tesseract_common/resource_locator.h>
#include <tesseract_common/manipulator_info.h>
#include <tesseract_common/types.h>
#include <tesseract_common/utils.h>
#include <tesseract_kinematics/core/joint_group.h>
#include <tesseract_kinematics/core/kinematic_group.h>
#include <tesseract_kinematics/core/kinematics_plugin_factory.h>

#ifdef SWIG
%shared_ptr(tesseract_environment::Environment)
%wrap_unique_ptr(EnvironmentUPtr,tesseract_environment::Environment)
#endif  // SWIG

namespace tesseract_environment
{
/**
 * @brief Function signature for adding additional callbacks for looking up TCP information
 *
 * The function should throw and exception if not located
 */
using FindTCPOffsetCallbackFn = std::function<Eigen::Isometry3d(const tesseract_common::ManipulatorInfo&)>;

class Environment
{
public:
  // LCOV_EXCL_START
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  // LCOV_EXCL_STOP

  using Ptr = std::shared_ptr<Environment>;
  using ConstPtr = std::shared_ptr<const Environment>;
  using UPtr = std::unique_ptr<Environment>;
  using ConstUPtr = std::unique_ptr<const Environment>;

  /** @brief Default constructor */
  Environment() = default;
  virtual ~Environment() = default;
  Environment(const Environment&) = delete;
  Environment& operator=(const Environment&) = delete;
  Environment(Environment&&) = delete;
  Environment& operator=(Environment&&) = delete;

  /**
   * @brief Initialize the Environment
   *
   * The template class provided should be a derived class from StateSolver.
   *
   * @param scene_graph The scene graph to initialize the environment.
   * @return True if successful, otherwise false
   */
  bool init(const Commands& commands);

  /**
   * @brief Initialize the Environment
   *
   * The template class provided should be a derived class from StateSolver.
   *
   * @param scene_graph The scene graph to initialize the environment.
   * @return True if successful, otherwise false
   */
  bool init(const tesseract_scene_graph::SceneGraph& scene_graph,
            const tesseract_srdf::SRDFModel::ConstPtr& srdf_model = nullptr);

  bool init(const std::string& urdf_string, const tesseract_common::ResourceLocator::ConstPtr& locator);

  bool init(const std::string& urdf_string,
            const std::string& srdf_string,
            const tesseract_common::ResourceLocator::ConstPtr& locator);

  bool init(const tesseract_common::fs::path& urdf_path, const tesseract_common::ResourceLocator::ConstPtr& locator);

  bool init(const tesseract_common::fs::path& urdf_path,
            const tesseract_common::fs::path& srdf_path,
            const tesseract_common::ResourceLocator::ConstPtr& locator);

  /**
   * @brief Clone the environment
   * @return A clone of the environment
   */
  Environment::UPtr clone() const;

  /**
   * @brief reset to initialized state
   * @details If the environment has not been initialized then this returns false
   * @return True if environment was successfully reset, otherwise false.
   */
  bool reset();

  /** @brief clear content and uninitialized */
  void clear();

  /** @brief check if the environment is initialized */
  bool isInitialized() const;

  /**
   * @brief Get the current revision number
   * @return Revision number
   */
  int getRevision() const;

  /**
   * @brief Get Environment command history post initialization
   * @return List of commands
   */
  Commands getCommandHistory() const;

  /**
   * @brief Applies the commands to the environment
   * @param commands Commands to be applied to the environment
   * @return true if successful. If returned false, then only a partial set of commands have been applied. Call
   * getCommandHistory to check. Some commands are not checked for success
   */
  bool applyCommands(const Commands& commands);

  /**
   * @brief Apply command to the environment
   * @param command Command to be applied to the environment
   * @return true if successful. If returned false, then the command have not been applied.
   * Some type of Command are not checked for success
   */
  bool applyCommand(Command::ConstPtr command);

  /**
   * @brief Get the Scene Graph
   * @return SceneGraphConstPtr
   */
  tesseract_scene_graph::SceneGraph::ConstPtr getSceneGraph() const;

  /**
   * @brief Get a groups joint names
   * @param group_name The group name
   * @return A vector of joint names
   */
  std::vector<std::string> getGroupJointNames(const std::string& group_name) const;

  /**
   * @brief Get a joint group by name
   * @param group_name The group name
   * @return A joint group
   */
  tesseract_kinematics::JointGroup::UPtr getJointGroup(const std::string& group_name) const;

  /**
   * @brief Get a joint group given a vector of joint names
   * @param name The name to assign to the joint group
   * @param joint_names The joint names that make up the group
   * @return A joint group
   */
  tesseract_kinematics::JointGroup::UPtr getJointGroup(const std::string& name,
                                                       const std::vector<std::string>& joint_names) const;

  /**
   * @brief Get a kinematic group given group name and solver name
   * @details If ik_solver_name is empty it will choose the first ik solver for the group
   * @param group_name The group name
   * @param ik_solver_name The IK solver name
   * @return A kinematics group
   */
  tesseract_kinematics::KinematicGroup::UPtr getKinematicGroup(const std::string& group_name,
                                                               std::string ik_solver_name = "") const;

  /**
   * @brief Find tool center point provided in the manipulator info
   *
   * If manipulator information tcp is defined as a string it does the following
   *    - First check if manipulator info is empty or already an Isometry3d, if so return identity
   *    - Next if not, it checks if the tcp offset name is a link in the environment if so throw an exception.
   *    - Next if not found, it looks up the tcp name in the SRDF kinematics information
   *    - Next if not found, it leverages the user defined callbacks to try an locate the tcp information.
   *    - Next throw an exception, because no tcp information was located.
   *
   * @param manip_info The manipulator info
   * @return The tool center point
   */
  Eigen::Isometry3d findTCPOffset(const tesseract_common::ManipulatorInfo& manip_info) const;

  /**
   * @brief This allows for user defined callbacks for looking up TCP information
   * @param fn User defined callback function for locating TCP information
   */
  void addFindTCPOffsetCallback(const FindTCPOffsetCallbackFn& fn);

  /**
   * @brief This get the current find tcp callbacks stored in the environment
   * @return A vector of callback functions
   */
  std::vector<FindTCPOffsetCallbackFn> getFindTCPOffsetCallbacks() const;

  /**
   * @brief Add an event callback function
   * @details When these get called they are protected by a unique lock internally so if the
   * callback is a long event it can impact performance.
   * @note These do not get cloned or serialized
   * @param hash The id associated with the callback to allow removal. It is recommended to use
   * std::hash<Object*>{}(this) to associate the callback with the class it associated with.
   * @param fn User defined callback function which gets called for different event triggers
   */
  void addEventCallback(std::size_t hash, const EventCallbackFn& fn);

  /**
   * @brief Remove event callbacks
   * @param hash the id associated with the callback to be removed
   */
  void removeEventCallback(std::size_t hash);

  /** @brief clear all event callbacks */
  void clearEventCallbacks();

  /**
   * @brief Get the current event callbacks stored in the environment
   * @return A map of callback functions
   */
  std::map<std::size_t, EventCallbackFn> getEventCallbacks() const;

  /**
   * @brief Set resource locator for environment
   * @param locator The resource locator
   */
  void setResourceLocator(tesseract_common::ResourceLocator::ConstPtr locator);

  /**
   * @brief Get the resource locator assigned
   * @details This can be a nullptr
   * @return The resource locator assigned to the environment
   */
  tesseract_common::ResourceLocator::ConstPtr getResourceLocator() const;

  /** @brief Give the environment a name */
  void setName(const std::string& name);

  /** @brief Get the name of the environment
   *
   * This may be empty, if so check urdf name
   */
  const std::string& getName() const;

  /**
   * @brief Set the current state of the environment
   *
   * After updating the current state these function must call currentStateChanged() which
   * will update the contact managers transforms
   *
   */
  void setState(const std::unordered_map<std::string, double>& joints);
  void setState(const std::vector<std::string>& joint_names, const Eigen::Ref<const Eigen::VectorXd>& joint_values);

  /**
   * @brief Get the state of the environment for a given set or subset of joint values.
   *
   * This does not change the internal state of the environment.
   *
   * @param joints A map of joint names to joint values to change.
   * @return A the state of the environment
   */
  tesseract_scene_graph::SceneState getState(const std::unordered_map<std::string, double>& joints) const;
  tesseract_scene_graph::SceneState getState(const std::vector<std::string>& joint_names,
                                             const Eigen::Ref<const Eigen::VectorXd>& joint_values) const;

  /** @brief Get the current state of the environment */
  tesseract_scene_graph::SceneState getState() const;

  /** @brief Last update time. Updated when any change to the environment occurs */
  std::chrono::system_clock::time_point getTimestamp() const;

  /** @brief Last update time to current state. Updated only when current state is updated */
  std::chrono::system_clock::time_point getCurrentStateTimestamp() const;

  /**
   * @brief Get a link in the environment
   * @param name The name of the link
   * @return Return nullptr if link name does not exists, otherwise a pointer to the link
   */
  tesseract_scene_graph::Link::ConstPtr getLink(const std::string& name) const;

  /**
   * @brief Get joint by name
   * @param name The name of the joint
   * @return Joint Const Pointer
   */
  tesseract_scene_graph::Joint::ConstPtr getJoint(const std::string& name) const;

  /**
   * @brief Gets the limits associated with a joint
   * @param joint_name Name of the joint to be updated
   * @return The joint limits set for the given joint
   */
  tesseract_scene_graph::JointLimits::ConstPtr getJointLimits(const std::string& joint_name) const;

  /**
   * @brief Get whether a link should be considered during collision checking
   * @return True if should be considered during collision checking, otherwise false
   */
  bool getLinkCollisionEnabled(const std::string& name) const;

  /**
   * @brief Get a given links visibility setting
   * @return True if should be visible, otherwise false
   */
  bool getLinkVisibility(const std::string& name) const;

  /**
   * @brief Get the allowed collision matrix
   * @return AllowedCollisionMatrixConstPtr
   */
  tesseract_common::AllowedCollisionMatrix::ConstPtr getAllowedCollisionMatrix() const;

  /**
   * @brief Get a vector of joint names in the environment
   * @return A vector of joint names
   */
  std::vector<std::string> getJointNames() const;

  /**
   * @brief Get a vector of active joint names in the environment
   * @return A vector of active joint names
   */
  std::vector<std::string> getActiveJointNames() const;

  /**
   * @brief Get the current state of the environment
   *
   * Order should be the same as getActiveJointNames()
   *
   * @return A vector of joint values
   */
  Eigen::VectorXd getCurrentJointValues() const;

  /**
   * @brief Get the current joint values for a vector of joints
   *
   * Order should be the same as the input vector
   *
   * @return A vector of joint values
   */
  Eigen::VectorXd getCurrentJointValues(const std::vector<std::string>& joint_names) const;

  /**
   * @brief Get the root link name
   * @return String
   */
  std::string getRootLinkName() const;

  /**
   * @brief Get a vector of link names in the environment
   * @return A vector of link names
   */
  std::vector<std::string> getLinkNames() const;

  /**
   * @brief Get a vector of active link names in the environment
   * @return A vector of active link names
   */
  std::vector<std::string> getActiveLinkNames() const;

  /**
   * @brief Get a vector of active link names affected by the provided joints in the environment
   * @param joint_names A list of joint names
   * @return A vector of active link names
   */
  std::vector<std::string> getActiveLinkNames(const std::vector<std::string>& joint_names) const;

  /**
   * @brief Get a vector of static link names in the environment
   * @return A vector of static link names
   */
  std::vector<std::string> getStaticLinkNames() const;

  /**
   * @brief Get a vector of static link names not affected by the provided joints in the environment
   * @param joint_names A list of joint names
   * @return A vector of static link names
   */
  std::vector<std::string> getStaticLinkNames(const std::vector<std::string>& joint_names) const;

  /**
   * @brief Get all of the links transforms
   *
   * Order should be the same as getLinkNames()
   *
   * @return Get a vector of transforms for all links in the environment.
   */
  tesseract_common::VectorIsometry3d getLinkTransforms() const;

  /**
   * @brief Get the transform corresponding to the link.
   * @return Transform and is identity when no transform is available.
   */
  Eigen::Isometry3d getLinkTransform(const std::string& link_name) const;

  /**
   * @brief Get transform between two links using the current state
   * @param from_link_name The link name the transform should be relative to
   * @param to_link_name The link name to get transform
   * @return The relative transform = inv(Transform(from_link_name)) * Transform(to_link_name)
   */
  Eigen::Isometry3d getRelativeLinkTransform(const std::string& from_link_name, const std::string& to_link_name) const;

  /**
   * @brief Returns a clone of the environments state solver
   *
   * The Environment::getState contains mutex's which is may not be needed in all motion planners. This allows the user
   * to get snap shot of the environment to calculate the state.
   *
   * @return A clone of the environments state solver
   */
  tesseract_scene_graph::StateSolver::UPtr getStateSolver() const;

  /**
   * @brief Get the kinematics information
   * @return The kinematics information
   */
  tesseract_srdf::KinematicsInformation getKinematicsInformation() const;

  /**
   * @brief Get the available group names
   * @return The group names
   */
  tesseract_srdf::GroupNames getGroupNames() const;

  /**
   * @brief Get the contact managers plugin information
   * @return The contact managers plugin information
   */
  tesseract_common::ContactManagersPluginInfo getContactManagersPluginInfo() const;

  /**
   * @brief Set the active discrete contact manager
   * @param name The name used to register the contact manager
   * @return True of name exists in DiscreteContactManagerFactory
   */
  bool setActiveDiscreteContactManager(const std::string& name);

  /** @brief Get a copy of the environments active discrete contact manager */
  tesseract_collision::DiscreteContactManager::UPtr getDiscreteContactManager() const;

  /**
   * @brief Set the cached internal copy of the environments active discrete contact manager not nullptr
   * @details This can be useful to save space in the event the environment is being saved
   */
  void clearCachedDiscreteContactManager() const;

  /** @brief Get a copy of the environments available discrete contact manager by name */
  tesseract_collision::DiscreteContactManager::UPtr getDiscreteContactManager(const std::string& name) const;

  /**
   * @brief Set the active continuous contact manager
   * @param name The name used to register the contact manager
   * @return True of name exists in ContinuousContactManagerFactory
   */
  bool setActiveContinuousContactManager(const std::string& name);

  /** @brief Get a copy of the environments active continuous contact manager */
  tesseract_collision::ContinuousContactManager::UPtr getContinuousContactManager() const;

  /**
   * @brief Set the cached internal copy of the environments active continuous contact manager not nullptr
   * @details This can be useful to save space in the event the environment is being saved
   */
  void clearCachedContinuousContactManager() const;

  /** @brief Get a copy of the environments available continuous contact manager by name */
  tesseract_collision::ContinuousContactManager::UPtr getContinuousContactManager(const std::string& name) const;

  /** @brief Get the environment collision margin data */
  tesseract_common::CollisionMarginData getCollisionMarginData() const;

  /**
   * @brief Lock the environment when wanting to make multiple reads
   * @details This is useful when making multiple read function calls and don't want the data
   * to get updated between function calls when there a multiple threads updating a single environment
   */
  std::shared_lock<std::shared_mutex> lockRead() const;

  /**
   * @brief These operators are to facilitate checking serialization but may have value elsewhere
   * @param rhs The environment to compare
   * @return True if they are equal otherwise false
   */
  bool operator==(const Environment& rhs) const;
  bool operator!=(const Environment& rhs) const;

protected:
  /**
   * @brief Identifies if the object has been initialized
   * @note This is intentionally not serialized it will auto updated
   */
  bool initialized_{ false };

  /**
   * @brief This increments when the scene graph is modified
   * @note This is intentionally not serialized it will auto updated
   */
  int revision_{ 0 };

  /** @brief This is the revision number after initialization used when reset is called */
  int init_revision_{ 0 };

  /** @brief The history of commands applied to the environment after initialization */
  Commands commands_;

  /**
   * @brief Tesseract Scene Graph
   * @note This is intentionally not serialized it will auto updated
   */
  tesseract_scene_graph::SceneGraph::Ptr scene_graph_;

  /**
   * @brief Tesseract Scene Graph Const
   * @note This is intentionally not serialized it will auto updated
   */
  tesseract_scene_graph::SceneGraph::ConstPtr scene_graph_const_;

  /**
   * @brief The kinematics information
   * @note This is intentionally not serialized it will auto updated
   */
  tesseract_srdf::KinematicsInformation kinematics_information_;

  /**
   * @brief The kinematics factory
   * @note This is intentionally not serialized it will auto updated
   */
  tesseract_kinematics::KinematicsPluginFactory kinematics_factory_;

  /** @brief Current state of the environment */
  tesseract_scene_graph::SceneState current_state_;

  /** @brief Environment timestamp */
  std::chrono::system_clock::time_point timestamp_{ std::chrono::system_clock::now() };

  /** @brief Current state timestamp */
  std::chrono::system_clock::time_point current_state_timestamp_{ std::chrono::system_clock::now() };

  /**
   * @brief Tesseract State Solver
   * @note This is intentionally not serialized it will auto updated
   */
  tesseract_scene_graph::MutableStateSolver::UPtr state_solver_;

  /**
   * @brief The function used to determine if two objects are allowed in collision
   * @todo This needs to be switched to class so it may be serialized
   */
  tesseract_collision::IsContactAllowedFn is_contact_allowed_fn_;

  /**
   * @brief A vector of user defined callbacks for locating tool center point
   * @todo This needs to be switched to class so it may be serialized
   */
  std::vector<FindTCPOffsetCallbackFn> find_tcp_cb_;

  /**
   * @brief A map of user defined event callback functions
   * @details This should not be cloned or serialized
   */
  std::map<std::size_t, EventCallbackFn> event_cb_;

  /** @brief Used when initialized by URDF_STRING, URDF_STRING_SRDF_STRING, URDF_PATH, and URDF_PATH_SRDF_PATH */
  tesseract_common::ResourceLocator::ConstPtr resource_locator_;

  /**
   * @brief The contact manager information
   * @note This is intentionally not serialized it will auto updated
   */
  tesseract_common::ContactManagersPluginInfo contact_managers_plugin_info_;

  /**
   * @brief The contact managers factory
   * @note This is intentionally not serialized it will auto updated
   */
  tesseract_collision::ContactManagersPluginFactory contact_managers_factory_;

  /**
   * @brief The collision margin data
   * @note This is intentionally not serialized it will auto updated
   */
  tesseract_collision::CollisionMarginData collision_margin_data_;

  /**
   * @brief The discrete contact manager object
   * @note This is intentionally not serialized it will auto updated
   */
  mutable tesseract_collision::DiscreteContactManager::UPtr discrete_manager_;
  mutable std::shared_mutex discrete_manager_mutex_;

  /**
   * @brief The continuous contact manager object
   * @note This is intentionally not serialized it will auto updated
   */
  mutable tesseract_collision::ContinuousContactManager::UPtr continuous_manager_;
  mutable std::shared_mutex continuous_manager_mutex_;

  /**
   * @brief A cache of group joint names to provide faster access
   * @details This will cleared when environment changes
   * @note This is intentionally not serialized it will auto updated
   */
  mutable std::unordered_map<std::string, std::vector<std::string>> group_joint_names_cache_;
  mutable std::shared_mutex group_joint_names_cache_mutex_;

  /**
   * @brief A cache of joint groups to provide faster access
   * @details This will cleared when environment changes
   * @note This is intentionally not serialized it will auto updated
   */
  mutable std::unordered_map<std::string, tesseract_kinematics::JointGroup::UPtr> joint_group_cache_;
  mutable std::shared_mutex joint_group_cache_mutex_;

  /**
   * @brief A cache of kinematic groups to provide faster access
   * @details This will cleared when environment changes
   * @note This is intentionally not serialized it will auto updated
   */
  mutable std::map<std::pair<std::string, std::string>, tesseract_kinematics::KinematicGroup::UPtr>
      kinematic_group_cache_;
  mutable std::shared_mutex kinematic_group_cache_mutex_;

  /** @brief The environment can be accessed from multiple threads, need use mutex throughout */
  mutable std::shared_mutex mutex_;

  /** This will update the contact managers transforms */
  void currentStateChanged();

  /** This will notify the state solver that the environment has changed */
  void environmentChanged();

  /**
   * @brief @brief Passes a current state changed event to the callbacks
   * @note This does not take a lock
   */
  void triggerCurrentStateChangedCallbacks();

  /**
   * @brief Passes a environment changed event to the callbacks
   * @note This does not take a lock
   */
  void triggerEnvironmentChangedCallbacks();

private:
  bool removeLinkHelper(const std::string& name);

  static void getCollisionObject(tesseract_collision::CollisionShapesConst& shapes,
                                 tesseract_common::VectorIsometry3d& shape_poses,
                                 const tesseract_scene_graph::Link& link);

  bool setActiveDiscreteContactManagerHelper(const std::string& name);
  bool setActiveContinuousContactManagerHelper(const std::string& name);

  tesseract_collision::DiscreteContactManager::UPtr getDiscreteContactManagerHelper(const std::string& name) const;

  tesseract_collision::ContinuousContactManager::UPtr getContinuousContactManagerHelper(const std::string& name) const;

  bool initHelper(const Commands& commands);
  static Commands getInitCommands(const tesseract_scene_graph::SceneGraph& scene_graph,
                                  const tesseract_srdf::SRDFModel::ConstPtr& srdf_model = nullptr);

  /** @brief Apply Command Helper which does not lock */
  bool applyCommandsHelper(const Commands& commands);

  // Command Helper function
  bool applyAddCommand(AddLinkCommand::ConstPtr cmd);
  bool applyMoveLinkCommand(const MoveLinkCommand::ConstPtr& cmd);
  bool applyMoveJointCommand(const MoveJointCommand::ConstPtr& cmd);
  bool applyRemoveLinkCommand(const RemoveLinkCommand::ConstPtr& cmd);
  bool applyRemoveJointCommand(const RemoveJointCommand::ConstPtr& cmd);
  bool applyReplaceJointCommand(const ReplaceJointCommand::ConstPtr& cmd);
  bool applyChangeLinkOriginCommand(const ChangeLinkOriginCommand::ConstPtr& cmd);
  bool applyChangeJointOriginCommand(const ChangeJointOriginCommand::ConstPtr& cmd);
  bool applyChangeLinkCollisionEnabledCommand(const ChangeLinkCollisionEnabledCommand::ConstPtr& cmd);
  bool applyChangeLinkVisibilityCommand(const ChangeLinkVisibilityCommand::ConstPtr& cmd);
  bool applyAddAllowedCollisionCommand(const AddAllowedCollisionCommand::ConstPtr& cmd);
  bool applyRemoveAllowedCollisionCommand(const RemoveAllowedCollisionCommand::ConstPtr& cmd);
  bool applyRemoveAllowedCollisionLinkCommand(const RemoveAllowedCollisionLinkCommand::ConstPtr& cmd);
  bool applyAddSceneGraphCommand(AddSceneGraphCommand::ConstPtr cmd);
  bool applyChangeJointPositionLimitsCommand(const ChangeJointPositionLimitsCommand::ConstPtr& cmd);
  bool applyChangeJointVelocityLimitsCommand(const ChangeJointVelocityLimitsCommand::ConstPtr& cmd);
  bool applyChangeJointAccelerationLimitsCommand(const ChangeJointAccelerationLimitsCommand::ConstPtr& cmd);
  bool applyAddKinematicsInformationCommand(const AddKinematicsInformationCommand::ConstPtr& cmd);
  bool applyChangeCollisionMarginsCommand(const ChangeCollisionMarginsCommand::ConstPtr& cmd);
  bool applyAddContactManagersPluginInfoCommand(const AddContactManagersPluginInfoCommand::ConstPtr& cmd);
  bool applySetActiveContinuousContactManagerCommand(const SetActiveContinuousContactManagerCommand::ConstPtr& cmd);
  bool applySetActiveDiscreteContactManagerCommand(const SetActiveDiscreteContactManagerCommand::ConstPtr& cmd);

  friend class boost::serialization::access;
  template <class Archive>
  void save(Archive& ar, const unsigned int version) const;  // NOLINT

  template <class Archive>
  void load(Archive& ar, const unsigned int version);  // NOLINT

  template <class Archive>
  void serialize(Archive& ar, const unsigned int version);  // NOLINT
};
}  // namespace tesseract_environment

#endif  // TESSERACT_ENVIRONMENT_ENVIRONMENT_H
