/**
 * @file Environment.h
 * @brief Tesseract Environment.
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
#ifndef TESSERACT_ENVIRONMENT_ENVIRONMENT_H
#define TESSERACT_ENVIRONMENT_ENVIRONMENT_H

#include <tesseract_common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <functional>
#include <vector>
#include <string>
#include <chrono>
#include <set>
#include <memory>
#include <shared_mutex>
#include <Eigen/Geometry>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract_common/fwd.h>
#include <tesseract_geometry/fwd.h>
#include <tesseract_scene_graph/fwd.h>
#include <tesseract_state_solver/fwd.h>
#include <tesseract_srdf/fwd.h>
#include <tesseract_kinematics/core/fwd.h>
#include <tesseract_collision/core/fwd.h>
#include <tesseract_environment/fwd.h>

#include <filesystem>
#include <tesseract_common/eigen_types.h>
#include <tesseract_common/any_poly.h>
#include <tesseract_common/contact_allowed_validator.h>

namespace tesseract_environment
{
/**
 * @brief Function signature for adding additional callbacks for looking up TCP information
 *
 * The function should throw and exception if not located
 */
using FindTCPOffsetCallbackFn = std::function<Eigen::Isometry3d(const tesseract_common::ManipulatorInfo&)>;

using EventCallbackFn = std::function<void(const Event& event)>;

class Environment;
template <class Archive>
void serialize(Archive& ar, Environment& obj);

class EnvironmentContactAllowedValidator;
template <class Archive>
void serialize(Archive& ar, EnvironmentContactAllowedValidator& obj);

class EnvironmentContactAllowedValidator : public tesseract_common::ContactAllowedValidator
{
public:
  EnvironmentContactAllowedValidator() = default;  // Required for serialization
  EnvironmentContactAllowedValidator(std::shared_ptr<const tesseract_scene_graph::SceneGraph> scene_graph);

  bool operator()(const std::string& link_name1, const std::string& link_name2) const override;

protected:
  std::shared_ptr<const tesseract_scene_graph::SceneGraph> scene_graph_;

  template <class Archive>
  friend void ::tesseract_environment::serialize(Archive& ar, EnvironmentContactAllowedValidator& obj);
};

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
  Environment();
  virtual ~Environment();
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
  bool init(const std::vector<std::shared_ptr<const Command>>& commands);

  /**
   * @brief Initialize the Environment
   *
   * The template class provided should be a derived class from StateSolver.
   *
   * @param scene_graph The scene graph to initialize the environment.
   * @return True if successful, otherwise false
   */
  bool init(const tesseract_scene_graph::SceneGraph& scene_graph,
            const std::shared_ptr<const tesseract_srdf::SRDFModel>& srdf_model = nullptr);

  bool init(const std::string& urdf_string, const std::shared_ptr<const tesseract_common::ResourceLocator>& locator);

  bool init(const std::string& urdf_string,
            const std::string& srdf_string,
            const std::shared_ptr<const tesseract_common::ResourceLocator>& locator);

  bool init(const std::filesystem::path& urdf_path,
            const std::shared_ptr<const tesseract_common::ResourceLocator>& locator);

  bool init(const std::filesystem::path& urdf_path,
            const std::filesystem::path& srdf_path,
            const std::shared_ptr<const tesseract_common::ResourceLocator>& locator);

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
   * @brief Get the initialization revision number
   * @return Initialization revision number
   */
  int getInitRevision() const;

  /**
   * @brief Get Environment command history post initialization
   * @return List of commands
   */
  std::vector<std::shared_ptr<const Command>> getCommandHistory() const;

  /**
   * @brief Applies the commands to the environment
   * @param commands Commands to be applied to the environment
   * @return true if successful. If returned false, then only a partial set of commands have been applied. Call
   * getCommandHistory to check. Some commands are not checked for success
   */
  bool applyCommands(const std::vector<std::shared_ptr<const Command>>& commands);

  /**
   * @brief Apply command to the environment
   * @param command Command to be applied to the environment
   * @return true if successful. If returned false, then the command have not been applied.
   * Some type of Command are not checked for success
   */
  bool applyCommand(std::shared_ptr<const Command> command);

  /**
   * @brief Get the Scene Graph
   * @return SceneGraphConstPtr
   */
  std::shared_ptr<const tesseract_scene_graph::SceneGraph> getSceneGraph() const;

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
  std::shared_ptr<const tesseract_kinematics::JointGroup> getJointGroup(const std::string& group_name) const;

  /**
   * @brief Get a joint group given a vector of joint names
   * @param name The name to assign to the joint group
   * @param joint_names The joint names that make up the group
   * @return A joint group
   */
  std::shared_ptr<const tesseract_kinematics::JointGroup>
  getJointGroup(const std::string& name, const std::vector<std::string>& joint_names) const;

  /**
   * @brief Get a kinematic group given group name and solver name
   * @details If ik_solver_name is empty it will choose the first ik solver for the group
   * @param group_name The group name
   * @param ik_solver_name The IK solver name
   * @return A kinematics group
   */
  std::shared_ptr<const tesseract_kinematics::KinematicGroup>
  getKinematicGroup(const std::string& group_name, const std::string& ik_solver_name = "") const;

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
  void setResourceLocator(std::shared_ptr<const tesseract_common::ResourceLocator> locator);

  /**
   * @brief Get the resource locator assigned
   * @details This can be a nullptr
   * @return The resource locator assigned to the environment
   */
  std::shared_ptr<const tesseract_common::ResourceLocator> getResourceLocator() const;

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
  void setState(const std::unordered_map<std::string, double>& joints,
                const tesseract_common::TransformMap& floating_joints = {});
  void setState(const std::vector<std::string>& joint_names,
                const Eigen::Ref<const Eigen::VectorXd>& joint_values,
                const tesseract_common::TransformMap& floating_joints = {});

  /**
   * @brief Set the current state of the floating joint values
   * @param floating_joint_values The floating joint values to set
   */
  void setState(const tesseract_common::TransformMap& floating_joints);

  /**
   * @brief Get the state of the environment for a given set or subset of joint values.
   *
   * This does not change the internal state of the environment.
   *
   * @param joints A map of joint names to joint values to change.
   * @return A the state of the environment
   */
  tesseract_scene_graph::SceneState getState(const std::unordered_map<std::string, double>& joints,
                                             const tesseract_common::TransformMap& floating_joints = {}) const;
  tesseract_scene_graph::SceneState getState(const std::vector<std::string>& joint_names,
                                             const Eigen::Ref<const Eigen::VectorXd>& joint_values,
                                             const tesseract_common::TransformMap& floating_joints = {}) const;

  /**
   * @brief Get the state given floating joint values
   * @param floating_joint_values The floating joint values to leverage
   * @return A the state of the environment
   */
  tesseract_scene_graph::SceneState getState(const tesseract_common::TransformMap& floating_joints) const;

  /** @brief Get the current state of the environment */
  tesseract_scene_graph::SceneState getState() const;

  /**
   * @brief Get the link transforms of the scene for a given set or subset of joint values.
   *
   * This is provided to optimize motion planning where link_transforms are poplated multiple time
   *
   * This does not change the internal state of the solver.
   *
   * @param link_transforms The link_transforms to populate with data.
   * @param joints A map of joint names to joint values to change.
   * @param joint_values The joint values
   * @param floating_joints The floating joint origin transform
   */
  void getLinkTransforms(tesseract_common::TransformMap& link_transforms,
                         const std::vector<std::string>& joint_names,
                         const Eigen::Ref<const Eigen::VectorXd>& joint_values,
                         const tesseract_common::TransformMap& floating_joints) const;

  /**
   * @brief Get the link transforms of the scene for a given set or subset of joint values.
   *
   * This is provided to optimize motion planning where link_transforms are poplated multiple time
   *
   * This does not change the internal state of the solver.
   *
   * @param link_transforms The link_transforms to populate with data.
   * @param joints A map of joint names to joint values to change.
   * @param joint_values The joint values
   */
  void getLinkTransforms(tesseract_common::TransformMap& link_transforms,
                         const std::vector<std::string>& joint_names,
                         const Eigen::Ref<const Eigen::VectorXd>& joint_values) const;

  /** @brief Last update time. Updated when any change to the environment occurs */
  std::chrono::system_clock::time_point getTimestamp() const;

  /** @brief Last update time to current state. Updated only when current state is updated */
  std::chrono::system_clock::time_point getCurrentStateTimestamp() const;

  /**
   * @brief Get a link in the environment
   * @param name The name of the link
   * @return Return nullptr if link name does not exists, otherwise a pointer to the link
   */
  std::shared_ptr<const tesseract_scene_graph::Link> getLink(const std::string& name) const;

  /**
   * @brief Get joint by name
   * @param name The name of the joint
   * @return Joint Const Pointer
   */
  std::shared_ptr<const tesseract_scene_graph::Joint> getJoint(const std::string& name) const;

  /**
   * @brief Gets the limits associated with a joint
   * @param joint_name Name of the joint to be updated
   * @return The joint limits set for the given joint
   */
  std::shared_ptr<const tesseract_scene_graph::JointLimits> getJointLimits(const std::string& joint_name) const;

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
  std::shared_ptr<const tesseract_common::AllowedCollisionMatrix> getAllowedCollisionMatrix() const;

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
   * @brief Get the current floating joint values
   * @return The joint origin transform for the floating joint
   */
  tesseract_common::TransformMap getCurrentFloatingJointValues() const;

  /**
   * @brief Get the current floating joint values
   * @return The joint origin transform for the floating joint
   */
  tesseract_common::TransformMap getCurrentFloatingJointValues(const std::vector<std::string>& joint_names) const;

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
  std::unique_ptr<tesseract_scene_graph::StateSolver> getStateSolver() const;

  /**
   * @brief Get the kinematics information
   * @return The kinematics information
   */
  tesseract_srdf::KinematicsInformation getKinematicsInformation() const;

  /**
   * @brief Get the available group names
   * @return The group names
   */
  std::set<std::string> getGroupNames() const;

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
  std::unique_ptr<tesseract_collision::DiscreteContactManager> getDiscreteContactManager() const;

  /**
   * @brief Set the cached internal copy of the environments active discrete contact manager not nullptr
   * @details This can be useful to save space in the event the environment is being saved
   */
  void clearCachedDiscreteContactManager() const;

  /** @brief Get a copy of the environments available discrete contact manager by name */
  std::unique_ptr<tesseract_collision::DiscreteContactManager> getDiscreteContactManager(const std::string& name) const;

  /**
   * @brief Set the active continuous contact manager
   * @param name The name used to register the contact manager
   * @return True of name exists in ContinuousContactManagerFactory
   */
  bool setActiveContinuousContactManager(const std::string& name);

  /** @brief Get a copy of the environments active continuous contact manager */
  std::unique_ptr<tesseract_collision::ContinuousContactManager> getContinuousContactManager() const;

  /**
   * @brief Set the cached internal copy of the environments active continuous contact manager not nullptr
   * @details This can be useful to save space in the event the environment is being saved
   */
  void clearCachedContinuousContactManager() const;

  /** @brief Get a copy of the environments available continuous contact manager by name */
  std::unique_ptr<tesseract_collision::ContinuousContactManager>
  getContinuousContactManager(const std::string& name) const;

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

private:
  /** @brief The environment can be accessed from multiple threads, need use mutex throughout */
  mutable std::shared_mutex mutex_;

  /** @brief This hides specific implemenation details to allow forward declarations */
  struct Implementation;
  std::unique_ptr<Implementation> impl_;

  /** @brief This is provided for serialization */
  void init(const std::vector<std::shared_ptr<const Command>>& commands,
            int init_revision,
            const std::chrono::system_clock::time_point& timestamp,
            const tesseract_scene_graph::SceneState& current_state,
            const std::chrono::system_clock::time_point& current_state_timestamp,
            const std::shared_ptr<const tesseract_common::ResourceLocator>& resource_locator);

  template <class Archive>
  friend void ::tesseract_environment::serialize(Archive& ar, Environment& obj);

public:
  /** @brief This should only be used by the clone method */
  explicit Environment(std::unique_ptr<Implementation> impl);
};

using EnvironmentPtrAnyPoly = tesseract_common::AnyWrapper<std::shared_ptr<tesseract_environment::Environment>>;
using EnvironmentConstPtrAnyPoly =
    tesseract_common::AnyWrapper<std::shared_ptr<const tesseract_environment::Environment>>;
}  // namespace tesseract_environment

#endif  // TESSERACT_ENVIRONMENT_ENVIRONMENT_H
