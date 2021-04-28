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

#include <tesseract_environment/core/types.h>
#include <tesseract_environment/core/commands.h>
#include <tesseract_environment/core/state_solver.h>
#include <tesseract_environment/core/manipulator_manager.h>
#include <tesseract_collision/core/discrete_contact_manager.h>
#include <tesseract_collision/core/discrete_contact_manager_factory.h>
#include <tesseract_collision/core/continuous_contact_manager.h>
#include <tesseract_collision/core/continuous_contact_manager_factory.h>
#include <tesseract_scene_graph/graph.h>
#include <tesseract_scene_graph/utils.h>
#include <tesseract_scene_graph/resource_locator.h>
#include <tesseract_urdf/urdf_parser.h>
#include <tesseract_srdf/srdf_model.h>
#include <tesseract_common/manipulator_info.h>
#include <tesseract_common/types.h>
#include <tesseract_common/utils.h>

#ifdef SWIG
%shared_ptr(tesseract_environment::Environment)
#endif  // SWIG

namespace tesseract_environment
{
/**
 * @brief Function signature for adding additional callbacks for looking up TCP information
 *
 * The function should throw and exception if not located
 */
using FindTCPCallbackFn = std::function<Eigen::Isometry3d(const tesseract_common::ManipulatorInfo&)>;

class Environment
{
public:
  // LCOV_EXCL_START
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  // LCOV_EXCL_STOP

  using Ptr = std::shared_ptr<Environment>;
  using ConstPtr = std::shared_ptr<const Environment>;

  /**
   * @brief Default constructor
   * @param register_default_contant_managers Indicate if the default contact managers should be registered
   */
  Environment(bool register_default_contact_managers = true);

  virtual ~Environment() = default;
  Environment(const Environment&) = delete;
  Environment& operator=(const Environment&) = delete;
  Environment(Environment&&) = delete;
  Environment& operator=(Environment&&) = delete;

  /**
   * @brief Initialize the Environment
   *
   * The templated class provided should be a derived class from StateSolver.
   *
   * @param scene_graph The scene graph to initialize the environment.
   * @return True if successful, otherwise false
   */
  template <typename S>
  bool init(const Commands& commands)
  {
    std::unique_lock<std::shared_mutex> lock(mutex_);
    state_solver_ = std::make_shared<S>();
    return initHelper(commands);
  }

  /**
   * @brief Initialize the Environment
   *
   * The templated class provided should be a derived class from StateSolver.
   *
   * @param scene_graph The scene graph to initialize the environment.
   * @return True if successful, otherwise false
   */
  template <typename S>
  bool init(const tesseract_scene_graph::SceneGraph& scene_graph,
            const tesseract_srdf::SRDFModel::ConstPtr& srdf_model = nullptr)
  {
    Commands commands = getInitCommands(scene_graph, srdf_model);
    return init<S>(commands);
  }

  template <typename S>
  bool init(const std::string& urdf_string, const tesseract_scene_graph::ResourceLocator::Ptr& locator)
  {
    resource_locator_ = locator;

    // Parse urdf string into Scene Graph
    tesseract_scene_graph::SceneGraph::Ptr scene_graph;
    try
    {
      scene_graph = tesseract_urdf::parseURDFString(urdf_string, resource_locator_);
    }
    catch (const std::exception& e)
    {
      CONSOLE_BRIDGE_logError("Failed to parse URDF.");
      tesseract_common::printNestedException(e);
      return false;
    }

    Commands commands = getInitCommands(*scene_graph);
    return init<S>(commands);
  }

  template <typename S>
  bool init(const std::string& urdf_string,
            const std::string& srdf_string,
            const tesseract_scene_graph::ResourceLocator::Ptr& locator)
  {
    resource_locator_ = locator;

    // Parse urdf string into Scene Graph
    tesseract_scene_graph::SceneGraph::Ptr scene_graph;
    try
    {
      scene_graph = tesseract_urdf::parseURDFString(urdf_string, resource_locator_);
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
      srdf->initString(*scene_graph, srdf_string);
    }
    catch (const std::exception& e)
    {
      CONSOLE_BRIDGE_logError("Failed to parse SRDF.");
      tesseract_common::printNestedException(e);
      return false;
    }

    Commands commands = getInitCommands(*scene_graph, srdf);
    return init<S>(commands);
  }

  template <typename S>
  bool init(const tesseract_common::fs::path& urdf_path, const tesseract_scene_graph::ResourceLocator::Ptr& locator)
  {
    resource_locator_ = locator;

    // Parse urdf file into Scene Graph
    tesseract_scene_graph::SceneGraph::Ptr scene_graph;
    try
    {
      scene_graph = tesseract_urdf::parseURDFFile(urdf_path.string(), resource_locator_);
    }
    catch (const std::exception& e)
    {
      CONSOLE_BRIDGE_logError("Failed to parse URDF.");
      tesseract_common::printNestedException(e);
      return false;
    }

    Commands commands = getInitCommands(*scene_graph);
    return init<S>(commands);
  }

  template <typename S>
  bool init(const tesseract_common::fs::path& urdf_path,
            const tesseract_common::fs::path& srdf_path,
            const tesseract_scene_graph::ResourceLocator::Ptr& locator)
  {
    resource_locator_ = locator;

    // Parse urdf file into Scene Graph
    tesseract_scene_graph::SceneGraph::Ptr scene_graph;
    try
    {
      scene_graph = tesseract_urdf::parseURDFFile(urdf_path.string(), resource_locator_);
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
      srdf->initFile(*scene_graph, srdf_path.string());
    }
    catch (const std::exception& e)
    {
      CONSOLE_BRIDGE_logError("Failed to parse SRDF.");
      tesseract_common::printNestedException(e);
      return false;
    }

    Commands commands = getInitCommands(*scene_graph, srdf);
    return init<S>(commands);
  }

  /**
   * @brief Clone the environment
   * @return A clone of the environment
   */
  Environment::Ptr clone() const;

  /**
   * @brief reset to initialized state
   * @details If the environment has not been initialized then this returns false
   * @return True if environment was successfully reset, otherwise false.
   */
  bool reset();

  /** @brief clear content and uninitialize */
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
  bool applyCommand(const Command::ConstPtr& command);

  /**
   * @brief Get the Scene Graph
   * @return SceneGraphConstPtr
   */
  virtual const tesseract_scene_graph::SceneGraph::ConstPtr& getSceneGraph() const;

  /**
   * @brief Get the manipulator manager
   * @todo This should go away and should expose methods for adding and removing objects from the manager and storing
   *       as commands
   * @warning Changes made to the manager are not capture in the command history.
   * @return The manipulator manager
   */
  virtual ManipulatorManager::Ptr getManipulatorManager();

  /**
   * @brief Get the manipulator manager const
   * @return The manipulator manager const
   */
  virtual ManipulatorManager::ConstPtr getManipulatorManager() const;

  /**
   * @brief Find tool center point provided in the manipulator info
   *
   * If manipulator information tcp is defined as a string it does the following
   *    - First check if manipulator info is empty, if so return identity
   *    - Next if not empty, it checks if the manipulator manager has tcp defined for the manipulator group
   *    - Next if not found, it looks up the tcp name in the EnvState along with manipulator tip link to calculate tcp
   *    - Next if not found, it leverages the user defind callbacks to try an locate the tcp information.
   *    - Next throw an exception, because no tcp information was located.
   *
   * @param manip_info The manipulator info
   * @return The tool center point
   */
  Eigen::Isometry3d findTCP(const tesseract_common::ManipulatorInfo& manip_info) const;

  /**
   * @brief This allows for user defined callbacks for looking up TCP information
   * @param fn User defind callback function for locating TCP information
   */
  void addFindTCPCallback(FindTCPCallbackFn fn);

  /**
   * @brief This get the current find tcp callbacks stored in the environment
   * @return A vector of callback functions
   */
  std::vector<FindTCPCallbackFn> getFindTCPCallbacks() const;

  /**
   * @brief Set resource locator for environment
   * @param locator The resource locator
   */
  void setResourceLocator(tesseract_scene_graph::ResourceLocator::Ptr locator);

  /**
   * @brief Get the resource locator assigned
   * @details This can be a nullptr
   * @return The resource locator assigned to the environment
   */
  tesseract_scene_graph::ResourceLocator::Ptr getResourceLocator() const;

  /** @brief Give the environment a name */
  virtual void setName(const std::string& name);

  /** @brief Get the name of the environment
   *
   * This may be empty, if so check urdf name
   */
  virtual const std::string& getName() const;

  /**
   * @brief Set the current state of the environment
   *
   * After updating the current state these function must call currentStateChanged() which
   * will update the contact managers transforms
   *
   */
  virtual void setState(const std::unordered_map<std::string, double>& joints);
  virtual void setState(const std::vector<std::string>& joint_names, const std::vector<double>& joint_values);
  virtual void setState(const std::vector<std::string>& joint_names,
                        const Eigen::Ref<const Eigen::VectorXd>& joint_values);

  /**
   * @brief Get the state of the environment for a given set or subset of joint values.
   *
   * This does not change the internal state of the environment.
   *
   * @param joints A map of joint names to joint values to change.
   * @return A the state of the environment
   */
  virtual EnvState::Ptr getState(const std::unordered_map<std::string, double>& joints) const;
  virtual EnvState::Ptr getState(const std::vector<std::string>& joint_names,
                                 const std::vector<double>& joint_values) const;
  virtual EnvState::Ptr getState(const std::vector<std::string>& joint_names,
                                 const Eigen::Ref<const Eigen::VectorXd>& joint_values) const;

  /** @brief Get the current state of the environment */
  virtual EnvState::ConstPtr getCurrentState() const;

  /** @brief Get the current state timestamp */
  virtual std::chrono::high_resolution_clock::duration getCurrentStateTimestamp() const;

  /**
   * @brief Get a link in the environment
   * @param name The name of the link
   * @return Return nullptr if link name does not exists, otherwise a pointer to the link
   */
  virtual tesseract_scene_graph::Link::ConstPtr getLink(const std::string& name) const;

  /**
   * @brief Get joint by name
   * @param name The name of the joint
   * @return Joint Const Pointer
   */
  virtual tesseract_scene_graph::Joint::ConstPtr getJoint(const std::string& name) const;

  /**
   * @brief Gets the limits associated with a joint
   * @param joint_name Name of the joint to be updated
   * @return The joint limits set for the given joint
   */
  virtual tesseract_scene_graph::JointLimits::ConstPtr getJointLimits(const std::string& joint_name) const;

  /**
   * @brief Get whether a link should be considered during collision checking
   * @return True if should be condisdered during collision checking, otherwise false
   */
  virtual bool getLinkCollisionEnabled(const std::string& name) const;

  /**
   * @brief Get a given links visibility setting
   * @return True if should be visible, otherwise false
   */
  virtual bool getLinkVisibility(const std::string& name) const;

  /**
   * @brief Get the allowed collision matrix
   * @return AllowedCollisionMatrixConstPtr
   */
  virtual tesseract_scene_graph::AllowedCollisionMatrix::ConstPtr getAllowedCollisionMatrix() const;

  /**
   * @brief Get a vector of joint names in the environment
   * @return A vector of joint names
   */
  virtual std::vector<std::string> getJointNames() const;

  /**
   * @brief Get a vector of active joint names in the environment
   * @return A vector of active joint names
   */
  virtual std::vector<std::string> getActiveJointNames() const;

  /**
   * @brief Get the current state of the environment
   *
   * Order should be the same as getActiveJointNames()
   *
   * @return A vector of joint values
   */
  virtual Eigen::VectorXd getCurrentJointValues() const;

  /**
   * @brief Get the current joint values for a vector of joints
   *
   * Order should be the same as the input vector
   *
   * @return A vector of joint values
   */
  virtual Eigen::VectorXd getCurrentJointValues(const std::vector<std::string>& joint_names) const;

  /**
   * @brief Get the root link name
   * @return String
   */
  virtual const std::string& getRootLinkName() const;

  /**
   * @brief Get a vector of link names in the environment
   * @return A vector of link names
   */
  virtual std::vector<std::string> getLinkNames() const;

  /**
   * @brief Get a vector of active link names in the environment
   * @return A vector of active link names
   */
  virtual std::vector<std::string> getActiveLinkNames() const;

  /**
   * @brief Get all of the links transforms
   *
   * Order should be the same as getLinkNames()
   *
   * @return Get a vector of transforms for all links in the environment.
   */
  virtual tesseract_common::VectorIsometry3d getLinkTransforms() const;

  /**
   * @brief Get the transform corresponding to the link.
   * @return Transform and is identity when no transform is available.
   */
  virtual Eigen::Isometry3d getLinkTransform(const std::string& link_name) const;

  /**
   * @brief Returns a clone of the environments state solver
   *
   * The Environment::getState contains mutex's which is may not be needed in all motion planners. This allows the user
   * to get snap shot of the environment to calculate the state.
   *
   * @return A clone of the environments state solver
   */
  virtual StateSolver::Ptr getStateSolver() const;

  /**
   * @brief Set the active discrete contact manager
   * @param name The name used to registar the contact manager
   * @return True of name exists in DiscreteContactManagerFactory
   */
  virtual bool setActiveDiscreteContactManager(const std::string& name);

  /** @brief Get a copy of the environments active discrete contact manager */
  virtual tesseract_collision::DiscreteContactManager::Ptr getDiscreteContactManager() const;

  /** @brief Get a copy of the environments available discrete contact manager by name */
  virtual tesseract_collision::DiscreteContactManager::Ptr getDiscreteContactManager(const std::string& name) const;

  /**
   * @brief Set the active continuous contact manager
   * @param name The name used to registar the contact manager
   * @return True of name exists in ContinuousContactManagerFactory
   */
  virtual bool setActiveContinuousContactManager(const std::string& name);

  /** @brief Get a copy of the environments active continuous contact manager */
  virtual tesseract_collision::ContinuousContactManager::Ptr getContinuousContactManager() const;

  /** @brief Get a copy of the environments available continuous contact manager by name */
  virtual tesseract_collision::ContinuousContactManager::Ptr getContinuousContactManager(const std::string& name) const;

  /** @brief Get the environment collision margin data */
  virtual tesseract_common::CollisionMarginData getCollisionMarginData() const;

  /**
   * @brief Set the discrete contact manager
   *
   * This method should clear the contents of the manager and reload it with the objects
   * in the environment.
   */
  bool registerDiscreteContactManager(const std::string& name,
                                      tesseract_collision::DiscreteContactManagerFactoryCreateMethod create_function);

  /**
   * @brief Set the discrete contact manager
   *
   * This method should clear the contents of the manager and reload it with the objects
   * in the environment.
   */
  bool
  registerContinuousContactManager(const std::string& name,
                                   tesseract_collision::ContinuousContactManagerFactoryCreateMethod create_function);

  /**
   * @brief Register Default Contact Managers
   * @return True if successful, otherwis false
   */
  bool registerDefaultContactManagers();

  /**
   * @brief Add kinematics information to the environment
   * @param kin_info The kinematics information
   * @return true if successful, otherwise false
   */
  virtual bool addKinematicsInformation(const tesseract_srdf::KinematicsInformation& kin_info);

  /**
   * @brief Adds a link to the environment
   *
   *        This method should attach the link to the root link with a fixed joint
   *        with a joint name of joint_{link name}".
   *
   * @param link The link to be added to the graph
   * @return Return False if a link with the same name allready exists, otherwise true
   */
  virtual bool addLink(const tesseract_scene_graph::Link& link);

  /**
   * @brief Adds a link to the environment
   * @param link The link to be added to the graph
   * @return Return False if a link with the same name allready exists, otherwise true
   */
  virtual bool addLink(const tesseract_scene_graph::Link& link, const tesseract_scene_graph::Joint& joint);

  /**
   * @brief Move a link in the environment
   *
   *        This should delete the parent joint of the child link. All child links and joints follow.
   *
   * @param joint The new joint.
   * @return Return False if a link does not exists or has no parent joint, otherwise true
   */
  virtual bool moveLink(const tesseract_scene_graph::Joint& joint);

  /**
   * @brief Removes a link from the environment
   *
   *        Parent joint and all child components (links/joints) should be removed
   *
   * @param name Name of the link to be removed
   * @return Return False if a link does not exists, otherwise true
   */
  virtual bool removeLink(const std::string& name);

  /**
   * @brief Removes a joint from the environment
   *
   *        All child components (links/joints) should be removed
   *
   * @param name Name of the joint to be removed
   * @return Return False if a joint does not exists, otherwise true
   */
  virtual bool removeJoint(const std::string& name);

  /**
   * @brief Move a joint from one link to another
   *
   *        All child links & joints should follow
   *
   * @param joint_name The name of the joint to move
   * @param new_parent_link The name of the link to move to.
   * @return Return False if parent_link does not exists, otherwise true
   */
  virtual bool moveJoint(const std::string& joint_name, const std::string& parent_link);

  /**
   * @brief Changes the origin associated with the joint
   *
   * Note: This is the origin as in the "origin" tag in the URDF. This is the location of the
   * joint in the frame of the parent link.
   * @param joint_name Name of the joint to be updated
   * @param new_origin New transform to be set as the origin
   * @return
   */
  virtual bool changeJointOrigin(const std::string& joint_name, const Eigen::Isometry3d& new_origin);

  /**
   * @brief Changes the position limits associated with a joint
   * @param joint_name Name of the joint to be updated
   * @param limits New position limits to be set as the joint limits
   * @return
   */
  virtual bool changeJointPositionLimits(const std::string& joint_name, double lower, double upper);

  /**
   * @brief Changes the position limits associated with one or more joints
   * @param limits A map of joint names to new position limits.
   * For each limit pair, first is the lower limit second is the upper limit
   * @return
   */
  virtual bool changeJointPositionLimits(const std::unordered_map<std::string, std::pair<double, double>>& limits);

  /**
   * @brief Changes the velocity limits associated with a joint
   * @param joint_name Name of the joint to be updated
   * @param limits New velocity limits to be set as the joint limits
   * @return
   */
  virtual bool changeJointVelocityLimits(const std::string& joint_name, double limit);

  /**
   * @brief Changes the velocity limits associated with a joint
   * @param limits A map of joint names to new velocity limits
   * @return
   */
  virtual bool changeJointVelocityLimits(const std::unordered_map<std::string, double>& limits);

  /**
   * @brief Changes the acceleration limits associated with a joint
   * @param joint_name Name of the joint to be updated
   * @param limits New acceleration limits to be set as the joint limits
   * @return
   */
  virtual bool changeJointAccelerationLimits(const std::string& joint_name, double limit);

  /**
   * @brief Changes the acceleration limits associated with a joint
   * @param limits A map of joint names to new acceleration limits
   * @return
   */
  virtual bool changeJointAccelerationLimits(const std::unordered_map<std::string, double>& limits);

  /**
   * @brief Disable collision between two collision objects
   * @param link_name1 Collision object name
   * @param link_name2 Collision object name
   * @param reason The reason for disabling collison
   * @return True if successful, otherwise false
   */
  virtual bool addAllowedCollision(const std::string& link_name1,
                                   const std::string& link_name2,
                                   const std::string& reason);

  /**
   * @brief Remove disabled collision pair from allowed collision matrix
   * @param link_name1 Collision object name
   * @param link_name2 Collision object name
   * @return True if successful, otherwise false
   */
  virtual bool removeAllowedCollision(const std::string& link_name1, const std::string& link_name2);

  /**
   * @brief Remove disabled collision for any pair with link_name from allowed collision matrix
   * @param link_name Collision object name
   * @return True if successful, otherwise false
   */
  virtual bool removeAllowedCollision(const std::string& link_name);

  /**
   * @brief Set whether a link should be considered during collision checking
   * @param enabled True if should be condisdered during collision checking, otherwise false
   * @return True if successful, otherwise false
   */
  virtual bool setLinkCollisionEnabled(const std::string& name, bool enabled);

  /**
   * @brief Set a links visibility
   * @param name The name of the link
   * @param visibility True if should be visible, otherwise false
   * @return True if successful, otherwise false
   */
  virtual bool setLinkVisibility(const std::string& name, bool visibility);

  /** @brief Merge a graph into the current environment
   * @param scene_graph Const ref to the graph to be merged (said graph will be copied)
   * @param prefix string Will be prepended to every link and joint of the merged graph
   * @return Return False if any link or joint name collides with current environment, otherwise True
   * Merge a subgraph into the current environment, considering that the root of the merged graph is attached to the
   * root of the environment by a fixed joint and no displacement. Every joint and link of the subgraph will be copied
   * into the environment graph. The prefix argument is meant to allow adding multiple copies of the same subgraph with
   * different names */
  virtual bool addSceneGraph(const tesseract_scene_graph::SceneGraph& scene_graph, const std::string& prefix = "");

  /** @brief Merge a graph into the current environment
   * @param scene_graph Const ref to the graph to be merged (said graph will be copied)
   * @param root_joint Const ptr to the joint that connects current environment with root of the merged graph
   * @param prefix string Will be prepended to every link and joint of the merged graph
   * @return Return False if any link or joint name collides with current environment, otherwise True
   * Merge a subgraph into the current environment. Every joint and link of the subgraph will be copied into the
   * environment graph. The prefix argument is meant to allow adding multiple copies of the same subgraph with different
   * names */
  virtual bool addSceneGraph(const tesseract_scene_graph::SceneGraph& scene_graph,
                             const tesseract_scene_graph::Joint& joint,
                             const std::string& prefix = "");

protected:
  bool initialized_{ false }; /**< Identifies if the object has been initialized */
  int revision_{ 0 };         /**< This increments when the scene graph is modified */
  int init_revision_{ 0 };    /**< This is the revision number after initialization used when reset is called */
  Commands commands_;         /**< The history of commands applied to the environment after intialization */
  tesseract_scene_graph::SceneGraph::Ptr scene_graph_;                   /**< Tesseract Scene Graph */
  tesseract_scene_graph::SceneGraph::ConstPtr scene_graph_const_;        /**< Tesseract Scene Graph Const */
  ManipulatorManager::Ptr manipulator_manager_;                          /**< Managers for the kinematics objects */
  EnvState::Ptr current_state_;                                          /**< Current state of the environment */
  std::chrono::high_resolution_clock::duration current_state_timestamp_; /**< Current state timestamp */
  StateSolver::Ptr state_solver_;                                        /**< Tesseract State Solver */
  std::vector<std::string> link_names_;                                  /**< A vector of link names */
  std::vector<std::string> joint_names_;                                 /**< A vector of joint names */
  std::vector<std::string> active_link_names_;                           /**< A vector of active link names */
  std::vector<std::string> active_joint_names_;                          /**< A vector of active joint names */
  tesseract_collision::IsContactAllowedFn is_contact_allowed_fn_; /**< The function used to determine if two objects are
                                                                     allowed in collision */

  /** @brief A vector of user defined callbacks for locating tool center point */
  std::vector<FindTCPCallbackFn> find_tcp_cb_;

  /** @brief This indicates that the default collision checker 'Bullet' should be registered */
  bool register_default_contact_managers_{ true };

  /** @brief Used when initialized by URDF_STRING, URDF_STRING_SRDF_STRING, URDF_PATH, and URDF_PATH_SRDF_PATH */
  tesseract_scene_graph::ResourceLocator::Ptr resource_locator_;
  tesseract_collision::CollisionMarginData collision_margin_data_;        /**< The collision margin data */
  tesseract_collision::DiscreteContactManager::Ptr discrete_manager_;     /**< The discrete contact manager object */
  tesseract_collision::ContinuousContactManager::Ptr continuous_manager_; /**< The continuous contact manager object */
  std::string discrete_manager_name_;   /**< Name of active descrete contact manager */
  std::string continuous_manager_name_; /**< Name of active continuous contact manager */
  tesseract_collision::DiscreteContactManagerFactory discrete_factory_;     /**< Descrete contact manager factory */
  tesseract_collision::ContinuousContactManagerFactory continuous_factory_; /**< Continuous contact manager factory */

  /** @brief The environment can be accessed from multiple threads, need use mutex throughout */
  mutable std::shared_mutex mutex_;

  /** This will update the contact managers transforms */
  void currentStateChanged();

  /** This will notify the state solver that the environment has changed */
  void environmentChanged();

private:
  bool removeLinkHelper(const std::string& name);

  void getCollisionObject(tesseract_collision::CollisionShapesConst& shapes,
                          tesseract_common::VectorIsometry3d& shape_poses,
                          const tesseract_scene_graph::Link& link) const;

  bool registerDefaultContactManagersHelper();
  bool setActiveDiscreteContactManagerHelper(const std::string& name);
  bool setActiveContinuousContactManagerHelper(const std::string& name);

  tesseract_collision::DiscreteContactManager::Ptr getDiscreteContactManagerHelper(const std::string& name) const;

  tesseract_collision::ContinuousContactManager::Ptr getContinuousContactManagerHelper(const std::string& name) const;

  bool initHelper(const Commands& commands);
  Commands getInitCommands(const tesseract_scene_graph::SceneGraph& scene_graph,
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
};
}  // namespace tesseract_environment

#ifdef SWIG
namespace tesseract_environment
{
  class KDLStateSolver;
}

%template(init) tesseract_environment::Environment::init<tesseract_environment::KDLStateSolver>;

#endif  // SWIG

#endif  // TESSERACT_ENVIRONMENT_ENVIRONMENT_H
