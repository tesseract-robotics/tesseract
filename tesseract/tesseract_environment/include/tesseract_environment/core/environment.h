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
#include <mutex>
#include <console_bridge/console.h>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract_environment/core/types.h>
#include <tesseract_environment/core/commands.h>
#include <tesseract_collision/core/discrete_contact_manager.h>
#include <tesseract_collision/core/discrete_contact_manager_factory.h>
#include <tesseract_collision/core/continuous_contact_manager.h>
#include <tesseract_collision/core/continuous_contact_manager_factory.h>
#include <tesseract_scene_graph/graph.h>
#include <tesseract_environment/core/state_solver.h>

namespace tesseract_environment
{
class Environment
{
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  using Ptr = std::shared_ptr<Environment>;
  using ConstPtr = std::shared_ptr<const Environment>;

  Environment() = default;
  virtual ~Environment() = default;
  Environment(const Environment&) = delete;
  Environment& operator=(const Environment&) = delete;
  Environment(Environment&&) = delete;
  Environment& operator=(Environment&&) = delete;

  /**
   * @brief Initialize the Environment
   *
   * Inherited class must call the protected create function
   *
   * @param scene_graph
   * @return
   */
  virtual bool init(tesseract_scene_graph::SceneGraph::Ptr scene_graph) = 0;

  /**
   * @brief Get the current revision number
   * @return Revision number
   */
  int getRevision() const { return revision_; }

  /**
   * @brief Get Environment command history post initialization
   * @return List of commands
   */
  const Commands& getCommandHistory() const { return commands_; }

  /**
   * @brief Check if environment has been initialized
   * @return True if initialized otherwise false
   */
  virtual bool checkInitialized() const { return initialized_; }

  /**
   * @brief Get the Scene Graph
   * @return SceneGraphConstPtr
   */
  virtual const tesseract_scene_graph::SceneGraph::ConstPtr& getSceneGraph() const { return scene_graph_const_; }

  /** @brief Give the environment a name */
  virtual void setName(const std::string& name) { scene_graph_->setName(name); }

  /** @brief Get the name of the environment
   *
   * This may be empty, if so check urdf name
   */
  virtual const std::string& getName() const { return scene_graph_->getName(); }

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
  virtual EnvState::ConstPtr getCurrentState() const { return current_state_; }

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
   * @brief Removes a link from the environment
   *
   *        Parent joint and all child components (links/joints) should be removed
   *
   * @param name Name of the link to be removed
   * @return Return False if a link does not exists, otherwise true
   */
  virtual bool removeLink(const std::string& name);

  /**
   * @brief Move a link in the environment
   *
   *        This should delete the parent joint of the child link. All child links and joints follow.
   *
   * @param joint The new joint.
   * @return Return False if a link does not exists or has no parent joint, otherwise true
   */
  virtual bool moveLink(tesseract_scene_graph::Joint joint);

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
   * @brief Set whether a link should be considered during collision checking
   * @param enabled True if should be condisdered during collision checking, otherwise false
   */
  virtual void setLinkCollisionEnabled(const std::string& name, bool enabled);

  /**
   * @brief Get whether a link should be considered during collision checking
   * @return True if should be condisdered during collision checking, otherwise false
   */
  virtual bool getLinkCollisionEnabled(const std::string& name) const;

  /**
   * @brief Set a links visibility
   * @param name The name of the link
   * @param visibility True if should be visible, otherwise false
   */
  virtual void setLinkVisibility(const std::string& name, bool visibility);

  /**
   * @brief Get a given links visibility setting
   * @return True if should be visible, otherwise false
   */
  virtual bool getLinkVisibility(const std::string& name) const;

  /**
   * @brief Disable collision between two collision objects
   * @param link_name1 Collision object name
   * @param link_name2 Collision object name
   * @param reason The reason for disabling collison
   */
  virtual void addAllowedCollision(const std::string& link_name1,
                                   const std::string& link_name2,
                                   const std::string& reason);

  /**
   * @brief Remove disabled collision pair from allowed collision matrix
   * @param link_name1 Collision object name
   * @param link_name2 Collision object name
   */
  virtual void removeAllowedCollision(const std::string& link_name1, const std::string& link_name2);

  /**
   * @brief Remove disabled collision for any pair with link_name from allowed collision matrix
   * @param link_name Collision object name
   */
  virtual void removeAllowedCollision(const std::string& link_name);

  /**
   * @brief Get the allowed collision matrix
   * @return AllowedCollisionMatrixConstPtr
   */
  virtual tesseract_scene_graph::AllowedCollisionMatrix::ConstPtr getAllowedCollisionMatrix() const;

  /**
   * @brief Get a vector of joint names in the environment
   * @return A vector of joint names
   */
  virtual std::vector<std::string> getJointNames() const { return joint_names_; }

  /**
   * @brief Get a vector of active joint names in the environment
   * @return A vector of active joint names
   */
  virtual std::vector<std::string> getActiveJointNames() const { return active_joint_names_; }

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
  virtual const std::string& getRootLinkName() const { return scene_graph_->getRoot(); }

  /**
   * @brief Get a vector of link names in the environment
   * @return A vector of link names
   */
  virtual std::vector<std::string> getLinkNames() const { return link_names_; }

  /**
   * @brief Get a vector of active link names in the environment
   * @return A vector of active link names
   */
  virtual std::vector<std::string> getActiveLinkNames() const { return active_link_names_; }

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
  virtual const Eigen::Isometry3d& getLinkTransform(const std::string& link_name) const;

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
  virtual tesseract_collision::DiscreteContactManager::Ptr getDiscreteContactManager() const
  {
    return discrete_manager_->clone();
  }

  /** @brief Get a copy of the environments available discrete contact manager by name */
  virtual tesseract_collision::DiscreteContactManager::Ptr getDiscreteContactManager(const std::string& name) const;

  /**
   * @brief Set the active continuous contact manager
   * @param name The name used to registar the contact manager
   * @return True of name exists in ContinuousContactManagerFactory
   */
  virtual bool setActiveContinuousContactManager(const std::string& name);

  /** @brief Get a copy of the environments active continuous contact manager */
  virtual tesseract_collision::ContinuousContactManager::Ptr getContinuousContactManager() const
  {
    return continuous_manager_->clone();
  }

  /** @brief Get a copy of the environments available continuous contact manager by name */
  virtual tesseract_collision::ContinuousContactManager::Ptr getContinuousContactManager(const std::string& name) const;

  /**
   * @brief Set the discrete contact manager
   *
   * This method should clear the contents of the manager and reload it with the objects
   * in the environment.
   */
  bool registerDiscreteContactManager(const std::string& name,
                                      tesseract_collision::DiscreteContactManagerFactory::CreateMethod create_function)
  {
    return discrete_factory_.registar(name, std::move(create_function));
  }

  /**
   * @brief Set the discrete contact manager
   *
   * This method should clear the contents of the manager and reload it with the objects
   * in the environment.
   */
  bool
  registerContinuousContactManager(const std::string& name,
                                   tesseract_collision::ContinuousContactManagerFactory::CreateMethod create_function)
  {
    return continuous_factory_.registar(name, std::move(create_function));
  }

protected:
  bool initialized_{ false }; /**< Identifies if the object has been initialized */
  int revision_{ 0 };         /**< This increments when the scene graph is modified */
  Commands commands_;         /**< The history of commands applied to the environment after intialization */
  tesseract_scene_graph::SceneGraph::Ptr scene_graph_;            /**< Tesseract Scene Graph */
  tesseract_scene_graph::SceneGraph::ConstPtr scene_graph_const_; /**< Tesseract Scene Graph Const */
  EnvState::Ptr current_state_;                                   /**< Current state of the environment */
  StateSolver::Ptr state_solver_;                                 /**< Tesseract State Solver */
  std::vector<std::string> link_names_;                           /**< A vector of link names */
  std::vector<std::string> joint_names_;                          /**< A vector of joint names */
  std::vector<std::string> active_link_names_;                    /**< A vector of active link names */
  std::vector<std::string> active_joint_names_;                   /**< A vector of active joint names */
  tesseract_collision::IsContactAllowedFn is_contact_allowed_fn_; /**< The function used to determine if two objects are
                                                                     allowed in collision */
  tesseract_collision::DiscreteContactManager::Ptr discrete_manager_;     /**< The discrete contact manager object */
  tesseract_collision::ContinuousContactManager::Ptr continuous_manager_; /**< The continuous contact manager object */
  std::string discrete_manager_name_;   /**< Name of active descrete contact manager */
  std::string continuous_manager_name_; /**< Name of active continuous contact manager */
  tesseract_collision::DiscreteContactManagerFactory discrete_factory_;     /**< Descrete contact manager factory */
  tesseract_collision::ContinuousContactManagerFactory continuous_factory_; /**< Continuous contact manager factory */
  mutable std::mutex mutex_; /**< The environment can be accessed from multiple threads, need use mutex throughout */

  /** This will update the contact managers transforms */
  void currentStateChanged();

  /** This will notify the state solver that the environment has changed */
  void environmentChanged();

  /**
   * @brief Initialize the Environment
   * @param scene_graph
   * @param state_solver
   * @return
   */
  template <typename S>
  bool create(tesseract_scene_graph::SceneGraph::Ptr scene_graph)
  {
    initialized_ = false;
    revision_ = 0;
    scene_graph_ = std::move(scene_graph);
    scene_graph_const_ = scene_graph_;
    commands_.clear();
    link_names_.clear();
    joint_names_.clear();
    active_link_names_.clear();
    active_joint_names_.clear();

    if (scene_graph_ == nullptr)
    {
      CONSOLE_BRIDGE_logError("Null pointer to Scene Graph");
      return false;
    }

    if (!scene_graph_->getLink(scene_graph_->getRoot()))
    {
      CONSOLE_BRIDGE_logError("The scene graph has an invalid root.");
      return false;
    }

    state_solver_ = std::make_shared<S>();
    if (!state_solver_->init(scene_graph_))
    {
      CONSOLE_BRIDGE_logError("The environment state solver failed to initialize");
      return false;
    }

    is_contact_allowed_fn_ = std::bind(&tesseract_scene_graph::SceneGraph::isCollisionAllowed,
                                       scene_graph_,
                                       std::placeholders::_1,
                                       std::placeholders::_2);

    initialized_ = true;

    environmentChanged();

    return initialized_;
  }

private:
  bool removeLinkHelper(const std::string& name);

  void getCollisionObject(tesseract_collision::CollisionShapesConst& shapes,
                          tesseract_common::VectorIsometry3d& shape_poses,
                          const tesseract_scene_graph::Link& link) const;

  tesseract_collision::DiscreteContactManager::Ptr getDiscreteContactManagerHelper(const std::string& name) const;

  tesseract_collision::ContinuousContactManager::Ptr getContinuousContactManagerHelper(const std::string& name) const;
};
}  // namespace tesseract_environment

#endif  // TESSERACT_ENVIRONMENT_ENVIRONMENT_H
