/**
 * @file inverse_kinematics_manager.h
 * @brief Inverse kinematics Manager.
 *
 * @author Levi Armstrong
 * @date April 15, 2018
 * @version TODO
 * @bug No known bugs
 *
 * @copyright Copyright (c) 2013, Southwest Research Institute
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
#ifndef TESSERACT_INVERSE_KINEMATICS_MANAGER_H
#define TESSERACT_INVERSE_KINEMATICS_MANAGER_H
#include <tesseract_kinematics/core/inverse_kinematics_factory.h>

namespace tesseract
{
class InverseKinematicsManager
{
public:
  using Ptr = std::shared_ptr<InverseKinematicsManager>;
  using ConstPtr = std::shared_ptr<const InverseKinematicsManager>;

  InverseKinematicsManager() = default;
  virtual ~InverseKinematicsManager() = default;
  InverseKinematicsManager(const InverseKinematicsManager&) = default;
  InverseKinematicsManager& operator=(const InverseKinematicsManager&) = default;
  InverseKinematicsManager(InverseKinematicsManager&&) = default;
  InverseKinematicsManager& operator=(InverseKinematicsManager&&) = default;

  /**
   * @brief Register a inverse kinematics factory
   * @param factory The factory to register
   * @return False if factory already exists, otherwise true.
   */
  bool registerInvKinematicsFactory(tesseract_kinematics::InverseKinematicsFactory::Ptr factory)
  {
    std::string name = factory->getName();
    if (inv_kin_factories_.find(name) == inv_kin_factories_.end())
    {
      inv_kin_factories_[name] = std::move(factory);
      return true;
    }
    return false;
  }

  /**
   * @brief Removes a registered inverse kinematics factory
   * @param name The name of the factory to remove
   */
  void removeInvKinematicsFactory(const std::string& name) { inv_kin_factories_.erase(name); }

  /**
   * @brief Get a list of all available inverse kinematics solvers
   * @return Vector of names
   */
  std::vector<std::string> getAvailableInvKinematicsSolvers() const
  {
    std::vector<std::string> names;
    names.reserve(inv_kin_factories_.size());
    for (const auto& factory : inv_kin_factories_)
      names.push_back(factory.first);

    return names;
  }

  /**
   * @brief Get a list of inverse kinematics solver for a specific type {CHAIN, TREE, GRAPH}
   * @param type The type of solver {CHAIN, TREE, GRAPH}
   * @return Vector of names
   */
  std::vector<std::string>
  getAvailableInvKinematicsSolvers(tesseract_kinematics::InverseKinematicsFactoryType type) const
  {
    std::vector<std::string> names;
    names.reserve(inv_kin_factories_.size());
    for (const auto& factory : inv_kin_factories_)
      if (factory.second->getType() == type)
        names.push_back(factory.first);

    return names;
  }

  /**
   * @brief This will return the inverse kinematics solver factory
   * @param name The name of the solver
   * @return If not found it returns a nullptr, otherwise a new instance of the solver.
   */
  tesseract_kinematics::InverseKinematicsFactory::ConstPtr getInvKinematicFactory(const std::string& name) const
  {
    auto it = inv_kin_factories_.find(name);
    if (it != inv_kin_factories_.end())
      return it->second;

    return nullptr;
  }

  /**
   * @brief Add a manipulator inverse kinematics solver
   * @param manipulator The manipulator name
   * @param solver The solver
   * @return
   */
  bool addInvKinematicSolver(const tesseract_kinematics::InverseKinematics::ConstPtr& solver)
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

  /**
   * @brief Remove a inverse kinematic solver for a given manipulator
   * @param manipulator The name of the manipulator
   * @param name The name of the solver
   */
  void removeFwdKinematicSolver(const std::string& manipulator, const std::string& name)
  {
    inv_kin_manipulators_.erase(std::make_pair(manipulator, name));
  }

  /**
   * @brief Get a list of all available forward kinematics manipulators
   * @return Vector of names
   */
  std::vector<std::string> getAvailableInvKinematicsManipulators() const
  {
    std::vector<std::string> names;
    names.reserve(inv_kin_manipulators_default_.size());
    for (const auto& manip : inv_kin_manipulators_default_)
      names.push_back(manip.first);

    return names;
  }

  /**
   * @brief Set default inverse kinematic solver for manipulator
   * @param manipulator The name of the manipulator
   * @param name The name of the solver
   * @return True if manipulator solver pair exist, otherwise false
   */
  bool setDefaultInvKinematicSolver(const std::string& manipulator, const std::string& name)
  {
    auto it = inv_kin_manipulators_.find(std::make_pair(manipulator, name));
    if (it == inv_kin_manipulators_.end())
      return false;

    inv_kin_manipulators_default_[manipulator] = it->second;

    return true;
  }

  /**
   * @brief Get inverse kinematic solver for manipulator
   * @param manipulator The name of the manipulator
   * @param name The name of the solver
   * @return If not found returns a nullptr, otherwise a instance of the solver.
   */
  tesseract_kinematics::InverseKinematics::Ptr getInvKinematicSolver(const std::string& manipulator,
                                                                     const std::string& name) const
  {
    auto it = inv_kin_manipulators_.find(std::make_pair(manipulator, name));
    if (it != inv_kin_manipulators_.end())
      return it->second->clone();

    return nullptr;
  }

  /**
   * @brief Get default inverse kinematic solver for manipulator
   * @param manipulator The name of the manipulator
   * @return If not found returns a nullptr, otherwise a instance of the solver.
   */
  tesseract_kinematics::InverseKinematics::Ptr getInvKinematicSolver(const std::string& manipulator) const
  {
    auto it = inv_kin_manipulators_default_.find(manipulator);
    if (it != inv_kin_manipulators_default_.end())
      return it->second->clone();

    return nullptr;
  }

private:
  std::unordered_map<std::string, tesseract_kinematics::InverseKinematicsFactory::ConstPtr> inv_kin_factories_;
  std::map<std::pair<std::string, std::string>, tesseract_kinematics::InverseKinematics::ConstPtr>
      inv_kin_manipulators_;
  std::unordered_map<std::string, tesseract_kinematics::InverseKinematics::ConstPtr> inv_kin_manipulators_default_;
};
}  // namespace tesseract
#endif  // TESSERACT_INVERSE_KINEMATICS_MANAGER_H
