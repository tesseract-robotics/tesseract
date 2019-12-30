/**
 * @file inverse_kinematics_manager.cpp
 * @brief This is a manager for all inverse kinematics objects
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

#include <tesseract/inverse_kinematics_manager.h>

namespace tesseract
{
bool InverseKinematicsManager::registerInvKinematicsFactory(tesseract_kinematics::InverseKinematicsFactory::Ptr factory)
{
  std::string name = factory->getName();
  if (inv_kin_factories_.find(name) == inv_kin_factories_.end())
  {
    inv_kin_factories_[name] = std::move(factory);
    return true;
  }
  return false;
}

void InverseKinematicsManager::removeInvKinematicsFactory(const std::string& name) { inv_kin_factories_.erase(name); }

std::vector<std::string> InverseKinematicsManager::getAvailableInvKinematicsSolvers() const
{
  std::vector<std::string> names;
  names.reserve(inv_kin_factories_.size());
  for (const auto& factory : inv_kin_factories_)
    names.push_back(factory.first);

  return names;
}

std::vector<std::string> InverseKinematicsManager::getAvailableInvKinematicsSolvers(
    tesseract_kinematics::InverseKinematicsFactoryType type) const
{
  std::vector<std::string> names;
  names.reserve(inv_kin_factories_.size());
  for (const auto& factory : inv_kin_factories_)
    if (factory.second->getType() == type)
      names.push_back(factory.first);

  return names;
}

tesseract_kinematics::InverseKinematicsFactory::ConstPtr
InverseKinematicsManager::getInvKinematicFactory(const std::string& name) const
{
  auto it = inv_kin_factories_.find(name);
  if (it != inv_kin_factories_.end())
    return it->second;

  return nullptr;
}

bool InverseKinematicsManager::addInvKinematicSolver(const tesseract_kinematics::InverseKinematics::ConstPtr& solver)
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

void InverseKinematicsManager::removeFwdKinematicSolver(const std::string& manipulator, const std::string& name)
{
  inv_kin_manipulators_.erase(std::make_pair(manipulator, name));
}

std::vector<std::string> InverseKinematicsManager::getAvailableInvKinematicsManipulators() const
{
  std::vector<std::string> names;
  names.reserve(inv_kin_manipulators_default_.size());
  for (const auto& manip : inv_kin_manipulators_default_)
    names.push_back(manip.first);

  return names;
}

bool InverseKinematicsManager::setDefaultInvKinematicSolver(const std::string& manipulator, const std::string& name)
{
  auto it = inv_kin_manipulators_.find(std::make_pair(manipulator, name));
  if (it == inv_kin_manipulators_.end())
    return false;

  inv_kin_manipulators_default_[manipulator] = it->second;

  return true;
}

tesseract_kinematics::InverseKinematics::Ptr
InverseKinematicsManager::getInvKinematicSolver(const std::string& manipulator, const std::string& name) const
{
  auto it = inv_kin_manipulators_.find(std::make_pair(manipulator, name));
  if (it != inv_kin_manipulators_.end())
    return it->second->clone();

  return nullptr;
}

tesseract_kinematics::InverseKinematics::Ptr
InverseKinematicsManager::getInvKinematicSolver(const std::string& manipulator) const
{
  auto it = inv_kin_manipulators_default_.find(manipulator);
  if (it != inv_kin_manipulators_default_.end())
    return it->second->clone();

  return nullptr;
}
}  // namespace tesseract
