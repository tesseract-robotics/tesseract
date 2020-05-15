/**
 * @file inverse_kinematics_manager.i
 * @brief SWIG interface file for tesseract/inverse_kinematics_manager.h
 *
 * @author John Wason
 * @date December 10, 2019
 * @version TODO
 * @bug No known bugs
 *
 * @copyright Copyright (c) 2019, Wason Technology, LLC
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

%{
#include <tesseract/inverse_kinematics_manager.h>
%}

%shared_ptr(tesseract::InverseKinematicsManager)

namespace tesseract
{
class InverseKinematicsManager
{
public:
  using Ptr = std::shared_ptr<InverseKinematicsManager>;
  using ConstPtr = std::shared_ptr<const InverseKinematicsManager>;

  InverseKinematicsManager();
  virtual ~InverseKinematicsManager();

  bool registerInvKinematicsFactory(tesseract_kinematics::InverseKinematicsFactory::Ptr factory);

  void removeInvKinematicsFactory(const std::string& name);

  std::vector<std::string> getAvailableInvKinematicsSolvers() const;

  std::vector<std::string>
  getAvailableInvKinematicsSolvers(tesseract_kinematics::InverseKinematicsFactoryType type) const;

  tesseract_kinematics::InverseKinematicsFactory::ConstPtr getInvKinematicFactory(const std::string& name) const;

  bool addInvKinematicSolver(tesseract_kinematics::InverseKinematics::ConstPtr solver);

  void removeInvKinematicSolver(const std::string& manipulator, const std::string& name);

  void removeInvKinematicSolver(const std::string& manipulator);

  std::vector<std::string> getAvailableInvKinematicsManipulators() const;

  bool setDefaultInvKinematicSolver(const std::string& manipulator, const std::string& name);

  tesseract_kinematics::InverseKinematics::Ptr getInvKinematicSolver(const std::string& manipulator,
                                                                     const std::string& name) const;

  tesseract_kinematics::InverseKinematics::Ptr getInvKinematicSolver(const std::string& manipulator) const;

};
}  // namespace tesseract
