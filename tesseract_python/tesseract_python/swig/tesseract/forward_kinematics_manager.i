/**
 * @file forward_kinematics_manager.i
 * @brief SWIG interface file for tesseract/forward_kinematics_manager.h
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
#include <tesseract/forward_kinematics_manager.h>
%}

%shared_ptr(tesseract::ForwardKinematicsManager)

namespace tesseract
{
class ForwardKinematicsManager
{
public:
  using Ptr = std::shared_ptr<ForwardKinematicsManager>;
  using ConstPtr = std::shared_ptr<const ForwardKinematicsManager>;

  ForwardKinematicsManager();
  virtual ~ForwardKinematicsManager();

  bool registerFwdKinematicsFactory(tesseract_kinematics::ForwardKinematicsFactory::Ptr factory);
  
  void removeFwdKinematicsFactory(const std::string& name);

  std::vector<std::string> getAvailableFwdKinematicsSolvers() const;
  
  std::vector<std::string>
  getAvailableFwdKinematicsSolvers(tesseract_kinematics::ForwardKinematicsFactoryType type) const;
  
  tesseract_kinematics::ForwardKinematicsFactory::ConstPtr getFwdKinematicFactory(const std::string& name) const;
  
  bool addFwdKinematicSolver(const tesseract_kinematics::ForwardKinematics::ConstPtr& solver);
  
  void removeFwdKinematicSolver(const std::string& manipulator, const std::string& name);
  
  void removeFwdKinematicSolver(const std::string& manipulator);

  std::vector<std::string> getAvailableFwdKinematicsManipulators() const;
  
  bool setDefaultFwdKinematicSolver(const std::string& manipulator, const std::string& name);
    
  tesseract_kinematics::ForwardKinematics::Ptr getFwdKinematicSolver(const std::string& manipulator,
                                                                     const std::string& name) const;
  
  tesseract_kinematics::ForwardKinematics::Ptr getFwdKinematicSolver(const std::string& manipulator) const;
  
};
}  // namespace tesseract
