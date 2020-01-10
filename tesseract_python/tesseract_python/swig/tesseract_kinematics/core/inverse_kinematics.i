/**
 * @file inverse_kinematics.i
 * @brief SWIG interface file for tesseract_kinematics/core/inverse_kinematics.h
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
#include <tesseract_kinematics/core/inverse_kinematics.h>
%}

%shared_ptr(tesseract_kinematics::InverseKinematics)

namespace tesseract_kinematics
{
/** @brief Inverse kinematics functions. */
class InverseKinematics
{
public:
  
  using Ptr = std::shared_ptr<InverseKinematics>;
  using ConstPtr = std::shared_ptr<const InverseKinematics>;

  virtual ~InverseKinematics() = default;

%extend {

  //virtual bool calcInvKin(Eigen::VectorXd& solutions,
  //                        const Eigen::Isometry3d& pose,
  //                        const Eigen::Ref<const Eigen::VectorXd>& seed) const = 0;

  Eigen::VectorXd calcInvKin(const Eigen::Isometry3d& pose, const Eigen::Ref<const Eigen::VectorXd>& seed) const
  {
    Eigen::VectorXd solutions;
    if (!$self->calcInvKin(solutions, pose, seed))
    {
      throw std::runtime_error("calcInvKin failed");
    }
    return solutions;
  }

  //virtual bool calcInvKin(Eigen::VectorXd& solutions,
  //                        const Eigen::Isometry3d& pose,
  //                        const Eigen::Ref<const Eigen::VectorXd>& seed,
  //                        const std::string& link_name) const = 0;
  Eigen::VectorXd calcInvKin(const Eigen::Isometry3d& pose, const Eigen::Ref<const Eigen::VectorXd>& seed, const std::string& link_name) const
  {
    Eigen::VectorXd solutions;
    if (!$self->calcInvKin(solutions, pose, seed, link_name))
    {
      throw std::runtime_error("calcInvKin failed");
    }
    return solutions;
  }
}

  virtual bool checkJoints(const Eigen::Ref<const Eigen::VectorXd>& vec) const = 0;

  virtual const std::vector<std::string> getJointNames() const = 0;

  virtual const std::vector<std::string> getLinkNames() const = 0;

  virtual const std::vector<std::string> getActiveLinkNames() const = 0;

  virtual const Eigen::MatrixX2d getLimits() const = 0;

  virtual unsigned int numJoints() const = 0;

  virtual const std::string getBaseLinkName() const = 0;

  virtual const std::string getTipLinkName() const = 0;

  virtual const std::string getName() const = 0;

  virtual const std::string getSolverName() const = 0;

  virtual std::shared_ptr<InverseKinematics> clone() const = 0;
};

}  // namespace tesseract_kinematics