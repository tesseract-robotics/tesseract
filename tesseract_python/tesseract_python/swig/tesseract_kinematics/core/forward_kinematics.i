/**
 * @file forward_kinematics.i
 * @brief SWIG interface file for tesseract_kinematics/core/forward_kinematics.h
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
#include <tesseract_kinematics/core/forward_kinematics.h>
%}

%shared_ptr(tesseract_kinematics::ForwardKinematics)

namespace tesseract_kinematics
{
class ForwardKinematics
{
public:
  
  using Ptr = std::shared_ptr<ForwardKinematics>;
  using ConstPtr = std::shared_ptr<const ForwardKinematics>;

  virtual ~ForwardKinematics() = default;

%extend {

  //virtual bool calcFwdKin(Eigen::Isometry3d& pose, const Eigen::Ref<const Eigen::VectorXd>& joint_angles) const = 0;
  
  Eigen::Isometry3d calcFwdKin(const Eigen::Ref<const Eigen::VectorXd>& joint_angles) const
  {
      Eigen::Isometry3d pose;
      if (!$self->calcFwdKin(pose,joint_angles))
      {
          throw std::runtime_error("calcFwdKin failed");
      }
      return pose;
  }

  //virtual bool calcFwdKin(tesseract_common::VectorIsometry3d& poses,
  //                        const Eigen::Ref<const Eigen::VectorXd>& joint_angles) const = 0;

  tesseract_common::VectorIsometry3d calcFwdKin2(const Eigen::Ref<const Eigen::VectorXd>& joint_angles) const
  {
      tesseract_common::VectorIsometry3d poses;
      if (!$self->calcFwdKin(poses,joint_angles))
      {
          throw std::runtime_error("calcFwdKin2 failed");
      }
      return poses;
  }

  //virtual bool calcFwdKin(Eigen::Isometry3d& pose,
  //                        const Eigen::Ref<const Eigen::VectorXd>& joint_angles,
  //                        const std::string& link_name) const = 0;

  Eigen::Isometry3d calcFwdKin(const Eigen::Ref<const Eigen::VectorXd>& joint_angles, const std::string& link_name) const
  {
      Eigen::Isometry3d pose;
      if (!$self->calcFwdKin(pose,joint_angles, link_name))
      {
          throw std::runtime_error("calcFwdKin failed");
      }
      return pose;
  }

  //virtual bool calcJacobian(Eigen::Ref<Eigen::MatrixXd> jacobian,
  //                          const Eigen::Ref<const Eigen::VectorXd>& joint_angles) const = 0;

  Eigen::MatrixXd calcJacobian(const Eigen::Ref<const Eigen::VectorXd>& joint_angles) const
  {
    Eigen::MatrixXd jacobian(6, joint_angles.rows());
    if (!$self->calcJacobian(jacobian, joint_angles))
    {
        throw std::runtime_error("calcJacobian failed");
    }
    return jacobian;
  }

  //virtual bool calcJacobian(Eigen::Ref<Eigen::MatrixXd> jacobian,
  //                          const Eigen::Ref<const Eigen::VectorXd>& joint_angles,
  //                          const std::string& link_name) const = 0;
  Eigen::MatrixXd calcJacobian(const Eigen::Ref<const Eigen::VectorXd>& joint_angles, const std::string& link_name) const
  {
    Eigen::MatrixXd jacobian(6, joint_angles.rows());
    if (!$self->calcJacobian(jacobian, joint_angles, link_name))
    {
        throw std::runtime_error("calcJacobian failed");
    }
    return jacobian;
  }
}

  virtual bool checkJoints(const Eigen::Ref<const Eigen::VectorXd>& vec) const = 0;


  virtual const std::vector<std::string>& getJointNames() const = 0;

  virtual const std::vector<std::string>& getLinkNames() const = 0;

  virtual const std::vector<std::string>& getActiveLinkNames() const = 0;

  virtual const Eigen::MatrixX2d& getLimits() const = 0;

  virtual unsigned int numJoints() const = 0;

  virtual const std::string& getBaseLinkName() const = 0;

  virtual const std::string& getTipLinkName() const = 0;

  virtual const std::string& getName() const = 0;

  virtual const std::string& getSolverName() const = 0;

  virtual std::shared_ptr<ForwardKinematics> clone() const = 0;
};

}  // namespace tesseract_kinematics

