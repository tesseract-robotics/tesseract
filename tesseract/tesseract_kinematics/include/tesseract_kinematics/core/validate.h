/**
 * @file validate.h
 * @brief This contains utility function validate things like forward kinematics match inverse kinematics
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
#ifndef TESSERACT_KINEMATICS_VALIDATE_H
#define TESSERACT_KINEMATICS_VALIDATE_H
#include <tesseract_common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <Eigen/Geometry>
#include <console_bridge/console.h>
#include <algorithm>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract_kinematics/core/forward_kinematics.h>
#include <tesseract_kinematics/core/inverse_kinematics.h>

namespace tesseract_kinematics
{
/**
 * @brief This compares a forward kinematics object to an inverse kinematics object and check that they are the same.
 *
 * It checks that the following are the same:
 *   - Manipulator Name
 *   - Number of joints
 *   - Link names
 *   - Active link names
 *   - Joint names
 *   - Joint limits
 *   - The forward kinematic solution matches the inverse kinematic solutions
 * @param fwd_kin The forward kinematics object for a manipulator
 * @param inv_kin The inverse kinematics object for a manipulator
 * @return True it they match, otherwise false.
 */
inline bool checkKinematics(const tesseract_kinematics::ForwardKinematics::ConstPtr& fwd_kin,
                            const tesseract_kinematics::InverseKinematics::ConstPtr& inv_kin)
{
  double tol = 1e-5;

  // Check name
  if (fwd_kin->getName() != inv_kin->getName())
    return false;

  // Check if number of joints
  if (fwd_kin->numJoints() != inv_kin->numJoints())
    return false;

  // Check link names
  if (fwd_kin->getLinkNames().size() != inv_kin->getLinkNames().size() ||
      !std::equal(fwd_kin->getLinkNames().begin(), fwd_kin->getLinkNames().end(), inv_kin->getLinkNames().begin()))
    return false;

  // Check active link names
  if (fwd_kin->getActiveLinkNames().size() != inv_kin->getActiveLinkNames().size() ||
      !std::equal(fwd_kin->getActiveLinkNames().begin(),
                  fwd_kin->getActiveLinkNames().end(),
                  inv_kin->getActiveLinkNames().begin()))
    return false;

  // Check joint names
  if (fwd_kin->getJointNames().size() != inv_kin->getJointNames().size() ||
      !std::equal(fwd_kin->getJointNames().begin(), fwd_kin->getJointNames().end(), inv_kin->getJointNames().begin()))
    return false;

  // Check joint limits
  if (!fwd_kin->getLimits().isApprox(inv_kin->getLimits(), tol))
    return false;

  Eigen::Isometry3d test1;
  Eigen::Isometry3d test2;
  Eigen::VectorXd seed_angles(fwd_kin->numJoints());
  Eigen::VectorXd joint_angles2(fwd_kin->numJoints());
  seed_angles.setZero();
  joint_angles2.setZero();

  for (int t = 0; t < fwd_kin->numJoints(); ++t)
  {
    joint_angles2[t] = M_PI / 2;

    fwd_kin->calcFwdKin(test1, joint_angles2);
    Eigen::VectorXd sols;
    inv_kin->calcInvKin(sols, test1, seed_angles);
    int num_sols = sols.size() / fwd_kin->numJoints();
    for (int i = 0; i < num_sols; ++i)
    {
      fwd_kin->calcFwdKin(test2, sols.middleRows(fwd_kin->numJoints() * i, fwd_kin->numJoints()));
      if (!test1.isApprox(test2, tol))
        return false;
    }

    joint_angles2[t] = 0;
  }

  return true;
}

/**
 * @brief This compares a forward kinematics object to a forward kinematics object and check that they are the same.
 *
 * It checks that the following are the same:
 *   - Manipulator Name
 *   - Number of joints
 *   - Link names
 *   - Active link names
 *   - Joint names
 *   - Joint limits
 *   - The forward kinematic solution matches the forward kinematic solutions
 * @param fwd_kin1 The forward kinematics object for a manipulator
 * @param fwd_kin2 The forward kinematics object for a manipulator
 * @return True it they match, otherwise false.
 */
inline bool checkKinematics(const tesseract_kinematics::ForwardKinematics::ConstPtr& fwd_kin1,
                            const tesseract_kinematics::ForwardKinematics::ConstPtr& fwd_kin2)
{
  double tol = 1e-5;

  // Check name
  if (fwd_kin1->getName() != fwd_kin2->getName())
    return false;

  // Check if number of joints
  if (fwd_kin1->numJoints() != fwd_kin2->numJoints())
    return false;

  // Check link names
  if (fwd_kin1->getLinkNames().size() != fwd_kin2->getLinkNames().size() ||
      !std::equal(fwd_kin1->getLinkNames().begin(), fwd_kin1->getLinkNames().end(), fwd_kin2->getLinkNames().begin()))
    return false;

  // Check active link names
  if (fwd_kin1->getActiveLinkNames().size() != fwd_kin2->getActiveLinkNames().size() ||
      !std::equal(fwd_kin1->getActiveLinkNames().begin(),
                  fwd_kin1->getActiveLinkNames().end(),
                  fwd_kin2->getActiveLinkNames().begin()))
    return false;

  // Check joint names
  if (fwd_kin1->getJointNames().size() != fwd_kin2->getJointNames().size() ||
      !std::equal(
          fwd_kin1->getJointNames().begin(), fwd_kin1->getJointNames().end(), fwd_kin2->getJointNames().begin()))
    return false;

  // Check joint limits
  if (!fwd_kin1->getLimits().isApprox(fwd_kin2->getLimits(), tol))
    return false;

  Eigen::Isometry3d test1;
  Eigen::Isometry3d test2;
  Eigen::VectorXd joint_angles2(fwd_kin1->numJoints());
  joint_angles2.setZero();

  for (int t = 0; t < fwd_kin1->numJoints(); ++t)
  {
    joint_angles2[t] = M_PI / 2;

    fwd_kin1->calcFwdKin(test1, joint_angles2);
    fwd_kin2->calcFwdKin(test2, joint_angles2);
    if (!test1.isApprox(test2, tol))
      return false;

    joint_angles2[t] = 0;
  }

  return true;
}

}  // namespace tesseract_kinematics
#endif  // TESSERACT_KINEMATICS_VALIDATE_H
