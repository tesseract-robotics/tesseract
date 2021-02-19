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

#include <tesseract_common/utils.h>
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
                            const tesseract_kinematics::InverseKinematics::ConstPtr& inv_kin,
                            double tol = 1e-3)
{
  // Check name
  if (fwd_kin->getName() != inv_kin->getName())
  {
    CONSOLE_BRIDGE_logError("checkKinematics: Manipulator names do not match, '%s' and '%s'!",
                            fwd_kin->getName().c_str(),
                            inv_kin->getName().c_str());
    return false;
  }

  // Check if number of joints
  if (fwd_kin->numJoints() != inv_kin->numJoints())
  {
    CONSOLE_BRIDGE_logError("checkKinematics: Manipulator number of joints do not match, '%f' and '%f'!",
                            fwd_kin->numJoints(),
                            inv_kin->numJoints());
    return false;
  }

  // Check link names
  if (fwd_kin->getLinkNames().size() != inv_kin->getLinkNames().size())
  {
    CONSOLE_BRIDGE_logWarn("checkKinematics: Manipulator link names size do not match!");
  }

  if (!tesseract_common::isIdentical(fwd_kin->getLinkNames(), inv_kin->getLinkNames(), false))
  {
    CONSOLE_BRIDGE_logWarn("checkKinematics: Manipulator link names do not match!");
  }

  // Check active link names
  if (fwd_kin->getActiveLinkNames().size() != inv_kin->getActiveLinkNames().size())
  {
    CONSOLE_BRIDGE_logWarn("checkKinematics: Manipulator active link names size do not match!");
  }

  if (!tesseract_common::isIdentical(fwd_kin->getActiveLinkNames(), inv_kin->getActiveLinkNames(), false))
  {
    CONSOLE_BRIDGE_logWarn("checkKinematics: Manipulator active link names do not match!");
  }

  // Check joint names
  if (fwd_kin->getJointNames().size() != inv_kin->getJointNames().size())
  {
    CONSOLE_BRIDGE_logError("checkKinematics: Manipulator joint names size do not match!");
    return false;
  }

  if (!tesseract_common::isIdentical(fwd_kin->getJointNames(), inv_kin->getJointNames()))
  {
    CONSOLE_BRIDGE_logError("checkKinematics: Manipulator joint names do not match!");
    return false;
  }

  // Check joint limits
  if (!fwd_kin->getLimits().joint_limits.isApprox(inv_kin->getLimits().joint_limits, tol))
  {
    CONSOLE_BRIDGE_logWarn("checkKinematics: Manipulator joint limits do not match within tolerance %f!", tol);
  }

  // Check velocity limits
  if (!fwd_kin->getLimits().velocity_limits.isApprox(inv_kin->getLimits().velocity_limits, tol))
  {
    CONSOLE_BRIDGE_logWarn("checkKinematics: Manipulator velocity limits do not match within tolerance %f!", tol);
  }

  // Check acceleration limits
  if (!fwd_kin->getLimits().acceleration_limits.isApprox(inv_kin->getLimits().acceleration_limits, tol))
  {
    CONSOLE_BRIDGE_logWarn("checkKinematics: Manipulator acceleration limits do not match within tolerance %f!", tol);
  }

  Eigen::Isometry3d test1;
  Eigen::Isometry3d test2;
  Eigen::VectorXd seed_angles(fwd_kin->numJoints());
  Eigen::VectorXd joint_angles2(fwd_kin->numJoints());
  seed_angles.setZero();
  joint_angles2.setZero();

  const int nj = static_cast<int>(fwd_kin->numJoints());
  for (int t = 0; t < nj; ++t)
  {
    joint_angles2[t] = M_PI / 2;

    test1 = fwd_kin->calcFwdKin(joint_angles2);
    IKSolutions sols = inv_kin->calcInvKin(test1, seed_angles);
    for (const auto& sol : sols)
    {
      test2 = fwd_kin->calcFwdKin(sol);

      if ((test1.translation() - test2.translation()).norm() > tol)
      {
        CONSOLE_BRIDGE_logError("checkKinematics: Manipulator translation norm is greater than tolerance %f!", tol);
        return false;
      }

      if (Eigen::Quaterniond(test1.linear()).angularDistance(Eigen::Quaterniond(test2.linear())) > tol)
      {
        CONSOLE_BRIDGE_logError("checkKinematics: Manipulator orientation angular distance is greater than tolerance "
                                "%f!",
                                tol);
        return false;
      }
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
                            const tesseract_kinematics::ForwardKinematics::ConstPtr& fwd_kin2,
                            double tol = 1e-5)
{
  // Check name
  if (fwd_kin1->getName() != fwd_kin2->getName())
  {
    CONSOLE_BRIDGE_logError("checkKinematics: Manipulator names do not match, '%s' and '%s'!",
                            fwd_kin1->getName().c_str(),
                            fwd_kin2->getName().c_str());
    return false;
  }

  // Check if number of joints
  if (fwd_kin1->numJoints() != fwd_kin2->numJoints())
  {
    CONSOLE_BRIDGE_logError("checkKinematics: Manipulator number of joints do not match, '%f' and '%f'!",
                            fwd_kin1->numJoints(),
                            fwd_kin2->numJoints());
    return false;
  }

  // Check link names
  if (fwd_kin1->getLinkNames().size() != fwd_kin2->getLinkNames().size())
  {
    CONSOLE_BRIDGE_logWarn("checkKinematics: Manipulator link names size do not match!");
  }

  if (!tesseract_common::isIdentical(fwd_kin1->getLinkNames(), fwd_kin2->getLinkNames(), false))
  {
    CONSOLE_BRIDGE_logWarn("checkKinematics: Manipulator link names do not match!");
  }

  // Check active link names
  if (fwd_kin1->getActiveLinkNames().size() != fwd_kin2->getActiveLinkNames().size())
  {
    CONSOLE_BRIDGE_logWarn("checkKinematics: Manipulator active link names size do not match!");
  }

  if (!tesseract_common::isIdentical(fwd_kin1->getActiveLinkNames(), fwd_kin2->getActiveLinkNames(), false))
  {
    CONSOLE_BRIDGE_logWarn("checkKinematics: Manipulator active link names do not match!");
  }

  // Check joint names
  if (fwd_kin1->getJointNames().size() != fwd_kin2->getJointNames().size())
  {
    CONSOLE_BRIDGE_logError("checkKinematics: Manipulator joint names size do not match!");
    return false;
  }

  if (!std::equal(
          fwd_kin1->getJointNames().begin(), fwd_kin1->getJointNames().end(), fwd_kin2->getJointNames().begin()))
  {
    CONSOLE_BRIDGE_logError("checkKinematics: Manipulator joint names do not match!");
    return false;
  }

  // Check joint limits
  if (!fwd_kin1->getLimits().joint_limits.isApprox(fwd_kin2->getLimits().joint_limits, tol))
  {
    CONSOLE_BRIDGE_logWarn("checkKinematics: Manipulator joint limits do not match within tolerance %f!", tol);
  }

  // Check velocity limits
  if (!fwd_kin1->getLimits().velocity_limits.isApprox(fwd_kin2->getLimits().velocity_limits, tol))
  {
    CONSOLE_BRIDGE_logWarn("checkKinematics: Manipulator velocity limits do not match within tolerance %f!", tol);
  }

  // Check acceleration limits
  if (!fwd_kin1->getLimits().acceleration_limits.isApprox(fwd_kin2->getLimits().acceleration_limits, tol))
  {
    CONSOLE_BRIDGE_logWarn("checkKinematics: Manipulator acceleration limits do not match within tolerance %f!", tol);
  }

  Eigen::Isometry3d test1;
  Eigen::Isometry3d test2;
  Eigen::VectorXd joint_angles2(fwd_kin1->numJoints());
  joint_angles2.setZero();

  for (int t = 0; t < static_cast<int>(fwd_kin1->numJoints()); ++t)
  {
    joint_angles2[t] = M_PI / 2;

    test1 = fwd_kin1->calcFwdKin(joint_angles2);
    test2 = fwd_kin2->calcFwdKin(joint_angles2);

    if ((test1.translation() - test2.translation()).norm() > tol)
    {
      CONSOLE_BRIDGE_logError("checkKinematics: Manipulator translation norm is greater than tolerance %f!", tol);
      return false;
    }

    if (Eigen::Quaterniond(test1.linear()).angularDistance(Eigen::Quaterniond(test2.linear())) > tol)
    {
      CONSOLE_BRIDGE_logError("checkKinematics: Manipulator orientation angular distance is greater than tolerance %f!",
                              tol);
      return false;
    }

    joint_angles2[t] = 0;
  }

  return true;
}

}  // namespace tesseract_kinematics
#endif  // TESSERACT_KINEMATICS_VALIDATE_H
