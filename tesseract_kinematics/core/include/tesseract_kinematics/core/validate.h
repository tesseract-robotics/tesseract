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
#include <tesseract_kinematics/core/kinematic_group.h>

namespace tesseract_kinematics
{
/**
 * @brief This compares calcFwdKin to calcInvKin for a KinematicGroup.
 *
 * It checks that the following are the same:
 *   - The forward kinematic solution matches the inverse kinematic solutions
 * @param manip The kinematic group for a manipulator
 * @return True it they match, otherwise false.
 */
inline bool checkKinematics(const KinematicGroup& manip, double tol = 1e-3)
{
  Eigen::Isometry3d test1;
  Eigen::Isometry3d test2;
  Eigen::VectorXd seed_angles(manip.numJoints());
  Eigen::VectorXd joint_angles2(manip.numJoints());
  std::string tip_link = manip.getAllPossibleTipLinkNames().at(0);
  std::string working_frame = manip.getAllValidWorkingFrames().at(0);
  seed_angles.setZero();
  joint_angles2.setZero();

  const int nj = static_cast<int>(manip.numJoints());
  for (int t = 0; t < nj; ++t)
  {
    joint_angles2[t] = M_PI / 2;

    auto poses1 = manip.calcFwdKin(joint_angles2);
    test1 = poses1.at(working_frame).inverse() * poses1.at(tip_link);
    KinGroupIKInput ik_input(test1, working_frame, tip_link);
    IKSolutions sols = manip.calcInvKin({ ik_input }, seed_angles);
    for (const auto& sol : sols)
    {
      auto poses2 = manip.calcFwdKin(sol);
      test2 = poses2.at(working_frame).inverse() * poses2.at(tip_link);

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

}  // namespace tesseract_kinematics
#endif  // TESSERACT_KINEMATICS_VALIDATE_H
