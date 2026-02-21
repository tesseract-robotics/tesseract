/**
 * @file validate.cpp
 * @brief This contains utility function validate things like forward kinematics match inverse kinematics
 *
 * @author Levi Armstrong
 * @date April 15, 2018
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

#include <tesseract_common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <Eigen/Geometry>
#include <console_bridge/console.h>
#include <sstream>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract_kinematics/core/validate.h>
#include <tesseract_common/utils.h>
#include <tesseract_kinematics/core/kinematic_group.h>

namespace tesseract::kinematics
{
bool checkKinematics(const KinematicGroup& manip, double tol)
{
  Eigen::Isometry3d test1;
  Eigen::Isometry3d test2;
  Eigen::VectorXd seed_angles(manip.numJoints());
  Eigen::VectorXd joint_angles2(manip.numJoints());
  std::vector<std::string> tip_links = manip.getAllPossibleTipLinkNames();
  std::vector<std::string> working_frames = manip.getAllValidWorkingFrames();
  const int nj = static_cast<int>(manip.numJoints());

  std::vector<std::vector<double>> passed_data;
  std::vector<std::vector<double>> failed_data;
  int translation_failures{ 0 };
  int angular_failures{ 0 };
  double translation_max{ 0 };
  double angular_max{ 0 };

  std::vector<std::pair<std::string, std::string>> checks;
  checks.reserve(tip_links.size() + working_frames.size());

  for (const auto& tip_link : tip_links)
    checks.emplace_back(working_frames[0], tip_link);

  for (const auto& working_frame : working_frames)
    checks.emplace_back(working_frame, tip_links[0]);

  for (const auto& check : checks)
  {
    seed_angles.setZero();
    joint_angles2.setZero();

    for (int t = 0; t < nj; ++t)
    {
      joint_angles2[t] = M_PI_4;

      auto poses1 = manip.calcFwdKin(joint_angles2);
      test1 = poses1.at(check.first).inverse() * poses1.at(check.second);
      KinGroupIKInput ik_input(test1, check.first, check.second);
      IKSolutions sols = manip.calcInvKin({ ik_input }, seed_angles);
      for (const auto& sol : sols)
      {
        auto poses2 = manip.calcFwdKin(sol);
        test2 = poses2.at(check.first).inverse() * poses2.at(check.second);

        double translation_distance = (test1.translation() - test2.translation()).norm();
        double angular_distance =
            Eigen::Quaterniond(test1.linear()).angularDistance(Eigen::Quaterniond(test2.linear()));
        if (translation_distance > tol || angular_distance > tol)
        {
          // LCOV_EXCL_START
          if (translation_distance > tol)
            ++translation_failures;

          if (angular_distance > tol)
            ++angular_failures;

          if (angular_distance > angular_max)
            angular_max = angular_distance;

          if (translation_distance > translation_max)
            translation_max = translation_distance;

          std::vector<double> data{ translation_distance, tol, angular_distance, tol };
          for (Eigen::Index i = 0; i < sol.rows(); ++i)
            data.push_back(sol(i));

          failed_data.push_back(data);
          // LCOV_EXCL_STOP
        }
        else
        {
          std::vector<double> data{ translation_distance, tol, angular_distance, tol };
          for (Eigen::Index i = 0; i < sol.rows(); ++i)
            data.push_back(sol(i));

          passed_data.push_back(data);
        }
      }

      joint_angles2[t] = 0;
    }
  }

  // LCOV_EXCL_START
  if (!failed_data.empty())
  {
    CONSOLE_BRIDGE_logError("checkKinematics failed %d out of %d\n           Translation failures %d out of %d (max: "
                            "%f)\n           Angular failures %d out of %d (max: %f)",
                            failed_data.size(),
                            (failed_data.size() + passed_data.size()),
                            translation_failures,
                            failed_data.size(),
                            translation_max,
                            angular_failures,
                            failed_data.size(),
                            angular_max);
    std::stringstream msg;
    msg << "\n";
    msg << "*****************************\n";
    msg << "******** Failed Data ********\n";
    msg << "*****************************\n";

    std::string header = "Trans. Dist. (m), tol, Angle Dist. (rad), tol";
    for (const auto& jn : manip.getJointNames())
      header += ", " + jn;

    msg << header << "\n";
    for (const auto& d : failed_data)
    {
      for (const auto& val : d)
        msg << val << ", ";
      msg << "\n";
    }

    msg << "*****************************\n";
    msg << "******** Passed Data ********\n";
    msg << "*****************************\n";
    if (passed_data.empty())
    {
      msg << "No Data!"
          << "\n";
    }
    else
    {
      for (const auto& d : passed_data)
      {
        for (const auto& val : d)
          msg << val << ", ";
        msg << "\n";
      }
    }

    CONSOLE_BRIDGE_logError("%s", msg.str().c_str());
    return false;
  }
  // LCOV_EXCL_STOP

  return true;
}
}  // namespace tesseract::kinematics
