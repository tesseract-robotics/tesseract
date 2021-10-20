/**
 * @file types.h
 * @brief Kinematics types.
 *
 * @author Levi Armstrong
 * @date March 1, 2021
 * @version TODO
 * @bug No known bugs
 *
 * @copyright Copyright (c) 2021, Southwest Research Institute
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
#ifndef TESSERACT_KINEMATICS_TYPES_H
#define TESSERACT_KINEMATICS_TYPES_H

#include <tesseract_common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <string>
#include <vector>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract_common/types.h>
#include <tesseract_common/utils.h>

namespace tesseract_kinematics
{
/** @brief The inverse kinematics solutions container */
using IKSolutions = std::vector<Eigen::VectorXd>;

/** @brief The Universal Robot kinematic parameters */
struct URParameters
{
  URParameters() = default;
  URParameters(double d1, double a2, double a3, double d4, double d5, double d6)
    : d1(d1), a2(a2), a3(a3), d4(d4), d5(d5), d6(d6)
  {
  }

  double d1{ 0 };
  double a2{ 0 };
  double a3{ 0 };
  double d4{ 0 };
  double d5{ 0 };
  double d6{ 0 };
};

/** @brief The UR10 kinematic parameters */
const static URParameters UR10Parameters(0.1273, -0.612, -0.5723, 0.163941, 0.1157, 0.0922);

/** @brief The UR5 kinematic parameters */
const static URParameters UR5Parameters(0.089159, -0.42500, -0.39225, 0.10915, 0.09465, 0.0823);

/** @brief The UR3 kinematic parameters */
const static URParameters UR3Parameters(0.1519, -0.24365, -0.21325, 0.11235, 0.08535, 0.0819);

/** @brief The UR10e kinematic parameters */
const static URParameters UR10eParameters(0.181, -0.613, -0.571, 0.135 + 0.176 - 0.137, 0.120, 0.117);

/** @brief The UR5e kinematic parameters */
const static URParameters UR5eParameters(0.163, -0.425, -0.392, 0.127 + 0.138 - 0.131, 0.100, 0.100);

/** @brief The UR3e kinematic parameters */
const static URParameters UR3eParameters(0.152, -0.244, -0.213, 0.104 + 0.138 - 0.131, 0.085, 0.092);

}  // namespace tesseract_kinematics
#endif  // TESSERACT_KINEMATICS_TYPES_H
