/*
 * Software License Agreement (Apache License)
 *
 * Copyright (c) 2016, Southwest Research Institute
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include <tesseract_common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include "iiwa7_ikfast_kinematics.h"
TESSERACT_COMMON_IGNORE_WARNINGS_POP

namespace tesseract_kinematics
{
iiwa7Kinematics::iiwa7Kinematics(std::string base_link_name,
                                 std::string tip_link_name,
                                 std::vector<std::string> joint_names,
                                 std::string solver_name,
                                 std::vector<std::vector<double> > free_joint_combos)
  : IKFastInvKin(base_link_name, tip_link_name, joint_names, solver_name, free_joint_combos)
{
}

}  // namespace tesseract_kinematics
