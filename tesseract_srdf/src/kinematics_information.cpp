/**
 * @file kinematics_information.cpp
 * @brief This hold the kinematics information
 *
 * @author Levi Armstrong
 * @date May 12, 2020
 * @version TODO
 * @bug No known bugs
 *
 * @copyright Copyright (c) 2020, Southwest Research Institute
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

#include <tesseract_srdf/kinematics_information.h>

namespace tesseract_srdf
{
bool OPWKinematicParameters::operator==(const OPWKinematicParameters& rhs) const
{
  bool success = true;
  success &= (tesseract_common::almostEqualRelativeAndAbs(a1, rhs.a1, 1e-6));
  success &= (tesseract_common::almostEqualRelativeAndAbs(a2, rhs.a2, 1e-6));
  success &= (tesseract_common::almostEqualRelativeAndAbs(b, rhs.b, 1e-6));
  success &= (tesseract_common::almostEqualRelativeAndAbs(c1, rhs.c1, 1e-6));
  success &= (tesseract_common::almostEqualRelativeAndAbs(c2, rhs.c2, 1e-6));
  success &= (tesseract_common::almostEqualRelativeAndAbs(c3, rhs.c3, 1e-6));
  success &= (tesseract_common::almostEqualRelativeAndAbs(c4, rhs.c4, 1e-6));

  for (std::size_t i = 0; i < 6; i++)
  {
    success &= (tesseract_common::almostEqualRelativeAndAbs(offsets[i], rhs.offsets[i], 1e-6));
    success &= (sign_corrections[i] == rhs.sign_corrections[i]);
  }

  return success;
}

bool OPWKinematicParameters::operator!=(const OPWKinematicParameters& rhs) const { return !(*this == rhs); }

bool ROPKinematicParameters::operator==(const ROPKinematicParameters& rhs) const
{
  bool success = true;
  success &= (solver_name == rhs.solver_name);
  success &= (manipulator_group == rhs.manipulator_group);
  success &= (manipulator_ik_solver == rhs.manipulator_ik_solver);
  success &= (tesseract_common::almostEqualRelativeAndAbs(manipulator_reach, rhs.manipulator_reach, 1e-6));
  success &= (positioner_group == rhs.positioner_group);
  success &= (positioner_fk_solver == rhs.positioner_fk_solver);
  success &= (positioner_sample_resolution.size() == rhs.positioner_sample_resolution.size());
  for (const auto& joint_pair : positioner_sample_resolution)
  {
    auto it = rhs.positioner_sample_resolution.find(joint_pair.first);
    success &= (it != rhs.positioner_sample_resolution.end());
    if (!success)
      break;

    success &= (tesseract_common::almostEqualRelativeAndAbs(joint_pair.second, it->second, 1e-6));
    if (!success)
      break;
  }

  return success;
}

bool ROPKinematicParameters::operator!=(const ROPKinematicParameters& rhs) const { return !(*this == rhs); }

bool REPKinematicParameters::operator==(const REPKinematicParameters& rhs) const
{
  bool success = true;
  success &= (solver_name == rhs.solver_name);
  success &= (manipulator_group == rhs.manipulator_group);
  success &= (manipulator_ik_solver == rhs.manipulator_ik_solver);
  success &= (tesseract_common::almostEqualRelativeAndAbs(manipulator_reach, rhs.manipulator_reach, 1e-6));
  success &= (positioner_group == rhs.positioner_group);
  success &= (positioner_fk_solver == rhs.positioner_fk_solver);
  success &= (positioner_sample_resolution.size() == rhs.positioner_sample_resolution.size());
  for (const auto& joint_pair : positioner_sample_resolution)
  {
    auto it = rhs.positioner_sample_resolution.find(joint_pair.first);
    success &= (it != rhs.positioner_sample_resolution.end());
    if (!success)
      break;

    success &= (tesseract_common::almostEqualRelativeAndAbs(joint_pair.second, it->second, 1e-6));
    if (!success)
      break;
  }

  return success;
}

bool REPKinematicParameters::operator!=(const REPKinematicParameters& rhs) const { return !(*this == rhs); }

void KinematicsInformation::clear()
{
  group_names.clear();
  chain_groups.clear();
  joint_groups.clear();
  link_groups.clear();
  group_rop_kinematics.clear();
  group_rep_kinematics.clear();
  group_states.clear();
  group_tcps.clear();
  group_opw_kinematics.clear();
  group_default_fwd_kin.clear();
  group_default_inv_kin.clear();
}

}  // namespace tesseract_srdf
