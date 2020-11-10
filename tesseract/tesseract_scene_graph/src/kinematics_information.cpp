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

#include <tesseract_scene_graph/kinematics_information.h>

namespace tesseract_scene_graph
{
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

}  // namespace tesseract_scene_graph
