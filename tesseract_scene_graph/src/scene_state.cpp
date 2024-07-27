/**
 * @file scene_state.cpp
 * @brief This holds a state of the scene
 *
 * It provides a way to look up the location of a link/joint in world coordinates system by link/joint name. It is
 * possible to get the joint transform using the child link transfrom of the joint, but they are both provided for
 * convience. Also the joint values used to calculated the link/joint transfroms is provided.
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

#include <tesseract_common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <boost/serialization/access.hpp>
#include <boost/serialization/map.hpp>
#include <boost/serialization/nvp.hpp>
#include <boost/serialization/unordered_map.hpp>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract_common/eigen_serialization.h>
#include <tesseract_common/utils.h>
#include <tesseract_scene_graph/scene_state.h>

namespace tesseract_scene_graph
{
Eigen::VectorXd SceneState::getJointValues(const std::vector<std::string>& joint_names) const
{
  Eigen::VectorXd jv;
  jv.resize(static_cast<long int>(joint_names.size()));
  for (auto j = 0U; j < joint_names.size(); ++j)
    jv(j) = joints.at(joint_names[j]);

  return jv;
}

bool SceneState::operator==(const SceneState& rhs) const
{
  auto isometry_equal = [](const Eigen::Isometry3d& iso_1, const Eigen::Isometry3d& iso_2) {
    return iso_1.isApprox(iso_2, 1e-5);
  };

  using namespace tesseract_common;
  bool equal = true;
  equal &= isIdenticalMap<std::unordered_map<std::string, double>, double>(joints, rhs.joints);
  equal &= isIdenticalMap<TransformMap, Eigen::Isometry3d>(link_transforms, rhs.link_transforms, isometry_equal);
  equal &= isIdenticalMap<TransformMap, Eigen::Isometry3d>(joint_transforms, rhs.joint_transforms, isometry_equal);

  return equal;
}
bool SceneState::operator!=(const SceneState& rhs) const { return !operator==(rhs); }

template <class Archive>
void SceneState::serialize(Archive& ar, const unsigned int /*version*/)
{
  ar& BOOST_SERIALIZATION_NVP(joints);
  ar& BOOST_SERIALIZATION_NVP(link_transforms);
  ar& BOOST_SERIALIZATION_NVP(joint_transforms);
}

}  // namespace tesseract_scene_graph

#include <tesseract_common/serialization.h>
BOOST_CLASS_EXPORT_IMPLEMENT(tesseract_scene_graph::SceneState)
TESSERACT_SERIALIZE_ARCHIVES_INSTANTIATE(tesseract_scene_graph::SceneState)
