/**
 * @file scene_state.h
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
#ifndef TESSERACT_SCENE_GRAPH_SCENE_STATE_H
#define TESSERACT_SCENE_GRAPH_SCENE_STATE_H

#include <tesseract_common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <Eigen/Core>
#include <unordered_map>
#include <vector>
#include <memory>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract_common/types.h>

#ifdef SWIG
%shared_ptr(tesseract_scene_graph::SceneState)
#endif  // SWIG

namespace tesseract_scene_graph
{
/**
 * @brief This holds a state of the scene
 *
 * It provides a way to look up the location of a link/joint in world coordinates system by link/joint name. It is
 * possible to get the joint transform using the child link transfrom of the joint, but they are both provided for
 * convience. Also the joint values used to calculated the link/joint transfroms is provided.
 */
struct SceneState
{
  // LCOV_EXCL_START
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  // LCOV_EXCL_STOP

  using Ptr = std::shared_ptr<SceneState>;
  using ConstPtr = std::shared_ptr<const SceneState>;
  using UPtr = std::unique_ptr<SceneState>;
  using ConstUPtr = std::unique_ptr<const SceneState>;

  /**  @brief The joint values used for calculating the joint and link transforms */
  std::unordered_map<std::string, double> joints;

  /** @brief The link transforms in world coordinate system */
  tesseract_common::TransformMap link_transforms;

  /** @brief The joint transforms in world coordinate system */
  tesseract_common::TransformMap joint_transforms;

  Eigen::VectorXd getJointValues(const std::vector<std::string>& joint_names) const;
};

}  // namespace tesseract_scene_graph
#endif  // TESSERACT_SCENE_GRAPH_SCENE_STATE_H
