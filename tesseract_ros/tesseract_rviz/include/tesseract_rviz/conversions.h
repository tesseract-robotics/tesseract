/**
 * @file conversions.h
 * @brief Common conversion used
 *
 * @author Levi Armstrong
 * @date Dec 18, 2017
 * @version TODO
 * @bug No known bugs
 *
 * @copyright Copyright (c) 2017, Southwest Research Institute
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
#ifndef TESSERACT_RVIZ_CONVERSIONS_H
#define TESSERACT_RVIZ_CONVERSIONS_H

#include <Eigen/Geometry>
#include <OgreVector3.h>
#include <OgreQuaternion.h>

namespace tesseract_rviz
{
static inline void toEigen(Eigen::Isometry3d& transform,
                           const Ogre::Vector3& position,
                           const Ogre::Quaternion& orientation)
{
  transform.linear() = Eigen::Quaterniond(orientation.w, orientation.x, orientation.y, orientation.z).matrix();
  transform.translation() = Eigen::Vector3d(position.x, position.y, position.z);
}

static inline void toOgre(Ogre::Vector3& position, Ogre::Quaternion& orientation, const Eigen::Isometry3d& transform)
{
  Eigen::Vector3f robot_visual_position = transform.translation().cast<float>();
  Eigen::Quaternionf robot_visual_orientation(transform.rotation().cast<float>());
  position = Ogre::Vector3(robot_visual_position.x(), robot_visual_position.y(), robot_visual_position.z());
  orientation = Ogre::Quaternion(robot_visual_orientation.w(),
                                 robot_visual_orientation.x(),
                                 robot_visual_orientation.y(),
                                 robot_visual_orientation.z());
}

}  // namespace tesseract_rviz
#endif  // TESSERACT_RVIZ_CONVERSIONS_H
