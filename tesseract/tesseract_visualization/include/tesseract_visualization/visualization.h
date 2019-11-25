/**
 * @file visualization.h
 * @brief Visualization Class.
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
#ifndef TESSERACT_VISUALIZATION_VISUALIZATION_H
#define TESSERACT_VISUALIZATION_VISUALIZATION_H

#include <tesseract_common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <Eigen/Core>
#include <Eigen/Geometry>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract_common/types.h>
#include <tesseract_collision/core/types.h>

namespace tesseract_visualization
{
/** @brief The Vizualization class */
class Visualization
{
public:
  using Ptr = std::shared_ptr<Visualization>;
  using ConstPtr = std::shared_ptr<const Visualization>;

  Visualization() = default;
  virtual ~Visualization() = default;
  Visualization(const Visualization&) = default;
  Visualization& operator=(const Visualization&) = default;
  Visualization(Visualization&&) = default;
  Visualization& operator=(Visualization&&) = default;

  /**
   * @brief Plot a trajectory
   * @param traj
   */
  virtual void plotTrajectory(const std::vector<std::string>& joint_names,
                              const Eigen::Ref<const tesseract_common::TrajArray>& traj) = 0;

  /**
   * @brief Plot the collision results data
   * @param link_names List of link names for which to plot data
   * @param dist_results The collision results data
   * @param safety_distance Vector of safety Distance corresponding to dist_results (Must be in the same order and
   * length).
   */
  virtual void plotContactResults(const std::vector<std::string>& link_names,
                                  const tesseract_collision::ContactResultVector& dist_results,
                                  const Eigen::Ref<const Eigen::VectorXd>& safety_distances) = 0;

  /**
   * @brief Plot arrow defined by two points
   * @param pt1 Start position of the arrow
   * @param pt2 Final position of the arrow
   * @param rgba Color of the arrow
   * @param scale The size of the arrow (related to diameter)
   */
  virtual void plotArrow(const Eigen::Ref<const Eigen::Vector3d>& pt1,
                         const Eigen::Ref<const Eigen::Vector3d>& pt2,
                         const Eigen::Ref<const Eigen::Vector4d>& rgba,
                         double scale) = 0;

  /**
   * @brief Plot axis
   * @param axis The axis
   * @param scale The size of the axis
   */
  virtual void plotAxis(const Eigen::Isometry3d& axis, double scale) = 0;

  /**
   * @brief This is called at the start of the plotting for each iteration
   *        to clear previous iteration graphics if neccessary.
   */
  virtual void clear() = 0;

  /** @brief Pause code and wait for enter key in terminal*/
  virtual void waitForInput() = 0;
};

}  // namespace tesseract_visualization

#endif  // TESSERACT_VISUALIZATION_VISUALIZATION_H
