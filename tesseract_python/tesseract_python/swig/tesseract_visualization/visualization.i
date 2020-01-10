/**
 * @file visualization.i
 * @brief SWIG interface file for tesseract_visualization/visualization.h
 *
 * @author John Wason
 * @date December 10, 2019
 * @version TODO
 * @bug No known bugs
 *
 * @copyright Copyright (c) 2019, Wason Technology, LLC
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

%{
#include <tesseract_visualization/visualization.h>
%}

%shared_ptr(tesseract_visualization::Visualization)

namespace tesseract_visualization
{
class Visualization
{
public:
  using Ptr = std::shared_ptr<Visualization>;
  using ConstPtr = std::shared_ptr<const Visualization>;

  virtual ~Visualization();
  virtual void plotTrajectory(const std::vector<std::string>& joint_names,
                              const Eigen::Ref<const tesseract_common::TrajArray>& traj) = 0;

  virtual void plotContactResults(const std::vector<std::string>& link_names,
                                  const tesseract_collision::ContactResultVector& dist_results,
                                  const Eigen::Ref<const Eigen::VectorXd>& safety_distances) = 0;

  virtual void plotArrow(const Eigen::Ref<const Eigen::Vector3d>& pt1,
                         const Eigen::Ref<const Eigen::Vector3d>& pt2,
                         const Eigen::Ref<const Eigen::Vector4d>& rgba,
                         double scale) = 0;

  virtual void plotAxis(const Eigen::Isometry3d& axis, double scale) = 0;

  virtual void clear() = 0;

  virtual void waitForInput() = 0;
};

}  // namespace tesseract_visualization
