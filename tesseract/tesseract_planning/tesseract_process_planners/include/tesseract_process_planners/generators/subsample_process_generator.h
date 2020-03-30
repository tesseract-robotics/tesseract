/**
 * @file subsample_process_generator.h
 * @brief Generator that takes a tool path and subsamples to meet the minimum number of waypoints. It does this by
 * subdividing.
 *
 * @todo Should update to add longest valid cartesian distance.
 *
 * @author Levi Armstrong
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

#ifndef TESSERACT_PROCESS_PLANNING_SUBSAMPLE_GENERATOR_H
#define TESSERACT_PROCESS_PLANNING_SUBSAMPLE_GENERATOR_H

#include <tesseract_process_planners/process_definition.h>

#include <tesseract_common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <cmath>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

namespace tesseract_process_planners
{
class SubsampleProcessGenerator : public ProcessStepGenerator
{
public:
  SubsampleProcessGenerator(int min_num_waypoints) : min_num_waypoints_(min_num_waypoints) {}
  ~SubsampleProcessGenerator() override = default;
  SubsampleProcessGenerator(const SubsampleProcessGenerator&) = default;
  SubsampleProcessGenerator& operator=(const SubsampleProcessGenerator&) = default;
  SubsampleProcessGenerator(SubsampleProcessGenerator&&) = default;
  SubsampleProcessGenerator& operator=(SubsampleProcessGenerator&&) = default;

  std::vector<tesseract_motion_planners::Waypoint::Ptr>
  generate(const std::vector<tesseract_motion_planners::Waypoint::Ptr>& waypoints,
           const ProcessDefinitionConfig& config) const override
  {
    std::vector<tesseract_motion_planners::Waypoint::Ptr> new_waypoints;
    new_waypoints.reserve(waypoints.size());
    for (const auto& waypoint : waypoints)
    {
      assert(waypoint->getType() == tesseract_motion_planners::WaypointType::CARTESIAN_WAYPOINT);
      const tesseract_motion_planners::CartesianWaypoint::Ptr& cur_waypoint =
          std::static_pointer_cast<tesseract_motion_planners::CartesianWaypoint>(waypoint);
      tesseract_motion_planners::CartesianWaypoint::Ptr new_waypoint =
          std::make_shared<tesseract_motion_planners::CartesianWaypoint>(
              config.world_offset_direction * cur_waypoint->getTransform() * config.local_offset_direction,
              cur_waypoint->getParentLinkName());
      new_waypoint->setCoefficients(cur_waypoint->getCoefficients());
      new_waypoint->setIsCritical(cur_waypoint->isCritical());
      new_waypoints.push_back(new_waypoint);
    }

    if (new_waypoints.size() >= min_num_waypoints_)
      return new_waypoints;

    int cnt = static_cast<int>(std::ceil(double(min_num_waypoints_) / double(new_waypoints.size())));
    for (int i = 0; i < cnt; ++i)
    {
      std::vector<tesseract_motion_planners::Waypoint::Ptr> temp;
      temp.reserve(new_waypoints.size() * 2);
      for (std::size_t j = 0; j < new_waypoints.size() - 1; ++j)
      {
        temp.push_back(new_waypoints[j]);

        const auto& cwp1 = std::static_pointer_cast<tesseract_motion_planners::CartesianWaypoint>(new_waypoints[j]);
        const auto& cwp2 = std::static_pointer_cast<tesseract_motion_planners::CartesianWaypoint>(new_waypoints[j + 1]);

        Eigen::Isometry3d delta = cwp1->getTransform().inverse() * cwp2->getTransform();
        Eigen::Isometry3d scale;
        scale.translation() = 0.5 * delta.translation();
        scale.linear() =
            Eigen::Quaterniond(1.0, 0.0, 0.0, 0.0).slerp(0.5, Eigen::Quaterniond(delta.rotation())).toRotationMatrix();

        auto new_waypoint = std::make_shared<tesseract_motion_planners::CartesianWaypoint>(cwp1->getTransform() * scale,
                                                                                           cwp1->getParentLinkName());
        new_waypoint->setCoefficients(cwp1->getCoefficients());
        new_waypoint->setIsCritical(cwp1->isCritical());
        temp.push_back(new_waypoint);
      }
      temp.push_back(new_waypoints.back());
      new_waypoints = temp;
    }

    return new_waypoints;
  }

private:
  int min_num_waypoints_;
};

}  // namespace tesseract_process_planners
#endif  // TESSERACT_PROCESS_PLANNING_SUBSAMPLE_GENERATOR_H
