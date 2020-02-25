/**
 * @file extension_approach_generator.cpp
 * @brief Tesseract process approach implementation
 *
 * @author David Merz, Jr.
 * @date February 25, 2020
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

#ifndef EXTENSION_APPROACH_GENERATOR_H
#define EXTENSION_APPROACH_GENERATOR_H

namespace tesseract_process_planners
{
/**
 * @brief The ExtensionApproachGenerator class
 * For a series of waypoints that progresses along either the positive or
 * negative x-axis of the waypoints, this class generates an extension at
 * the start of that path.  A positive X-direction in the supplied vector
 * will automatically be transformed to align with the direction the path
 * will travel, reducing complexity of calling code for alternating paths.
 */
class ExtensionApproachGenerator : public ProcessStepGenerator
{
public:
  /**
   * @brief ExtensionApproachGenerator - constructor. Supply a vector for
   * the approach to follow along.
   * @param approach - the vector along which the approach should be generated
   * @param step_count - how many waypoints should be generated
   */
  ExtensionApproachGenerator(const Eigen::Vector3d& approach, int step_count)
    : approach_(Eigen::Isometry3d::Identity()), step_count_(step_count)
  {
    approach_.translation() = approach;
  }
  /**
   * @brief ExtensionApproachGenerator - alternate constructor to ease use with
   * ProcessPlannerConfig, which uses Isometry3d for approach_direction.  Uses
   * only the translation portion of a provided isometry.
   * @param approach - the vector along which the approach should be generated
   * @param step_count - how many waypoints should be generated
   */
  ExtensionApproachGenerator(const Eigen::Isometry3d& approach, int step_count)
    : approach_(Eigen::Isometry3d::Identity()), step_count_(step_count)
  {
    approach_.translation() = approach.translation();
  }
  ~ExtensionApproachGenerator() override = default;
  ExtensionApproachGenerator(const ExtensionApproachGenerator&) = default;
  ExtensionApproachGenerator& operator=(const ExtensionApproachGenerator&) = default;
  ExtensionApproachGenerator(ExtensionApproachGenerator&&) = default;
  ExtensionApproachGenerator& operator=(ExtensionApproachGenerator&&) = default;

  std::vector<tesseract_motion_planners::Waypoint::Ptr>
  generate(const std::vector<tesseract_motion_planners::Waypoint::Ptr>& waypoints,
           const ProcessDefinitionConfig& config) const override
  {
    assert(waypoints.front()->getType() == tesseract_motion_planners::WaypointType::CARTESIAN_WAYPOINT);
    const tesseract_motion_planners::CartesianWaypoint::Ptr& cur_waypoint =
        std::static_pointer_cast<tesseract_motion_planners::CartesianWaypoint>(waypoints.front());

    Eigen::Isometry3d alt_approach = approach_;
    // Ensure that negative x/y 'double back' and positive x/y move in the direction of the path
    if (waypoints.size() >= 2)
    {
      assert(waypoints[1]->getType() == tesseract_motion_planners::WaypointType::CARTESIAN_WAYPOINT);
      const tesseract_motion_planners::CartesianWaypoint::Ptr& next_waypoint =
          std::static_pointer_cast<tesseract_motion_planners::CartesianWaypoint>(*(waypoints.begin() + 1));
      Eigen::Isometry3d next_in_first = cur_waypoint->getTransform().inverse() * next_waypoint->getTransform();

      // If the first waypoint x-axis is pointing away from the next waypoint,
      // reverse the requested x-translation and y translation.  This assumes that
      // the x-axis is the intended direction of travel.
      if (next_in_first.translation().x() > 0)
      {
        alt_approach.translation().x() *= -1;
        alt_approach.translation().y() *= -1;
      }
    }

    std::vector<tesseract_motion_planners::Waypoint::Ptr> approach;
    approach.reserve(static_cast<size_t>(step_count_) + 1);

    for (int i = step_count_; i-- > 0;)
    {
      Eigen::Isometry3d scaled = alt_approach;

      // Interpolate the transform
      double progress = static_cast<double>(i + 1) / static_cast<double>(step_count_);
      scaled.translation() = progress * alt_approach.translation();
      scaled.linear() = Eigen::Quaterniond(1.0, 0.0, 0.0, 0.0)
                            .slerp(progress, Eigen::Quaterniond(alt_approach.rotation()))
                            .toRotationMatrix();

      tesseract_motion_planners::CartesianWaypoint::Ptr new_waypoint =
          std::make_shared<tesseract_motion_planners::CartesianWaypoint>(
              config.world_offset_direction * cur_waypoint->getTransform() * config.local_offset_direction * scaled,
              cur_waypoint->getParentLinkName());
      new_waypoint->setCoefficients(cur_waypoint->getCoefficients());
      new_waypoint->setIsCritical(cur_waypoint->isCritical());
      approach.push_back(new_waypoint);
    }
    return approach;
  }

private:
  Eigen::Isometry3d approach_;
  int step_count_;
};
}  // namespace tesseract_process_planners

#endif  // EXTENSION_APPROACH_GENERATOR_H
