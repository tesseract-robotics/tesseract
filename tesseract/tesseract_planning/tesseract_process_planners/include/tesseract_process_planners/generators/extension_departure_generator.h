/**
 * @file extension_departure_generator.cpp
 * @brief Tesseract process departure implementation
 *
 * @author David Merz, Jr.
 * @date February 17, 2020
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

#ifndef TESSERACT_PLANNING_EXTENSION_DEPARTURE_GENERATOR_H
#define TESSERACT_PLANNING_EXTENSION_DEPARTURE_GENERATOR_H

#include <tesseract_process_planners/process_definition.h>
#include <Eigen/Core>

namespace tesseract_process_planners
{
/**
 * @brief The ExtensionDepartureGenerator class
 * For a series of waypoints that progresses along either the positive or
 * negative x-axis of the waypoints, this class generates an extension at
 * the end of that path.  A positive X-direction in the supplied vector
 * will automatically be transformed to align with the direction the path
 * was traveling, reducing complexity of calling code for alternating paths.
 */
class ExtensionDepartureGenerator : public ProcessStepGenerator
{
public:
  /**
   * @brief ExtensionDepartureGenerator - constructor. Supply a vector for
   * the departure to follow along.
   * @param departure - the vector along which the departure should be generated
   * @param step_count - how many waypoints should be generated
   */
  ExtensionDepartureGenerator(const Eigen::Vector3d& departure, int step_count)
    : departure_(Eigen::Isometry3d::Identity()), step_count_(step_count)
  {
    departure_.translation() = departure;
  }
  /**
   * @brief ExtensionDepartureGenerator - alternate constructor to ease use with
   * ProcessPlannerConfig, which uses Isometry3d for departure_direction.  Uses
   * only the translation portion of a provided isometry.
   * @param departure - the vector along which the departure should be generated
   * @param step_count - how many waypoints should be generated
   */
  ExtensionDepartureGenerator(const Eigen::Isometry3d& departure, int step_count)
    : departure_(Eigen::Isometry3d::Identity()), step_count_(step_count)
  {
    departure_.translation() = departure.translation();
  }
  ~ExtensionDepartureGenerator() override = default;
  ExtensionDepartureGenerator(const ExtensionDepartureGenerator&) = default;
  ExtensionDepartureGenerator& operator=(const ExtensionDepartureGenerator&) = default;
  ExtensionDepartureGenerator(ExtensionDepartureGenerator&&) = default;
  ExtensionDepartureGenerator& operator=(ExtensionDepartureGenerator&&) = default;

  std::vector<tesseract_motion_planners::Waypoint::Ptr>
  generate(const std::vector<tesseract_motion_planners::Waypoint::Ptr>& waypoints,
           const ProcessDefinitionConfig& config) const override
  {
    assert(waypoints.back()->getType() == tesseract_motion_planners::WaypointType::CARTESIAN_WAYPOINT);
    const tesseract_motion_planners::CartesianWaypoint::Ptr& cur_waypoint =
        std::static_pointer_cast<tesseract_motion_planners::CartesianWaypoint>(waypoints.back());

    Eigen::Isometry3d alt_departure = departure_;
    // Ensure that negative x/y 'double back' and positive x/y move in the direction of the path
    if (waypoints.size() >= 2)
    {
      assert(waypoints[waypoints.size() - 2]->getType() == tesseract_motion_planners::WaypointType::CARTESIAN_WAYPOINT);
      const tesseract_motion_planners::CartesianWaypoint::Ptr& prev_waypoint =
          std::static_pointer_cast<tesseract_motion_planners::CartesianWaypoint>(*(waypoints.end() - 2));
      Eigen::Isometry3d penultimate_in_final = cur_waypoint->getTransform().inverse() * prev_waypoint->getTransform();

      // If the final waypoint x-axis is pointing 'towards' the previous waypoint,
      // reverse the requested x-translation and y translation.  This assumes that
      // the x-axis is the intended direction of travel.
      if (penultimate_in_final.translation().x() > 0)
      {
        alt_departure.translation().x() *= -1;
        alt_departure.translation().y() *= -1;
      }
    }

    std::vector<tesseract_motion_planners::Waypoint::Ptr> departure;
    departure.reserve(static_cast<size_t>(step_count_) + 1);

    for (int i = 0; i < step_count_; i++)
    {
      Eigen::Isometry3d scaled = alt_departure;

      // Interpolate the transform
      double progress = static_cast<double>(i + 1) / static_cast<double>(step_count_);
      scaled.translation() = progress * alt_departure.translation();
      scaled.linear() = Eigen::Quaterniond(1.0, 0.0, 0.0, 0.0)
                            .slerp(progress, Eigen::Quaterniond(alt_departure.rotation()))
                            .toRotationMatrix();

      tesseract_motion_planners::CartesianWaypoint::Ptr new_waypoint =
          std::make_shared<tesseract_motion_planners::CartesianWaypoint>(
              config.world_offset_direction * cur_waypoint->getTransform() * config.local_offset_direction * scaled,
              cur_waypoint->getParentLinkName());
      new_waypoint->setCoefficients(cur_waypoint->getCoefficients());
      new_waypoint->setIsCritical(cur_waypoint->isCritical());
      departure.push_back(new_waypoint);
    }
    return departure;
  }

private:
  Eigen::Isometry3d departure_;
  int step_count_;
};

}  // namespace tesseract_process_planners

#endif  // TESSERACT_PLANNING_EXTENSION_DEPARTURE_GENERATOR_H
