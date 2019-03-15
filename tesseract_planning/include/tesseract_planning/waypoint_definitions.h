/**
 * @file waypoint_definitions.h
 * @brief The class that defines common types of waypoints that can be sent to the planners
 *
 * @author Matthew Powelson
 * @date February 29, 2019
 * @version TODO
 * @bug No known bugs
 *
 * @copyright Copyright (c) 2019, Southwest Research Institute
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
#ifndef TESSERACT_PLANNING_WAYPOINT_DEFINITIONS_H
#define TESSERACT_PLANNING_WAYPOINT_DEFINITIONS_H

#include <Eigen/Dense>
#include <memory>

namespace tesseract
{
namespace tesseract_planning
{
/** @brief Used to specify the type of waypoint. Corresponds to a derived class of Waypoint*/
enum class WaypointType
{
  JOINT_WAYPOINT = 0x1,      // 0000 0001
  CARTESIAN_WAYPOINT = 0x2,  // 0000 0010
};
// TODO: Doxygen
/** @brief Defines a generic way of sending waypoints to a Tesseract Planner */
class Waypoint
{
public:
  Waypoint() {}
  virtual ~Waypoint() {}
  /** @brief Returns the type of waypoint so that it may be cast back to the derived type */
  WaypointType getType() const { return waypoint_type_; }
protected:
  /** @brief Should be set by the derived class for casting Waypoint back to appropriate derived class type */
  WaypointType waypoint_type_;
};
/** @brief Defines a joint position waypoint for use with Tesseract Planners*/
class JointWaypoint : public Waypoint
{
public:
  // TODO: constructor that takes joint position vector
  JointWaypoint() { waypoint_type_ = WaypointType::JOINT_WAYPOINT; }
  virtual ~JointWaypoint() {}
  /** @brief Stores the joint values associated with this waypoint (radians) */
  std::vector<double> joint_positions_;
};
/** @brief Defines a cartesian position waypoint for use with Tesseract Planners */
class CartesianWaypoint : public Waypoint
{
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  CartesianWaypoint() { waypoint_type_ = WaypointType::CARTESIAN_WAYPOINT; }
  virtual ~CartesianWaypoint() {}
  /** @brief Contains the position and orientation of this waypoint */
  Eigen::Isometry3d cartesian_position_;

  /** @brief Convenience function that returns the xyz cartesian position contained in cartesian_position_ */
  Eigen::Vector3d getPosition() { return cartesian_position_.translation(); }
  /** @brief Convenience function that returns the xyzw rotation quarternion contained in cartesian_position_ */
  Eigen::Vector4d getOrientation()
  {
    Eigen::Quaterniond q(cartesian_position_.rotation());
    return Eigen::Vector4d(q.x(), q.y(), q.z(), q.w());
  }
};

typedef std::shared_ptr<Waypoint> WaypointPtr;
typedef std::shared_ptr<const Waypoint> WaypointConstPtr;
typedef std::shared_ptr<JointWaypoint> JointWaypointPtr;
typedef std::shared_ptr<const JointWaypoint> JointWaypointConstPtr;
typedef std::shared_ptr<CartesianWaypoint> CartesianWaypointPtr;
typedef std::shared_ptr<const CartesianWaypoint> CartesianWaypointConstPtr;
}  // namespace tesseract_planning
}  // namespace tesseract
#endif
