/**
 * @file waypoint.i
 * @brief SWIG interface file for tesseract_planning/core/waypoint.h
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
#include <tesseract_motion_planners/core/waypoint.h>
%}

%shared_ptr(tesseract_motion_planners::Waypoint)
%shared_ptr(tesseract_motion_planners::JointWaypoint)
%shared_ptr(tesseract_motion_planners::CartesianWaypoint)
%shared_ptr(tesseract_motion_planners::JointTolerancedWaypoint)

namespace tesseract_motion_planners
{
enum class WaypointType
{
  JOINT_WAYPOINT,
  JOINT_TOLERANCED_WAYPOINT,
  CARTESIAN_WAYPOINT
};

inline bool isCartesianWaypointType(WaypointType type);

inline bool isJointWaypointType(WaypointType type);

class Waypoint
{
public:
  
  using Ptr = std::shared_ptr<Waypoint>;
  using ConstPtr = std::shared_ptr<const Waypoint>;

  Waypoint(WaypointType type);
  virtual ~Waypoint();
  
  WaypointType getType();

  virtual bool isCritical() const;
  
  virtual void setIsCritical(bool is_critical);

  virtual bool setCoefficients(Eigen::VectorXd coefficients);

  virtual const Eigen::VectorXd getCoefficients() const;
};

class JointWaypoint : public Waypoint
{
public:
  
  using Ptr = std::shared_ptr<JointWaypoint>;
  using ConstPtr = std::shared_ptr<const JointWaypoint>;

  JointWaypoint(Eigen::VectorXd joint_positions, std::vector<std::string> joint_names);

  const Eigen::VectorXd getPositions();

  Eigen::VectorXd getPositions(const std::vector<std::string>& joint_names) const;

  const Eigen::VectorXd getCoefficients() const override;

  Eigen::VectorXd getCoefficients(const std::vector<std::string>& joint_names) const;

  const std::vector<std::string> getNames() const;

  bool compare(const std::vector<std::string>& joint_names) const;

};

class CartesianWaypoint : public Waypoint
{
public:
  
  using Ptr = std::shared_ptr<CartesianWaypoint>;
  using ConstPtr = std::shared_ptr<const CartesianWaypoint>;

  CartesianWaypoint(const Eigen::Isometry3d& cartesian_position, std::string parent_link = "");

  const Eigen::Isometry3d getTransform();

  Eigen::Vector3d getPosition() const;
  
  Eigen::Vector4d getOrientation() const;

  std::string getParentLinkName() const;
};

class JointTolerancedWaypoint : public JointWaypoint
{
public:
  using Ptr = std::shared_ptr<JointTolerancedWaypoint>;
  using ConstPtr = std::shared_ptr<const JointTolerancedWaypoint>;

  JointTolerancedWaypoint(Eigen::VectorXd joint_positions, std::vector<std::string> joint_names);

  bool setUpperTolerance(Eigen::VectorXd upper_tolerance);

  const Eigen::VectorXd getUpperTolerance() const;

  Eigen::VectorXd getUpperTolerance(const std::vector<std::string>& joint_names) const;

  bool setLowerTolerance(Eigen::VectorXd lower_tolerance);

  const Eigen::VectorXd getLowerTolerance() const;

   Eigen::VectorXd getLowerTolerance(const std::vector<std::string>& joint_names) const;
};

}  // namespace tesseract_motion_planners