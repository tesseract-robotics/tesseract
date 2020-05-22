#ifndef TESSERACT_COMMAND_LANGUAGE_CARTESIAN_WAYPOINT_H
#define TESSERACT_COMMAND_LANGUAGE_CARTESIAN_WAYPOINT_H

#include <tesseract_common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <Eigen/Core>
#include <Eigen/Geometry>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract_command_language/waypoint_type.h>

namespace tesseract_planning
{

class CartesianWaypoint : public Eigen::Isometry3d
{
public:

  CartesianWaypoint(void):Eigen::Isometry3d() {}

  // This constructor allows you to construct MyVectorType from Eigen expressions
  template<typename OtherDerived>
  CartesianWaypoint(const Eigen::MatrixBase<OtherDerived>& other)
      : Eigen::Isometry3d(other)
  { }

  // This method allows you to assign Eigen expressions to MyVectorType
  template<typename OtherDerived>
  CartesianWaypoint& operator=(const Eigen::MatrixBase<OtherDerived>& other)
  {
      this->Eigen::Isometry3d::operator=(other);
      return *this;
  }

  CartesianWaypoint(const Eigen::Isometry3d& other)
      : Eigen::Isometry3d(other)
  { }

  CartesianWaypoint& operator=(const Eigen::Isometry3d& other)
  {
      this->Eigen::Isometry3d::operator=(other);
      return *this;
  }

  int getType() const { return static_cast<int>(WaypointType::CARTESIAN_WAYPOINT); }

};

}

#endif // TESSERACT_COMMAND_LANGUAGE_CARTESIAN_WAYPOINT_H
