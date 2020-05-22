#ifndef TESSERACT_COMMAND_LANGUAGE_JOINT_WAYPOINT_H
#define TESSERACT_COMMAND_LANGUAGE_JOINT_WAYPOINT_H

#include <tesseract_common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <Eigen/Core>
#include <memory>
#include <vector>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract_command_language/waypoint_type.h>

namespace tesseract_planning
{
class JointWaypoint : public Eigen::VectorXd
{
public:

  JointWaypoint(void):Eigen::VectorXd() {}

  // This constructor allows you to construct MyVectorType from Eigen expressions
  template<typename OtherDerived>
  JointWaypoint(const Eigen::MatrixBase<OtherDerived>& other)
      : Eigen::VectorXd(other)
  { }

  // This method allows you to assign Eigen expressions to MyVectorType
  template<typename OtherDerived>
  JointWaypoint& operator=(const Eigen::MatrixBase <OtherDerived>& other)
  {
      this->Eigen::VectorXd::operator=(other);
      return *this;
  }

  int getType() const { return static_cast<int>(WaypointType::JOINT_WAYPOINT); }

  std::vector<std::string> joint_names;
};
}

#endif // TESSERACT_COMMAND_LANGUAGE_JOINT_WAYPOINT_H
