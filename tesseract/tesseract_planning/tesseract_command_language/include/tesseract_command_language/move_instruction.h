#ifndef TESSERACT_COMMAND_LANGUAGE_MOVE_INSTRUCTION_H
#define TESSERACT_COMMAND_LANGUAGE_MOVE_INSTRUCTION_H

#include <tesseract_command_language/core/component_info.h>
#include <tesseract_command_language/core/waypoint.h>
#include <tesseract_command_language/instruction_type.h>
#include <vector>
#include <Eigen/Geometry>

namespace tesseract_planning
{

enum class MoveInstructionType : int
{
  LINEAR,
  FREESPACE,
  CIRCULAR
};

class MoveInstruction
{
public:
  using Ptr = std::shared_ptr<MoveInstruction>;
  using ConstPtr = std::shared_ptr<const MoveInstruction>;

  MoveInstruction(Waypoint waypoint, MoveInstructionType type);

  void setWaypoint(Waypoint waypoint);
  const Waypoint& getWaypoint() const;

  void setTCP(Eigen::Isometry3d tcp);
  const Eigen::Isometry3d& getTCP() const;

  void setWorkingFrame(std::string working_frame);
  const std::string& getWorkingFrame() const;

  void setPosition(Eigen::VectorXd position);
  const Eigen::VectorXd& getPosition() const;

  void setVelocity(Eigen::VectorXd velocity);
  const Eigen::VectorXd& getVelocity() const;

  void setAcceleration(Eigen::VectorXd acceleration);
  const Eigen::VectorXd& getAcceleration() const;

  void setEffort(Eigen::VectorXd effort);
  const Eigen::VectorXd& getEffort() const;

  void setTime(double time);
  const double& getTime() const;

  int getType() const;

  const std::string& getDescription() const;

  void setDescription(const std::string& description);

  bool isComposite() const;

  bool isPlan() const;

  bool isMove() const;

  void print() const;

  bool isLinear() const;

  bool isFreespace() const;

  bool isCircular() const;

private:
  int type_ { static_cast<int>(InstructionType::MOVE_INSTRUCTION) };

  MoveInstructionType move_type_;
  std::string description_;

  /** @brief The assigned waypoint (Cartesian or Joint) */
  Waypoint waypoint_;

  /** @brief The tool center point */
  Eigen::Isometry3d tcp_ { Eigen::Isometry3d::Identity() };

  /** @brief The working frame the waypoint is relative to */
  std::string working_frame_;

  /**
   * @brief The joint position at the waypoint
   *
   * This is different from waypoint because it can be cartesian or joint and this stores the joint position solved
   * for the planned waypoint. This also can be used for determining the robot configuration if provided a cartesian
   * waypoint for generating a native robot program.
   */
  Eigen::VectorXd position_;

  /** @brief The velocity at the waypoint */
  Eigen::VectorXd velocity_;

  /** @brief The Acceleration at the waypoint */
  Eigen::VectorXd acceleration_;

  /** @brief The Effort at the waypoint */
  Eigen::VectorXd effort_;

  /** @brief The Time from start at the waypoint */
  double time_;
};

}

#endif // TESSERACT_COMMAND_LANGUAGE_MOVE_INSTRUCTION_H
