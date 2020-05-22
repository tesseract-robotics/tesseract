#include <tesseract_command_language/move_instruction.h>

namespace tesseract_planning
{
MoveInstruction::MoveInstruction(Waypoint waypoint, MoveInstructionType type)
  : move_type_(type)
  , waypoint_(std::move(waypoint)) {}

void MoveInstruction::setWaypoint(Waypoint waypoint) { waypoint_ = waypoint; }
const Waypoint& MoveInstruction::getWaypoint() const { return waypoint_; }

void MoveInstruction::setTCP(Eigen::Isometry3d tcp) { tcp_ = tcp; }
const Eigen::Isometry3d& MoveInstruction::getTCP() const { return tcp_; }

void MoveInstruction::setWorkingFrame(std::string working_frame) { working_frame_ = working_frame; }
const std::string& MoveInstruction::getWorkingFrame() const { return working_frame_; }

void MoveInstruction::setPosition(Eigen::VectorXd position) { position_ = position; }
const Eigen::VectorXd& MoveInstruction::getPosition() const { return position_; }

void MoveInstruction::setVelocity(Eigen::VectorXd velocity) { velocity_ = velocity; }
const Eigen::VectorXd& MoveInstruction::getVelocity() const { return velocity_; }

void MoveInstruction::setAcceleration(Eigen::VectorXd acceleration) { acceleration_ = acceleration; }
const Eigen::VectorXd& MoveInstruction::getAcceleration() const { return acceleration_; }

void MoveInstruction::setEffort(Eigen::VectorXd effort) { effort_ = effort; }
const Eigen::VectorXd& MoveInstruction::getEffort() const { return effort_; }

void MoveInstruction::setTime(double time) { time_ = time; }
const double& MoveInstruction::getTime() const { return time_; }

int MoveInstruction::getType() const { return type_; }

const std::string& MoveInstruction::getDescription() const { return description_; }

void MoveInstruction::setDescription(const std::string& description) { description_ = description; }

bool MoveInstruction::isComposite() const { return false; }

bool MoveInstruction::isPlan() const { return false; }

bool MoveInstruction::isMove() const { return true; }

void MoveInstruction::print() const { }

bool MoveInstruction::isLinear() const { return (move_type_ == MoveInstructionType::LINEAR); }

bool MoveInstruction::isFreespace() const { return (move_type_ == MoveInstructionType::FREESPACE); }

bool MoveInstruction::isCircular() const { return (move_type_ == MoveInstructionType::CIRCULAR); }
}
