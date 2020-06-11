#include <tesseract_command_language/plan_instruction.h>

namespace tesseract_planning
{
PlanInstruction::PlanInstruction(Waypoint waypoint, PlanInstructionType type, std::string profile)
  : plan_type_(type)
  , waypoint_(std::move(waypoint))
  , profile_(std::move(profile)) {}

void PlanInstruction::setWaypoint(Waypoint waypoint) { waypoint_ = waypoint; }
const Waypoint& PlanInstruction::getWaypoint() const { return waypoint_; }

void PlanInstruction::setTCP(Eigen::Isometry3d tcp) { tcp_ = tcp; }
const Eigen::Isometry3d& PlanInstruction::getTCP() const { return tcp_; }

void PlanInstruction::setWorkingFrame(std::string working_frame) { working_frame_ = working_frame; }
const std::string& PlanInstruction::getWorkingFrame() const { return working_frame_; }

void PlanInstruction::setProfile(const std::string& profile)
{
  profile_ = (profile.empty()) ? "DEFAULT" : profile;
}
const std::string& PlanInstruction::getProfile() const { return profile_; }

int PlanInstruction::getType() const { return type_; }

const std::string& PlanInstruction::getDescription() const { return description_; }

void PlanInstruction::setDescription(const std::string& description) { description_ = description; }

bool PlanInstruction::isComposite() const { return false; }

bool PlanInstruction::isPlan() const { return true; }

bool PlanInstruction::isMove() const { return false; }

void PlanInstruction::print() const { }

bool PlanInstruction::isLinear() const { return (plan_type_ == PlanInstructionType::LINEAR); }

bool PlanInstruction::isFreespace() const { return (plan_type_ == PlanInstructionType::FREESPACE); }

bool PlanInstruction::isCircular() const { return (plan_type_ == PlanInstructionType::CIRCULAR); }
}
