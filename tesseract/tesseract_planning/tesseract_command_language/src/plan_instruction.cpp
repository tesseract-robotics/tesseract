#include <tesseract_command_language/plan_instruction.h>

namespace tesseract_planning
{
PlanInstruction::PlanInstruction(Waypoint waypoint, PlanInstructionType type)
  : plan_type_(type)
  , waypoint_(std::move(waypoint)) {}

void PlanInstruction::setWaypoint(Waypoint waypoint) { waypoint_ = waypoint; }
const Waypoint& PlanInstruction::getWaypoint() const { return waypoint_; }

void PlanInstruction::setTCP(Eigen::Isometry3d tcp) { tcp_ = tcp; }
const Eigen::Isometry3d& PlanInstruction::getTCP() const { return tcp_; }

void PlanInstruction::setWorkingFrame(std::string working_frame) { working_frame_ = working_frame; }
const std::string& PlanInstruction::getWorkingFrame() const { return working_frame_; }

void PlanInstruction::addCost(ComponentInfo component) { costs_.push_back(component); }
const std::vector<ComponentInfo>& PlanInstruction::getCosts() const { return costs_; }

void PlanInstruction::addConstraint(ComponentInfo component) { constraints_.push_back(component); }
const std::vector<ComponentInfo>& PlanInstruction::getConstraints() const { return constraints_; }

void PlanInstruction::addPathCost(ComponentInfo component) { path_costs_.push_back(component); }
const std::vector<ComponentInfo>& PlanInstruction::getPathCosts() const { return path_costs_; }

void PlanInstruction::addPathConstraint(ComponentInfo component) { path_constraints_.push_back(component); }
const std::vector<ComponentInfo>& PlanInstruction::getPathConstraints() const { return path_constraints_; }

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
