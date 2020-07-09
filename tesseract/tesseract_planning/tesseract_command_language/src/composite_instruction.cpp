#include <tesseract_command_language/composite_instruction.h>
#include <tesseract_command_language/plan_instruction.h>
#include <stdexcept>
#include <iostream>

namespace tesseract_planning
{
CompositeInstruction::CompositeInstruction(std::string profile, CompositeInstructionOrder order)
  : profile_(std::move(profile)), order_(order)
{
}

CompositeInstructionOrder CompositeInstruction::getOrder() const { return order_; }

int CompositeInstruction::getType() const { return type_; }

const std::string& CompositeInstruction::getDescription() const { return description_; }

void CompositeInstruction::setDescription(const std::string& description) { description_ = description; }

void CompositeInstruction::setProfile(const std::string& profile)
{
  profile_ = (profile.empty()) ? "DEFAULT" : profile;
}
const std::string& CompositeInstruction::getProfile() const { return profile_; }

void CompositeInstruction::setStartWaypoint(Waypoint waypoint)
{
  start_waypoint_ = waypoint;
}

const Waypoint& CompositeInstruction::getStartWaypoint() const
{
  return start_waypoint_;
}

bool CompositeInstruction::hasStartWaypoint() const
{
  return (!isNullWaypoint(start_waypoint_.getType()));
}

void CompositeInstruction::setEndWaypoint(Waypoint waypoint, bool process_child_composites)
{
  assert(!process_child_composites);
  for (auto it = this->rbegin(); it != this->rend(); ++it)
  {
    if(isPlanInstruction(it->getType()))
    {
      it->cast<PlanInstruction>()->setWaypoint(waypoint);
      return;
    }
  }

  throw std::runtime_error("Failed to set end waypoint, composite instruction does not contain any plan instructions!");
}

bool CompositeInstruction::isComposite() const { return true; }

bool CompositeInstruction::isPlan() const { return false; }

bool CompositeInstruction::isMove() const { return false; }

void CompositeInstruction::print(std::string prefix) const
{
  std::cout << prefix + "Composite Instruction, Description: " << getDescription() << std::endl;
  std::cout << prefix + "{" << std::endl;
  for (const auto& i : *this)
    i.print(prefix + "  ");
  std::cout << prefix + "}" << std::endl;
}

CompositeInstruction CompositeInstruction::flatten() const
{
  CompositeInstruction flattened;
  flattenHelper(flattened, *this);
  return flattened;
}

void CompositeInstruction::flattenHelper(CompositeInstruction& flattened, const CompositeInstruction& composite) const
{
  for (const auto& i : composite)
  {
    if (i.isComposite())
      flattenHelper(flattened, *(i.cast_const<CompositeInstruction>()));
    else
      flattened.push_back(i);
  }
}

}  // namespace tesseract_planning
