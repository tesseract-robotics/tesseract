#include <tesseract_command_language/composite_instruction.h>
#include <stdexcept>

namespace tesseract_planning
{
CompositeInstruction::CompositeInstruction(CompositeInstructionOrder order) : order_(order) {}

CompositeInstructionOrder CompositeInstruction::getOrder() const { return order_; }

void CompositeInstruction::addCost(ComponentInfo component)
{
  if (!component.isCompositeInstructionSupported())
    throw std::runtime_error("Component is not supported for a composite instruction!");

  costs_.push_back(component);
}
const std::vector<ComponentInfo>& CompositeInstruction::getCosts() const { return costs_; }

void CompositeInstruction::addConstraint(ComponentInfo component)
{
  if (!component.isCompositeInstructionSupported())
    throw std::runtime_error("Component is not supported for a composite instruction!");

  constraints_.push_back(component);
}

const std::vector<ComponentInfo>& CompositeInstruction::getConstraints() const { return constraints_; }

int CompositeInstruction::getType() const { return type_; }

const std::string& CompositeInstruction::getDescription() const { return description_; }

void CompositeInstruction::setDescription(const std::string& description) { description_ = description; }

bool CompositeInstruction::isComposite() const { return true; }

bool CompositeInstruction::isPlan() const { return false; }

bool CompositeInstruction::isMove() const { return false; }

void CompositeInstruction::print() const  {}

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

}
