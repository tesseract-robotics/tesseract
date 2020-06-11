#include <tesseract_command_language/composite_instruction.h>
#include <stdexcept>

namespace tesseract_planning
{
CompositeInstruction::CompositeInstruction(std::string profile, CompositeInstructionOrder order)
  : profile_(std::move(profile))
  , order_(order) {}

CompositeInstructionOrder CompositeInstruction::getOrder() const { return order_; }

int CompositeInstruction::getType() const { return type_; }

const std::string& CompositeInstruction::getDescription() const { return description_; }

void CompositeInstruction::setDescription(const std::string& description) { description_ = description; }

void CompositeInstruction::setProfile(const std::string& profile)
{
  profile_ = (profile.empty()) ? "DEFAULT" : profile;
}
const std::string& CompositeInstruction::getProfile() const { return profile_; }

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
