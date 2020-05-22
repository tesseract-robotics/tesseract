#ifndef TESSERACT_COMMAND_LANGUAGE_INSTRUCTION_H
#define TESSERACT_COMMAND_LANGUAGE_INSTRUCTION_H

#include <tesseract_command_language/core/waypoint.h>
#include <tesseract_command_language/core/component_info.h>

namespace tesseract_planning
{

namespace detail
{

struct InstructionInnerBase
{
  InstructionInnerBase() = default;
  virtual ~InstructionInnerBase() = default;
  InstructionInnerBase(const InstructionInnerBase&) = delete;
  InstructionInnerBase& operator=(const InstructionInnerBase&) = delete;
  InstructionInnerBase(InstructionInnerBase&&) = delete;
  InstructionInnerBase& operator=(InstructionInnerBase&&) = delete;

  virtual int getType() const = 0;

  virtual const std::string& getDescription() const = 0;

  virtual void setDescription(const std::string& description) = 0;

  virtual bool isComposite() const = 0;

  virtual bool isPlan() const = 0;

  virtual bool isMove() const = 0;

  virtual void print() const = 0;

  // This is not required for user defined implementation
  virtual void* recover() = 0;

  // This is not required for user defined implementation
  virtual std::unique_ptr<InstructionInnerBase> clone() const = 0;
};

template <typename T>
struct InstructionInner final : InstructionInnerBase
{
  InstructionInner() = default;
  ~InstructionInner() override = default;
  InstructionInner(const InstructionInner &) = delete;
  InstructionInner(InstructionInner &&) = delete;
  InstructionInner &operator=(const InstructionInner &) = delete;
  InstructionInner &operator=(InstructionInner &&) = delete;

  // Constructors from T (copy and move variants).
  explicit InstructionInner(T instruction) : instruction_(std::move(instruction)) {}
  explicit InstructionInner(T &&instruction) : instruction_(std::move(instruction)) {}

  std::unique_ptr<InstructionInnerBase> clone() const override
  {
    return std::make_unique<InstructionInner>(instruction_);
  }

  void* recover() { return &instruction_; }

  int getType() const { return instruction_.getType(); }

  const std::string& getDescription() const { return instruction_.getDescription(); }

  void setDescription(const std::string& description) { instruction_.setDescription(description); }

  bool isComposite() const { return instruction_.isComposite(); }

  bool isPlan() const { return instruction_.isPlan(); }

  bool isMove() const { return instruction_.isMove(); }

  void print() const { instruction_.print(); }

  T instruction_;
};

}

class Instruction
{
  template <typename T>
  using uncvref_t = std::remove_cv_t<std::remove_reference_t<T>>;

  // Enable the generic ctor only if ``T`` is not a ForwardKinematics (after removing const/reference qualifiers)
  // If ``T`` is of type ForwardKinematics we disable so it will use the copy or move constructors of this class.
  template <typename T>
  using generic_ctor_enabler = std::enable_if_t<!std::is_same<Instruction, uncvref_t<T>>::value, int>;

public:
  using Ptr = std::shared_ptr<Instruction>;
  using ConstPtr = std::shared_ptr<const Instruction>;

  template <typename T, generic_ctor_enabler<T> = 0>
  Instruction(T &&instruction) // NOLINT
    : instruction_(std::make_unique<detail::InstructionInner<uncvref_t<T>>>(instruction))
  {
  }

  // Destructor
  ~Instruction() = default;

  // Copy constructor
  Instruction(const Instruction &other) : instruction_(other.instruction_->clone()) {}

  // Move ctor.
  Instruction(Instruction &&other) noexcept { instruction_.swap(other.instruction_); }
  // Move assignment.
  Instruction &operator=(Instruction &&other) noexcept { instruction_.swap(other.instruction_); return (*this); }

  // Copy assignment.
  Instruction &operator=(const Instruction &other)
  {
    (*this) = Instruction(other);
    return (*this);
  }

  template <typename T, generic_ctor_enabler<T> = 0>
  Instruction &operator=(T &&other)
  {
    (*this) = Instruction(std::forward<T>(other));
    return (*this);
  }

  int getType() const { return instruction_->getType(); }

  const std::string& getDescription() const { return instruction_->getDescription(); }

  void setDescription(const std::string& description) { instruction_->setDescription(description); }

  bool isComposite() const { return instruction_->isComposite(); };

  bool isPlan() const { return instruction_->isPlan(); }

  bool isMove() const { return instruction_->isMove(); }

  void print() const { instruction_->print(); }

  template<typename T>
  T* cast() { return static_cast<T*>(instruction_->recover()); }

  template<typename T>
  const T* cast_const() const { return static_cast<const T*>(instruction_->recover()); }

private:
  std::unique_ptr<detail::InstructionInnerBase> instruction_;
};

}

#endif // TESSERACT_COMMAND_LANGUAGE_INSTRUCTION_H
