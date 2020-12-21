/**
 * @file instruction.h
 * @brief
 *
 * @author Levi Armstrong
 * @date June 15, 2020
 * @version TODO
 * @bug No known bugs
 *
 * @copyright Copyright (c) 2020, Southwest Research Institute
 *
 * @par License
 * Software License Agreement (Apache License)
 * @par
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 * http://www.apache.org/licenses/LICENSE-2.0
 * @par
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */
#ifndef TESSERACT_COMMAND_LANGUAGE_INSTRUCTION_H
#define TESSERACT_COMMAND_LANGUAGE_INSTRUCTION_H

#include <tesseract_common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <string>
#include <tinyxml2.h>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract_command_language/core/waypoint.h>
#include <tesseract_common/sfinae_utils.h>

#ifdef SWIG
%ignore std::vector<tesseract_planning::Instruction>::vector(size_type);
%ignore std::vector<tesseract_planning::Instruction>::resize(size_type);
%pythondynamic tesseract_planning::Instruction;
%template(Instructions) std::vector<tesseract_planning::Instruction>;
#endif  // SWIG

namespace tesseract_planning
{
#ifndef SWIG
namespace detail_instruction
{
CREATE_MEMBER_CHECK(getType);
CREATE_MEMBER_CHECK(getDescription);
CREATE_MEMBER_CHECK(setDescription);
CREATE_MEMBER_CHECK(print);
CREATE_MEMBER_CHECK(toXML);
CREATE_MEMBER_FUNC_SIGNATURE_NOARGS_CHECK(getType, int);
CREATE_MEMBER_FUNC_SIGNATURE_NOARGS_CHECK(getDescription, const std::string&);
CREATE_MEMBER_FUNC_SIGNATURE_CHECK(setDescription, void, const std::string&);
CREATE_MEMBER_FUNC_SIGNATURE_CHECK(print, void, std::string);
CREATE_MEMBER_FUNC_SIGNATURE_CHECK(toXML, tinyxml2::XMLElement*, tinyxml2::XMLDocument&);

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

  virtual void print(std::string prefix) const = 0;

  virtual tinyxml2::XMLElement* toXML(tinyxml2::XMLDocument& doc) const = 0;

  // This is not required for user defined implementation
  virtual void* recover() = 0;

  // This is not required for user defined implementation
  virtual std::unique_ptr<InstructionInnerBase> clone() const = 0;
};

template <typename T>
struct InstructionInner final : InstructionInnerBase
{
  InstructionInner()
  {
    static_assert(has_member_getType<T>::value, "Class does not have member function 'getType'");
    static_assert(has_member_getDescription<T>::value, "Class does not have member function 'getDescription'");
    static_assert(has_member_setDescription<T>::value, "Class does not have member function 'setDescription'");
    static_assert(has_member_print<T>::value, "Class does not have member function 'print'");
    static_assert(has_member_toXML<T>::value, "Class does not have member function 'toXML'");
    static_assert(has_member_func_signature_getType<T>::value, "Class 'getType' function has incorrect signature");
    static_assert(has_member_func_signature_getDescription<T>::value,
                  "Class 'getDescription' function has incorrect signature");
    static_assert(has_member_func_signature_setDescription<T>::value,
                  "Class 'setDescription' function has incorrect signature");
    static_assert(has_member_func_signature_print<T>::value, "Class 'print' function has incorrect signature");
    static_assert(has_member_func_signature_toXML<T>::value, "Class 'toXML' function has incorrect signature");
  }
  ~InstructionInner() override = default;
  InstructionInner(const InstructionInner&) = delete;
  InstructionInner(InstructionInner&&) = delete;
  InstructionInner& operator=(const InstructionInner&) = delete;
  InstructionInner& operator=(InstructionInner&&) = delete;

  // Constructors from T (copy and move variants).
  explicit InstructionInner(T instruction) : instruction_(std::move(instruction))
  {
    static_assert(has_member_getType<T>::value, "Class does not have member function 'getType'");
    static_assert(has_member_getDescription<T>::value, "Class does not have member function 'getDescription'");
    static_assert(has_member_setDescription<T>::value, "Class does not have member function 'setDescription'");
    static_assert(has_member_print<T>::value, "Class does not have member function 'print'");
    static_assert(has_member_toXML<T>::value, "Class does not have member function 'toXML'");
    static_assert(has_member_func_signature_getType<T>::value, "Class 'getType' function has incorrect signature");
    static_assert(has_member_func_signature_getDescription<T>::value,
                  "Class 'getDescription' function has incorrect signature");
    static_assert(has_member_func_signature_setDescription<T>::value,
                  "Class 'setDescription' function has incorrect signature");
    static_assert(has_member_func_signature_print<T>::value, "Class 'print' function has incorrect signature");
    static_assert(has_member_func_signature_toXML<T>::value, "Class 'toXML' function has incorrect signature");
  }

  explicit InstructionInner(T&& instruction) : instruction_(std::move(instruction))
  {
    static_assert(has_member_getType<T>::value, "Class does not have member function 'getType'");
    static_assert(has_member_getDescription<T>::value, "Class does not have member function 'getDescription'");
    static_assert(has_member_setDescription<T>::value, "Class does not have member function 'setDescription'");
    static_assert(has_member_print<T>::value, "Class does not have member function 'print'");
    static_assert(has_member_toXML<T>::value, "Class does not have member function 'toXML'");
    static_assert(has_member_func_signature_getType<T>::value, "Class 'getType' function has incorrect signature");
    static_assert(has_member_func_signature_getDescription<T>::value,
                  "Class 'getDescription' function has incorrect signature");
    static_assert(has_member_func_signature_setDescription<T>::value,
                  "Class 'setDescription' function has incorrect signature");
    static_assert(has_member_func_signature_print<T>::value, "Class 'print' function has incorrect signature");
    static_assert(has_member_func_signature_toXML<T>::value, "Class 'toXML' function has incorrect signature");
  }

  std::unique_ptr<InstructionInnerBase> clone() const override
  {
    return std::make_unique<InstructionInner>(instruction_);
  }

  void* recover() final { return &instruction_; }

  int getType() const final { return instruction_.getType(); }

  const std::string& getDescription() const final { return instruction_.getDescription(); }

  void setDescription(const std::string& description) final { instruction_.setDescription(description); }

  void print(std::string prefix) const final { instruction_.print(prefix); }

  tinyxml2::XMLElement* toXML(tinyxml2::XMLDocument& doc) const final { return instruction_.toXML(doc); }

  T instruction_;
};

}  // namespace detail_instruction

#endif  // SWIG

class Instruction
{
  template <typename T>
  using uncvref_t = std::remove_cv_t<typename std::remove_reference<T>::type>;

  // Enable the generic ctor only if ``T`` is not a ForwardKinematics (after removing const/reference qualifiers)
  // If ``T`` is of type ForwardKinematics we disable so it will use the copy or move constructors of this class.
  template <typename T>
  using generic_ctor_enabler = std::enable_if_t<!std::is_same<Instruction, uncvref_t<T>>::value, int>;

public:
  template <typename T, generic_ctor_enabler<T> = 0>
  Instruction(T&& instruction)  // NOLINT
    : instruction_(std::make_unique<detail_instruction::InstructionInner<uncvref_t<T>>>(instruction))
  {
  }

  // Destructor
  ~Instruction() = default;

  // Copy constructor
  Instruction(const Instruction& other) : instruction_(other.instruction_->clone()) {}

  // Move ctor.
  Instruction(Instruction&& other) noexcept { instruction_.swap(other.instruction_); }
  // Move assignment.
  Instruction& operator=(Instruction&& other) noexcept
  {
    instruction_.swap(other.instruction_);
    return (*this);
  }

  // Copy assignment.
  Instruction& operator=(const Instruction& other)
  {
    (*this) = Instruction(other);
    return (*this);
  }

  template <typename T, generic_ctor_enabler<T> = 0>
  Instruction& operator=(T&& other)
  {
    (*this) = Instruction(std::forward<T>(other));
    return (*this);
  }

  int getType() const { return instruction_->getType(); }

  const std::string& getDescription() const { return instruction_->getDescription(); }

  void setDescription(const std::string& description) { instruction_->setDescription(description); }

  void print(std::string prefix = "") const { instruction_->print(std::move(prefix)); }

  tinyxml2::XMLElement* toXML(tinyxml2::XMLDocument& doc) const { return instruction_->toXML(doc); }

  template <typename T>
  T* cast()
  {
    return static_cast<T*>(instruction_->recover());
  }

  template <typename T>
  const T* cast_const() const
  {
    return static_cast<const T*>(instruction_->recover());
  }

private:
  std::unique_ptr<detail_instruction::InstructionInnerBase> instruction_;
};

}  // namespace tesseract_planning

#endif  // TESSERACT_COMMAND_LANGUAGE_INSTRUCTION_H
