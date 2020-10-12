/**
 * @file status_code.h
 * @brief A status code class
 *
 * @author Levi Armstrong
 * @date January 18, 2018
 * @version TODO
 * @bug No known bugs
 *
 * @copyright Copyright (c) 2017, Southwest Research Institute
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
#ifndef TESSERACT_COMMON_STATUS_CODE_H
#define TESSERACT_COMMON_STATUS_CODE_H

#include <tesseract_common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <string>
#include <functional>
#include <memory>
#include <cassert>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

namespace tesseract_common
{
class StatusCategory
{
public:
  using Ptr = std::shared_ptr<StatusCategory>;
  using ConstPtr = std::shared_ptr<const StatusCategory>;

  constexpr StatusCategory() noexcept = default;
  virtual ~StatusCategory() = default;
  StatusCategory(const StatusCategory&) = delete;
  StatusCategory& operator=(const StatusCategory&) = delete;
  StatusCategory(StatusCategory&&) = delete;
  StatusCategory& operator=(StatusCategory&&) = delete;

  virtual const std::string& name() const noexcept = 0;
  virtual std::string message(int code) const = 0;

  bool operator==(const StatusCategory& rhs) const noexcept
  {
    return std::equal_to<const StatusCategory*>()(this, &rhs);  // NOLINT
  }

  bool operator!=(const StatusCategory& rhs) const noexcept
  {
    return std::not_equal_to<const StatusCategory*>()(this, &rhs);  // NOLINT
  }
};

class GeneralStatusCategory : public StatusCategory
{
public:
  GeneralStatusCategory(std::string name = "GeneralStatus") : name_(std::move(name)) {}
  const std::string& name() const noexcept override { return name_; }
  std::string message(int code) const override
  {
    switch (code)
    {
      case IsConfigured:
      {
        return "Is configured";
      }
      case Success:
      {
        return "Sucessful";
      }
      case Failure:
      {
        return "Failure";
      }
      case IsNotConfigured:
      {
        return "Is not configured";
      }
      default:
      {
        assert(false);
        return "";
      }
    }
  }

  enum
  {
    IsConfigured = 1,
    Success = 0,
    Failure = -1,
    IsNotConfigured = -2
  };

private:
  std::string name_;
};

class StatusCode
{
public:
  using Ptr = std::shared_ptr<StatusCode>;
  using ConstPtr = std::shared_ptr<const StatusCode>;

  StatusCode(StatusCode::ConstPtr child = nullptr)
    : cat_(std::make_shared<GeneralStatusCategory>()), child_(std::move(child))
  {
  }
  StatusCode(int val, StatusCategory::ConstPtr cat, StatusCode::ConstPtr child = nullptr)
    : val_(val), cat_(std::move(cat)), child_(std::move(child))
  {
  }
  ~StatusCode() = default;
  StatusCode(const StatusCode&) = default;
  StatusCode& operator=(const StatusCode&) = default;
  StatusCode(StatusCode&&) = default;
  StatusCode& operator=(StatusCode&&) = default;

  int value() const noexcept { return val_; }
  const StatusCategory::ConstPtr& category() const noexcept { return cat_; }
  std::string message() const
  {
    if (child_ != nullptr)
      return category()->message(value()) + child_->messageIndent("         ");

    return category()->message(value());
  }

  /**
   * @brief Set the child status code
   * @param child The child status code to assign
   */
  void setChild(StatusCode::ConstPtr child) { child_ = std::move(child); }

  /**
   * @brief Get the child status code if it exist
   * @return Child status code, nullptr if it does not have a child.
   */
  const StatusCode::ConstPtr& getChild() const { return child_; }

  /**
   * @brief This return true if status value is greater or equal to zero which is not in an error state
   */
  explicit operator bool() const noexcept { return (value() >= 0); }

  bool operator==(const StatusCode& rhs) noexcept
  {
    return ((this->value() == rhs.value()) && (this->category() == rhs.category()));
  }

  bool operator!=(const StatusCode& rhs) noexcept
  {
    return ((this->value() != rhs.value()) || (this->category() != rhs.category()));
  }

protected:
  std::string messageIndent(const std::string& previous_indent) const
  {
    std::string indent = previous_indent + "  ";
    if (child_ != nullptr)
      return "\n" + indent + category()->message(value()) + child_->messageIndent(indent);

    return "\n" + indent + category()->message(value());
  }

private:
  int val_{ 0 };
  StatusCategory::ConstPtr cat_;
  StatusCode::ConstPtr child_;
};

}  // namespace tesseract_common
#endif  // TESSERACT_COMMON_STATUS_CODE_H
