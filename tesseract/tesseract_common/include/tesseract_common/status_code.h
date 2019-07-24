#ifndef TESSERACT_COMMON_STATUS_CODE_H
#define TESSERACT_COMMON_STATUS_CODE_H

#include <string>
#include <functional>
#include <memory>

namespace tesseract_common
{

class StatusCategory
{
public:
  using Ptr = std::shared_ptr<StatusCategory>;
  using ConstPtr = std::shared_ptr<const StatusCategory>;

  constexpr StatusCategory() noexcept;
  StatusCategory( const StatusCategory& other ) = delete;
  virtual const std::string& name() const noexcept = 0;
  virtual std::string message(int code) const = 0;

  bool operator==( const StatusCategory& rhs ) const noexcept
  {
    return std::equal_to<const StatusCategory*>()(this, &rhs);
  }

  bool operator!=( const StatusCategory& rhs ) const noexcept
  {
    return std::not_equal_to<const StatusCategory*>()(this, &rhs);
  }
};

class GeneralStatusCategory : public StatusCategory
{
public:
  GeneralStatusCategory(std::string name = "GeneralStatus") : name_(name) {}
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
  StatusCode() : val_(0), cat_(std::make_shared<GeneralStatusCategory>()) {}
  StatusCode(int val, StatusCategory::ConstPtr cat) : val_(val), cat_(std::move(cat)) {}
  ~StatusCode() = default;
  int value() const noexcept { return val_; }
  const StatusCategory::ConstPtr& category() const noexcept  { return cat_; }
  std::string message() const { return category()->message(value()); }

  /**
   * @brief This return true if status value is greater or equal to zero which is not in an error state
   */
  explicit operator bool() const noexcept
  {
    return (value() >= 0);
  }

  bool operator==(const StatusCode& rhs) noexcept
  {
    return ((this->value() == rhs.value()) && (this->category() == rhs.category()));
  }

  bool operator!=(const StatusCode& rhs) noexcept
  {
    return ((this->value() != rhs.value()) || (this->category() != rhs.category()));
  }

private:
  int val_;
  StatusCategory::ConstPtr cat_;
};




}
#endif // TESSERACT_COMMON_STATUS_CODE_H
