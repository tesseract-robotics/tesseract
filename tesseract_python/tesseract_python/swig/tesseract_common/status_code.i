/**
 * @file status_code.i
 * @brief SWIG interface file for tesseract_common/status_code.h
 *
 * @author John Wason
 * @date December 10, 2019
 * @version TODO
 * @bug No known bugs
 *
 * @copyright Copyright (c) 2019, Wason Technology, LLC
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

%{
#include <tesseract_common/status_code.h>
%}

%shared_ptr(tesseract_common::StatusCategory)
%shared_ptr(tesseract_common::GeneralStatusCategory)
%shared_ptr(tesseract_common::StatusCode)

namespace tesseract_common
{
class StatusCategory
{
public:
  using Ptr = std::shared_ptr<StatusCategory>;
  using ConstPtr = std::shared_ptr<const StatusCategory>;

  constexpr StatusCategory() noexcept;
  StatusCategory(const StatusCategory& other) = delete;
  virtual ~StatusCategory() = default;

  virtual const std::string& name() const noexcept = 0;
  virtual std::string message(int code) const = 0;

  bool operator==(const StatusCategory& rhs) const noexcept;

  bool operator!=(const StatusCategory& rhs) const noexcept;
};

class GeneralStatusCategory : public StatusCategory
{
public:
  GeneralStatusCategory(std::string name = "GeneralStatus");
  const std::string& name() const noexcept override;
  std::string message(int code) const override;

  enum
  {
    IsConfigured = 1,
    Success = 0,
    Failure = -1,
    IsNotConfigured = -2
  };
};

class StatusCode
{
public:
  using Ptr = std::shared_ptr<StatusCode>;
  using ConstPtr = std::shared_ptr<const StatusCode>;

  StatusCode(StatusCode::ConstPtr child = nullptr);
  StatusCode(int val, StatusCategory::ConstPtr cat, StatusCode::ConstPtr child = nullptr);
  ~StatusCode() = default;
  int value() const noexcept;
  const StatusCategory::ConstPtr& category() const noexcept;
  std::string message() const;
  void setChild(StatusCode::ConstPtr child);
  const StatusCode::ConstPtr& getChild() const;
  explicit operator bool() const noexcept;
  bool operator==(const StatusCode& rhs) noexcept;
  bool operator!=(const StatusCode& rhs) noexcept;
};

}  // namespace tesseract_common