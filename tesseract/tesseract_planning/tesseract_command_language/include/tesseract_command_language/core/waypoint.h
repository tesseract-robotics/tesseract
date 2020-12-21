/**
 * @file waypoint.h
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
#ifndef TESSERACT_COMMAND_LANGUAGE_WAYPOINT_H
#define TESSERACT_COMMAND_LANGUAGE_WAYPOINT_H

#include <tesseract_common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <memory>
#include <string>
#include <tinyxml2.h>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract_common/sfinae_utils.h>

#ifdef SWIG
//%template(Waypoints) std::vector<tesseract_planning::Waypoint>;
#endif  // SWIG

namespace tesseract_planning
{
#ifndef SWIG
namespace detail_waypoint
{
CREATE_MEMBER_CHECK(getType);
CREATE_MEMBER_CHECK(print);
CREATE_MEMBER_CHECK(toXML);
CREATE_MEMBER_FUNC_SIGNATURE_NOARGS_CHECK(getType, int);
CREATE_MEMBER_FUNC_SIGNATURE_CHECK(print, void, std::string);
CREATE_MEMBER_FUNC_SIGNATURE_CHECK(toXML, tinyxml2::XMLElement*, tinyxml2::XMLDocument&);

struct WaypointInnerBase
{
  WaypointInnerBase() = default;
  virtual ~WaypointInnerBase() = default;
  WaypointInnerBase(const WaypointInnerBase&) = delete;
  WaypointInnerBase& operator=(const WaypointInnerBase&) = delete;
  WaypointInnerBase(WaypointInnerBase&&) = delete;
  WaypointInnerBase& operator=(WaypointInnerBase&&) = delete;

  virtual int getType() const = 0;

  virtual void print(const std::string& prefix) const = 0;

  virtual tinyxml2::XMLElement* toXML(tinyxml2::XMLDocument& doc) const = 0;

  // This is not required for user defined implementation
  virtual void* recover() = 0;

  // This is not required for user defined implementation
  virtual std::unique_ptr<WaypointInnerBase> clone() const = 0;
};

template <typename T>
struct WaypointInner final : WaypointInnerBase
{
  WaypointInner()
  {
    static_assert(has_member_getType<T>::value, "Class does not have member function 'getType'");
    static_assert(has_member_print<T>::value, "Class does not have member function 'print'");
    static_assert(has_member_toXML<T>::value, "Class does not have member function 'toXML'");
    static_assert(has_member_func_signature_getType<T>::value, "Class 'getType' function has incorrect signature");
    static_assert(has_member_func_signature_print<T>::value, "Class 'print' function has incorrect signature");
    static_assert(has_member_func_signature_toXML<T>::value, "Class 'toXML' function has incorrect signature");
  }
  ~WaypointInner() override = default;
  WaypointInner(const WaypointInner&) = delete;
  WaypointInner(WaypointInner&&) = delete;
  WaypointInner& operator=(const WaypointInner&) = delete;
  WaypointInner& operator=(WaypointInner&&) = delete;

  // Constructors from T (copy and move variants).
  explicit WaypointInner(T waypoint) : waypoint_(std::move(waypoint))
  {
    static_assert(has_member_getType<T>::value, "Class does not have member function 'getType'");
    static_assert(has_member_print<T>::value, "Class does not have member function 'print'");
    static_assert(has_member_toXML<T>::value, "Class does not have member function 'toXML'");
    static_assert(has_member_func_signature_getType<T>::value, "Class 'getType' function has incorrect signature");
    static_assert(has_member_func_signature_print<T>::value, "Class 'print' function has incorrect signature");
    static_assert(has_member_func_signature_toXML<T>::value, "Class 'toXML' function has incorrect signature");
  }
  explicit WaypointInner(T&& waypoint) : waypoint_(std::move(waypoint))
  {
    static_assert(has_member_getType<T>::value, "Class does not have member function 'getType'");
    static_assert(has_member_print<T>::value, "Class does not have member function 'print'");
    static_assert(has_member_toXML<T>::value, "Class does not have member function 'toXML'");
    static_assert(has_member_func_signature_getType<T>::value, "Class 'getType' function has incorrect signature");
    static_assert(has_member_func_signature_print<T>::value, "Class 'print' function has incorrect signature");
    static_assert(has_member_func_signature_toXML<T>::value, "Class 'toXML' function has incorrect signature");
  }

  std::unique_ptr<WaypointInnerBase> clone() const override { return std::make_unique<WaypointInner>(waypoint_); }

  int getType() const final { return waypoint_.getType(); }

  void print(const std::string& prefix) const final { waypoint_.print(prefix); }

  tinyxml2::XMLElement* toXML(tinyxml2::XMLDocument& doc) const final { return waypoint_.toXML(doc); }

  void* recover() final { return &waypoint_; }

  T waypoint_;
};

}  // namespace detail_waypoint
#endif  // SWIG

class Waypoint
{
  template <typename T>
  using uncvref_t = std::remove_cv_t<typename std::remove_reference<T>::type>;

  // Enable the generic ctor only if ``T`` is not a ForwardKinematics (after removing const/reference qualifiers)
  // If ``T`` is of type ForwardKinematics we disable so it will use the copy or move constructors of this class.
  template <typename T>
  using generic_ctor_enabler = std::enable_if_t<!std::is_same<Waypoint, uncvref_t<T>>::value, int>;

public:
  template <typename T, generic_ctor_enabler<T> = 0>
  Waypoint(T&& waypoint)  // NOLINT
    : waypoint_(std::make_unique<detail_waypoint::WaypointInner<uncvref_t<T>>>(waypoint))
  {
  }

  // Destructor
  ~Waypoint() = default;

  // Copy constructor
  Waypoint(const Waypoint& other) : waypoint_(other.waypoint_->clone()) {}

  // Move ctor.
  Waypoint(Waypoint&& other) noexcept { waypoint_.swap(other.waypoint_); }
  // Move assignment.
  Waypoint& operator=(Waypoint&& other) noexcept
  {
    waypoint_.swap(other.waypoint_);
    return (*this);
  }

  // Copy assignment.
  Waypoint& operator=(const Waypoint& other)
  {
    (*this) = Waypoint(other);
    return (*this);
  }

  template <typename T, generic_ctor_enabler<T> = 0>
  Waypoint& operator=(T&& other)
  {
    (*this) = Waypoint(std::forward<T>(other));
    return (*this);
  }

  int getType() const { return waypoint_->getType(); }

  void print(const std::string& prefix = "") const { waypoint_->print(prefix); }

  tinyxml2::XMLElement* toXML(tinyxml2::XMLDocument& doc) const { return waypoint_->toXML(doc); }

  template <typename T>
  T* cast()
  {
    return static_cast<T*>(waypoint_->recover());
  }

  template <typename T>
  const T* cast_const() const
  {
    return static_cast<const T*>(waypoint_->recover());
  }

private:
  std::unique_ptr<detail_waypoint::WaypointInnerBase> waypoint_;
};

}  // namespace tesseract_planning

#endif  // TESSERACT_COMMAND_LANGUAGE_WAYPOINT_H
