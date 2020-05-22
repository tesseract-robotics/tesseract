#ifndef TESSERACT_COMMAND_LANGUAGE_WAYPOINT_H
#define TESSERACT_COMMAND_LANGUAGE_WAYPOINT_H

#include <tesseract_common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <memory>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

namespace tesseract_planning
{

namespace detail
{

struct WaypointInnerBase
{
  WaypointInnerBase() = default;
  virtual ~WaypointInnerBase() = default;
  WaypointInnerBase(const WaypointInnerBase&) = delete;
  WaypointInnerBase& operator=(const WaypointInnerBase&) = delete;
  WaypointInnerBase(WaypointInnerBase&&) = delete;
  WaypointInnerBase& operator=(WaypointInnerBase&&) = delete;

  virtual int getType() const = 0;

  // This is not required for user defined implementation
  virtual void* recover() = 0;

  // This is not required for user defined implementation
  virtual std::unique_ptr<WaypointInnerBase> clone() const = 0;
};

template <typename T>
struct WaypointInner final : WaypointInnerBase
{
  WaypointInner() = default;
  ~WaypointInner() override = default;
  WaypointInner(const WaypointInner &) = delete;
  WaypointInner(WaypointInner &&) = delete;
  WaypointInner &operator=(const WaypointInner &) = delete;
  WaypointInner &operator=(WaypointInner &&) = delete;

  // Constructors from T (copy and move variants).
  explicit WaypointInner(T waypoint) : waypoint_(std::move(waypoint)) {}
  explicit WaypointInner(T &&waypoint) : waypoint_(std::move(waypoint)) {}

  std::unique_ptr<WaypointInnerBase> clone() const override
  {
    return std::make_unique<WaypointInner>(waypoint_);
  }

  int getType() const { return waypoint_.getType(); }

  void* recover() { return &waypoint_; }

  T waypoint_;
};

}

class Waypoint
{
  template <typename T>
  using uncvref_t = std::remove_cv_t<std::remove_reference_t<T>>;

  // Enable the generic ctor only if ``T`` is not a ForwardKinematics (after removing const/reference qualifiers)
  // If ``T`` is of type ForwardKinematics we disable so it will use the copy or move constructors of this class.
  template <typename T>
  using generic_ctor_enabler = std::enable_if_t<!std::is_same<Waypoint, uncvref_t<T>>::value, int>;

public:
  using Ptr = std::shared_ptr<Waypoint>;
  using ConstPtr = std::shared_ptr<const Waypoint>;

  template <typename T, generic_ctor_enabler<T> = 0>
  Waypoint(T &&waypoint) // NOLINT
    : waypoint_(std::make_unique<detail::WaypointInner<uncvref_t<T>>>(waypoint))
  {
  }

  // Destructor
  ~Waypoint() = default;

  // Copy constructor
  Waypoint(const Waypoint &other) : waypoint_(other.waypoint_->clone()) {}

  // Move ctor.
  Waypoint(Waypoint &&other) noexcept { waypoint_.swap(other.waypoint_); }
  // Move assignment.
  Waypoint &operator=(Waypoint &&other) noexcept { waypoint_.swap(other.waypoint_); return (*this); }

  // Copy assignment.
  Waypoint &operator=(const Waypoint &other)
  {
    (*this) = Waypoint(other);
    return (*this);
  }

  template <typename T, generic_ctor_enabler<T> = 0>
  Waypoint &operator=(T &&other)
  {
    (*this) = Waypoint(std::forward<T>(other));
    return (*this);
  }

  int getType() const { return waypoint_->getType(); }

  template<typename T>
  T* cast() { return static_cast<T*>(waypoint_->recover()); }

  template<typename T>
  const T* cast_const() const { return static_cast<const T*>(waypoint_->recover()); }

private:
  std::unique_ptr<detail::WaypointInnerBase> waypoint_;
};

}

#endif // TESSERACT_COMMAND_LANGUAGE_WAYPOINT_H
