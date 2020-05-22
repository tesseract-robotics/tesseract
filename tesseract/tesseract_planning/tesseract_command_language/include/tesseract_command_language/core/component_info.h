#ifndef TESSERACT_COMMAND_LANGUAGE_COMPONENT_INFO_H
#define TESSERACT_COMMAND_LANGUAGE_COMPONENT_INFO_H

#include <memory>

namespace tesseract_planning
{
  namespace detail
  {

  struct ComponentInfoInnerBase
  {
    ComponentInfoInnerBase() = default;
    virtual ~ComponentInfoInnerBase() = default;
    ComponentInfoInnerBase(const ComponentInfoInnerBase&) = delete;
    ComponentInfoInnerBase& operator=(const ComponentInfoInnerBase&) = delete;
    ComponentInfoInnerBase(ComponentInfoInnerBase&&) = delete;
    ComponentInfoInnerBase& operator=(ComponentInfoInnerBase&&) = delete;

    virtual int getType() const = 0;

    /** @brief This is to allow a mask for different planners */
    virtual int getMask() const = 0;

    virtual const std::string& getName() const = 0;

    virtual bool isCompositeInstructionSupported() const = 0;

    // This is not required for user defined implementation
    virtual void* recover() = 0;

    // This is not required for user defined implementation
    virtual std::unique_ptr<ComponentInfoInnerBase> clone() const = 0;
  };

  template <typename T>
  struct ComponentInfoInner final : ComponentInfoInnerBase
  {
    ComponentInfoInner() = default;
    ~ComponentInfoInner() override = default;
    ComponentInfoInner(const ComponentInfoInner &) = delete;
    ComponentInfoInner(ComponentInfoInner &&) = delete;
    ComponentInfoInner &operator=(const ComponentInfoInner &) = delete;
    ComponentInfoInner &operator=(ComponentInfoInner &&) = delete;

    // Constructors from T (copy and move variants).
    explicit ComponentInfoInner(T component_info) : component_info_(std::move(component_info)) {}
    explicit ComponentInfoInner(T &&component_info) : component_info_(std::move(component_info)) {}

    std::unique_ptr<ComponentInfoInnerBase> clone() const override
    {
      return std::make_unique<ComponentInfoInner>(component_info_);
    }

    int getType() const override { return component_info_.getType(); }

    int getMask() const override { return component_info_.getMask(); }

    const std::string& getName() const override { return component_info_.getName(); }

    bool isCompositeInstructionSupported() const override { return component_info_.isCompositeInstructionSupported(); }

    void* recover() override { return &component_info_; }

    T component_info_;
  };

  }

  class ComponentInfo
  {
    template <typename T>
    using uncvref_t = std::remove_cv_t<std::remove_reference_t<T>>;

    // Enable the generic ctor only if ``T`` is not a ForwardKinematics (after removing const/reference qualifiers)
    // If ``T`` is of type ForwardKinematics we disable so it will use the copy or move constructors of this class.
    template <typename T>
    using generic_ctor_enabler = std::enable_if_t<!std::is_same<ComponentInfo, uncvref_t<T>>::value, int>;

  public:
    using Ptr = std::shared_ptr<ComponentInfo>;
    using ConstPtr = std::shared_ptr<const ComponentInfo>;

    template <typename T, generic_ctor_enabler<T> = 0>
    ComponentInfo(T &&component_info) // NOLINT
      : component_info_(std::make_unique<detail::ComponentInfoInner<uncvref_t<T>>>(component_info))
    {
    }

    // Destructor
    ~ComponentInfo() = default;

    // Copy constructor
    ComponentInfo(const ComponentInfo &other) : component_info_(other.component_info_->clone()) {}

    // Move ctor.
    ComponentInfo(ComponentInfo &&other) noexcept { component_info_.swap(other.component_info_); }
    // Move assignment.
    ComponentInfo &operator=(ComponentInfo &&other) noexcept { component_info_.swap(other.component_info_); return (*this); }

    // Copy assignment.
    ComponentInfo &operator=(const ComponentInfo &other)
    {
      (*this) = ComponentInfo(other);
      return (*this);
    }

    template <typename T, generic_ctor_enabler<T> = 0>
    ComponentInfo &operator=(T &&other)
    {
      (*this) = ComponentInfo(std::forward<T>(other));
      return (*this);
    }

    int getType() const { return component_info_->getType(); }

    int getMask() const { return component_info_->getMask(); }

    const std::string& getName() const { return component_info_->getName(); }

    bool isCompositeInstructionSupported() const { return component_info_->isCompositeInstructionSupported(); }

    template<typename T>
    T* cast() { return static_cast<T*>(component_info_->recover()); }

    template<typename T>
    const T* cast_const() const { return static_cast<const T*>(component_info_->recover()); }

  private:
    std::unique_ptr<detail::ComponentInfoInnerBase> component_info_;
  };
}

#endif // TESSERACT_COMMAND_LANGUAGE_COMPONENT_INFO_H
