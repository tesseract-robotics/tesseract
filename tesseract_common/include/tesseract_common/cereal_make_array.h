#ifndef TESSERACT_COMMON_CEREAL_MAKE_ARRAY_H
#define TESSERACT_COMMON_CEREAL_MAKE_ARRAY_H

#include <cstddef>
#include <type_traits>
#include <stdexcept>
#include <cmath>  // std::isfinite, std::fpclassify

#include <cereal/cereal.hpp>
#include <cereal/details/traits.hpp>

#ifndef TESSERACT_CEREAL_SANITIZE_DENORMALS
#define TESSERACT_CEREAL_SANITIZE_DENORMALS 1
#endif

namespace tesseract_common::serialization
{
// -------- helpers --------
namespace detail
{
template <typename U>
inline U sanitize_for_text_io(U x)
{
#if TESSERACT_CEREAL_SANITIZE_DENORMALS
  if constexpr (std::is_floating_point<U>::value)
  {
    // Collapse NaN/Inf/subnormals to 0 to avoid fragile string roundtrips
    if (!std::isfinite(x) || std::fpclassify(x) == FP_SUBNORMAL)
      return U(0);
  }
#endif
  return x;
}
}  // namespace detail

// Forward decl
template <class T>
class array_wrapper;

// ----------------------
// Non-const specialization
// ----------------------
template <class T>
class array_wrapper
{
public:
  using value_type = T;

  array_wrapper(T* ptr, std::size_t count) : ptr_(ptr), count_(count) {}

  T* address() const noexcept { return ptr_; }
  std::size_t count() const noexcept { return count_; }

  // -------- Text archives (JSON/XML) → proper arrays with sanitization --------
  template <class Archive, std::enable_if_t<cereal::traits::is_text_archive<Archive>::value, int> = 0>
  void save(Archive& ar) const
  {
    auto n = static_cast<cereal::size_type>(count_);
    ar(cereal::make_size_tag(n));
    for (std::size_t i = 0; i < count_; ++i)
      ar(detail::sanitize_for_text_io(ptr_[i]));  // unnamed array entries
  }

  template <class Archive, std::enable_if_t<cereal::traits::is_text_archive<Archive>::value, int> = 0>
  void load(Archive& ar)
  {
    cereal::size_type n{};
    ar(cereal::make_size_tag(n));
    if (static_cast<std::size_t>(n) != count_)
      throw std::runtime_error("array_wrapper: size mismatch when loading text archive");

    for (std::size_t i = 0; i < count_; ++i)
    {
      T tmp{};
      ar(tmp);
      ptr_[i] = detail::sanitize_for_text_io(tmp);
    }
  }

  // -------- Binary archives + trivially copyable → raw bytes (fast path) --------
  template <class Archive,
            std::enable_if_t<!cereal::traits::is_text_archive<Archive>::value &&
                                 std::is_trivially_copyable<std::remove_const_t<T>>::value,
                             int> = 0>
  void save(Archive& ar) const
  {
    ar(cereal::binary_data(ptr_, count_ * sizeof(T)));
  }

  template <class Archive,
            std::enable_if_t<!cereal::traits::is_text_archive<Archive>::value &&
                                 std::is_trivially_copyable<std::remove_const_t<T>>::value,
                             int> = 0>
  void load(Archive& ar)
  {
    ar(cereal::binary_data(ptr_, count_ * sizeof(T)));
  }

  // -------- Binary archives + non-trivial types → element-wise fallback --------
  template <class Archive,
            std::enable_if_t<!cereal::traits::is_text_archive<Archive>::value &&
                                 !std::is_trivially_copyable<std::remove_const_t<T>>::value,
                             int> = 0>
  void serialize(Archive& ar)
  {
    for (std::size_t i = 0; i < count_; ++i)
      ar(ptr_[i]);
  }

private:
  T* ptr_;
  std::size_t count_;
};

// ------------------
// Const specialization
// ------------------
template <class T>
class array_wrapper<const T>
{
public:
  using value_type = const T;

  array_wrapper(const T* ptr, std::size_t count) : ptr_(ptr), count_(count) {}

  const T* address() const noexcept { return ptr_; }
  std::size_t count() const noexcept { return count_; }

  // -------- Text archives (JSON/XML) → proper arrays with sanitization (save-only) --------
  template <class Archive, std::enable_if_t<cereal::traits::is_text_archive<Archive>::value, int> = 0>
  void save(Archive& ar) const
  {
    auto n = static_cast<cereal::size_type>(count_);
    ar(cereal::make_size_tag(n));
    for (std::size_t i = 0; i < count_; ++i)
      ar(detail::sanitize_for_text_io(ptr_[i]));
  }

  // -------- Binary archives + trivially copyable → raw bytes (save-only) --------
  template <class Archive,
            std::enable_if_t<!cereal::traits::is_text_archive<Archive>::value && std::is_trivially_copyable<T>::value,
                             int> = 0>
  void save(Archive& ar) const
  {
    ar(cereal::binary_data(ptr_, count_ * sizeof(T)));
  }

  // -------- Binary archives + non-trivial types → element-wise (save-only) --------
  template <class Archive,
            std::enable_if_t<!cereal::traits::is_text_archive<Archive>::value && !std::is_trivially_copyable<T>::value,
                             int> = 0>
  void save(Archive& ar) const
  {
    for (std::size_t i = 0; i < count_; ++i)
      ar(ptr_[i]);
  }

private:
  const T* ptr_;
  std::size_t count_;
};

// ----------------------
// Helper: make_array()
// ----------------------
template <class T, class Size>
inline array_wrapper<T> make_array(T* ptr, Size count)
{
  return array_wrapper<T>(ptr, static_cast<std::size_t>(count));
}

template <class T, class Size>
inline array_wrapper<const T> make_array(const T* ptr, Size count)
{
  return array_wrapper<const T>(ptr, static_cast<std::size_t>(count));
}

}  // namespace tesseract_common::serialization

#endif  // TESSERACT_COMMON_CEREAL_MAKE_ARRAY_H
