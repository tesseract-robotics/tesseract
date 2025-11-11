#ifndef TESSERACT_COMMON_CEREAL_MAKE_ARRAY_H
#define TESSERACT_COMMON_CEREAL_MAKE_ARRAY_H

#include <cstddef>
#include <type_traits>

#include <cereal/cereal.hpp>          // cereal core
#include <cereal/details/traits.hpp>  // traits::is_text_archive

namespace tesseract_common::serialization
{
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

  // Text archives → element-wise (portable)
  template <class Archive, std::enable_if_t<cereal::traits::is_text_archive<Archive>::value, int> = 0>
  void serialize(Archive& ar)
  {
    for (std::size_t i = 0; i < count_; ++i)
      ar(cereal::make_nvp("item", ptr_[i]));
  }

  // Binary archives + trivially copyable → raw bytes (fast path)
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

  // Binary archives + non-trivial types → element-wise fallback
  template <class Archive,
            std::enable_if_t<!cereal::traits::is_text_archive<Archive>::value &&
                                 !std::is_trivially_copyable<std::remove_const_t<T>>::value,
                             int> = 0>
  void serialize(Archive& ar)
  {
    for (std::size_t i = 0; i < count_; ++i)
      ar(cereal::make_nvp("item", ptr_[i]));
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

  // Text → element-wise
  template <class Archive, std::enable_if_t<cereal::traits::is_text_archive<Archive>::value, int> = 0>
  void serialize(Archive& ar) const
  {
    for (std::size_t i = 0; i < count_; ++i)
      ar(cereal::make_nvp("item", ptr_[i]));
  }

  // Binary + trivially copyable → raw bytes
  template <class Archive,
            std::enable_if_t<!cereal::traits::is_text_archive<Archive>::value && std::is_trivially_copyable<T>::value,
                             int> = 0>
  void save(Archive& ar) const
  {
    ar(cereal::binary_data(ptr_, count_ * sizeof(T)));
  }

  // Binary + non-trivial → element-wise
  template <class Archive,
            std::enable_if_t<!cereal::traits::is_text_archive<Archive>::value && !std::is_trivially_copyable<T>::value,
                             int> = 0>
  void serialize(Archive& ar) const
  {
    for (std::size_t i = 0; i < count_; ++i)
      ar(cereal::make_nvp("item", ptr_[i]));
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
