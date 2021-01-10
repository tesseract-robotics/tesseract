#ifndef TESSEACT_COMMON_SFINAE_UTILS_H
#define TESSEACT_COMMON_SFINAE_UTILS_H

#include <tesseract_common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <type_traits>
#include <array>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

/*
 *  https://stackoverflow.com/questions/257288/templated-check-for-the-existence-of-a-class-member-function
 *
 *   - Multiple inheritance forces ambiguity of member names.
 *   - SFINAE is used to make aliases to member names.
 *   - Expression SFINAE is used in just one generic has_member that can accept
 *     any alias we pass it.
 */

namespace tesseract_common
{
// Variadic to force ambiguity of class members.  C++11 and up.
template <typename... Args>
struct ambiguate : public Args...
{
};

template <typename A, typename = void>
struct got_type : std::false_type
{
};

template <typename A>
struct got_type<A> : std::true_type
{
  using type = A;
};

template <typename T, T>
struct sig_check : std::true_type
{
};

template <typename Alias, typename AmbiguitySeed>
struct has_member
{
  template <typename C>
  static std::array<char, 1>& f(decltype(&C::value));
  template <typename C>
  static std::array<char, 2>& f(...);

  // Make sure the member name is consistently spelled the same.
  static_assert((sizeof(f<AmbiguitySeed>(nullptr)) == 1),
                "Member name specified in AmbiguitySeed is different from member name specified in Alias, or wrong "
                "Alias/AmbiguitySeed has been specified.");

  static bool const value = sizeof(f<Alias>(nullptr)) == 2;
};

/**
 * @brief Check for member x in a given class. Could be var, func, class, union, or enum:
 * @details
 *    CREATE_MEMBER_CHECK(x);
 *    bool has_x = has_member_x<class_to_check_for_x>::value;
 *
 * @note Also useful to add static assert in constructor to provide useful message
 *    static_assert(has_member_x<class_to_check_for_x>::value, "Class does not have member function 'x'");
 */
#define CREATE_MEMBER_CHECK(member)                                                                                    \
                                                                                                                       \
  template <typename T, typename = std::true_type>                                                                     \
  struct Alias_##member;                                                                                               \
                                                                                                                       \
  template <typename T>                                                                                                \
  struct Alias_##member<T, std::integral_constant<bool, tesseract_common::got_type<decltype(&T::member)>::value>>      \
  {                                                                                                                    \
    static const decltype(&T::member) value;                                                                           \
  };                                                                                                                   \
                                                                                                                       \
  struct AmbiguitySeed_##member                                                                                        \
  {                                                                                                                    \
    char member; /* NOLINT */                                                                                          \
  };                                                                                                                   \
                                                                                                                       \
  template <typename T>                                                                                                \
  struct has_member_##member                                                                                           \
  {                                                                                                                    \
    static const bool value =                                                                                          \
        tesseract_common::has_member<Alias_##member<tesseract_common::ambiguate<T, AmbiguitySeed_##member>>,           \
                                     Alias_##member<AmbiguitySeed_##member>>::value;                                   \
  }

/**
 * @brief Check if a member function is invocable with the provided input types
 * @details
 *   Check if the class has a function named add which takes two double as arguments
 *   CREATE_MEMBER_FUNC_INVOCABLE_CHECK(add, double, double)
 *   bool has_add_invocable = has_member_func_invocable_add<class_to_check_for_add_invocable>::value;
 */
#define CREATE_MEMBER_FUNC_INVOCABLE_CHECK(func_name, ...)                                                             \
                                                                                                                       \
  template <typename T, typename = std::true_type>                                                                     \
  struct has_member_func_invocable_##func_name : std::false_type                                                       \
  {                                                                                                                    \
  };                                                                                                                   \
                                                                                                                       \
  template <typename T>                                                                                                \
  struct has_member_func_invocable_##func_name<                                                                        \
      T,                                                                                                               \
      std::integral_constant<bool, std::is_invocable<decltype(&T::func_name), T, __VA_ARGS__>::value>>                 \
    : std::true_type                                                                                                   \
  {                                                                                                                    \
  };

/**
 * @brief Check if a member function has a given return type with input parameters
 * @details
 *   Check if the class has a function named add with return type followed function parameters
 *   CREATE_MEMBER_FUNC_RETURN_TYPE_CHECK(add, double, double, double)
 *   bool has_add_return_type = has_member_func_return_type_add<class_to_check_for_add_return_type>::value;
 */
#define CREATE_MEMBER_FUNC_RETURN_TYPE_CHECK(func_name, return_type, ...)                                              \
                                                                                                                       \
  template <typename T, typename = std::true_type>                                                                     \
  struct has_member_func_return_type_##func_name : std::false_type                                                     \
  {                                                                                                                    \
  };                                                                                                                   \
                                                                                                                       \
  template <typename T>                                                                                                \
  struct has_member_func_return_type_##func_name<                                                                      \
      T,                                                                                                               \
      std::integral_constant<bool,                                                                                     \
                             std::is_same<typename std::invoke_result<decltype(&T::func_name), T, __VA_ARGS__>::type,  \
                                          return_type>::value>> : std::true_type                                       \
  {                                                                                                                    \
  };

/**
 * @brief Check if a member function has a given return type that takes no arguments
 * @details
 *   Check if the class has a function named add with return type
 *   CREATE_MEMBER_FUNC_RETURN_TYPE_NOARGS_CHECK(update, bool)
 *   bool has_update_return_type = has_member_func_return_type_update<class_to_check_for_update_return_type>::value;
 */
#define CREATE_MEMBER_FUNC_RETURN_TYPE_NOARGS_CHECK(func_name, return_type)                                            \
                                                                                                                       \
  template <typename T, typename = std::true_type>                                                                     \
  struct has_member_func_return_type_##func_name : std::false_type                                                     \
  {                                                                                                                    \
  };                                                                                                                   \
                                                                                                                       \
  template <typename T>                                                                                                \
  struct has_member_func_return_type_##func_name<                                                                      \
      T,                                                                                                               \
      std::integral_constant<                                                                                          \
          bool,                                                                                                        \
          std::is_same<typename std::invoke_result<decltype(&T::func_name), T>::type, return_type>::value>>            \
    : std::true_type                                                                                                   \
  {                                                                                                                    \
  };

/**
 * @brief Check if a member function has a given return type
 * @details
 *   Check if the class has a function named add with return type followed function parameters
 *   CREATE_MEMBER_FUNC_SIGNATURE_CHECK(add, double, double, double)
 *   bool has_add_signature = has_member_func_signature_add<class_to_check_for_add_signature>::value;
 */
#define CREATE_MEMBER_FUNC_SIGNATURE_CHECK(func_name, return_type, ...)                                                \
                                                                                                                       \
  template <typename T, typename = std::true_type>                                                                     \
  struct has_member_func_signature_##func_name : std::false_type                                                       \
  {                                                                                                                    \
  };                                                                                                                   \
                                                                                                                       \
  template <typename T>                                                                                                \
  struct has_member_func_signature_##func_name<                                                                        \
      T,                                                                                                               \
      std::integral_constant<                                                                                          \
          bool,                                                                                                        \
          std::is_invocable<decltype(&T::func_name), T, __VA_ARGS__>::value &&                                         \
              std::is_same<typename std::invoke_result<decltype(&T::func_name), T, __VA_ARGS__>::type,                 \
                           return_type>::value>> : std::true_type                                                      \
  {                                                                                                                    \
  };

/**
 * @brief Check if a member function has a given return type that takes no arguments
 * @details
 *   Check if the class has a function named add with return type
 *   CREATE_MEMBER_FUNC_SIGNATURE_NOARGS_CHECK(add, double)
 *   bool has_add_signature = has_member_func_signature_add<class_to_check_for_add_signature>::value;
 */
#define CREATE_MEMBER_FUNC_SIGNATURE_NOARGS_CHECK(func_name, return_type)                                              \
                                                                                                                       \
  template <typename T, typename = std::true_type>                                                                     \
  struct has_member_func_signature_##func_name : std::false_type                                                       \
  {                                                                                                                    \
  };                                                                                                                   \
                                                                                                                       \
  template <typename T>                                                                                                \
  struct has_member_func_signature_##func_name<                                                                        \
      T,                                                                                                               \
      std::integral_constant<                                                                                          \
          bool,                                                                                                        \
          std::is_invocable<decltype(&T::func_name), T>::value &&                                                      \
              std::is_same<typename std::invoke_result<decltype(&T::func_name), T>::type, return_type>::value>>        \
    : std::true_type                                                                                                   \
  {                                                                                                                    \
  };

}  // namespace tesseract_common

#endif  // TESSEACT_COMMON_SFINAE_UTILS_H
