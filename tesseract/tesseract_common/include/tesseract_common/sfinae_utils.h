#ifndef TESSEACT_COMMON_SFINAE_UTILS_H
#define TESSEACT_COMMON_SFINAE_UTILS_H

#include <type_traits>

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
  static char (&f(decltype(&C::value)))[1];
  template <typename C>
  static char (&f(...))[2];

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
}  // namespace tesseract_common

#endif  // TESSEACT_COMMON_SFINAE_UTILS_H
