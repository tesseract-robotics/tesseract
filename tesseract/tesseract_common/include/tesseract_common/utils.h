#ifndef TESSERACT_COMMON_UTILS_H
#define TESSERACT_COMMON_UTILS_H

#include <tesseract_common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <vector>
#include <string>
#include <stdexcept>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

namespace tesseract_common
{
/**
 * @brief Determine if a string is a number
 * @param s The string to evaluate
 * @return True if numeric, otherwise false.
 */
inline bool isNumeric(const std::string& s)
{
  if (s.empty())
    return false;

  try
  {
    std::stod(s);
  }
  catch (const std::invalid_argument& /*ia*/)
  {
    return false;
  }

  return true;
}

inline bool isNumeric(const std::vector<std::string>& sv)
{
  for (const auto& s : sv)
  {
    if (!isNumeric(s))
      return false;
  }

  return true;
}

}  // namespace tesseract_common
#endif  // TESSERACT_COMMON_UTILS_H
