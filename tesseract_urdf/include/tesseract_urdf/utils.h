#ifndef TESSERACT_URDF_UTILS_H
#define TESSERACT_URDF_UTILS_H

#include <tesseract_common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <sstream>
#include <string>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

namespace tesseract_urdf
{

template <typename T>
inline std::string toString(const T& float_value, const int precision = 3)
{
  std::stringstream sstring;
  sstring.precision(precision);
  sstring << float_value;
  return sstring.str();
}

}  // namespace tesseract_urdf

#endif  // TESSERACT_URDF_UTILS_H
