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

inline std::string trailingSlash(const std::string& path)
{
  std::string ret;
  if (path.empty())
    ret = "/";
  else
  {
    if (path.back() == '/')
      ret = path;
    else
      ret = path + "/";
  }
  return ret;
}

inline std::string noTrailingSlash(const std::string& path)
{
  std::string ret = path;
  while (!ret.empty() && (ret.back() == '/' || ret.back() == '\\'))
  {
    ret = ret.substr(0, ret.size() - 1);
  }
  return ret;
}

inline std::string noLeadingSlash(const std::string& filename)
{
  std::string ret = filename;
  while (!ret.empty() && (ret.front() == '/' || ret.front() == '\\'))
  {
    ret = ret.substr(1);  // from second char to end
  }
  return ret;
}

}  // namespace tesseract_urdf

#endif  // TESSERACT_URDF_UTILS_H
