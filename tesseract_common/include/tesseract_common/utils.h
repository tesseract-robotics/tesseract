/**
 * @file utils.h
 * @brief Common Tesseract Utility Functions
 *
 * @author Levi Armstrong
 * @date January 18, 2018
 * @version TODO
 * @bug No known bugs
 *
 * @copyright Copyright (c) 2017, Southwest Research Institute
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
#ifndef TESSERACT_COMMON_UTILS_H
#define TESSERACT_COMMON_UTILS_H

#include <tesseract_common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <array>
#include <vector>
#include <string>
#include <sstream>
#include <stdexcept>
#include <random>
#include <iomanip>
#include <Eigen/Core>
#include <iostream>
#include <tinyxml2.h>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract_common/allowed_collision_matrix.h>
#include <tesseract_common/types.h>

namespace tesseract_common
{
#if __cplusplus < 201703L
/** @brief Random number generator */
static std::mt19937 mersenne{ static_cast<std::mt19937::result_type>(std::time(nullptr)) };
#else
/** @brief Random number generator */
// NOLINTNEXTLINE(cppcoreguidelines-avoid-non-const-global-variables)
inline std::mt19937 mersenne{ static_cast<std::mt19937::result_type>(std::time(nullptr)) };
#endif

template <typename... Args>
std::string strFormat(const std::string& format, Args... args)
{
  int size_s = std::snprintf(nullptr, 0, format.c_str(), args...) + 1;  // Extra space for '\0'
  if (size_s <= 0)
    throw std::runtime_error("Error during formatting.");

  auto size = static_cast<size_t>(size_s);
  auto buf = std::make_unique<char[]>(size);  // NOLINT
  std::snprintf(buf.get(), size, format.c_str(), args...);
  return std::string{ buf.get(), buf.get() + size - 1 };  // We don't want the '\0' inside
}

/**
 * @brief Change the reference point of the provided Twist
 * @param twist The current Twist which gets modified in place
 * @param ref_point Is expressed in the same base frame of the Twist
 *                  and is a vector from the old point to the new point.
 */
void twistChangeRefPoint(Eigen::Ref<Eigen::VectorXd> twist, const Eigen::Ref<const Eigen::Vector3d>& ref_point);

/**
 * @brief Change the base coordinate system of the Twist
 * @param twist The current Twist which gets modified in place
 * @param change_base The transform from the desired frame to the current base frame of the Twist
 */
void twistChangeBase(Eigen::Ref<Eigen::VectorXd> twist, const Eigen::Isometry3d& change_base);

/**
 * @brief Change the base coordinate system of the jacobian
 * @param jacobian The current Jacobian which gets modified in place
 * @param change_base The transform from the desired frame to the current base frame of the jacobian
 */
void jacobianChangeBase(Eigen::Ref<Eigen::MatrixXd> jacobian, const Eigen::Isometry3d& change_base);

/**
 * @brief Change the reference point of the jacobian
 * @param jacobian The current Jacobian which gets modified in place
 * @param ref_point Is expressed in the same base frame of the jacobian
 *                  and is a vector from the old point to the new point.
 */
void jacobianChangeRefPoint(Eigen::Ref<Eigen::MatrixXd> jacobian, const Eigen::Ref<const Eigen::Vector3d>& ref_point);

/** @brief Concatenate two vector */
Eigen::VectorXd concat(const Eigen::VectorXd& a, const Eigen::VectorXd& b);

/**
 * @brief Calculate the rotation error vector given a rotation error matrix where the angle is between [-pi, pi]
 * @details This should be used only for calculating the error. Do not use for numerically calculating jacobians
 * because it breaks down a -PI and PI
 * @param R rotation error matrix
 * @return Rotation error vector = Eigen::AngleAxisd.axis() * Eigen::AngleAxisd.angle()
 */
Eigen::Vector3d calcRotationalError(const Eigen::Ref<const Eigen::Matrix3d>& R);

/**
 * @brief Calculate the rotation error vector given a rotation error matrix where the angle is between [0, 2 * pi]
 * @details This function does not break down when the angle is near zero or 2pi when calculating the numerical
 * jacobian. This is because when using Eigen's angle axis it converts the angle to be between [0, PI] where internally
 * if the angle is between [-PI, 0] it flips the sign of the axis. Both this function and calcRotationalError both check
 * for this flip and reverts it. Since the angle is always between [-PI, PI], switching the range to [0, PI] will
 * never be close to 2PI. In the case of zero, it also does not break down because we are making sure that the angle
 * axis aligns with the quaternion axis eliminating this issue. As you can see the quaternion keeps the angle small but
 * flips the axis so the correct delta rotation is calculated.
 *
 * Angle: 0.001 results in an axis: [0, 0, 1]
 * Angle: -0.001 results in and axis: [0, 0, -1]
 * e1 = angle * axis = [0, 0, 0.001]
 * e2 = angle * axis = [0, 0, -0.001]
 * delta = e2 - e1 = [0, 0, 0.002]
 *
 * @details This should be used when numerically calculating rotation jacobians
 * @param R rotation error matrix
 * @return Rotation error vector = Eigen::AngleAxisd.axis() * Eigen::AngleAxisd.angle()
 */
Eigen::Vector3d calcRotationalError2(const Eigen::Ref<const Eigen::Matrix3d>& R);

/**
 * @brief Calculate error between two transforms expressed in t1 coordinate system
 * @param t1 Target Transform
 * @param t2 Current Transform
 * @return error [Position, Rotational(Angle Axis)]
 */
Eigen::VectorXd calcTransformError(const Eigen::Isometry3d& t1, const Eigen::Isometry3d& t2);

/**
 * @brief This computes a random color RGBA [0, 1] with alpha set to 1
 * @return A random RGBA color
 */
Eigen::Vector4d computeRandomColor();

/**
 * @brief Print a nested exception
 * @param e The exception to print
 * @param level The exception level which controls the indentation
 */
void printNestedException(const std::exception& e, int level = 0);

/**
 * @brief Get the host temp directory path
 * @return The host temp directory path
 */
std::string getTempPath();

/**
 * @brief Determine if a string is a number
 * @param s The string to evaluate
 * @return True if numeric, otherwise false.
 */
bool isNumeric(const std::string& s);

/**
 * @brief Determine if each string in vector is a number
 * @param sv The vector of strings to evaluate
 * @return True if all strings are numeric, otherwise false.
 */
bool isNumeric(const std::vector<std::string>& sv);

/**
 * @brief Given a set of limits it will generate a vector of random numbers between the limit.
 * @param limits The limits to generated numbers based on.
 * @return A vector of random numbers between the provided limits.
 */
Eigen::VectorXd generateRandomNumber(const Eigen::Ref<const Eigen::MatrixX2d>& limits);

/**
 * @brief Left trim string
 * @param s The string to left trim
 */
void ltrim(std::string& s);

/**
 * @brief Right trim string
 * @param s The string to right trim
 */
void rtrim(std::string& s);

/**
 * @brief Trim left and right of string
 * @param s The string to trim
 */
void trim(std::string& s);

/**
 * @brief Check if two vector of strings are identical
 * @param vec1 Vector strings
 * @param vec2 Vector strings
 * @param ordered If true order is relavent, othwise if false order is not relavent
 * @param equal_pred Binary predicate passed into std::equals to determine if an element is equal. Useful for vectors of
 * pointers
 * @param comp Binary function passed into std::sort. Only used it ordered == false. Useful for vectors of pointers
 */
template <typename T>
bool isIdentical(
    const std::vector<T>& vec1,
    const std::vector<T>& vec2,
    bool ordered = true,
    const std::function<bool(const T&, const T&)>& equal_pred = [](const T& v1, const T& v2) { return v1 == v2; },
    const std::function<bool(const T&, const T&)>& comp = [](const T& v1, const T& v2) { return v1 < v2; })
{
  if (vec1.size() != vec2.size())
    return false;

  if (ordered)
    return std::equal(vec1.begin(), vec1.end(), vec2.begin(), equal_pred);

  std::vector<T> v1 = vec1;
  std::vector<T> v2 = vec2;
  std::sort(v1.begin(), v1.end(), comp);
  std::sort(v2.begin(), v2.end(), comp);
  return std::equal(v1.begin(), v1.end(), v2.begin(), equal_pred);
}

/**
 * @brief Checks if 2 maps are identical
 * @param map_1 First map
 * @param map_2 Second map
 * @return True if they are identical
 */
template <typename KeyValueContainerType, typename ValueType>
bool isIdenticalMap(
    const KeyValueContainerType& map_1,
    const KeyValueContainerType& map_2,
    const std::function<bool(const ValueType&, const ValueType&)>& value_eq =
        [](const ValueType& v1, const ValueType& v2) { return v1 == v2; })
{
  if (map_1.size() != map_2.size())
    return false;

  for (const auto& entry : map_1)
  {
    // Check if the key exists
    const auto cp = map_2.find(entry.first);
    if (cp == map_2.end())
      return false;
    // Check if the value is the same
    if (!value_eq(cp->second, entry.second))
      return false;
  }
  return true;
}

/**
 * @brief Checks if 2 sets are identical
 * @param map_1 First map
 * @param map_2 Second map
 * @return True if they are identical
 */
template <typename ValueType>
bool isIdenticalSet(
    const std::set<ValueType>& set_1,
    const std::set<ValueType>& set_2,
    const std::function<bool(const ValueType&, const ValueType&)>& value_eq =
        [](const ValueType& v1, const ValueType& v2) { return v1 == v2; })
{
  if (set_1.size() != set_2.size())
    return false;

  for (const auto& entry : set_1)
  {
    // Check if the key exists
    const auto cp = set_2.find(entry);
    if (cp == set_2.end())
      return false;
    // Check if the value is the same
    if (!value_eq(*cp, entry))
      return false;
  }
  return true;
}

/**
 * @brief Checks if 2 arrays are identical
 * @param array_1 First array
 * @param array_2 Second array
 * @return True if they are identical
 */
template <typename ValueType, std::size_t Size>
bool isIdenticalArray(
    const std::array<ValueType, Size>& array_1,
    const std::array<ValueType, Size>& array_2,
    const std::function<bool(const ValueType&, const ValueType&)>& value_eq =
        [](const ValueType& v1, const ValueType& v2) { return v1 == v2; })
{
  if (array_1.size() != array_2.size())
    return false;

  for (std::size_t idx = 0; idx < array_1.size(); idx++)
  {
    if (!value_eq(array_1[idx], array_2[idx]))
      return false;
  }
  return true;
}

/**
 * @brief Checks if 2 pointers point to objects that are ==
 * @param p1 First pointer
 * @param p2 Second pointer
 * @return True if the objects are == or both pointers are nullptr
 */
template <typename T>
bool pointersEqual(const std::shared_ptr<T>& p1, const std::shared_ptr<T>& p2)
{
  return (p1 && p2 && *p1 == *p2) || (!p1 && !p2);
}
/**
 * @brief Comparison operator for the objects 2 points point to
 * @param p1 First pointer
 * @param p2 Second pointer
 * @return True if *p1 < *p2 and neither is nullptr. Non-nullptr is considered > than nullptr
 */
template <typename T>
bool pointersComparison(const std::shared_ptr<T>& p1, const std::shared_ptr<T>& p2)
{
  if (p1 && p2)
    return *p1 < *p2;
  // If p2 is !nullptr then return true. p1 or both are nullptr should be false
  return p2 != nullptr;
}
/**
 * @brief Get Timestamp string
 * @return Timestamp string
 */
std::string getTimestampString();

/**
 * @brief Reorder Eigen::VectorXd implace given index list
 * @param v The vector to reorder
 * @param order A vector of index which define the new order
 */
void reorder(Eigen::Ref<Eigen::VectorXd> v, std::vector<Eigen::Index> order);

/**
 * @brief Query a string value from xml element
 * @param xml_element The xml element to query string from
 * @param value The value to update from the xml element
 * @return tinyxml2::XML_SUCCESS if successful, otherwise returns tinyxml2::XML_NO_ATTRIBUTE
 */
tinyxml2::XMLError QueryStringValue(const tinyxml2::XMLElement* xml_element, std::string& value);

/**
 * @brief Query a string Text from xml element
 * @param xml_element The xml element to query string from
 * @param test The value to update from the xml element
 * @return tinyxml2::XML_SUCCESS if successful, otherwise returns tinyxml2::XML_NO_ATTRIBUTE
 */
tinyxml2::XMLError QueryStringText(const tinyxml2::XMLElement* xml_element, std::string& text);

/**
 * @brief Query a string value from xml attribute
 * @param xml_attribute The xml attribute to query string from
 * @param value The value to update from the xml attribute
 * @return tinyxml2::XML_SUCCESS if successful, otherwise returns tinyxml2::XML_WRONG_ATTRIBUTE_TYPE
 */
tinyxml2::XMLError QueryStringValue(const tinyxml2::XMLAttribute* xml_attribute, std::string& value);

/**
 * @brief Query a string attribute from an xml element
 * @param xml_element The xml attribute to query attribute
 * @param name The name of the attribute to query
 * @param value The value to update from the xml attribute
 * @return tinyxml2::XML_SUCCESS if successful, otherwise returns tinyxml2::XML_NO_ATTRIBUTE or
 * tinyxml2::XML_WRONG_ATTRIBUTE_TYPE
 */
tinyxml2::XMLError QueryStringAttribute(const tinyxml2::XMLElement* xml_element, const char* name, std::string& value);

/**
 * @brief Get string attribute if exist. If it does not exist it returns the default value.
 * @param xml_element The xml element to attempt to parse attribute
 * @param name The name of the attribute
 * @param default_value The default value to return if attribute does not exist
 * @return Parsed string attribute or default value if attribute does not exist.
 */
std::string StringAttribute(const tinyxml2::XMLElement* xml_element, const char* name, std::string default_value);

/**
 * @brief Query a string attribute from an xml element and print error log
 *
 * This is the same QueryStringAttribute but it will print out error messages for the failure conditions so the user
 * only needs to check for the tinyxml2::XML_SUCCESS since it is a required attribute.
 *
 * @param xml_element The xml attribute to query attribute
 * @param name The name of the attribute to query
 * @param value The value to update from the xml attribute
 * @return tinyxml2::XML_SUCCESS if successful, otherwise returns tinyxml2::XML_NO_ATTRIBUTE or
 * tinyxml2::XML_WRONG_ATTRIBUTE_TYPE
 */
tinyxml2::XMLError QueryStringAttributeRequired(const tinyxml2::XMLElement* xml_element,
                                                const char* name,
                                                std::string& value);

/**
 * @brief Query a double attribute from an xml element and print error log
 *
 * This is the same QueryDoubleAttribute but it will print out error messages for the failure conditions so the user
 * only needs to check for the tinyxml2::XML_SUCCESS since it is a required attribute.
 *
 * @param xml_element The xml attribute to query attribute
 * @param name The name of the attribute to query
 * @param value The value to update from the xml attribute
 * @return tinyxml2::XML_SUCCESS if successful, otherwise returns tinyxml2::XML_NO_ATTRIBUTE or
 * tinyxml2::XML_WRONG_ATTRIBUTE_TYPE
 */
tinyxml2::XMLError QueryDoubleAttributeRequired(const tinyxml2::XMLElement* xml_element,
                                                const char* name,
                                                double& value);

/**
 * @brief Query a int attribute from an xml element and print error log
 *
 * This is the same QueryIntAttribute but it will print out error messages for the failure conditions so the user
 * only needs to check for the tinyxml2::XML_SUCCESS since it is a required attribute.
 *
 * @param xml_element The xml attribute to query attribute
 * @param name The name of the attribute to query
 * @param value The value to update from the xml attribute
 * @return tinyxml2::XML_SUCCESS if successful, otherwise returns tinyxml2::XML_NO_ATTRIBUTE or
 * tinyxml2::XML_WRONG_ATTRIBUTE_TYPE
 */
tinyxml2::XMLError QueryIntAttributeRequired(const tinyxml2::XMLElement* xml_element, const char* name, int& value);

/**
 * @brief Check if two double are relatively equal
 * @details The max_diff is for handling when comparing numbers near zero
 * @param a Double
 * @param b Double
 * @param max_diff The max diff when comparing std::abs(a - b) <= max_diff, if true they are considered equal
 * @param max_rel_diff The max relative diff std::abs(a - b) <= largest * max_rel_diff, if true considered equal. The
 * largest is the largest of the absolute values of a and b.
 * @return True if they are relatively equal, otherwise false.
 */
bool almostEqualRelativeAndAbs(double a,
                               double b,
                               double max_diff = 1e-6,
                               double max_rel_diff = std::numeric_limits<double>::epsilon());

/**
 * @brief Check if two Eigen VectorXd are relatively and absolute equal
 * @details
 *    This is a vectorized implementation of the function above.
 *    The max_diff is for handling when comparing numbers near zero
 * @param a A vector of double
 * @param b A vector of double
 * @param max_diff The max diff when comparing max(abs(a - b)) <= max_diff, if true they are considered equal
 * @param max_rel_diff The max relative diff abs(a - b) <= largest * max_rel_diff, if true considered equal. The
 * largest is the largest of the absolute values of a and b.
 * @return True if they are relatively equal, otherwise false.
 */
bool almostEqualRelativeAndAbs(const Eigen::Ref<const Eigen::VectorXd>& v1,
                               const Eigen::Ref<const Eigen::VectorXd>& v2,
                               double max_diff = 1e-6,
                               double max_rel_diff = std::numeric_limits<double>::epsilon());

/**
 * @brief Check if two Eigen VectorXd are relatively and absolute equal
 * @details
 *    This is a vectorized implementation of the function above.
 *    The max_diff is for handling when comparing numbers near zero
 * @param a A vector of double
 * @param b A vector of double
 * @param max_diff The max diff when comparing max(abs(a - b)) <= max_diff, if true they are considered equal
 * @param max_rel_diff The max relative diff abs(a - b) <= largest * max_rel_diff, if true considered equal. The
 * largest is the largest of the absolute values of a and b.
 * @return True if they are relatively equal, otherwise false.
 */
bool almostEqualRelativeAndAbs(const Eigen::Ref<const Eigen::VectorXd>& v1,
                               const Eigen::Ref<const Eigen::VectorXd>& v2,
                               const Eigen::Ref<const Eigen::VectorXd>& max_diff,
                               const Eigen::Ref<const Eigen::VectorXd>& max_rel_diff);

/**
 * @brief Convert a string to a numeric value type
 * @param s The string to be converted
 * @param value The value to be loaded with converted string
 * @return True if successful, otherwise false
 */
template <typename FloatType>
bool toNumeric(const std::string& s, FloatType& value)
{
  if (s.empty())
    return false;

  std::stringstream ss;
  ss.imbue(std::locale::classic());

  ss << s;

  FloatType out;
  ss >> out;

  if (ss.fail() || !ss.eof())
    return false;

  value = out;
  return true;
}

/**
 * @brief Gets allowed collisions for a set of link names.
 * @param link_names Vector of link names for which we want the allowed collisions
 * @param acm_entries Entries in the ACM. Get this with AllowedCollisionMatrix::getAllAllowedCollisions()
 * @param remove_duplicates If true, duplicates will be removed. Default: true
 * @return vector of links that are allowed to collide with links given
 */
std::vector<std::string> getAllowedCollisions(const std::vector<std::string>& link_names,
                                              const AllowedCollisionEntries& acm_entries,
                                              bool remove_duplicates = true);

}  // namespace tesseract_common
#endif  // TESSERACT_COMMON_UTILS_H
