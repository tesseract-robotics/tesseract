/**
 * @file profile.h
 * @brief This is a profile base class
 *
 * @author Levi Armstrong
 * @date December 2, 2024
 * @version TODO
 * @bug No known bugs
 *
 * @copyright Copyright (c) 2024, Southwest Research Institute
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
#ifndef TESSERACT_COMMON_PROFILE_H
#define TESSERACT_COMMON_PROFILE_H

#include <memory>
#include <boost/serialization/access.hpp>
#include <boost/serialization/export.hpp>

namespace tesseract_common
{
/**
 * @brief The Profile class
 */
class Profile
{
public:
  using Ptr = std::shared_ptr<Profile>;
  using ConstPtr = std::shared_ptr<const Profile>;

  Profile() = default;
  Profile(std::size_t key);
  virtual ~Profile() = default;

  /**
   * @brief Get the hash code associated with the profile
   * @return The profile's hash code
   */
  std::size_t getKey() const;

protected:
  Profile(const Profile&) = default;
  Profile& operator=(const Profile&) = default;
  Profile(Profile&&) = default;
  Profile& operator=(Profile&&) = default;

  std::size_t key_{ 0 };
  friend class boost::serialization::access;
  template <class Archive>
  void serialize(Archive&, const unsigned int);  // NOLINT
};
}  // namespace tesseract_common

BOOST_CLASS_EXPORT_KEY(tesseract_common::Profile)

#endif  // TESSERACT_COMMON_PROFILE_H
