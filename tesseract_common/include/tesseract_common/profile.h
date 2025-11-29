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
#include <typeindex>

namespace tesseract_common
{
class Profile;

template <class Archive>
void serialize(Archive& ar, Profile& obj);

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

  template <typename KeyType>
  static std::size_t createKey()
  {
    return std::type_index(typeid(KeyType)).hash_code();
  }

protected:
  Profile(const Profile&) = default;
  Profile& operator=(const Profile&) = default;
  Profile(Profile&&) = default;
  Profile& operator=(Profile&&) = default;

  std::size_t key_{ 0 };

private:
  template <class Archive>
  friend void ::tesseract_common::serialize(Archive& ar, Profile& obj);
};
}  // namespace tesseract_common

#endif  // TESSERACT_COMMON_PROFILE_H
