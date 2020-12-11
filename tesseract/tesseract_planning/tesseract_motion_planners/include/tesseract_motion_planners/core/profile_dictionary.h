/**
 * @file profile_dictionary.h
 * @brief This is a profile dictionary for storing all profiles
 *
 * @author Levi Armstrong
 * @date December 2, 2020
 * @version TODO
 * @bug No known bugs
 *
 * @copyright Copyright (c) 2020, Southwest Research Institute
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
#ifndef TESSERACT_MOTION_PLANNERS_PROFILE_DICTIONARY_H
#define TESSERACT_MOTION_PLANNERS_PROFILE_DICTIONARY_H

#include <tesseract_common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <any>
#include <iostream>
#include <typeindex>
#include <unordered_map>
#include <memory>
#include <shared_mutex>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

namespace tesseract_planning
{
/**
 * @brief This class is used to store profiles for motion planning and process planning
 * @details This is a thread safe class
 *    A ProfileEntry<T> is a std::unordered_map<std::string, std::shared_ptr<const T>>
 *      - The key is the profile name
 *      - Where std::shared_ptr<const T> is the profile
 *    The ProfleEntry<T> is also stored in std::unordered_map where the key here is the std::type_index(typeid(T))
 * @note When adding a profile entry the T should be the base class type.
 */
class ProfileDictionary
{
public:
  using Ptr = std::shared_ptr<ProfileDictionary>;
  using ConstPtr = std::shared_ptr<const ProfileDictionary>;

  /**
   * @brief Check if a profile entry exists
   * @return True if exists, otherwise false
   */
  template <typename ProfileType>
  bool hasProfileEntry() const
  {
    std::shared_lock lock(mutex_);
    return (profiles_.find(std::type_index(typeid(ProfileType))) != profiles_.end());
  }

  /** @brief Remove a profile entry */
  template <typename ProfileType>
  void removeProfileEntry()
  {
    std::unique_lock lock(mutex_);
    profiles_.erase(std::type_index(typeid(ProfileType)));
  }

  /**
   * @brief Get a profile entry
   * @return The profile map associated with the profile entry
   */
  template <typename ProfileType>
  std::unordered_map<std::string, std::shared_ptr<const ProfileType>> getProfileEntry() const
  {
    std::shared_lock lock(mutex_);
    auto it = profiles_.find(std::type_index(typeid(ProfileType)));
    if (it != profiles_.end())
      return std::any_cast<const std::unordered_map<std::string, std::shared_ptr<const ProfileType>>&>(it->second);

    throw std::runtime_error("Profile entry does not exist for type name " +
                             std::string(std::type_index(typeid(ProfileType)).name()));
  }

  /**
   * @brief Add a profile
   * @details If the profile entry does not exist it will create one
   * @param profile_name The profile name
   * @param profile The profile to add
   */
  template <typename ProfileType>
  void addProfile(const std::string& profile_name, std::shared_ptr<const ProfileType> profile)
  {
    if (profile_name.empty())
      throw std::runtime_error("Adding profile with an empty string as the key!");

    if (profile == nullptr)
      throw std::runtime_error("Adding profile that is a nullptr");

    std::unique_lock lock(mutex_);
    auto it = profiles_.find(std::type_index(typeid(ProfileType)));
    if (it != profiles_.end())
    {
      std::any_cast<std::unordered_map<std::string, std::shared_ptr<const ProfileType>>&>(it->second)[profile_name] =
          profile;
    }
    else
    {
      std::unordered_map<std::string, std::shared_ptr<const ProfileType>> new_entry;
      new_entry[profile_name] = profile;
      profiles_[std::type_index(typeid(ProfileType))] = new_entry;
    }
  }

  /**
   * @brief Check if a profile exists
   * @details If profile entry does not exist it also returns false
   * @return True if profile exists, otherwise false
   */
  template <typename ProfileType>
  bool hasProfile(const std::string& profile_name) const
  {
    std::shared_lock lock(mutex_);
    auto it = profiles_.find(std::type_index(typeid(ProfileType)));
    if (it != profiles_.end())
    {
      const auto& profile_map =
          std::any_cast<const std::unordered_map<std::string, std::shared_ptr<const ProfileType>>&>(it->second);
      auto it2 = profile_map.find(profile_name);
      if (it2 != profile_map.end())
        return true;
    }
    return false;
  }

  /**
   * @brief Get a profile by name
   * @details Check if the profile exist before calling this function, if missing an exception is thrown
   * @param profile_name The profile name
   * @return The profile
   */
  template <typename ProfileType>
  std::shared_ptr<const ProfileType> getProfile(const std::string& profile_name) const
  {
    std::shared_lock lock(mutex_);
    auto it = profiles_.at(std::type_index(typeid(ProfileType)));
    const auto& profile_map =
        std::any_cast<const std::unordered_map<std::string, std::shared_ptr<const ProfileType>>&>(it);
    return profile_map.at(profile_name);
  }

  /**
   * @brief Remove a profile
   * @param profile_name The profile to be removed
   */
  template <typename ProfileType>
  void removeProfile(const std::string& profile_name)
  {
    std::unique_lock lock(mutex_);
    auto it = profiles_.find(std::type_index(typeid(ProfileType)));
    if (it != profiles_.end())
      std::any_cast<std::unordered_map<std::string, std::shared_ptr<const ProfileType>>&>(it->second)
          .erase(profile_name);
  }

protected:
  std::unordered_map<std::type_index, std::any> profiles_;
  mutable std::shared_mutex mutex_;
};
}  // namespace tesseract_planning
#endif  // TESSERACT_MOTION_PLANNERS_PROFILE_DICTIONARY_H
