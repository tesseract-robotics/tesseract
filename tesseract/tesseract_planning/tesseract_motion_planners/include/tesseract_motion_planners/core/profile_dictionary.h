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
TESSERACT_COMMON_IGNORE_WARNINGS_POP

namespace tesseract_planning
{
/**
 * @brief This class is used to store profiles for motion planning and process planning
 * @details
 *    A ProfileEntry<T> is a std::unordered_map<std::string, std::shared_ptr<T>>
 *      - The key is the profile name
 *      - Where std::shared_ptr<T> is the profile
 *    The ProfleEntry<T> is also stored in std::unordered_map where the key here is the std::type_index(typeid(T))
 */
class ProfileDictionary
{
public:
  using Ptr = std::shared_ptr<ProfileDictionary>;
  using ConstPtr = std::shared_ptr<const ProfileDictionary>;

  template <typename T>
  bool hasProfileEntry() const
  {
    return (profiles_.find(std::type_index(typeid(T))) != profiles_.end());
  }

  template <typename T>
  void removeProfileEntry()
  {
    profiles_.erase(std::type_index(typeid(T)));
  }

  template <typename T>
  const std::unordered_map<std::string, std::shared_ptr<T>>& getProfileEntry() const
  {
    auto it = profiles_.find(std::type_index(typeid(T)));
    if (it != profiles_.end())
      return std::any_cast<const std::unordered_map<std::string, std::shared_ptr<T>>&>(it->second);

    throw std::runtime_error("Profile entry does not exist for type name " +
                             std::string(std::type_index(typeid(T)).name()));
  }

  template <typename T>
  void addProfile(const std::string& profile_name, std::shared_ptr<T> profile)
  {
    auto it = profiles_.find(std::type_index(typeid(T)));
    if (it != profiles_.end())
    {
      std::any_cast<std::unordered_map<std::string, std::shared_ptr<T>>&>(it->second)[profile_name] = profile;
    }
    else
    {
      std::unordered_map<std::string, std::shared_ptr<T>> new_entry;
      new_entry[profile_name] = profile;
      profiles_[std::type_index(typeid(T))] = new_entry;
    }
  }

  template <typename T>
  std::shared_ptr<T> getProfile(const std::string& profile_name)
  {
    auto it = profiles_.find(std::type_index(typeid(T)));
    if (it != profiles_.end())
      return std::any_cast<std::unordered_map<std::string, std::shared_ptr<T>>&>(it->second)[profile_name];

    return nullptr;
  }

  template <typename T>
  void removeProfile(const std::string& profile_name)
  {
    auto it = profiles_.find(std::type_index(typeid(T)));
    if (it != profiles_.end())
      std::any_cast<std::unordered_map<std::string, std::shared_ptr<T>>&>(it->second).erase(profile_name);
  }

protected:
  std::unordered_map<std::type_index, std::any> profiles_;
};
}  // namespace tesseract_planning
#endif  // TESSERACT_MOTION_PLANNERS_PROFILE_DICTIONARY_H
