/**
 * @file profile_dictionary.cpp
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

#include <tesseract_common/profile_dictionary.h>
#include <boost/serialization/nvp.hpp>
#include <boost/serialization/shared_ptr.hpp>
#include <boost/serialization/unordered_map.hpp>

#include <mutex>

namespace tesseract_common
{
// NOLINTNEXTLINE(cppcoreguidelines-prefer-member-initializer)
ProfileDictionary::ProfileDictionary(const ProfileDictionary& other)
{
  std::unique_lock lhs_lock(mutex_, std::defer_lock);
  std::shared_lock rhs_lock(other.mutex_, std::defer_lock);
  std::scoped_lock lock{ lhs_lock, rhs_lock };

  profiles_ = other.profiles_;  // NOLINT(cppcoreguidelines-prefer-member-initializer)
}

// NOLINTNEXTLINE(cppcoreguidelines-prefer-member-initializer)
ProfileDictionary& ProfileDictionary::operator=(const ProfileDictionary& other)
{
  std::unique_lock lhs_lock(mutex_, std::defer_lock);
  std::shared_lock rhs_lock(other.mutex_, std::defer_lock);
  std::scoped_lock lock{ lhs_lock, rhs_lock };

  profiles_ = other.profiles_;  // NOLINT(cppcoreguidelines-prefer-member-initializer)
  return *this;
}

// NOLINTNEXTLINE(cppcoreguidelines-prefer-member-initializer)
ProfileDictionary::ProfileDictionary(ProfileDictionary&& other) noexcept
{
  std::unique_lock lhs_lock(mutex_, std::defer_lock);
  std::unique_lock rhs_lock(other.mutex_, std::defer_lock);
  std::scoped_lock lock{ lhs_lock, rhs_lock };

  profiles_ = std::move(other.profiles_);  // NOLINT(cppcoreguidelines-prefer-member-initializer)
}

// NOLINTNEXTLINE(cppcoreguidelines-prefer-member-initializer)
ProfileDictionary& ProfileDictionary::operator=(ProfileDictionary&& other) noexcept
{
  std::unique_lock lhs_lock(mutex_, std::defer_lock);
  std::unique_lock rhs_lock(other.mutex_, std::defer_lock);
  std::scoped_lock lock{ lhs_lock, rhs_lock };

  profiles_ = std::move(other.profiles_);  // NOLINT(cppcoreguidelines-prefer-member-initializer)
  return *this;
}

bool ProfileDictionary::hasProfileEntry(std::size_t key, const std::string& ns) const
{
  const std::shared_lock lock(mutex_);
  auto it = profiles_.find(ns);
  if (it == profiles_.end())
    return false;

  return (it->second.find(key) != it->second.end());
}

void ProfileDictionary::removeProfileEntry(std::size_t key, const std::string& ns)
{
  const std::unique_lock lock(mutex_);

  auto it = profiles_.find(ns);
  if (it == profiles_.end())
    return;

  it->second.erase(key);
  if (it->second.empty())
    profiles_.erase(it);
}

std::unordered_map<std::string, Profile::ConstPtr> ProfileDictionary::getProfileEntry(std::size_t key,
                                                                                      const std::string& ns) const
{
  const std::shared_lock lock(mutex_);
  auto it = profiles_.find(ns);
  if (it == profiles_.end())
    throw std::runtime_error("Profile namespace does not exist for '" + ns + "'!");

  auto it2 = it->second.find(key);
  if (it2 != it->second.end())
    return it2->second;

  throw std::runtime_error("Profile entry does not exist for type name '" + std::to_string(key) + "' in namespace '" +
                           ns + "'!");
}

std::unordered_map<std::string, std::unordered_map<std::size_t, std::unordered_map<std::string, Profile::ConstPtr>>>
ProfileDictionary::getAllProfileEntries() const
{
  const std::shared_lock lock(mutex_);
  return profiles_;
}

void ProfileDictionary::addProfile(const std::string& ns,
                                   const std::string& profile_name,
                                   const Profile::ConstPtr& profile)
{
  if (ns.empty())
    throw std::runtime_error("Adding profile with an empty namespace!");

  if (profile_name.empty())
    throw std::runtime_error("Adding profile with an empty string as the key!");

  if (profile == nullptr)
    throw std::runtime_error("Adding profile that is a nullptr");

  const std::unique_lock lock(mutex_);
  auto it = profiles_.find(ns);
  if (it == profiles_.end())
  {
    std::unordered_map<std::string, Profile::ConstPtr> new_entry;
    new_entry[profile_name] = profile;
    profiles_[ns][profile->getKey()] = new_entry;
  }
  else
  {
    auto it2 = it->second.find(profile->getKey());
    if (it2 != it->second.end())
    {
      it2->second[profile_name] = profile;
    }
    else
    {
      std::unordered_map<std::string, Profile::ConstPtr> new_entry;
      new_entry[profile_name] = profile;
      it->second[profile->getKey()] = new_entry;
    }
  }
}

void ProfileDictionary::addProfile(const std::string& ns,
                                   const std::vector<std::string>& profile_names,
                                   const Profile::ConstPtr& profile)
{
  if (ns.empty())
    throw std::runtime_error("Adding profile with an empty namespace!");

  if (profile_names.empty())
    throw std::runtime_error("Adding profile with an empty vector of keys!");

  if (profile == nullptr)
    throw std::runtime_error("Adding profile that is a nullptr");

  const std::unique_lock lock(mutex_);
  auto it = profiles_.find(ns);
  if (it == profiles_.end())
  {
    std::unordered_map<std::string, Profile::ConstPtr> new_entry;
    for (const auto& profile_name : profile_names)
    {
      if (profile_name.empty())
        throw std::runtime_error("Adding profile with an empty string as the key!");

      new_entry[profile_name] = profile;
    }
    profiles_[ns][profile->getKey()] = new_entry;
  }
  else
  {
    auto it2 = it->second.find(profile->getKey());
    if (it2 != it->second.end())
    {
      for (const auto& profile_name : profile_names)
      {
        if (profile_name.empty())
          throw std::runtime_error("Adding profile with an empty string as the key!");

        it2->second[profile_name] = profile;
      }
    }
    else
    {
      std::unordered_map<std::string, Profile::ConstPtr> new_entry;
      for (const auto& profile_name : profile_names)
      {
        if (profile_name.empty())
          throw std::runtime_error("Adding profile with an empty string as the key!");

        new_entry[profile_name] = profile;
      }
      it->second[profile->getKey()] = new_entry;
    }
  }
}

bool ProfileDictionary::hasProfile(std::size_t key, const std::string& ns, const std::string& profile_name) const
{
  const std::shared_lock lock(mutex_);
  auto it = profiles_.find(ns);
  if (it == profiles_.end())
    return false;

  auto it2 = it->second.find(key);
  if (it2 != it->second.end())
  {
    auto it3 = it2->second.find(profile_name);
    if (it3 != it2->second.end())
      return true;
  }
  return false;
}

Profile::ConstPtr ProfileDictionary::getProfile(std::size_t key,
                                                const std::string& ns,
                                                const std::string& profile_name) const
{
  const std::shared_lock lock(mutex_);
  return profiles_.at(ns).at(key).at(profile_name);
}

void ProfileDictionary::removeProfile(std::size_t key, const std::string& ns, const std::string& profile_name)
{
  const std::unique_lock lock(mutex_);
  auto it = profiles_.find(ns);
  if (it == profiles_.end())
    return;

  auto it2 = it->second.find(key);
  if (it2 != it->second.end())
  {
    it2->second.erase(profile_name);
    if (it2->second.empty())
    {
      it->second.erase(it2);
      if (it->second.empty())
        profiles_.erase(it);
    }
  }
}

void ProfileDictionary::clear()
{
  const std::unique_lock lock(mutex_);
  profiles_.clear();
}

template <class Archive>
void ProfileDictionary::serialize(Archive& ar, const unsigned int /*version*/)
{
  const std::shared_lock lock(mutex_);
  ar& boost::serialization::make_nvp("profiles", profiles_);
}

}  // namespace tesseract_common

#include <tesseract_common/serialization.h>
TESSERACT_SERIALIZE_ARCHIVES_INSTANTIATE(tesseract_common::ProfileDictionary)
BOOST_CLASS_EXPORT_IMPLEMENT(tesseract_common::ProfileDictionary)

TESSERACT_SERIALIZE_ARCHIVES_INSTANTIATE(tesseract_common::ProfileDictionaryPtrAnyPoly)
BOOST_CLASS_EXPORT_IMPLEMENT(tesseract_common::ProfileDictionaryPtrAnyPoly)
