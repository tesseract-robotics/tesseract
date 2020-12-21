/**
 * @file profile_switch_process_generator.h
 * @brief Process generator that returns a value based on the profile
 *
 * @author Matthew Powelson
 * @date October 26. 2020
 * @version TODO
 * @bug No known bugs
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
#ifndef TESSERACT_PROCESS_MANAGERS_PROFILE_SWITCH_PROCESS_GENERATOR_H
#define TESSERACT_PROCESS_MANAGERS_PROFILE_SWITCH_PROCESS_GENERATOR_H

#include <tesseract_process_managers/core/process_generator.h>

#ifdef SWIG
%shared_ptr(tesseract_planning::ProfileSwitchProfile)
%ignore ProfileSwitchProcessGenerator;
%ignore ProfileSwitchProcessInfo;
#endif  // SWIG

namespace tesseract_planning
{
struct ProfileSwitchProfile
{
  ProfileSwitchProfile(const int& return_value = 1);

  using Ptr = std::shared_ptr<ProfileSwitchProfile>;
  using ConstPtr = std::shared_ptr<const ProfileSwitchProfile>;

  int return_value;
};
using ProfileSwitchProfileMap = std::unordered_map<std::string, ProfileSwitchProfile::ConstPtr>;

/**
 * @brief This generator simply returns a value specified in the composite profile. This can be used to switch execution
 * based on the profile
 */
class ProfileSwitchProcessGenerator : public ProcessGenerator
{
public:
  using UPtr = std::unique_ptr<ProfileSwitchProcessGenerator>;

  ProfileSwitchProcessGenerator(std::string name = "Profile Switch");

  ~ProfileSwitchProcessGenerator() override = default;
  ProfileSwitchProcessGenerator(const ProfileSwitchProcessGenerator&) = delete;
  ProfileSwitchProcessGenerator& operator=(const ProfileSwitchProcessGenerator&) = delete;
  ProfileSwitchProcessGenerator(ProfileSwitchProcessGenerator&&) = delete;
  ProfileSwitchProcessGenerator& operator=(ProfileSwitchProcessGenerator&&) = delete;

  ProfileSwitchProfileMap composite_profiles;

  int conditionalProcess(ProcessInput input, std::size_t unique_id) const override;

  void process(ProcessInput input, std::size_t unique_id) const override;
};

class ProfileSwitchProcessInfo : public ProcessInfo
{
public:
  ProfileSwitchProcessInfo(std::size_t unique_id, std::string name = "Profile Switch");
};
}  // namespace tesseract_planning
#endif  // TESSERACT_PROCESS_MANAGERS_PROFILE_SWITCH_PROCESS_GENERATOR_H
