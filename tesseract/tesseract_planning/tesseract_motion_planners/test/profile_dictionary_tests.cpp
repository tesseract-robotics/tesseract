/**
 * @file profile_dictionary_tests.cpp
 * @brief
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
#include <tesseract_common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <gtest/gtest.h>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract_motion_planners/core/profile_dictionary.h>

struct ProfileBase
{
  int a{ 0 };
};

struct ProfileTest : public ProfileBase
{
  ProfileTest() = default;
  ProfileTest(int a) { this->a = a; }
};

using namespace tesseract_planning;

TEST(TesseractPlanningProfileDictionaryUnit, ProfileDictionaryTest)
{
  ProfileDictionary profiles;

  EXPECT_FALSE(profiles.hasProfileEntry<ProfileBase>());

  profiles.addProfile<ProfileBase>("key", std::make_shared<ProfileTest>());

  EXPECT_TRUE(profiles.hasProfileEntry<ProfileBase>());

  auto profile = profiles.getProfile<ProfileBase>("key");

  EXPECT_TRUE(profile != nullptr);
  EXPECT_EQ(profile->a, 0);

  profiles.addProfile<ProfileBase>("key", std::make_shared<ProfileTest>(10));
  auto profile_check = profiles.getProfile<ProfileBase>("key");
  EXPECT_TRUE(profile_check != nullptr);
  EXPECT_EQ(profile_check->a, 10);

  auto profile_map = profiles.getProfileEntry<ProfileBase>();
  auto it = profile_map.find("key");
  EXPECT_TRUE(it != profile_map.end());
  EXPECT_EQ(it->second->a, 10);

  profiles.addProfile<ProfileBase>("key", std::make_shared<ProfileTest>(20));
  auto profile_check2 = profiles.getProfile<ProfileBase>("key");
  EXPECT_TRUE(profile_check2 != nullptr);
  EXPECT_EQ(profile_check2->a, 20);
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);

  return RUN_ALL_TESTS();
}
