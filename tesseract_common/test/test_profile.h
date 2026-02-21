/**
 * @file test_profile.h
 * @brief Profile test plugin
 *
 * @author Levi Armstrong
 * @date March 25, 2025
 *
 * @copyright Copyright (c) 2025, Levi Armstrong
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
#ifndef TESSERACT_COMMON_TEST_PROFILE_H
#define TESSERACT_COMMON_TEST_PROFILE_H

#include <tesseract_common/profile.h>

namespace tesseract::common
{
class TestProfile : public Profile
{
public:
  TestProfile() = default;
  ~TestProfile() override = default;
  TestProfile(const TestProfile&) = default;
  TestProfile& operator=(const TestProfile&) = default;
  TestProfile(TestProfile&&) = default;
  TestProfile& operator=(TestProfile&&) = default;

  bool enabled{ false };
};
}  // namespace tesseract::common
#endif  // TESSERACT_COMMON_TEST_PROFILE_H
