/**
 * @file profile.cpp
 * @brief This is a profile base class
 *
 * @author Levi Armstrong
 * @date December 2, 2024
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

#include <tesseract_common/profile.h>

namespace tesseract::common
{
Profile::Profile(std::size_t key) : key_(key) {}

std::size_t Profile::getKey() const { return key_; }

}  // namespace tesseract::common
