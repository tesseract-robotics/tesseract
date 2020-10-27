/**
 * @file constants.h
 * @brief Containst Tesseract Command Language constants
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
#ifndef TESSERACT_COMMAND_LANGUAGE_CONSTANTS_H
#define TESSERACT_COMMAND_LANGUAGE_CONSTANTS_H

#include <tesseract_common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <string>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

namespace tesseract_planning
{
/** @brief Set to DEFAULT. Default profiles are given this name. */
static const std::string DEFAULT_PROFILE_KEY = "DEFAULT";

}  // namespace tesseract_planning
#endif  // TESSERACT_COMMAND_LANGUAGE_CONSTANTS_H
