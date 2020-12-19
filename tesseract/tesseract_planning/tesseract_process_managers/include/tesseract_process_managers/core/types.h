/**
 * @file types.h
 * @brief Tesseract process managers types
 *
 * @author Levi Armstrong
 * @date December 19, 2020
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
#ifndef TESSERACT_PROCESS_MANAGERS_TYPES_H
#define TESSERACT_PROCESS_MANAGERS_TYPES_H

#include <tesseract_common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <functional>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

namespace tesseract_planning
{
using TaskflowVoidFn = std::function<void()>;
using TaskflowIntFn = std::function<int()>;
}  // namespace tesseract_planning

#endif  // TESSERACT_PROCESS_MANAGERS_TYPES_H
