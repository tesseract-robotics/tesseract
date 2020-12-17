/**
 * @file types.h
 * @brief Contains common types used throughout command language
 *
 * @author Levi Armstrong
 * @date July 22, 2020
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
#ifndef TESSERACT_COMMAND_LANGUAGE_TYPES_H
#define TESSERACT_COMMAND_LANGUAGE_TYPES_H

#include <tesseract_common/manipulator_info.h>

namespace tesseract_planning
{
using ToolCenterPoint = tesseract_common::ToolCenterPoint;
using ManipulatorInfo = tesseract_common::ManipulatorInfo;
}  // namespace tesseract_planning

#endif  // TESSERACT_COMMAND_LANGUAGE_TYPES_H
