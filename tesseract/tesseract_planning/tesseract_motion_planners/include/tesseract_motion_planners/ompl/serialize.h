/**
 * @file serialize.h
 * @brief Provide methods for serializing ompl plans to xml
 *
 * @author Tyler Marr
 * @date August 24, 2020
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
#ifndef TESSERACT_MOTION_PLANNERS_OMPL_SERIALIZE_H
#define TESSERACT_MOTION_PLANNERS_OMPL_SERIALIZE_H

#include <tesseract_common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <tinyxml2.h>
TESSERACT_COMMON_IGNORE_WARNINGS_POP
#include <tesseract_motion_planners/ompl/profile/ompl_profile.h>

namespace tesseract_planning
{
std::shared_ptr<tinyxml2::XMLDocument> toXMLDocument(const OMPLPlanProfile& plan_profile);

bool toXMLFile(const OMPLPlanProfile& plan_profile, const std::string& file_path);

std::string toXMLString(const OMPLPlanProfile& plan_profile);

}  // namespace tesseract_planning
#endif  // TESSERACT_MOTION_PLANNERS_OMPL_SERIALIZE_H
