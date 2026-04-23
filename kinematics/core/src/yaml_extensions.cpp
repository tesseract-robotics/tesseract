/**
 * @file yaml_extensions.cpp
 * @brief YAML Type conversion implementations for kinematics types.
 *
 * @author Levi Armstrong
 * @date April 4, 2026
 *
 * @copyright Copyright (c) 2026, Southwest Research Institute
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
#include <tesseract/kinematics/yaml_extensions.h>
#include <tesseract/common/property_tree.h>
#include <tesseract/common/schema_registration.h>

using namespace tesseract::common;

PropertyTree YAML::convert<tesseract::kinematics::PositionerSampleResolution>::schema()
{
  return PropertyTreeBuilder()
      .string("name")
      .required()
      .done()
      .doubleNum("value")
      .required()
      .done()
      .doubleNum("min")
      .done()
      .doubleNum("max")
      .done()
      .build();
}

TESSERACT_SCHEMA_REGISTER(tesseract::kinematics::PositionerSampleResolution,
                          YAML::convert<tesseract::kinematics::PositionerSampleResolution>::schema)
