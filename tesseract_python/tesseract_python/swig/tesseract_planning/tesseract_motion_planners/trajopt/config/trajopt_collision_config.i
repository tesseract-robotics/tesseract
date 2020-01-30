/**
 * @file trajopt_collision_config.i
 * @brief SWIG interface file for tesseract_planning/trajopt/config/trajopt_collision_config.h
 *
 * @author Joseph Schornak
 * @date January 22, 2020
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

%{
#include <tesseract_motion_planners/trajopt/config/trajopt_collision_config.h>
%}

namespace tesseract_motion_planners
{
struct CollisionCostConfig
{

  bool enabled = true;

  trajopt::CollisionEvaluatorType type = trajopt::CollisionEvaluatorType::CAST_CONTINUOUS;

  double buffer_margin = 0.025;

  double safety_margin_buffer = 0.0;

  double coeff = 20;
};

struct CollisionConstraintConfig
{

  bool enabled = true;

  trajopt::CollisionEvaluatorType type = trajopt::CollisionEvaluatorType::CAST_CONTINUOUS;

  double safety_margin = 0.01;

  double safety_margin_buffer = 0.05;

  double coeff = 20;
};

}  // namespace tesseract_motion_planners
