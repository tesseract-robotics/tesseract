/**
 * @file trajopt_collision_config.h
 * @brief TrajOpt collision configuration settings
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
#ifndef TESSERACT_MOTION_PLANNERS_TRAJOPT_CONFIG_TRAJOPT_COLLISION_CONFIG_H
#define TESSERACT_MOTION_PLANNERS_TRAJOPT_CONFIG_TRAJOPT_COLLISION_CONFIG_H

#include <tesseract_common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <trajopt/problem_description.hpp>
#include <tinyxml2.h>
#include <boost/algorithm/string.hpp>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract_common/utils.h>

namespace tesseract_planning
{
/**
 * @brief Config settings for collision cost terms.
 */
struct CollisionCostConfig
{
  CollisionCostConfig() = default;
  CollisionCostConfig(const tinyxml2::XMLElement& xml_element);

  /** @brief If true, a collision cost term will be added to the problem. Default: true*/
  bool enabled = true;

  /**
   * @brief Use the weighted sum for each link pair. This reduces the number equations added to the problem
   * If set to true, it is recommended to start with the coeff set to one
   */
  bool use_weighted_sum = false;

  /** @brief The evaluator type that will be used for collision checking. */
  trajopt::CollisionEvaluatorType type = trajopt::CollisionEvaluatorType::DISCRETE_CONTINUOUS;

  /** @brief Max distance in which collision costs will be evaluated. */
  double safety_margin = 0.025;
  /** @brief Distance beyond buffer_margin in which collision optimization will be evaluated.
      This is set to 0 by default (effectively disabled) for collision costs.*/
  double safety_margin_buffer = 0.0;

  /** @brief The collision coeff/weight */
  double coeff = 20;

  tinyxml2::XMLElement* toXML(tinyxml2::XMLDocument& doc) const;
};

/**
 * @brief Config settings for collision constraint terms.
 */
struct CollisionConstraintConfig
{
  CollisionConstraintConfig() = default;
  CollisionConstraintConfig(const tinyxml2::XMLElement& xml_element);

  /** @brief If true, a collision cost term will be added to the problem. Default: true*/
  bool enabled = true;
  /**
   * @brief Use the weighted sum for each link pair. This reduces the number equations added to the problem
   * If set to true, it is recommended to start with the coeff set to one.
   */
  bool use_weighted_sum = false;
  /** @brief The evaluator type that will be used for collision checking. */
  trajopt::CollisionEvaluatorType type = trajopt::CollisionEvaluatorType::CAST_CONTINUOUS;
  /** @brief Max distance in which collision constraints will be evaluated. */
  double safety_margin = 0.01;
  /** @brief Distance beyond safety_margin in which collision optimization will be evaluated. */
  double safety_margin_buffer = 0.05;
  /** @brief The collision coeff/weight */
  double coeff = 20;

  tinyxml2::XMLElement* toXML(tinyxml2::XMLDocument& doc) const;
};
}  // namespace tesseract_planning

#endif  // TESSERACT_MOTION_PLANNERS_TRAJOPT_CONFIG_TRAJOPT_COLLISION_CONFIG_H
