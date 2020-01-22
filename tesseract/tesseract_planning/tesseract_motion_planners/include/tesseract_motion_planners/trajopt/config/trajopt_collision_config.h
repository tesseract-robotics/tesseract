#ifndef TESSERACT_MOTION_PLANNERS_TRAJOPT_CONFIG_TRAJOPT_COLLISION_CONFIG_H
#define TESSERACT_MOTION_PLANNERS_TRAJOPT_CONFIG_TRAJOPT_COLLISION_CONFIG_H

#include <trajopt/problem_description.hpp>

namespace tesseract_motion_planners
{
/**
 * @brief Config settings for collision cost terms.
 */
struct CollisionCostConfig
{
  /** @brief If true, a collision cost term will be added to the problem. Default: true*/
  bool check = true;
  /** @brief The evaluator type that will be used for collision checking. */
  trajopt::CollisionEvaluatorType type = trajopt::CollisionEvaluatorType::CAST_CONTINUOUS;
  /** @brief Max distance in which collision costs will be evaluated. */
  double buffer_margin = 0.025;
  /** @brief The collision coeff/weight */
  double coeff = 20;
};

/**
 * @brief Config settings for collision constraint terms.
 */
struct CollisionConstraintConfig
{
  /** @brief If true, a collision cost term will be added to the problem. Default: true*/
  bool check = true;
  /** @brief The evaluator type that will be used for collision checking. */
  trajopt::CollisionEvaluatorType type = trajopt::CollisionEvaluatorType::CAST_CONTINUOUS;
  /** @brief Max distance in which collision constraints will be evaluated. */
  double safety_margin = 0.01;
  /** @brief The collision coeff/weight */
  double coeff = 20;
};
}

#endif // TESSERACT_MOTION_PLANNERS_TRAJOPT_CONFIG_TRAJOPT_COLLISION_CONFIG_H
