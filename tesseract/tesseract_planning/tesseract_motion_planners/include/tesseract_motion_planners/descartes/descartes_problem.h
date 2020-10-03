/**
 * @file descartes_problem.h
 * @brief
 *
 * @author Levi Armstrong
 * @date June 18, 2020
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
#ifndef TESSERACT_MOTION_PLANNERS_DESCARTES_PROBLEM_H
#define TESSERACT_MOTION_PLANNERS_DESCARTES_PROBLEM_H

#include <tesseract_common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <descartes_light/interface/edge_evaluator.h>
#include <descartes_light/interface/position_sampler.h>
#include <descartes_light/ladder_graph.h>
#include <descartes_light/descartes_light.h>
#include <vector>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract/tesseract.h>
#include <tesseract_environment/core/environment.h>

namespace tesseract_planning
{
template <typename FloatType>
struct DescartesProblem
{
  // These are required for Tesseract to configure Descartes
  tesseract::Tesseract::ConstPtr tesseract;
  tesseract_environment::EnvState::ConstPtr env_state;

  // Kinematic Objects
  tesseract_kinematics::ForwardKinematics::ConstPtr manip_fwd_kin;
  tesseract_kinematics::InverseKinematics::ConstPtr manip_inv_kin;

  // These are required for descartes
  std::vector<typename descartes_light::EdgeEvaluator<FloatType>::Ptr> edge_evaluators;
  std::vector<typename descartes_light::PositionSampler<FloatType>::Ptr> samplers;
  int num_threads = descartes_light::Solver<double>::getMaxThreads();
};
using DescartesProblemF = DescartesProblem<float>;
using DescartesProblemD = DescartesProblem<double>;

}  // namespace tesseract_planning
#endif  // TESSERACT_MOTION_PLANNERS_DESCARTES_PROBLEM_H
