/**
 * @file weighted_real_vector_state_sampler.h
 * @brief Tesseract OMPL Weighted State Sampler.
 *
 * This allows you to provided a weight factor different
 * joint when sampling. This is useful when you have a
 * gantry with two linear axis with a robot attached. When
 * sampling near you may want to scale down the rail sampling.
 *
 * @author Ioan Sucan, Levi Armstrong
 *
 * @copyright Copyright (c) 2010, Rice University
 * @copyright Copyright (c) 2019, Southwest Research Institute
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
#ifndef TESSERACT_MOTION_PLANNERS_WEIGHTED_REAL_VECTOR_STATE_SAMPLER_H
#define TESSERACT_MOTION_PLANNERS_WEIGHTED_REAL_VECTOR_STATE_SAMPLER_H

#include <tesseract_common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <ompl/base/StateSampler.h>
#include <Eigen/Geometry>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

namespace tesseract_motion_planners
{
/**
 * @brief Tesseract OMPL Weighted State Sampler.
 *
 * This allows you to provided a weight factor different
 * joint when sampling. This is useful when you have a
 * gantry with two linear axis with a robot attached. When
 * sampling near you may want to scale down the rail sampling.
 */
class WeightedRealVectorStateSampler : public ompl::base::StateSampler
{
public:
  WeightedRealVectorStateSampler(const ompl::base::StateSpace* space, Eigen::VectorXd weights)
    : ompl::base::StateSampler(space), weights_(std::move(weights))
  {
  }

  void sampleUniform(ompl::base::State* state) override;

  /** \brief Sample a state such that each component state[i] is
      uniformly sampled from [near[i]-distance, near[i]+distance].
      If this interval exceeds the state space bounds, the
      interval is truncated. */
  void sampleUniformNear(ompl::base::State* state, const ompl::base::State* near, double distance) override;

  /** \brief Sample a state such that each component state[i] has
      a Gaussian distribution with mean mean[i] and standard
      deviation stdDev. If the sampled value exceeds the state
      space boundary, it is thresholded to the nearest boundary. */
  void sampleGaussian(ompl::base::State* state, const ompl::base::State* mean, double stdDev) override;

protected:
  Eigen::VectorXd weights_;
};

}  // namespace tesseract_motion_planners
#endif  // TESSERACT_MOTION_PLANNERS_WEIGHTED_REAL_VECTOR_STATE_SAMPLER_H
