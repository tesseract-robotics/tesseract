/**
 * @file weighted_real_vector_state_sampler.cpp
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
#include <tesseract_common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <tesseract_motion_planners/ompl/weighted_real_vector_state_sampler.h>
#include <ompl/base/spaces/RealVectorStateSpace.h>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

namespace tesseract_motion_planners
{
void WeightedRealVectorStateSampler::sampleUniform(ompl::base::State* state)
{
  const unsigned int dim = space_->getDimension();
  const ompl::base::RealVectorBounds& bounds =
      static_cast<const ompl::base::RealVectorStateSpace*>(space_)->getBounds();

  auto* rstate = static_cast<ompl::base::RealVectorStateSpace::StateType*>(state);
  for (unsigned int i = 0; i < dim; ++i)
    rstate->values[i] = rng_.uniformReal(bounds.low[i], bounds.high[i]);
}

void WeightedRealVectorStateSampler::sampleUniformNear(ompl::base::State* state,
                                                       const ompl::base::State* near,
                                                       const double distance)
{
  const unsigned int dim = space_->getDimension();
  const ompl::base::RealVectorBounds& bounds =
      static_cast<const ompl::base::RealVectorStateSpace*>(space_)->getBounds();

  auto* rstate = static_cast<ompl::base::RealVectorStateSpace::StateType*>(state);
  const auto* rnear = static_cast<const ompl::base::RealVectorStateSpace::StateType*>(near);
  for (unsigned int i = 0; i < dim; ++i)
    rstate->values[i] = rng_.uniformReal(std::max(bounds.low[i], rnear->values[i] - (distance * weights_[i])),
                                         std::min(bounds.high[i], rnear->values[i] + (distance * weights_[i])));
}

void WeightedRealVectorStateSampler::sampleGaussian(ompl::base::State* state,
                                                    const ompl::base::State* mean,
                                                    const double stdDev)
{
  const unsigned int dim = space_->getDimension();
  const ompl::base::RealVectorBounds& bounds =
      static_cast<const ompl::base::RealVectorStateSpace*>(space_)->getBounds();

  auto* rstate = static_cast<ompl::base::RealVectorStateSpace::StateType*>(state);
  const auto* rmean = static_cast<const ompl::base::RealVectorStateSpace::StateType*>(mean);
  for (unsigned int i = 0; i < dim; ++i)
  {
    double v = rng_.gaussian(rmean->values[i], stdDev * weights_[i]);
    if (v < bounds.low[i])
      v = bounds.low[i];
    else if (v > bounds.high[i])
      v = bounds.high[i];
    rstate->values[i] = v;
  }
}
}  // namespace tesseract_motion_planners
