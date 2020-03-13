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
#include <ompl/base/StateSpace.h>
#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <ompl/base/ScopedState.h>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract_motion_planners/ompl/weighted_real_vector_state_sampler.h>

namespace tesseract_motion_planners
{
WeightedRealVectorStateSampler::WeightedRealVectorStateSampler(const ompl::base::StateSpace* space,
                                                               const Eigen::Ref<const Eigen::VectorXd>& weights,
                                                               const Eigen::Ref<const Eigen::MatrixX2d>& bounds)
  : ompl::base::StateSampler(space), weights_(weights), bounds_(bounds)
{
  assert(bounds_.rows() == weights_.size());
}

void WeightedRealVectorStateSampler::sampleUniform(ompl::base::State* state)
{
  assert(dynamic_cast<ompl::base::RealVectorStateSpace::StateType*>(state) != nullptr);

  const unsigned int dim = space_->getDimension();
  auto* rstate = state->template as<ompl::base::RealVectorStateSpace::StateType>();

  for (unsigned int i = 0; i < dim; ++i)
    rstate->values[i] = rng_.uniformReal(bounds_(i, 0), bounds_(i, 1));
}

void WeightedRealVectorStateSampler::sampleUniformNear(ompl::base::State* state,
                                                       const ompl::base::State* near,
                                                       const double distance)
{
  assert(dynamic_cast<ompl::base::RealVectorStateSpace::StateType*>(state) != nullptr);
  assert(dynamic_cast<const ompl::base::RealVectorStateSpace::StateType*>(near) != nullptr);

  const unsigned int dim = space_->getDimension();
  auto* rstate = state->template as<ompl::base::RealVectorStateSpace::StateType>();
  const auto* rnear = near->template as<ompl::base::RealVectorStateSpace::StateType>();

  for (unsigned int i = 0; i < dim; ++i)
  {
    double lb = std::max(bounds_(i, 0), rnear->values[i] - (distance * weights_[i]));
    double ub = std::min(bounds_(i, 1), rnear->values[i] + (distance * weights_[i]));
    if (std::abs(ub - lb) < 1e-6 || ub < lb)
      rstate->values[i] = lb;
    else
      rstate->values[i] = rng_.uniformReal(lb, ub);
  }
}

void WeightedRealVectorStateSampler::sampleGaussian(ompl::base::State* state,
                                                    const ompl::base::State* mean,
                                                    const double stdDev)
{
  assert(dynamic_cast<ompl::base::RealVectorStateSpace::StateType*>(state) != nullptr);
  assert(dynamic_cast<const ompl::base::RealVectorStateSpace::StateType*>(mean) != nullptr);

  const unsigned int dim = space_->getDimension();
  auto* rstate = state->template as<ompl::base::RealVectorStateSpace::StateType>();
  const auto* rmean = mean->template as<ompl::base::RealVectorStateSpace::StateType>();

  for (unsigned int i = 0; i < dim; ++i)
  {
    double v = rng_.gaussian(rmean->values[i], stdDev * weights_[i]);
    if (v < bounds_(i, 0))
      v = bounds_(i, 0);
    else if (v > bounds_(i, 1))
      v = bounds_(i, 1);
    rstate->values[i] = v;
  }
}
}  // namespace tesseract_motion_planners
