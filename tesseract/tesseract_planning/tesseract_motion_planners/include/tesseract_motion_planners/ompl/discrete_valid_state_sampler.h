/**
 * @file discrete_valid_state_sampler.h
 * @brief Tesseract OMPL planner discrete valid state sampler.
 *
 * This not only generates the sample but it also performs discrete
 * collision checking here instead of the isValid function because
 * this is generated for every thread removing the multiple clones
 * of the contact manager in the isValid function.
 *
 * @author Levi Armstrong
 * @date April 18, 2018
 * @version TODO
 * @bug No known bugs
 *
 * @copyright Copyright (c) 2017, Southwest Research Institute
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
#ifndef TESSERACT_MOTION_PLANNERS_DISCRETE_VALID_STATE_SAMPLER_H
#define TESSERACT_MOTION_PLANNERS_DISCRETE_VALID_STATE_SAMPLER_H

#include <tesseract_common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <ompl/base/ValidStateSampler.h>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract_environment/core/environment.h>
#include <tesseract_kinematics/core/forward_kinematics.h>
#include <tesseract_collision/core/discrete_contact_manager.h>

namespace tesseract_motion_planners
{
/**
 * @brief Tesseract OMPL planner discrete valid state sampler.
 *
 * This not only generates the sample but it also performs discrete
 * collision checking here instead of the isValid function because
 * this is generated for every thread removing the multiple clones
 * of the contact manager in the isValid function.
 */
class DiscreteValidStateSampler : public ompl::base::ValidStateSampler
{
public:
  DiscreteValidStateSampler(const ompl::base::SpaceInformation* si,
                            tesseract_environment::Environment::ConstPtr env,
                            tesseract_kinematics::ForwardKinematics::ConstPtr kin,
                            tesseract_collision::DiscreteContactManager::Ptr contact_manager);

  ~DiscreteValidStateSampler() override = default;

  bool sample(ompl::base::State* state) override;
  bool sampleNear(ompl::base::State* state, const ompl::base::State* near, double distance) override;

private:
  bool isCollisionFree(ompl::base::State* state);

protected:
  ompl::base::StateSamplerPtr sampler_;
  tesseract_environment::Environment::ConstPtr env_;
  tesseract_kinematics::ForwardKinematics::ConstPtr kin_;
  tesseract_collision::DiscreteContactManager::Ptr contact_manager_;
};

}  // namespace tesseract_motion_planners

#endif  // TESSERACT_MOTION_PLANNERS_DISCRETE_VALID_STATE_SAMPLER_H
