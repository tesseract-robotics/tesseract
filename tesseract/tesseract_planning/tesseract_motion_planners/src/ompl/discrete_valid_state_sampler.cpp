/**
 * @file discrete_valid_state_sampler.cpp
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
#include <tesseract_common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <ompl/base/samplers/UniformValidStateSampler.h>
#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <ompl/base/SpaceInformation.h>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract_motion_planners/ompl/discrete_valid_state_sampler.h>

namespace tesseract_motion_planners
{
DiscreteValidStateSampler::DiscreteValidStateSampler(const ompl::base::SpaceInformation* si,
                                                     tesseract_environment::Environment::ConstPtr env,
                                                     tesseract_kinematics::ForwardKinematics::ConstPtr kin,
                                                     tesseract_collision::DiscreteContactManager::Ptr contact_manager)
  : ValidStateSampler(si)
  , sampler_(si->allocStateSampler())
  , env_(std::move(env))
  , kin_(std::move(kin))
  , contact_manager_(contact_manager->clone())
{
  name_ = "Tesseract Discrete Valid State Sampler";
}

bool DiscreteValidStateSampler::sample(ompl::base::State* state)
{
  unsigned int attempts = 0;
  bool valid = false;
  do
  {
    sampler_->sampleUniform(state);
    valid = si_->isValid(state);
    if (valid)
      valid = isCollisionFree(state);
    ++attempts;
  } while (!valid && attempts < attempts_);
  return valid;
}

bool DiscreteValidStateSampler::sampleNear(ompl::base::State* state,
                                           const ompl::base::State* near,
                                           const double distance)
{
  unsigned int attempts = 0;
  bool valid = false;
  do
  {
    sampler_->sampleUniformNear(state, near, distance);
    valid = si_->isValid(state);
    if (valid)
      valid = isCollisionFree(state);
    ++attempts;
  } while (!valid && attempts < attempts_);
  return valid;
}

bool DiscreteValidStateSampler::isCollisionFree(ompl::base::State* state)
{
  // Ompl Valid state sampler is created for each thread so
  // do not need to clone inside this function.
  const ompl::base::RealVectorStateSpace::StateType* s = state->as<ompl::base::RealVectorStateSpace::StateType>();
  const auto dof = kin_->numJoints();

  Eigen::Map<Eigen::VectorXd> joint_angles(s->values, long(dof));
  tesseract_environment::EnvState::ConstPtr env_state = env_->getState(kin_->getJointNames(), joint_angles);

  contact_manager_->setCollisionObjectsTransform(env_state->transforms);

  tesseract_collision::ContactResultMap contact_map;
  contact_manager_->contactTest(contact_map, tesseract_collision::ContactTestType::FIRST);

  return contact_map.empty();
}

}  // namespace tesseract_motion_planners
