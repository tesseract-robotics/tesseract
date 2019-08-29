#include <tesseract_common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <ompl/base/samplers/UniformValidStateSampler.h>
#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <ompl/base/SpaceInformation.h>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract_motion_planners/ompl/discrete_valid_state_sampler.h>

namespace tesseract_motion_planners
{

DiscreteValidStateSampler::DiscreteValidStateSampler(const ompl::base::SpaceInformation *si,
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

bool DiscreteValidStateSampler::sample(ompl::base::State *state)
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

bool DiscreteValidStateSampler::sampleNear(ompl::base::State *state, const ompl::base::State *near, const double distance)
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

bool DiscreteValidStateSampler::isCollisionFree(ompl::base::State *state)
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

}
