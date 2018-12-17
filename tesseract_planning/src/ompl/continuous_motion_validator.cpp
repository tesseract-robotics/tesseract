#include <tesseract_core/macros.h>
TESSERACT_IGNORE_WARNINGS_PUSH
#include <ompl/base/spaces/RealVectorStateSpace.h>
TESSERACT_IGNORE_WARNINGS_POP

#include "tesseract_planning/ompl/continuous_motion_validator.h"

namespace tesseract
{
namespace tesseract_planning
{
ContinuousMotionValidator::ContinuousMotionValidator(ompl::base::SpaceInformationPtr space_info,
                                                     tesseract::BasicEnvConstPtr env,
                                                     const std::string& manipulator)
  : MotionValidator(space_info), env_(std::move(env))
{
  joints_ = env_->getManipulator(manipulator)->getJointNames();
  links_ = env_->getManipulator(manipulator)->getLinkNames();
  is_allowed_cb_ =
      std::bind(&ContinuousMotionValidator::isContactAllowed, this, std::placeholders::_1, std::placeholders::_2);

  contact_manager_ = env_->getContinuousContactManager();
  contact_manager_->setActiveCollisionObjects(links_);
  contact_manager_->setContactDistanceThreshold(0);
  contact_manager_->setIsContactAllowedFn(is_allowed_cb_);
}

bool ContinuousMotionValidator::checkMotion(const ompl::base::State* s1, const ompl::base::State* s2) const
{
  std::pair<ompl::base::State*, double> dummy = { nullptr, 0.0 };
  return checkMotion(s1, s2, dummy);
}

bool ContinuousMotionValidator::checkMotion(const ompl::base::State* s1,
                                            const ompl::base::State* s2,
                                            std::pair<ompl::base::State*, double>& lastValid) const
{
  const ompl::base::StateSpace& state_space = *si_->getStateSpace();

  unsigned n_steps = state_space.validSegmentCount(s1, s2);

  ompl::base::State* start_interp = si_->allocState();
  ompl::base::State* end_interp = si_->allocState();

  bool is_valid = true;
  unsigned i = 1;
  for (i = 1; i <= n_steps; ++i)
  {
    state_space.interpolate(s1, s2, static_cast<double>(i - 1) / n_steps, start_interp);
    state_space.interpolate(s1, s2, static_cast<double>(i) / n_steps, end_interp);

    if (!continuousCollisionCheck(start_interp, end_interp))
    {
      is_valid = false;
      break;
    }
  }

  if (!is_valid)
  {
    lastValid.second = static_cast<double>(i - 1) / n_steps;
    if (lastValid.first != nullptr)
      state_space.interpolate(s1, s2, lastValid.second, lastValid.first);
  }

  si_->freeState(start_interp);
  si_->freeState(end_interp);
  return is_valid;
}

bool ContinuousMotionValidator::continuousCollisionCheck(const ompl::base::State* s1, const ompl::base::State* s2) const
{
  const ompl::base::RealVectorStateSpace::StateType* start = s1->as<ompl::base::RealVectorStateSpace::StateType>();
  const ompl::base::RealVectorStateSpace::StateType* finish = s2->as<ompl::base::RealVectorStateSpace::StateType>();

  // Need to get thread id
  tesseract::ContinuousContactManagerBasePtr cm = contact_manager_->clone();

  const auto dof = si_->getStateDimension();
  Eigen::Map<Eigen::VectorXd> start_joints(start->values, dof);
  Eigen::Map<Eigen::VectorXd> finish_joints(finish->values, dof);

  tesseract::EnvStatePtr state0 = env_->getState(joints_, start_joints);
  tesseract::EnvStatePtr state1 = env_->getState(joints_, finish_joints);

  for (const auto& link_name : links_)
    cm->setCollisionObjectsTransform(link_name, state0->transforms[link_name], state1->transforms[link_name]);

  tesseract::ContactResultMap contact_map;
  cm->contactTest(contact_map, tesseract::ContactTestTypes::FIRST);

  return contact_map.empty();
}
}
}
