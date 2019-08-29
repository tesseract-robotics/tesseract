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

class DiscreteValidStateSampler : public ompl::base::ValidStateSampler
{
public:

  DiscreteValidStateSampler(const ompl::base::SpaceInformation *si,
                            tesseract_environment::Environment::ConstPtr env,
                            tesseract_kinematics::ForwardKinematics::ConstPtr kin,
                            tesseract_collision::DiscreteContactManager::Ptr contact_manager);

  ~DiscreteValidStateSampler() override = default;

  bool sample(ompl::base::State *state) override;
  bool sampleNear(ompl::base::State *state, const ompl::base::State *near, double distance) override;

private:
  bool isCollisionFree(ompl::base::State *state);

protected:
  ompl::base::StateSamplerPtr sampler_;
  tesseract_environment::Environment::ConstPtr env_;
  tesseract_kinematics::ForwardKinematics::ConstPtr kin_;
  tesseract_collision::DiscreteContactManager::Ptr contact_manager_;
};

}

#endif // TESSERACT_MOTION_PLANNERS_DISCRETE_VALID_STATE_SAMPLER_H
