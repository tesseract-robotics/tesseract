#ifndef TESSERACT_ROS_PLANNING_CONTINUOUS_MOTION_VALIDATOR_H
#define TESSERACT_ROS_PLANNING_CONTINUOUS_MOTION_VALIDATOR_H

#include <tesseract_common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <ompl/base/DiscreteMotionValidator.h>
#include <ompl/base/MotionValidator.h>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract_environment/core/environment.h>
#include <tesseract_kinematics/core/forward_kinematics.h>

namespace tesseract_motion_planners
{
class ContinuousMotionValidator : public ompl::base::MotionValidator
{
public:
  ContinuousMotionValidator(ompl::base::SpaceInformationPtr space_info,
                            tesseract_environment::Environment::ConstPtr env,
                            tesseract_kinematics::ForwardKinematics::ConstPtr kin);

  bool checkMotion(const ompl::base::State* s1, const ompl::base::State* s2) const override;

  bool checkMotion(const ompl::base::State* s1,
                   const ompl::base::State* s2,
                   std::pair<ompl::base::State*, double>& lastValid) const override;

private:
  bool continuousCollisionCheck(const ompl::base::State* s1, const ompl::base::State* s2) const;

  tesseract_environment::Environment::ConstPtr env_;
  tesseract_kinematics::ForwardKinematics::ConstPtr kin_;
  tesseract_collision::ContinuousContactManager::Ptr contact_manager_;
  std::vector<std::string> links_;
  std::vector<std::string> joints_;
};
}

#endif  // TESSERACT_ROS_PLANNING_CONTINUOUS_MOTION_VALIDATOR_H
