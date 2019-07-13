#ifndef TESSERACT_PLANNERS_CHAIN_OMPL_INTERFACE_H
#define TESSERACT_PLANNERS_CHAIN_OMPL_INTERFACE_H

#include <tesseract_common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <ompl/geometric/SimpleSetup.h>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract_environment/core/environment.h>
#include <tesseract_kinematics/core/forward_kinematics.h>

namespace tesseract_motion_planners
{
struct OmplPlanParameters
{
  double planning_time = 5.0;
  bool simplify = true;
};

class ChainOmplInterface
{
public:
  ChainOmplInterface(tesseract_environment::Environment::ConstPtr env,
                     tesseract_kinematics::ForwardKinematics::ConstPtr kin);

  boost::optional<ompl::geometric::PathGeometric> plan(ompl::base::PlannerPtr planner,
                                                       const std::vector<double>& from,
                                                       const std::vector<double>& to,
                                                       const OmplPlanParameters& params);

  ompl::base::SpaceInformationPtr spaceInformation();

  void setMotionValidator(ompl::base::MotionValidatorPtr mv)
  {
    ss_->getSpaceInformation()->setMotionValidator(std::move(mv));
  }

private:
  bool isStateValid(const ompl::base::State* state) const;

private:
  ompl::geometric::SimpleSetupPtr ss_;
  tesseract_environment::Environment::ConstPtr env_;
  tesseract_kinematics::ForwardKinematics::ConstPtr kin_;
  tesseract_collision::DiscreteContactManager::Ptr contact_manager_;
  std::vector<std::string> joint_names_;
  std::vector<std::string> link_names_;
};
}

#endif
