#ifndef TESSERACT_MOTION_PLANNERS_TRAJOPT_PROFILE_H
#define TESSERACT_MOTION_PLANNERS_TRAJOPT_PROFILE_H

#include <trajopt/problem_description.hpp>
#include <vector>
#include <memory>
#include <tesseract_command_language/plan_instruction.h>

namespace tesseract_planning
{

class TrajOptPlanProfile
{
public:
  using Ptr = std::shared_ptr<TrajOptPlanProfile>;
  using ConstPtr = std::shared_ptr<const TrajOptPlanProfile>;

  virtual void apply(trajopt::ProblemConstructionInfo& pci,
                     const Eigen::Isometry3d& cartesian_waypoint,
                     const PlanInstruction& parent_instruction,
                     const std::vector<std::string> &active_links,
                     int index) = 0;

  virtual void apply(trajopt::ProblemConstructionInfo& pci,
                     const Eigen::VectorXd& joint_waypoint,
                     const PlanInstruction& parent_instruction,
                     const std::vector<std::string> &active_links,
                     int index) = 0;
};

class TrajOptCompositeProfile
{
public:
  using Ptr = std::shared_ptr<TrajOptCompositeProfile>;
  using ConstPtr = std::shared_ptr<const TrajOptCompositeProfile>;

  virtual void apply(trajopt::ProblemConstructionInfo& pci,
                     int start_index,
                     int end_index,
                     const std::vector<std::string> &active_links,
                     const std::vector<int>& fixed_indices) = 0;
};

}

#endif // TESSERACT_MOTION_PLANNERS_TRAJOPT_PROFILE_H
