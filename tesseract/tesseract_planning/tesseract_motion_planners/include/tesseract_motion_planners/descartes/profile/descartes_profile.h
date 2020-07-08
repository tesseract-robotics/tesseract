#ifndef TESSERACT_MOTION_PLANNERS_DESCARTES_DESCARTES_PROFILE_H
#define TESSERACT_MOTION_PLANNERS_DESCARTES_DESCARTES_PROFILE_H

#include <vector>
#include <memory>
#include <tesseract_command_language/plan_instruction.h>

#include <tesseract_motion_planners/descartes/descartes_problem.h>

namespace tesseract_planning
{
template <typename FloatType>
class DescartesPlanProfile
{
public:
  using Ptr = std::shared_ptr<DescartesPlanProfile<FloatType>>;
  using ConstPtr = std::shared_ptr<const DescartesPlanProfile<FloatType>>;

  virtual void apply(DescartesProblem<FloatType>& prob,
                     const Eigen::Isometry3d& cartesian_waypoint,
                     const PlanInstruction& parent_instruction,
                     const std::vector<std::string>& active_links,
                     int index) = 0;

  virtual void apply(DescartesProblem<FloatType>& prob,
                     const Eigen::VectorXd& joint_waypoint,
                     const PlanInstruction& parent_instruction,
                     const std::vector<std::string>& active_links,
                     int index) = 0;
};

template <typename FloatType>
using DescartesPlanProfileMap = std::unordered_map<std::string, typename DescartesPlanProfile<FloatType>::Ptr>;

/** @todo Currently descartes does not have support of composite profile everything is handled by the plan profile */
}  // namespace tesseract_planning

#endif  // TESSERACT_MOTION_PLANNERS_DESCARTES_DESCARTES_PROFILE_H
