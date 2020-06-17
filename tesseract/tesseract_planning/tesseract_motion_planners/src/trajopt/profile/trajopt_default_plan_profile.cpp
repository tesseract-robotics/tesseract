#include <tesseract_motion_planners/trajopt/trajopt_utils.h>
#include <tesseract_motion_planners/trajopt/profile/trajopt_default_plan_profile.h>

namespace tesseract_planning
{

void TrajOptDefaultPlanProfile::apply(trajopt::ProblemConstructionInfo& pci,
                                      const Eigen::Isometry3d& cartesian_waypoint,
                                      const PlanInstruction& parent_instruction,
                                      const std::vector<std::string>& active_links,
                                      int index)
{
  trajopt::TermInfo::Ptr ti {nullptr};

  /* Check if this cartesian waypoint is dynamic
   * (i.e. defined relative to a frame that will move with the kinematic chain)
   */
  auto it = std::find(active_links.begin(), active_links.end(), parent_instruction.getWorkingFrame());
  if (it != active_links.end())
  {
    ti = createDynamicCartesianWaypointTermInfo(cartesian_waypoint, index, parent_instruction.getWorkingFrame(), parent_instruction.getTCP(), cartesian_coeff, pci.kin->getTipLinkName(), term_type);
  }
  else
  {
    ti = createCartesianWaypointTermInfo(cartesian_waypoint, index, parent_instruction.getWorkingFrame(), parent_instruction.getTCP(), cartesian_coeff, pci.kin->getTipLinkName(), term_type);
  }

  if (term_type == trajopt::TermType::TT_CNT)
    pci.cnt_infos.push_back(ti);
  else
    pci.cost_infos.push_back(ti);

}

void TrajOptDefaultPlanProfile::apply(trajopt::ProblemConstructionInfo& pci,
                                      const Eigen::VectorXd& joint_waypoint,
                                      const PlanInstruction& parent_instruction,
                                      const std::vector<std::string>& active_links,
                                      int index)
{
  auto ti = createJointWaypointTermInfo(joint_waypoint, index, joint_coeff, term_type);

  if (term_type == trajopt::TermType::TT_CNT)
    pci.cnt_infos.push_back(ti);
  else
    pci.cost_infos.push_back(ti);
}

}
