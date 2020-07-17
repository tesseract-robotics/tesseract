#include <tesseract_motion_planners/trajopt/trajopt_utils.h>
#include <tesseract_motion_planners/trajopt/profile/trajopt_default_plan_profile.h>

#include <tesseract_command_language/move_instruction.h>
#include <tesseract_command_language/plan_instruction.h>
#include <tesseract_command_language/instruction_type.h>

namespace tesseract_planning
{
void TrajOptDefaultPlanProfile::apply(trajopt::ProblemConstructionInfo& pci,
                                      const Eigen::Isometry3d& cartesian_waypoint,
                                      const Instruction& parent_instruction,
                                      const std::vector<std::string>& active_links,
                                      int index)
{
  trajopt::TermInfo::Ptr ti{ nullptr };

  // Extract working frame and tcp
  std::string working_frame;
  Eigen::Isometry3d tcp = Eigen::Isometry3d::Identity();
  if (isMoveInstruction(parent_instruction))
  {
    const auto* temp = parent_instruction.cast_const<MoveInstruction>();
    tcp = temp->getTCP();
    working_frame = temp->getWorkingFrame();
  }
  else if (isPlanInstruction(parent_instruction))
  {
    const auto* temp = parent_instruction.cast_const<PlanInstruction>();
    tcp = temp->getTCP();
    working_frame = temp->getWorkingFrame();
  }
  else
  {
    throw std::runtime_error("TrajOptDefaultPlanProfile: Unsupported instruction type!");
  }

  /* Check if this cartesian waypoint is dynamic
   * (i.e. defined relative to a frame that will move with the kinematic chain)
   */
  auto it = std::find(active_links.begin(), active_links.end(), working_frame);
  if (it != active_links.end())
  {
    ti = createDynamicCartesianWaypointTermInfo(
        cartesian_waypoint, index, working_frame, tcp, cartesian_coeff, pci.kin->getTipLinkName(), term_type);
  }
  else
  {
    ti = createCartesianWaypointTermInfo(
        cartesian_waypoint, index, working_frame, tcp, cartesian_coeff, pci.kin->getTipLinkName(), term_type);
  }

  if (term_type == trajopt::TermType::TT_CNT)
    pci.cnt_infos.push_back(ti);
  else
    pci.cost_infos.push_back(ti);
}

void TrajOptDefaultPlanProfile::apply(trajopt::ProblemConstructionInfo& pci,
                                      const Eigen::VectorXd& joint_waypoint,
                                      const Instruction& /*parent_instruction*/,
                                      const std::vector<std::string>& /*active_links*/,
                                      int index)
{
  auto ti = createJointWaypointTermInfo(joint_waypoint, index, joint_coeff, term_type);

  if (term_type == trajopt::TermType::TT_CNT)
    pci.cnt_infos.push_back(ti);
  else
    pci.cost_infos.push_back(ti);
}

}  // namespace tesseract_planning
