#include <tesseract_command_language/planners/trajopt/trajopt_default_plan_profile.h>

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
//  auto it = std::find(adjacency_links.begin(), adjacency_links.end(), working_frame);
//  if (it != adjacency_links.end())
//  {
//    ti = createDynamicCartesianWaypointTermInfo(p_cpw, index, working_frame, tcp, coeffs, config.link, term_type);
//    pci.cnt_infos.push_back(ti);
//  }
//  else
//  {
//    ti = createCartesianWaypointTermInfo(p_cpw, index, working_frame, tcp, coeffs, config.link, term_type);
//    pci.cnt_infos.push_back(ti);
//  }

//  if (term_type == trajopt::TermType::TT_CNT)
//    pci.cnt_infos.push_back(ti);
//  else
//    pci.cost_infos.push_back(ti);

}

void TrajOptDefaultPlanProfile::apply(trajopt::ProblemConstructionInfo& pci,
                                      const Eigen::VectorXd& joint_waypoint,
                                      const PlanInstruction& parent_instruction,
                                      const std::vector<std::string>& active_links,
                                      int index)
{
//  auto ti = createJointWaypointTermInfo(*cur_wp, index, joint_coeffs, term_type);

//  if (term_type == trajopt::TermType::TT_CNT)
//    pci.cnt_infos.push_back(ti);
//  else
//    pci.cost_infos.push_back(ti);

}

}
