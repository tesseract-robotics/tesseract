#include <tesseract_motion_planners/trajopt/profile/trajopt_default_composite_profile.h>
#include <tesseract_motion_planners/trajopt/trajopt_utils.h>
#include <tesseract_motion_planners/core/utils.h>

#include <trajopt/problem_description.hpp>

#include <tesseract_command_language/core/instruction.h>
#include <tesseract_command_language/plan_instruction.h>
#include <tesseract_command_language/move_instruction.h>
#include <tesseract_command_language/cartesian_waypoint.h>
#include <tesseract_command_language/joint_waypoint.h>


static const double LONGEST_VALID_SEGMENT_FRACTION_DEFAULT = 0.01;

namespace tesseract_planning
{

void TrajOptDefaultCompositeProfile::apply(trajopt::ProblemConstructionInfo& pci,
                                           int start_index, int end_index,
                                           const std::vector<std::string> &/*active_links*/,
                                           const std::vector<int>& fixed_indices)
{
  // -------- Construct the problem ------------
  // -------------------------------------------
  if (collision_constraint_config.enabled)
    addCollisionConstraint(pci, start_index, end_index, fixed_indices);

  if (collision_cost_config.enabled)
    addCollisionCost(pci, start_index, end_index, fixed_indices);

  if (smooth_velocities)
    addVelocitySmoothing(pci, start_index, end_index, fixed_indices);

  if (smooth_accelerations)
    addAccelerationSmoothing(pci, start_index, end_index, fixed_indices);

  if (smooth_jerks)
    addJerkSmoothing(pci, start_index, end_index, fixed_indices);

//  if (!constraint_error_functions.empty())
//    addConstraintErrorFunctions(pci, start_index, end_index, fixed_indices);

  if (avoid_singularity)
    addAvoidSingularity(pci, start_index, end_index, pci.kin->getTipLinkName(), fixed_indices);
}

void TrajOptDefaultCompositeProfile::addCollisionCost(trajopt::ProblemConstructionInfo& pci,
                                             int start_index, int end_index,
                                             const std::vector<int>& fixed_indices) const
{
  // Calculate longest valid segment length
  const Eigen::MatrixX2d& limits = pci.kin->getLimits();
  double length = 0;
  double extent = (limits.col(1) - limits.col(0)).norm();
  if (longest_valid_segment_fraction > 0 && longest_valid_segment_length > 0)
  {
    length = std::min(longest_valid_segment_fraction * extent, longest_valid_segment_length);
  }
  else if (longest_valid_segment_fraction > 0)
  {
    length = longest_valid_segment_fraction * extent;
  }
  else if (longest_valid_segment_length > 0)
  {
    length = longest_valid_segment_length;
  }
  else
  {
    length = LONGEST_VALID_SEGMENT_FRACTION_DEFAULT * extent;
  }

  // Create a default collision term info
  trajopt::TermInfo::Ptr ti = createCollisionTermInfo(start_index,
                                                      end_index,
                                                      collision_cost_config.buffer_margin,
                                                      collision_constraint_config.safety_margin_buffer,
                                                      collision_cost_config.type,
                                                      collision_cost_config.use_weighted_sum,
                                                      collision_cost_config.coeff,
                                                      contact_test_type,
                                                      length,
                                                      trajopt::TermType::TT_COST);

  // Update the term info with the (possibly) new start and end state indices for which to apply this cost
  std::shared_ptr<trajopt::CollisionTermInfo> ct = std::static_pointer_cast<trajopt::CollisionTermInfo>(ti);
  if (special_collision_cost)
  {
    for (auto& info : ct->info)
    {
      info = special_collision_cost;
    }
  }
  ct->fixed_steps = fixed_indices;

  pci.cost_infos.push_back(ct);
}

void TrajOptDefaultCompositeProfile::addCollisionConstraint(trajopt::ProblemConstructionInfo& pci,
                                                            int start_index, int end_index,
                                                            const std::vector<int>& fixed_indices) const
{
  // Calculate longest valid segment length
  const Eigen::MatrixX2d& limits = pci.kin->getLimits();
  double length = 0;
  double extent = (limits.col(1) - limits.col(0)).norm();
  if (longest_valid_segment_fraction > 0 && longest_valid_segment_length > 0)
  {
    length = std::min(longest_valid_segment_fraction * extent, longest_valid_segment_length);
  }
  else if (longest_valid_segment_fraction > 0)
  {
    length = longest_valid_segment_fraction * extent;
  }
  else if (longest_valid_segment_length > 0)
  {
    length = longest_valid_segment_length;
  }
  else
  {
    length = LONGEST_VALID_SEGMENT_FRACTION_DEFAULT * extent;
  }

  // Create a default collision term info
  trajopt::TermInfo::Ptr ti = createCollisionTermInfo(start_index,
                                                      end_index,
                                                      collision_constraint_config.safety_margin,
                                                      collision_constraint_config.safety_margin_buffer,
                                                      collision_constraint_config.type,
                                                      collision_cost_config.use_weighted_sum,
                                                      collision_constraint_config.coeff,
                                                      contact_test_type,
                                                      length,
                                                      trajopt::TermType::TT_CNT);

  // Update the term info with the (possibly) new start and end state indices for which to apply this cost
  std::shared_ptr<trajopt::CollisionTermInfo> ct = std::static_pointer_cast<trajopt::CollisionTermInfo>(ti);
  if (special_collision_constraint)
  {
    for (auto& info : ct->info)
    {
      info = special_collision_constraint;
    }
  }
  ct->fixed_steps = fixed_indices;

  pci.cnt_infos.push_back(ct);
}

void TrajOptDefaultCompositeProfile::addVelocitySmoothing(trajopt::ProblemConstructionInfo& pci,
                                                          int start_index, int end_index,
                                                          const std::vector<int>& /*fixed_indices*/) const
{
  if (velocity_coeff.size() == 0)
    pci.cost_infos.push_back(createSmoothVelocityTermInfo(start_index, end_index, static_cast<int>(pci.kin->numJoints())));
  else
    pci.cost_infos.push_back(createSmoothVelocityTermInfo(start_index, end_index, velocity_coeff));
}

void TrajOptDefaultCompositeProfile::addAccelerationSmoothing(trajopt::ProblemConstructionInfo& pci,
                                                              int start_index, int end_index,
                                                              const std::vector<int>& /*fixed_indices*/) const
{
  if (acceleration_coeff.size() == 0)
    pci.cost_infos.push_back(
        createSmoothAccelerationTermInfo(start_index, end_index, static_cast<int>(pci.kin->numJoints())));
  else
    pci.cost_infos.push_back(createSmoothAccelerationTermInfo(start_index, end_index, acceleration_coeff));
}

void TrajOptDefaultCompositeProfile::addJerkSmoothing(trajopt::ProblemConstructionInfo& pci,
                                                      int start_index, int end_index,
                                                      const std::vector<int>& /*fixed_indices*/) const
{
  if (jerk_coeff.size() == 0)
    pci.cost_infos.push_back(createSmoothJerkTermInfo(start_index, end_index, static_cast<int>(pci.kin->numJoints())));
  else
    pci.cost_infos.push_back(createSmoothJerkTermInfo(start_index, end_index, jerk_coeff));
}

void TrajOptDefaultCompositeProfile::addAvoidSingularity(trajopt::ProblemConstructionInfo& pci,
                                                         int start_index, int end_index,
                                                         const std::string& link,
                                                         const std::vector<int>& /*fixed_indices*/) const
{
  trajopt::TermInfo::Ptr ti = createAvoidSingularityTermInfo(start_index, end_index, link, avoid_singularity_coeff);
  pci.cost_infos.push_back(ti);
}

}
