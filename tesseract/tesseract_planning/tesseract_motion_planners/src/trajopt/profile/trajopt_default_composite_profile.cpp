/**
 * @file trajopt_default_composite_profile.cpp
 * @brief
 *
 * @author Levi Armstrong
 * @date June 18, 2020
 * @version TODO
 * @bug No known bugs
 *
 * @copyright Copyright (c) 2020, Southwest Research Institute
 *
 * @par License
 * Software License Agreement (Apache License)
 * @par
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 * http://www.apache.org/licenses/LICENSE-2.0
 * @par
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include <tesseract_common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <trajopt/problem_description.hpp>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract_motion_planners/trajopt/profile/trajopt_default_composite_profile.h>
#include <tesseract_motion_planners/trajopt/trajopt_utils.h>
#include <tesseract_motion_planners/core/utils.h>

#include <tesseract_command_language/core/instruction.h>
#include <tesseract_command_language/plan_instruction.h>
#include <tesseract_command_language/move_instruction.h>
#include <tesseract_command_language/cartesian_waypoint.h>
#include <tesseract_command_language/joint_waypoint.h>

static const double LONGEST_VALID_SEGMENT_FRACTION_DEFAULT = 0.01;

namespace tesseract_planning
{
void TrajOptDefaultCompositeProfile::apply(trajopt::ProblemConstructionInfo& pci,
                                           int start_index,
                                           int end_index,
                                           const ManipulatorInfo& /*manip_info*/,
                                           const std::vector<std::string>& /*active_links*/,
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

tinyxml2::XMLElement* TrajOptDefaultCompositeProfile::toXML(tinyxml2::XMLDocument& doc) const
{
  Eigen::IOFormat eigen_format(Eigen::StreamPrecision, 0, " ", " ");

  tinyxml2::XMLElement* xml_planner = doc.NewElement("Planner");
  xml_planner->SetAttribute("type", std::to_string(1).c_str());

  tinyxml2::XMLElement* xml_trajopt = doc.NewElement("TrajoptCompositeProfile");

  tinyxml2::XMLElement* xml_contact_test_type = doc.NewElement("ContactTestType");
  xml_contact_test_type->SetText(static_cast<int>(contact_test_type));
  xml_trajopt->InsertEndChild(xml_contact_test_type);

  tinyxml2::XMLElement* xml_collision_cost_info = collision_cost_config.toXML(doc);
  xml_trajopt->InsertEndChild(xml_collision_cost_info);

  tinyxml2::XMLElement* xml_collision_constraint_info = collision_constraint_config.toXML(doc);
  xml_trajopt->InsertEndChild(xml_collision_constraint_info);

  tinyxml2::XMLElement* xml_smooth_velocities = doc.NewElement("SmoothVelocities");

  tinyxml2::XMLElement* xml_sv_enabled = doc.NewElement("Enabled");
  xml_sv_enabled->SetText(smooth_velocities);
  xml_smooth_velocities->InsertEndChild(xml_sv_enabled);

  tinyxml2::XMLElement* xml_sv_coeff = doc.NewElement("Coefficients");
  std::stringstream sv_coeff;
  sv_coeff << velocity_coeff.format(eigen_format);
  xml_sv_coeff->SetText(sv_coeff.str().c_str());
  xml_smooth_velocities->InsertEndChild(xml_sv_coeff);
  xml_trajopt->InsertEndChild(xml_smooth_velocities);

  tinyxml2::XMLElement* xml_smooth_accelerations = doc.NewElement("SmoothAccelerations");

  tinyxml2::XMLElement* xml_sa_enabled = doc.NewElement("Enabled");
  xml_sa_enabled->SetText(smooth_accelerations);
  xml_smooth_accelerations->InsertEndChild(xml_sa_enabled);

  tinyxml2::XMLElement* xml_sa_coeff = doc.NewElement("Coefficients");
  std::stringstream sa_coeff;
  sa_coeff << acceleration_coeff.format(eigen_format);
  xml_sa_coeff->SetText(sa_coeff.str().c_str());
  xml_smooth_accelerations->InsertEndChild(xml_sa_coeff);
  xml_trajopt->InsertEndChild(xml_smooth_accelerations);

  tinyxml2::XMLElement* xml_smooth_jerks = doc.NewElement("SmoothJerks");

  tinyxml2::XMLElement* xml_sj_enabled = doc.NewElement("Enabled");
  xml_sj_enabled->SetText(smooth_jerks);
  xml_smooth_jerks->InsertEndChild(xml_sj_enabled);

  tinyxml2::XMLElement* xml_sj_coeff = doc.NewElement("Coefficients");
  std::stringstream sj_coeff;
  sj_coeff << jerk_coeff.format(eigen_format);
  xml_sj_coeff->SetText(sj_coeff.str().c_str());
  xml_smooth_jerks->InsertEndChild(xml_sj_coeff);
  xml_trajopt->InsertEndChild(xml_smooth_jerks);

  tinyxml2::XMLElement* xml_avoid_singularity = doc.NewElement("AvoidSingularity");

  tinyxml2::XMLElement* xml_as_enabled = doc.NewElement("Enabled");
  xml_as_enabled->SetText(avoid_singularity);
  xml_avoid_singularity->InsertEndChild(xml_as_enabled);

  tinyxml2::XMLElement* xml_as_coeff = doc.NewElement("Coefficient");
  xml_as_coeff->SetText(avoid_singularity_coeff);
  xml_avoid_singularity->InsertEndChild(xml_as_coeff);
  xml_trajopt->InsertEndChild(xml_avoid_singularity);

  tinyxml2::XMLElement* xml_long_valid_seg_frac = doc.NewElement("LongestValidSegmentFraction");
  xml_long_valid_seg_frac->SetText(longest_valid_segment_fraction);
  xml_trajopt->InsertEndChild(xml_long_valid_seg_frac);

  tinyxml2::XMLElement* xml_long_valid_seg_len = doc.NewElement("LongestValidSegmentLength");
  xml_long_valid_seg_len->SetText(longest_valid_segment_length);
  xml_trajopt->InsertEndChild(xml_long_valid_seg_len);

  xml_planner->InsertEndChild(xml_trajopt);

  return xml_planner;
}

void TrajOptDefaultCompositeProfile::addCollisionCost(trajopt::ProblemConstructionInfo& pci,
                                                      int start_index,
                                                      int end_index,
                                                      const std::vector<int>& fixed_indices) const
{
  // Calculate longest valid segment length
  const Eigen::MatrixX2d& limits = pci.kin->getLimits().joint_limits;
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
                                                            int start_index,
                                                            int end_index,
                                                            const std::vector<int>& fixed_indices) const
{
  // Calculate longest valid segment length
  const Eigen::MatrixX2d& limits = pci.kin->getLimits().joint_limits;
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
                                                          int start_index,
                                                          int end_index,
                                                          const std::vector<int>& /*fixed_indices*/) const
{
  if (velocity_coeff.size() == 0)
    pci.cost_infos.push_back(
        createSmoothVelocityTermInfo(start_index, end_index, static_cast<int>(pci.kin->numJoints())));
  else
    pci.cost_infos.push_back(createSmoothVelocityTermInfo(start_index, end_index, velocity_coeff));
}

void TrajOptDefaultCompositeProfile::addAccelerationSmoothing(trajopt::ProblemConstructionInfo& pci,
                                                              int start_index,
                                                              int end_index,
                                                              const std::vector<int>& /*fixed_indices*/) const
{
  if (acceleration_coeff.size() == 0)
    pci.cost_infos.push_back(
        createSmoothAccelerationTermInfo(start_index, end_index, static_cast<int>(pci.kin->numJoints())));
  else
    pci.cost_infos.push_back(createSmoothAccelerationTermInfo(start_index, end_index, acceleration_coeff));
}

void TrajOptDefaultCompositeProfile::addJerkSmoothing(trajopt::ProblemConstructionInfo& pci,
                                                      int start_index,
                                                      int end_index,
                                                      const std::vector<int>& /*fixed_indices*/) const
{
  if (jerk_coeff.size() == 0)
    pci.cost_infos.push_back(createSmoothJerkTermInfo(start_index, end_index, static_cast<int>(pci.kin->numJoints())));
  else
    pci.cost_infos.push_back(createSmoothJerkTermInfo(start_index, end_index, jerk_coeff));
}

void TrajOptDefaultCompositeProfile::addAvoidSingularity(trajopt::ProblemConstructionInfo& pci,
                                                         int start_index,
                                                         int end_index,
                                                         const std::string& link,
                                                         const std::vector<int>& /*fixed_indices*/) const
{
  trajopt::TermInfo::Ptr ti = createAvoidSingularityTermInfo(start_index, end_index, link, avoid_singularity_coeff);
  pci.cost_infos.push_back(ti);
}

}  // namespace tesseract_planning
