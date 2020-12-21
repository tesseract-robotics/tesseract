/**
 * @file descartes_default_plan_profile.h
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
#ifndef TESSERACT_MOTION_PLANNERS_DESCARTES_DESCARTES_DEFAULT_PLAN_PROFILE_H
#define TESSERACT_MOTION_PLANNERS_DESCARTES_DESCARTES_DEFAULT_PLAN_PROFILE_H

#include <tesseract_common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <Eigen/Geometry>
#include <tesseract_collision/core/types.h>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract_motion_planners/descartes/profile/descartes_profile.h>
#include <tesseract_motion_planners/descartes/descartes_utils.h>
#include <tesseract_motion_planners/descartes/types.h>

#ifdef SWIG
%shared_ptr(tesseract_planning::DescartesDefaultPlanProfile<double>)
%ignore tesseract_planning::DescartesDefaultPlanProfile::edge_evaluator;
%ignore tesseract_planning::DescartesDefaultPlanProfile::is_valid;
#endif  // SWIG

namespace tesseract_planning
{
template <typename FloatType>
class DescartesDefaultPlanProfile : public DescartesPlanProfile<FloatType>
{
public:
  using Ptr = std::shared_ptr<DescartesDefaultPlanProfile<FloatType>>;
  using ConstPtr = std::shared_ptr<const DescartesDefaultPlanProfile<FloatType>>;

  DescartesDefaultPlanProfile() = default;
  ~DescartesDefaultPlanProfile() override = default;
  DescartesDefaultPlanProfile(const DescartesDefaultPlanProfile<FloatType>&) = default;
  DescartesDefaultPlanProfile& operator=(const DescartesDefaultPlanProfile&) = default;
  DescartesDefaultPlanProfile(DescartesDefaultPlanProfile&&) noexcept = default;
  DescartesDefaultPlanProfile& operator=(DescartesDefaultPlanProfile&&) noexcept = default;
  DescartesDefaultPlanProfile(const tinyxml2::XMLElement& xml_element);

  PoseSamplerFn target_pose_sampler = sampleFixed;

#ifndef SWIG
  DescartesEdgeEvaluatorAllocatorFn<FloatType> edge_evaluator{ nullptr };

  // If not provided it adds a joint limit is valid function
  DescartesVertexEvaluatorAllocatorFn<FloatType> vertex_evaluator{ nullptr };
#endif

  double timing_constraint = std::numeric_limits<FloatType>::max();

  // Applied to sampled states
  bool enable_collision{ true };
  double collision_safety_margin{ 0 };

  // Applied during edge evaluation
  bool enable_edge_collision{ false };
  tesseract_collision::CollisionCheckConfig edge_collision_check_config;
  int num_threads{ 1 };
  bool allow_collision{ false };
  bool debug{ false };

  void apply(DescartesProblem<FloatType>& prob,
             const Eigen::Isometry3d& cartesian_waypoint,
             const Instruction& parent_instruction,
             const ManipulatorInfo& manip_info,
             const std::vector<std::string>& active_links,
             int index) const override;

  void apply(DescartesProblem<FloatType>& prob,
             const Eigen::VectorXd& joint_waypoint,
             const Instruction& parent_instruction,
             const ManipulatorInfo& manip_info,
             const std::vector<std::string>& active_links,
             int index) const override;

  tinyxml2::XMLElement* toXML(tinyxml2::XMLDocument& doc) const override;
};

using DescartesDefaultPlanProfileF = DescartesDefaultPlanProfile<float>;
using DescartesDefaultPlanProfileD = DescartesDefaultPlanProfile<double>;
}  // namespace tesseract_planning

#ifdef SWIG
%template(DescartesDefaultPlanProfileD) tesseract_planning::DescartesDefaultPlanProfile<double>;
#endif  // SWIG

#endif  // TESSERACT_MOTION_PLANNERS_DESCARTES_DESCARTES_DEFAULT_PLAN_PROFILE_H
