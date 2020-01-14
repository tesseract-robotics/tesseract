/**
 * @file trajopt_planner_default_config.i
 * @brief SWIG interface file for tesseract_planning/trajopt/config/trajopt_planner_default_config.h
 *
 * @author John Wason
 * @date December 10, 2019
 * @version TODO
 * @bug No known bugs
 *
 * @copyright Copyright (c) 2019, Wason Technology, LLC
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

%{
#include <tesseract_motion_planners/trajopt/config/trajopt_planner_default_config.h>
%}

%shared_ptr(tesseract_motion_planners::TrajOptPlannerDefaultConfig)

namespace tesseract_motion_planners
{
struct TrajOptPlannerDefaultConfig : public TrajOptPlannerConfig
{
  
  TrajOptPlannerDefaultConfig(const tesseract::Tesseract::ConstPtr& tesseract_,
                              const std::string& manipulator_,
                              const std::string& link_,
                              const tesseract_common::VectorIsometry3d& tcp_);

  TrajOptPlannerDefaultConfig(const tesseract::Tesseract::ConstPtr& tesseract_,
                              const std::string& manipulator_,
                              const std::string& link_,
                              const Eigen::Isometry3d& tcp_);

  virtual std::shared_ptr<trajopt::ProblemConstructionInfo> generatePCI() const;

  virtual bool generate() override;

  tesseract::Tesseract::ConstPtr tesseract;
  
  std::string manipulator;
  
  std::string link;

  sco::ModelType optimizer = sco::ModelType::AUTO_SOLVER;

  tesseract_common::VectorIsometry3d tcp;

  std::vector<Waypoint::Ptr> target_waypoints;

  trajopt::InitInfo::Type init_type;
  
  trajopt::TrajArray seed_trajectory;
  
  JointWaypoint::ConstPtr configuration;

  tesseract_collision::ContactTestType contact_test_type = tesseract_collision::ContactTestType::ALL;

  bool collision_check = true;
  
  trajopt::CollisionEvaluatorType collision_type = trajopt::CollisionEvaluatorType::CAST_CONTINUOUS;
  
  double collision_safety_margin = 0.025;
  
  bool smooth_velocities = true;
  
  Eigen::VectorXd velocity_coeff;
  
  bool smooth_accelerations = true;
  
  Eigen::VectorXd acceleration_coeff;
  
  bool smooth_jerks = true;
  
  Eigen::VectorXd jerk_coeff;

  bool avoid_singularity = false;

  double avoid_singularity_coeff = 5.0;

  // std::vector<std::pair<sco::VectorOfVector::func, sco::MatrixOfVector::func>> constraint_error_functions;
};

}  // namespace tesseract_motion_planners
