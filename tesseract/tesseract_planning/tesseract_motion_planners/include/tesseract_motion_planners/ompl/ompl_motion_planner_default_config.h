/**
 * @file ompl_motion_planner_default_config.h
 * @brief Tesseract OMPL motion planner default config.
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
#ifndef TESSERACT_MOTION_PLANNERS_OMPL_OMPL_MOTION_PLANNER_DEFAULT_CONFIG_H
#define TESSERACT_MOTION_PLANNERS_OMPL_OMPL_MOTION_PLANNER_DEFAULT_CONFIG_H

#include <tesseract_common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <ompl/geometric/SimpleSetup.h>
#include <ompl/base/OptimizationObjective.h>
#include <ompl/tools/multiplan/ParallelPlan.h>
#include <ompl/base/objectives/PathLengthOptimizationObjective.h>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract_command_language/composite_instruction.h>
#include <tesseract_command_language/plan_instruction.h>
#include <tesseract_command_language/move_instruction.h>
#include <tesseract_command_language/cartesian_waypoint.h>
#include <tesseract_command_language/joint_waypoint.h>

#include <tesseract_motion_planners/ompl/utils.h>
#include <tesseract_motion_planners/ompl/ompl_motion_planner_config.h>
#include <tesseract_motion_planners/ompl/profile/ompl_profile.h>

#include <tesseract_collision/core/discrete_contact_manager.h>
namespace tesseract_planning
{
/** @brief The OMPLPlannerConfig struct */
struct OMPLMotionPlannerDefaultConfig : public OMPLMotionPlannerConfig
{
  explicit OMPLMotionPlannerDefaultConfig(tesseract::Tesseract::ConstPtr tesseract,
                                          tesseract_environment::EnvState::ConstPtr env_state,
                                          std::string manipulator);

  tesseract::Tesseract::ConstPtr tesseract;
  tesseract_environment::EnvState::ConstPtr env_state;

  OMPLProblemConfiguration configuration {OMPLProblemConfiguration::REAL_STATE_SPACE};

  std::string manipulator;
  std::string manipulator_ik_solver;
  std::string positioner;
  double manipulator_reach;

  /**
   * @brief The available plan profiles
   *
   * Plan instruction profiles are used to control waypoint specific information like fixed waypoint, toleranced
   * waypoint, corner distance waypoint, etc.
   */
  std::unordered_map<std::string, OMPLPlanProfile::Ptr> plan_profiles;

  /**
   * @brief The program instruction
   * This must containt a minimum of two move instruction the first move instruction is the start state
   */
  CompositeInstruction instructions;

  /**
   * @brief This should be a one to one match with the instructions where the PlanInstruction is replaced with a
   * composite instruction of MoveInstructions.
   */
  CompositeInstruction seed;

  bool generate() override;

private:
  std::vector<std::size_t> plan_instruction_indices_;

  // Kinematic Objects
  tesseract_kinematics::ForwardKinematics::ConstPtr manip_fwd_kin_;
  tesseract_kinematics::InverseKinematics::ConstPtr manip_inv_kin_;
  tesseract_kinematics::ForwardKinematics::ConstPtr positioner_fwd_kin_;
  std::vector<std::string> active_link_names_;
  OMPLStateExtractor extractor_;

  void init();

  void clear();

  OMPLProblem::UPtr createSubProblem();
};
}
#endif // TESSERACT_MOTION_PLANNERS_OMPL_OMPL_MOTION_PLANNER_DEFAULT_CONFIG_H
