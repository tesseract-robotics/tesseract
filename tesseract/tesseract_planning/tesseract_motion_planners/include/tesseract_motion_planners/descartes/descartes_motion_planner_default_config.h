#ifndef TESSERACT_MOTION_PLANNERS_DESCARTES_MOTION_PLANNER_DEFAULT_CONFIG_H
#define TESSERACT_MOTION_PLANNERS_DESCARTES_MOTION_PLANNER_DEFAULT_CONFIG_H

#include <tesseract/tesseract.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <descartes_light/interface/collision_interface.h>
#include <descartes_light/interface/kinematics_interface.h>
#include <descartes_light/interface/edge_evaluator.h>
#include <descartes_light/interface/position_sampler.h>
#include <descartes_light/descartes_light.h>
#include <Eigen/Geometry>
#include <vector>
#include <string>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract/tesseract.h>
#include <tesseract_kinematics/core/validate.h>
#include <tesseract_command_language/composite_instruction.h>
#include <tesseract_command_language/plan_instruction.h>
#include <tesseract_command_language/move_instruction.h>
#include <tesseract_command_language/cartesian_waypoint.h>
#include <tesseract_command_language/joint_waypoint.h>

#include <tesseract_kinematics/core/forward_kinematics.h>
#include <tesseract_kinematics/core/inverse_kinematics.h>

#include <tesseract_motion_planners/descartes/descartes_motion_planner_config.h>
#include <tesseract_motion_planners/descartes/descartes_collision.h>
#include <tesseract_motion_planners/descartes/descartes_utils.h>
#include <tesseract_motion_planners/descartes/descartes_problem.h>
#include <tesseract_motion_planners/descartes/profile/descartes_default_plan_profile.h>
#include <tesseract_motion_planners/core/utils.h>

namespace tesseract_planning
{
template <typename FloatType>
struct DescartesMotionPlannerDefaultConfig : public DescartesMotionPlannerConfig<FloatType>
{
  using Ptr = std::shared_ptr<DescartesMotionPlannerDefaultConfig<FloatType>>;
  using ConstPtr = std::shared_ptr<const DescartesMotionPlannerDefaultConfig<FloatType>>;

  DescartesMotionPlannerDefaultConfig(tesseract::Tesseract::ConstPtr tesseract,
                                      tesseract_environment::EnvState::ConstPtr env_state,
                                      std::string manipulator,
                                      double manipulator_reach);

  virtual ~DescartesMotionPlannerDefaultConfig() = default;
  DescartesMotionPlannerDefaultConfig(const DescartesMotionPlannerDefaultConfig&) = default;
  DescartesMotionPlannerDefaultConfig& operator=(const DescartesMotionPlannerDefaultConfig&) = default;
  DescartesMotionPlannerDefaultConfig(DescartesMotionPlannerDefaultConfig&&) = default;             // NOLINT
  DescartesMotionPlannerDefaultConfig& operator=(DescartesMotionPlannerDefaultConfig&&) = default;  // NOLINT

  typename DescartesProblem<FloatType>::Configuration configuration {DescartesProblem<FloatType>::Configuration::ROBOT_ONLY};

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
  std::unordered_map<std::string, typename DescartesPlanProfile<FloatType>::Ptr> plan_profiles;

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

  void getManipulatorInfo();
};

using DescartesMotionPlannerDefaultConfigD = DescartesMotionPlannerDefaultConfig<double>;
using DescartesMotionPlannerDefaultConfigF = DescartesMotionPlannerDefaultConfig<float>;
}
#endif // TESSERACT_MOTION_PLANNERS_DESCARTES_MOTION_PLANNER_DEFAULT_CONFIG_H
