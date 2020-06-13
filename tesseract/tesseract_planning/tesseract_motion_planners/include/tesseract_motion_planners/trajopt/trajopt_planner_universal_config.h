#ifndef TESSERACT_MOTION_PLANNERS_TRAJOPT_PLANNER_UNIVERSAL_CONFIG_H
#define TESSERACT_MOTION_PLANNERS_TRAJOPT_PLANNER_UNIVERSAL_CONFIG_H

#include <tesseract_motion_planners/trajopt/trajopt_planner_config.h>
#include <tesseract_motion_planners/trajopt/trajopt_collision_config.h>
#include <tesseract_motion_planners/trajopt/profile/trajopt_profile.h>
#include <tesseract_command_language/composite_instruction.h>

namespace tesseract_planning
{

/**
 * @brief Default configuration to setup TrajOpt planner.
 *
 * While there are many parameters that can be set, the majority of them have defaults that will be suitable for many
 * problems. These are always required: tesseract_, maninpulator_, link_, tcp_
 *
 */
struct TrajOptPlannerUniversalConfig : public TrajOptPlannerConfig
{
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  using Ptr = std::shared_ptr<TrajOptPlannerUniversalConfig>;
  using ConstPtr = std::shared_ptr<const TrajOptPlannerUniversalConfig>;

  TrajOptPlannerUniversalConfig(tesseract::Tesseract::ConstPtr tesseract_,
                                std::string manipulator_);

  /** @brief Generates the TrajOpt problem and saves the result internally */
  bool generate() override;

  /** @brief This is used to process the results into the seed trajectory
   *
   * This is currently required because the base class is not aware of instruction
   *
   */
  void processResults(const tesseract_common::JointTrajectory& trajectory);

  /** @brief Tesseract object. ***REQUIRED*** */
  tesseract::Tesseract::ConstPtr tesseract;

  /** @brief Manipulator used for pathplanning ***REQUIRED*** */
  std::string manipulator;

  /** @brief The QP solver used in the SQP optimization routine */
  sco::ModelType optimizer = sco::ModelType::AUTO_SOLVER;

  /**
   * @brief The available composite profiles
   *
   * Composite instruction is a way to namespace or organize your planning problem. The composite instruction has a
   * profile which is used for applying multy waypoint costs and constraints like joint smoothing, collision avoidance,
   * and velocity smoothing.
   */
  std::unordered_map<std::string, TrajOptCompositeProfile::Ptr> composite_profiles;

  /**
   * @brief The available plan profiles
   *
   * Plan instruction profiles are used to control waypoint specific information like fixed waypoint, toleranced
   * waypoint, corner distance waypoint, etc.
   */
  std::unordered_map<std::string, TrajOptPlanProfile::Ptr> plan_profiles;

  /**
   * @brief The program instruction
   * This must containt a minimum of two move instruction the first move instruction is the start state
   */
  tesseract_planning::CompositeInstruction instructions;

  /**
   * @brief This should be a one to one match with the instructions where the PlanInstruction is replaced with a
   * composite instruction of MoveInstructions.
   */
  tesseract_planning::CompositeInstruction seed;

protected:
  std::vector<std::size_t> plan_instruction_indices_;
  bool checkUserInput() const;
};

}

#endif // TESSERACT_MOTION_PLANNERS_TRAJOPT_PLANNER_UNIVERSAL_CONFIG_H
