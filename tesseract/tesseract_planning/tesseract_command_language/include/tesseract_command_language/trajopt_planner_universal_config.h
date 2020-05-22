#ifndef TESSERACT_MOTION_PLANNERS_TRAJOPT_PLANNER_UNIVERSAL_CONFIG_H
#define TESSERACT_MOTION_PLANNERS_TRAJOPT_PLANNER_UNIVERSAL_CONFIG_H

#include <tesseract_motion_planners/trajopt/config/trajopt_planner_config.h>
#include <tesseract_motion_planners/trajopt/config/trajopt_collision_config.h>
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
struct TrajOptPlannerUniversalConfig : public tesseract_motion_planners::TrajOptPlannerConfig
{
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  using Ptr = std::shared_ptr<TrajOptPlannerUniversalConfig>;
  using ConstPtr = std::shared_ptr<const TrajOptPlannerUniversalConfig>;

  TrajOptPlannerUniversalConfig(tesseract::Tesseract::ConstPtr tesseract_,
                                std::string manipulator_,
                                std::string link_,
                                tesseract_common::VectorIsometry3d tcp_);

  TrajOptPlannerUniversalConfig(const tesseract::Tesseract::ConstPtr& tesseract_,
                                const std::string& manipulator_,
                                const std::string& link_,
                                const Eigen::Isometry3d& tcp_);

  /** @brief Function for creating a ProblemConstructionInfo from the planner configuration */
  std::shared_ptr<trajopt::ProblemConstructionInfo> generatePCI();

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
  /** @brief This is the tip link in the kinematics object used for the cartesian positions ***REQUIRED*** */
  std::string link;

  /** @brief The QP solver used in the SQP optimization routine */
  sco::ModelType optimizer = sco::ModelType::AUTO_SOLVER;

  /**
   * @brief Vector of TCP transforms. This should contain either one transform to be applied to all waypoints
   * or a separate transform for each waypoint
   */
  tesseract_common::VectorIsometry3d tcp;

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
  bool checkUserInput() const;
  void addInstructions(trajopt::ProblemConstructionInfo& pci, std::vector<int>& fixed_steps);

  std::vector<std::size_t> plan_instruction_indices_;
};

}

#endif // TRAJOPT_PLANNER_UNIVERSAL_CONFIG_H
