/**
 * @file ompl_default_plan_profile.h
 * @brief Tesseract OMPL default plan profile
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
#ifndef TESSERACT_MOTION_PLANNERS_OMPL_OMPL_DEFAULT_PLAN_PROFILE_H
#define TESSERACT_MOTION_PLANNERS_OMPL_OMPL_DEFAULT_PLAN_PROFILE_H

#include <tesseract_motion_planners/ompl/utils.h>
#include <tesseract_motion_planners/ompl/ompl_planner_configurator.h>
#include <tesseract_motion_planners/ompl/profile/ompl_profile.h>
#include <tesseract_motion_planners/ompl/types.h>

namespace tesseract_planning
{

/**
 * @brief OMPL does not support the concept of multi waypoint planning like descartes and trajopt. Because of this
 * every plan instruction will be its a seperate ompl motion plan and therefore planning information is relevent
 * for this motion planner in the profile.
 */
class OMPLDefaultPlanProfile : public OMPLPlanProfile
{
public:
  using Ptr = std::shared_ptr<OMPLDefaultPlanProfile>;
  using ConstPtr = std::shared_ptr<const OMPLDefaultPlanProfile>;

  /** @brief Max planning time allowed in seconds */
  double planning_time = 5.0;

  /** @brief The max number of solutions. If max solutions are hit it will exit even if other threads are running. */
  int max_solutions = 10;

  /**
   * @brief Simplify trajectory.
   *
   * Note: If set to true it ignores n_output_states and returns the simplest trajectory.
   */
  bool simplify = true;

  /**
   * @brief This uses all available planning time to create the most optimized trajectory given the objective function.
   *
   * This is required because not all OMPL planners are optimize graph planners. If the planner you choose is an
   * optimize graph planner then setting this to true has no affect. In the case of non-optimize planners they still
   * use the OptimizeObjective function but only when searching the graph to find the most optimize solution based
   * on the provided optimize objective function. In the case of these type of planners like RRT and RRTConnect if set
   * to true it will leverage all planning time to keep finding solutions up to your max solutions count to find the
   * most optimal solution.
   */
  bool optimize = false;

  /**
   * @brief The planner configurators ***REQUIRED***
   *
   * This will create a new thead for each planner configurator provided. T
   */
  std::vector<OMPLPlannerConfigurator::ConstPtr> planners;

  /** @brief If true, collision checking will be enabled. Default: true*/
  bool collision_check = true;

  /** @brief If true, use continuous collision checking */
  bool collision_continuous = true;

  /** @brief Max distance over which collisions are checked */
  double collision_safety_margin = 0.025;

  /** @brief Set the resolution at which state validity needs to be verified in order for a motion between two states
   * to be considered valid in post checking of trajectory returned by trajopt.
   *
   * The resolution is equal to longest_valid_segment_fraction * state_space.getMaximumExtent()
   *
   * Note: The planner takes the conservative of either longest_valid_segment_fraction or longest_valid_segment_length.
   */
  double longest_valid_segment_fraction = 0.01;  // 1%

  /** @brief Set the resolution at which state validity needs to be verified in order for a motion between two states
   * to be considered valid. If norm(state1 - state0) > longest_valid_segment_length.
   *
   * Note: This gets converted to longest_valid_segment_fraction.
   *       longest_valid_segment_fraction = longest_valid_segment_length / state_space.getMaximumExtent()
   */
  double longest_valid_segment_length = 0.5;

  /** @brief This scales the variables search space. Must be same size as number of joints.
   *         If empty it defaults to all ones */
  Eigen::VectorXd weights;

  /** @brief The state sampler allocator. This can be null and it will use Tesseract default state sampler allocator. */
  ompl::base::StateSamplerAllocator state_sampler_allocator;

  /** @brief Set the optimization objective function allocator. Default is to minimize path length */
  OptimizationObjectiveAllocator optimization_objective_allocator;

  /** @brief The ompl state validity checker. If nullptr and collision checking enabled it uses
   * StateCollisionValidator */
  StateValidityCheckerAllocator svc_allocator;

  /** @brief The ompl motion validator. If nullptr and continuous collision checking enabled it used
   * ContinuousMotionValidator */
  MotionValidatorAllocator mv_allocator;

  void apply(OMPLProblem& prob,
             const Eigen::Isometry3d& cartesian_waypoint,
             const PlanInstruction& parent_instruction,
             const std::vector<std::string> &active_links,
             int index) override;

  void apply(OMPLProblem& prob,
             const Eigen::VectorXd& joint_waypoint,
             const PlanInstruction& parent_instruction,
             const std::vector<std::string> &active_links,
             int index) override;

  /**
   * @brief Default State sampler which uses the weights information to scale the sampled state. This is use full
   * when you state space has mixed units like meters and radian.
   * @param space The ompl state space.
   * @return OMPL state sampler shared pointer
   */
  ompl::base::StateSamplerPtr allocWeightedRealVectorStateSampler(const ompl::base::StateSpace* space,
                                                                  const Eigen::MatrixX2d &limits) const;

protected:
  bool processStartAndGoalState(OMPLProblem& prob,
                                const tesseract_environment::Environment::ConstPtr& env,
                                const tesseract_kinematics::ForwardKinematics::ConstPtr& kin);
  ompl::base::StateValidityCheckerPtr processStateValidator(OMPLProblem& prob,
                                                            const tesseract_environment::Environment::ConstPtr& env,
                                                            const tesseract_kinematics::ForwardKinematics::ConstPtr& kin);
  void processMotionValidator(ompl::base::StateValidityCheckerPtr svc_without_collision,
                              OMPLProblem& prob,
                              const tesseract_environment::Environment::ConstPtr& env,
                              const tesseract_kinematics::ForwardKinematics::ConstPtr& kin);
  void processOptimizationObjective(OMPLProblem& prob);
};
}
#endif // TESSERACT_MOTION_PLANNERS_OMPL_OMPL_DEFAULT_PLAN_PROFILE_H
