#include <tesseract_common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <ompl/geometric/SimpleSetup.h>
#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <ompl/tools/multiplan/ParallelPlan.h>
#include <ompl/base/objectives/PathLengthOptimizationObjective.h>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract_motion_planners/ompl/profile/ompl_default_plan_profile.h>
#include <tesseract_motion_planners/ompl/utils.h>

#include <tesseract_motion_planners/ompl/continuous_motion_validator.h>
#include <tesseract_motion_planners/ompl/discrete_motion_validator.h>
#include <tesseract_motion_planners/ompl/weighted_real_vector_state_sampler.h>
#include <tesseract_motion_planners/ompl/state_collision_validator.h>
#include <tesseract_motion_planners/ompl/compound_state_validator.h>

namespace tesseract_planning
{
void OMPLDefaultPlanProfile::apply(OMPLProblem& prob,
                                   const Eigen::Isometry3d& cartesian_waypoint,
                                   const PlanInstruction& parent_instruction,
                                   const std::vector<std::string> &active_links,
                                   int index)
{
  assert(false);
}

void OMPLDefaultPlanProfile::apply(OMPLProblem& prob,
                                   const Eigen::VectorXd& joint_waypoint,
                                   const PlanInstruction& parent_instruction,
                                   const std::vector<std::string> &active_links,
                                   int index)
{
  prob.planners = planners;
  prob.max_solutions = max_solutions;
  prob.simplify = simplify;
  prob.optimize = optimize;
  prob.contact_checker->setContactDistanceThreshold(collision_safety_margin);

  const tesseract_environment::Environment::ConstPtr& env = prob.tesseract->getEnvironmentConst();
  const std::vector<std::string>& joint_names = prob.manip_fwd_kin->getJointNames();
  const auto dof = prob.manip_fwd_kin->numJoints();
  const auto& limits = prob.manip_fwd_kin->getLimits();

  if (weights.size() == 1)
    weights = Eigen::VectorXd::Constant(1, dof, weights(0));
  else if (weights.size() != dof)
    weights = Eigen::VectorXd::Ones(dof);

  if (prob.configuration == OMPLProblemConfiguration::REAL_STATE_SPACE)
  {
      // Construct the OMPL state space for this manipulator
      ompl::base::StateSpacePtr state_space_ptr;

      auto rss = std::make_shared<ompl::base::RealVectorStateSpace>();
      for (unsigned i = 0; i < dof; ++i)
        rss->addDimension(joint_names[i], limits(i, 0), limits(i, 1));

      if (state_sampler_allocator)
      {
        rss->setStateSamplerAllocator(state_sampler_allocator);
      }
      else
      {
        rss->setStateSamplerAllocator(std::bind(&OMPLDefaultPlanProfile::allocWeightedRealVectorStateSampler, this, std::placeholders::_1, limits));
      }

      state_space_ptr = rss;

      // Setup Longest Valid Segment
      processLongestValidSegment(state_space_ptr, longest_valid_segment_fraction, longest_valid_segment_length);

      // Create Simple Setup from state space
      prob.simple_setup = std::make_shared<ompl::geometric::SimpleSetup>(state_space_ptr);

      // Get descrete contact manager for testing provided start and end position
      // This is required because collision checking happens in motion validators now
      // instead of the isValid function to avoid unnecessary collision checks.
      if (checkStateInCollision(prob, joint_waypoint))
      {
        CONSOLE_BRIDGE_logError("In OMPLPlannerFreespaceConfig: Start state is in collision");
      }

      ompl::base::ScopedState<> goal_state(prob.simple_setup->getStateSpace());
      for (unsigned i = 0; i < dof; ++i)
        goal_state[i] = joint_waypoint[i];

      prob.simple_setup->setGoalState(goal_state);

      // Setup state checking functionality
      ompl::base::StateValidityCheckerPtr svc_without_collision = processStateValidator(prob, env, prob.manip_fwd_kin);

      // Setup motion validation (i.e. collision checking)
      processMotionValidator(svc_without_collision, prob, env, prob.manip_fwd_kin);

      // make sure the planners run until the time limit, and get the best possible solution
      processOptimizationObjective(prob);
  }
}

ompl::base::StateValidityCheckerPtr
OMPLDefaultPlanProfile::processStateValidator(OMPLProblem& prob,
                                              const tesseract_environment::Environment::ConstPtr& env,
                                              const tesseract_kinematics::ForwardKinematics::ConstPtr& kin)
{
  ompl::base::StateValidityCheckerPtr svc_without_collision;
  auto csvc = std::make_shared<CompoundStateValidator>();
  if (svc_allocator != nullptr)
  {
    svc_without_collision = svc_allocator(prob.simple_setup->getSpaceInformation(), prob);
    csvc->addStateValidator(svc_without_collision);
  }

  if (collision_check && !collision_continuous)
  {
    auto svc = std::make_shared<StateCollisionValidator>(
        prob.simple_setup->getSpaceInformation(), env, kin, collision_safety_margin, prob.extractor);
    csvc->addStateValidator(svc);
  }
  prob.simple_setup->setStateValidityChecker(csvc);

  return svc_without_collision;
}

void OMPLDefaultPlanProfile::processMotionValidator(ompl::base::StateValidityCheckerPtr svc_without_collision,
                                                    OMPLProblem& prob,
                                                    const tesseract_environment::Environment::ConstPtr& env,
                                                    const tesseract_kinematics::ForwardKinematics::ConstPtr& kin)
{
  if (mv_allocator != nullptr)
  {
    auto mv = mv_allocator(prob.simple_setup->getSpaceInformation(), prob);
    prob.simple_setup->getSpaceInformation()->setMotionValidator(mv);
  }
  else
  {
    if (collision_check)
    {
      ompl::base::MotionValidatorPtr mv;
      if (collision_continuous)
      {
        mv = std::make_shared<ContinuousMotionValidator>(
            prob.simple_setup->getSpaceInformation(), svc_without_collision, env, kin, collision_safety_margin, prob.extractor);
      }
      else
      {
        // Collision checking is preformed using the state validator which this calles.
        mv = std::make_shared<DiscreteMotionValidator>(prob.simple_setup->getSpaceInformation());
      }
      prob.simple_setup->getSpaceInformation()->setMotionValidator(mv);
    }
  }
}

void OMPLDefaultPlanProfile::processOptimizationObjective(OMPLProblem &prob)
{
  if (optimization_objective_allocator)
  {
    prob.simple_setup->getProblemDefinition()->setOptimizationObjective(
        optimization_objective_allocator(prob.simple_setup->getSpaceInformation(), prob));
  }
}

ompl::base::StateSamplerPtr
OMPLDefaultPlanProfile::allocWeightedRealVectorStateSampler(const ompl::base::StateSpace* space,
                                                            const Eigen::MatrixX2d& limits) const
{
  return std::make_shared<WeightedRealVectorStateSampler>(space, weights, limits);
}

}
