#include <tesseract_planning/core/macros.h>
TESSERACT_PLANNING_IGNORE_WARNINGS_PUSH
#include <ompl/base/spaces/RealVectorStateSpace.h>
TESSERACT_PLANNING_IGNORE_WARNINGS_POP

#include "tesseract_planning/ompl/chain_ompl_interface.h"

namespace tesseract_planning
{
ChainOmplInterface::ChainOmplInterface(tesseract_environment::EnvironmentConstPtr env,
                                       tesseract_kinematics::ForwardKinematicsConstPtr kin)
  : env_(std::move(env)), kin_(std::move(kin))
{
  joint_names_ = kin_->getJointNames();

  // kinematics objects does not know of every link affected by its motion so must compute adjacency map
  // to determine all active links.
  tesseract_environment::AdjacencyMap adj_map(env_->getSceneGraph(), kin_->getActiveLinkNames(), env_->getCurrentState()->transforms);
  link_names_ = adj_map.getActiveLinkNames();

  const auto dof = kin_->numJoints();
  const auto& limits = kin_->getLimits();

  // Construct the OMPL state space for this manipulator
  ompl::base::RealVectorStateSpace* space = new ompl::base::RealVectorStateSpace();
  for (unsigned i = 0; i < dof; ++i)
  {
    space->addDimension(joint_names_[i], limits(i, 0), limits(i, 1));
  }

  ompl::base::StateSpacePtr state_space_ptr(space);
  ss_.reset(new ompl::geometric::SimpleSetup(state_space_ptr));

  // Setup state checking functionality
  ss_->setStateValidityChecker(std::bind(&ChainOmplInterface::isStateValid, this, std::placeholders::_1));

  contact_manager_ = env_->getDiscreteContactManager();
  contact_manager_->setActiveCollisionObjects(link_names_);
  contact_manager_->setContactDistanceThreshold(0);

  // We need to set the planner and call setup before it can run
}

boost::optional<ompl::geometric::PathGeometric> ChainOmplInterface::plan(ompl::base::PlannerPtr planner,
                                                                         const std::vector<double>& from,
                                                                         const std::vector<double>& to,
                                                                         const OmplPlanParameters& params)
{
  ss_->setPlanner(planner);
  planner->clear();

  const auto dof = ss_->getStateSpace()->getDimension();
  ompl::base::ScopedState<> start_state(ss_->getStateSpace());
  for (unsigned i = 0; i < dof; ++i)
    start_state[i] = from[i];

  ompl::base::ScopedState<> goal_state(ss_->getStateSpace());
  for (unsigned i = 0; i < dof; ++i)
    goal_state[i] = to[i];

  ss_->setStartAndGoalStates(start_state, goal_state);

  ompl::base::PlannerStatus status = ss_->solve(params.planning_time);

  if (status)
  {
    if (params.simplify)
    {
      ss_->simplifySolution();
    }

    ompl::geometric::PathGeometric& path = ss_->getSolutionPath();
    return boost::optional<ompl::geometric::PathGeometric>{ path };
  }
  else
  {
    return {};
  }
}

ompl::base::SpaceInformationPtr ChainOmplInterface::spaceInformation() { return ss_->getSpaceInformation(); }
bool ChainOmplInterface::isStateValid(const ompl::base::State* state) const
{
  const ompl::base::RealVectorStateSpace::StateType* s = state->as<ompl::base::RealVectorStateSpace::StateType>();
  const auto dof = joint_names_.size();

  Eigen::Map<Eigen::VectorXd> joint_angles(s->values, long(dof));
  tesseract_environment::EnvStateConstPtr env_state = env_->getState(joint_names_, joint_angles);

  // Need to get thread id
  tesseract_collision::DiscreteContactManagerPtr cm = contact_manager_->clone();
  cm->setCollisionObjectsTransform(env_state->transforms);

  tesseract_collision::ContactResultMap contact_map;
  cm->contactTest(contact_map, tesseract_collision::ContactTestTypes::FIRST);

  return contact_map.empty();
}
}
