
/**
 * @file ompl_motion_planner_default_config.cpp
 * @brief Tesseract OMPL motion planner default config implementation.
 *
 * @author Levi Armstrong
 * @date January 22, 2020
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

#include <tesseract_motion_planners/ompl/ompl_motion_planner_default_config.h>
#include <tesseract_motion_planners/ompl/profile/ompl_default_plan_profile.h>
#include <tesseract_kinematics/core/validate.h>
#include <tesseract/tesseract.h>

namespace tesseract_planning
{
OMPLMotionPlannerDefaultConfig::OMPLMotionPlannerDefaultConfig(tesseract::Tesseract::ConstPtr tesseract,
                                                               tesseract_environment::EnvState::ConstPtr env_state,
                                                               std::string manipulator)
  : tesseract(std::move(tesseract)), env_state(std::move(env_state)), manipulator(std::move(manipulator))
{
}

void OMPLMotionPlannerDefaultConfig::init()
{
  manip_fwd_kin_ = tesseract->getFwdKinematicsManagerConst()->getFwdKinematicSolver(manipulator);
  if (manipulator_ik_solver.empty())
    manip_inv_kin_ = tesseract->getInvKinematicsManagerConst()->getInvKinematicSolver(manipulator);
  else
    manip_inv_kin_ =
        tesseract->getInvKinematicsManagerConst()->getInvKinematicSolver(manipulator, manipulator_ik_solver);

  // Get Active Link Names
  std::vector<std::string> active_link_names = manip_inv_kin_->getActiveLinkNames();
  auto adjacency_map = std::make_shared<tesseract_environment::AdjacencyMap>(
      tesseract->getEnvironmentConst()->getSceneGraph(), active_link_names, env_state->link_transforms);
  active_link_names_ = adjacency_map->getActiveLinkNames();
}

void OMPLMotionPlannerDefaultConfig::clear()
{
  prob.clear();

  manip_fwd_kin_ = nullptr;
  manip_inv_kin_ = nullptr;
  positioner_fwd_kin_ = nullptr;
  active_link_names_.clear();
}

OMPLProblem::UPtr OMPLMotionPlannerDefaultConfig::createSubProblem()
{
  auto sub_prob = std::make_unique<OMPLProblem>();
  sub_prob->tesseract = tesseract;
  sub_prob->env_state = env_state;
  sub_prob->state_solver = tesseract->getEnvironmentConst()->getStateSolver();
  sub_prob->state_solver->setState(env_state->joints);
  sub_prob->manip_fwd_kin = manip_fwd_kin_;
  sub_prob->manip_inv_kin = manip_inv_kin_;
  sub_prob->contact_checker = tesseract->getEnvironmentConst()->getDiscreteContactManager();
  sub_prob->contact_checker->setCollisionObjectsTransform(env_state->link_transforms);
  sub_prob->contact_checker->setActiveCollisionObjects(active_link_names_);
  return sub_prob;
}

bool OMPLMotionPlannerDefaultConfig::generate()
{
  // Clear ompl data
  clear();

  // Initialize Information
  init();

  // Process instructions
  if (!tesseract_kinematics::checkKinematics(manip_fwd_kin_, manip_inv_kin_))
    CONSOLE_BRIDGE_logError("Check Kinematics failed. This means that Inverse Kinematics does not agree with KDL "
                            "(TrajOpt). Did you change the URDF recently?");

  // Check and make sure it does not contain any composite instruction
  for (const auto& instruction : instructions)
    if (instruction.isComposite())
      throw std::runtime_error("OMPL planner does not support child composite instructions.");

  Waypoint start_waypoint = NullWaypoint();
  if (instructions.hasStartWaypoint())
  {
    start_waypoint = instructions.getStartWaypoint();
  }
  else
  {
    Eigen::VectorXd current_jv = env_state->getJointValues(manip_fwd_kin_->getJointNames());
    JointWaypoint temp(current_jv);
    temp.joint_names = manip_fwd_kin_->getJointNames();
    start_waypoint = temp;
  }

  // Transform plan instructions into ompl problem
  int index = 0;
  for (std::size_t i = 0; i < instructions.size(); ++i)
  {
    const auto& instruction = instructions[i];
    if (instruction.isPlan())
    {
      // Save plan index for process trajectory
      plan_instruction_indices_.push_back(i);

      assert(instruction.getType() == static_cast<int>(InstructionType::PLAN_INSTRUCTION));
      const auto* plan_instruction = instruction.cast_const<PlanInstruction>();

      assert(seed[i].isComposite());
      const auto* seed_composite = seed[i].cast_const<tesseract_planning::CompositeInstruction>();

      // Get Plan Profile
      std::string profile = plan_instruction->getProfile();
      if (profile.empty())
        profile = "DEFAULT";

      typename OMPLPlanProfile::Ptr cur_plan_profile{ nullptr };
      auto it = plan_profiles.find(profile);
      if (it == plan_profiles.end())
        cur_plan_profile = std::make_shared<OMPLDefaultPlanProfile>();
      else
        cur_plan_profile = it->second;

      /** @todo Should check that the joint names match the order of the manipulator */
      OMPLProblem::UPtr sub_prob = createSubProblem();
      cur_plan_profile->setup(*sub_prob);
      sub_prob->n_output_states = static_cast<int>(seed_composite->size());

      if (plan_instruction->isLinear())
      {
        /** @todo Add support for linear motion to ompl planner */
        if (isCartesianWaypoint(plan_instruction->getWaypoint().getType()))
        {
          // TODO Currently skipping linear moves until SE3 motion planning is implemented.
          prob.push_back(nullptr);
          ++index;
        }
        else if (isJointWaypoint(plan_instruction->getWaypoint().getType()))
        {
          // TODO Currently skipping linear moves until SE3 motion planning is implemented.
          prob.push_back(nullptr);
          ++index;
        }
        else
        {
          throw std::runtime_error("OMPLMotionPlannerDefaultConfig: unknown waypoint type");
        }
      }
      else if (plan_instruction->isFreespace())
      {
        if (isJointWaypoint(plan_instruction->getWaypoint().getType()))
        {
          const auto* cur_wp = plan_instruction->getWaypoint().cast_const<tesseract_planning::JointWaypoint>();
          cur_plan_profile->applyGoalStates(*sub_prob, *cur_wp, *plan_instruction, active_link_names_, index);

          if (index == 0)
          {
            ompl::base::ScopedState<> start_state(sub_prob->simple_setup->getStateSpace());
            if (isJointWaypoint(start_waypoint.getType()))
            {
              const auto* prev_wp = start_waypoint.cast_const<tesseract_planning::JointWaypoint>();
              cur_plan_profile->applyStartStates(*sub_prob, *prev_wp, *plan_instruction, active_link_names_, index);
            }
            else if (isCartesianWaypoint(start_waypoint.getType()))
            {
              const auto* prev_wp = start_waypoint.cast_const<tesseract_planning::CartesianWaypoint>();
              cur_plan_profile->applyStartStates(*sub_prob, *prev_wp, *plan_instruction, active_link_names_, index);
            }
            else
            {
              throw std::runtime_error("OMPLMotionPlannerDefaultConfig: unknown waypoint type");
            }

            prob.push_back(std::move(sub_prob));
            ++index;
          }
          else
          {
            /** @todo Update. Extract the solution for the previous plan and set as the start */
            assert(false);
          }
        }
        else if (isCartesianWaypoint(plan_instruction->getWaypoint().getType()))
        {
          const auto* cur_wp = plan_instruction->getWaypoint().cast_const<tesseract_planning::CartesianWaypoint>();
          cur_plan_profile->applyGoalStates(*sub_prob, *cur_wp, *plan_instruction, active_link_names_, index);

          if (index == 0)
          {
            ompl::base::ScopedState<> start_state(sub_prob->simple_setup->getStateSpace());
            if (isJointWaypoint(start_waypoint.getType()))
            {
              const auto* prev_wp = start_waypoint.cast_const<tesseract_planning::JointWaypoint>();
              cur_plan_profile->applyStartStates(*sub_prob, *prev_wp, *plan_instruction, active_link_names_, index);
            }
            else if (isCartesianWaypoint(start_waypoint.getType()))
            {
              const auto* prev_wp = start_waypoint.cast_const<tesseract_planning::CartesianWaypoint>();
              cur_plan_profile->applyStartStates(*sub_prob, *prev_wp, *plan_instruction, active_link_names_, index);
            }
            else
            {
              throw std::runtime_error("OMPLMotionPlannerDefaultConfig: unknown waypoint type");
            }
          }
          else
          {
            /** @todo Update. Extract the solution for the previous plan and set as the start */
            assert(false);
          }

          prob.push_back(std::move(sub_prob));
          ++index;
        }
      }
      else
      {
        throw std::runtime_error("OMPLMotionPlannerDefaultConfig: Unsupported!");
      }

      start_waypoint = plan_instruction->getWaypoint(); /** @todo need to extract the solution */
    }
  }

  return OMPLMotionPlannerConfig::generate();
}
}  // namespace tesseract_planning
