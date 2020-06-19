
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
  : tesseract(std::move(tesseract))
  , env_state(std::move(env_state))
  , manipulator(std::move(manipulator))
{
}

void OMPLMotionPlannerDefaultConfig::init()
{
  manip_fwd_kin_ = tesseract->getFwdKinematicsManagerConst()->getFwdKinematicSolver(manipulator);
  if (manipulator_ik_solver.empty())
    manip_inv_kin_ = tesseract->getInvKinematicsManagerConst()->getInvKinematicSolver(manipulator);
  else
    manip_inv_kin_ = tesseract->getInvKinematicsManagerConst()->getInvKinematicSolver(manipulator, manipulator_ik_solver);

//  const std::vector<std::string>& joint_names = manip_fwd_kin_->getJointNames();

//  this->prob.dof = static_cast<int>(this->prob.manip_fwd_kin->numJoints());
//  this->prob.joint_limits = this->prob.manip_fwd_kin->getLimits();
//  this->prob.joint_names = joint_names;
//  this->prob.configuration = configuration;
  if (configuration == OMPLProblemConfiguration::SE3_STATE_SPACE_ROBOT_ON_POSITIONER || configuration == OMPLProblemConfiguration::SE3_STATE_SPACE_ROBOT_WITH_EXTERNAL_POSITIONER)
  {
    positioner_fwd_kin_ = tesseract->getFwdKinematicsManagerConst()->getFwdKinematicSolver(positioner);
//    this->prob.dof += static_cast<int>(this->prob.positioner_fwd_kin->numJoints());
//    this->prob.joint_limits = Eigen::MatrixX2d(this->prob.dof, 2);
//    this->prob.joint_limits << this->prob.positioner_fwd_kin->getLimits(), this->prob.manip_fwd_kin->getLimits();
//    this->prob.joint_names = this->prob.positioner_fwd_kin->getJointNames();
//    this->prob.joint_names.insert(this->prob.joint_names.end(), joint_names.begin(), joint_names.end());
  }

  if (configuration == OMPLProblemConfiguration::REAL_STATE_SPACE)
    extractor_ = std::bind(&tesseract_planning::RealVectorStateSpaceExtractor, std::placeholders::_1, manip_inv_kin_->numJoints());
  else if (configuration == OMPLProblemConfiguration::REAL_CONSTRAINTED_STATE_SPACE)
    extractor_ = tesseract_planning::ConstrainedStateSpaceExtractor;
  else
    throw std::runtime_error("OMPLMotionPlannerDefaultConfig: Unsupported configuration!");

  // Get Active Link Names
  std::vector<std::string> active_link_names = manip_inv_kin_->getActiveLinkNames();
  if (configuration == OMPLProblemConfiguration::SE3_STATE_SPACE_ROBOT_ON_POSITIONER || configuration == OMPLProblemConfiguration::SE3_STATE_SPACE_ROBOT_WITH_EXTERNAL_POSITIONER)
  {
    const std::vector<std::string>& positioner_active_link_names = positioner_fwd_kin_->getActiveLinkNames();
    active_link_names.insert(active_link_names.end(), positioner_active_link_names.begin(), positioner_active_link_names.end());
  }

  auto adjacency_map = std::make_shared<tesseract_environment::AdjacencyMap>(tesseract->getEnvironmentConst()->getSceneGraph(), active_link_names, env_state->link_transforms);
  active_link_names_ = adjacency_map->getActiveLinkNames();
}

void OMPLMotionPlannerDefaultConfig::clear()
{
  prob.clear();

  manip_fwd_kin_ = nullptr;
  manip_inv_kin_ = nullptr;
  positioner_fwd_kin_ = nullptr;
  extractor_ = nullptr;
  active_link_names_.clear();
}

OMPLProblem::UPtr OMPLMotionPlannerDefaultConfig::createSubProblem()
{
  auto sub_prob = std::make_unique<OMPLProblem>();
  sub_prob->tesseract = tesseract;
  sub_prob->env_state = env_state;
  sub_prob->state_solver = tesseract->getEnvironmentConst()->getStateSolver();
  sub_prob->state_solver->setState(env_state->joints);
  sub_prob->configuration = configuration;
  sub_prob->manip_reach = manipulator_reach;
  sub_prob->manip_fwd_kin = manip_fwd_kin_;
  sub_prob->manip_inv_kin = manip_inv_kin_;
  sub_prob->positioner_fwd_kin = positioner_fwd_kin_;
  sub_prob->extractor = extractor_;
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
    CONSOLE_BRIDGE_logError("Check Kinematics failed. This means that Inverse Kinematics does not agree with KDL (TrajOpt). Did you change the URDF recently?");

  // Check and make sure it does not contain any composite instruction
  for (const auto& instruction : instructions)
  {
    if (instruction.isComposite())
      throw std::runtime_error("OMPL planner does not support child composite instructions.");
  }

  // Transform plan instructions into descartes samplers
  const PlanInstruction* prev_plan_instruction {nullptr};
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
//        const Waypoint& wp = plan_instruction->getWaypoint();
      const std::string& working_frame = plan_instruction->getWorkingFrame();
//        const Eigen::Isometry3d& tcp = plan_instruction->getTCP();

      assert(seed[i].isComposite());
      const auto* seed_composite = seed[i].cast_const<tesseract_planning::CompositeInstruction>();

      // Get Plan Profile
      std::string profile = plan_instruction->getProfile();
      if (profile.empty())
        profile = "DEFAULT";

      typename OMPLPlanProfile::Ptr cur_plan_profile {nullptr};
      auto it = plan_profiles.find(profile);
      if (it == plan_profiles.end())
        cur_plan_profile = std::make_shared<OMPLDefaultPlanProfile>();
      else
        cur_plan_profile = it->second;

      if (plan_instruction->isLinear())
      {
        if (isCartesianWaypoint(plan_instruction->getWaypoint().getType()))
        {
          const auto* cur_wp = plan_instruction->getWaypoint().cast_const<tesseract_planning::CartesianWaypoint>();
          if (prev_plan_instruction)
          {
            assert(prev_plan_instruction->getTCP().isApprox(plan_instruction->getTCP(), 1e-5));

//            Eigen::Isometry3d prev_pose = Eigen::Isometry3d::Identity();
//            if (isCartesianWaypoint(prev_plan_instruction->getWaypoint().getType()))
//            {
//              prev_pose = (*prev_plan_instruction->getWaypoint().cast_const<Eigen::Isometry3d>());
//            }
//            else if (isJointWaypoint(prev_plan_instruction->getWaypoint().getType()))
//            {
//              // This currently only works for ROBOT_ONLY configuration need to update to use state solver
//              assert(this->prob.configuration == DescartesProblem<FloatType>::ROBOT_ONLY);
//              const auto* jwp = prev_plan_instruction->getWaypoint().cast_const<JointWaypoint>();
//              if (!this->prob.manip_fwd_kin->calcFwdKin(prev_pose, *jwp))
//                throw std::runtime_error("DescartesMotionPlannerConfig: failed to solve forward kinematics!");

//              prev_pose = this->prob.env_state->link_transforms.at(this->prob.manip_fwd_kin->getBaseLinkName()) * prev_pose * plan_instruction->getTCP();
//            }
//            else
//            {
//              throw std::runtime_error("DescartesMotionPlannerConfig: uknown waypoint type.");
//            }

//            tesseract_common::VectorIsometry3d poses = interpolate(prev_pose, *cur_wp, static_cast<int>(seed_composite->size()));
//            // Add intermediate points with path costs and constraints
//            for (std::size_t p = 1; p < poses.size() - 1; ++p)
//            {
//              cur_plan_profile->apply(this->prob, poses[p], *plan_instruction, active_links, index);

//              ++index;
//            }
          }
          else
          {
            assert(seed_composite->size()==1);
          }

//          // Add final point with waypoint
//          cur_plan_profile->apply(this->prob, *cur_wp, *plan_instruction, active_links, index);

//          ++index;

          // TODO Currently skipping linear moves until SE3 motion planning is implemented.
          prob.push_back(nullptr);
          ++index;
        }
        else if (isJointWaypoint(plan_instruction->getWaypoint().getType()))
        {
//          // This currently only works for ROBOT_ONLY configuration Need to update to use state solver
//          assert(configuration == OMPLProblemConfiguration::REAL_STATE_SPACE || configuration == OMPLProblemConfiguration::REAL_CONSTRAINTED_STATE_SPACE || configuration == OMPLProblemConfiguration::SE3_STATE_SPACE_ROBOT_ONLY);
//          const auto* cur_wp = plan_instruction->getWaypoint().cast_const<JointWaypoint>();
//          Eigen::Isometry3d cur_pose = Eigen::Isometry3d::Identity();
//          if (!manip_fwd_kin_->calcFwdKin(cur_pose, *cur_wp))
//            throw std::runtime_error("DescartesMotionPlannerConfig: failed to solve forward kinematics!");

//          cur_pose = env_state->link_transforms.at(manip_fwd_kin_->getBaseLinkName()) * cur_pose * plan_instruction->getTCP();
//          if (prev_plan_instruction)
//          {
//            assert(prev_plan_instruction->getTCP().isApprox(plan_instruction->getTCP(), 1e-5));

//            /** @todo This should also handle if waypoint type is joint */
//            Eigen::Isometry3d prev_pose = Eigen::Isometry3d::Identity();
//            if (isCartesianWaypoint(prev_plan_instruction->getWaypoint().getType()))
//            {
//              prev_pose = (*prev_plan_instruction->getWaypoint().cast_const<Eigen::Isometry3d>());
//            }
//            else if (isJointWaypoint(prev_plan_instruction->getWaypoint().getType()))
//            {
//              // This currently only works for ROBOT_ONLY configuration need to update to use state solver
//              assert(configuration == OMPLProblemConfiguration::REAL_STATE_SPACE || configuration == OMPLProblemConfiguration::REAL_CONSTRAINTED_STATE_SPACE || configuration == OMPLProblemConfiguration::SE3_STATE_SPACE_ROBOT_ONLY);
//              const auto* jwp = prev_plan_instruction->getWaypoint().cast_const<JointWaypoint>();
//              if (!manip_fwd_kin_->calcFwdKin(prev_pose, *jwp))
//                throw std::runtime_error("DescartesMotionPlannerConfig: failed to solve forward kinematics!");

//              prev_pose = env_state->link_transforms.at(manip_fwd_kin_->getBaseLinkName()) * prev_pose * plan_instruction->getTCP();
//            }
//            else
//            {
//              throw std::runtime_error("DescartesMotionPlannerConfig: uknown waypoint type.");
//            }

//            tesseract_common::VectorIsometry3d poses = interpolate(prev_pose, cur_pose, static_cast<int>(seed_composite->size()));
//            // Add intermediate points with path costs and constraints
//            for (std::size_t p = 1; p < poses.size() - 1; ++p)
//            {
//              cur_plan_profile->apply(this->prob, poses[p], *plan_instruction, active_links, index);

//              ++index;
//            }
//          }
//          else
//          {
//            assert(seed_composite->size()==1);
//          }

//          // Add final point with waypoint
//          cur_plan_profile->apply(this->prob, *cur_wp, *plan_instruction, active_links, index);

//          ++index;

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
        /** @todo This should also handle if waypoint type is cartesian */
        if (isJointWaypoint(plan_instruction->getWaypoint().getType()))
        {
          const auto* cur_wp = plan_instruction->getWaypoint().cast_const<tesseract_planning::JointWaypoint>();
          if (!prev_plan_instruction)
          {
            assert(seed_composite->size()==1);
          }
          else
          {
            /** @todo Should check that the joint names match the order of the manipulator */
            OMPLProblem::UPtr sub_prob = createSubProblem();
            cur_plan_profile->apply(*sub_prob, *cur_wp, *plan_instruction, active_link_names_, index);
            sub_prob->n_output_states = static_cast<int>(seed_composite->size());

            ompl::base::ScopedState<> start_state(sub_prob->simple_setup->getStateSpace());
            if (isJointWaypoint(prev_plan_instruction->getWaypoint().getType()))
            {
              const auto* prev_wp = prev_plan_instruction->getWaypoint().cast_const<tesseract_planning::JointWaypoint>();
              for (long i = 0; i < prev_wp->size(); ++i)
                start_state[static_cast<unsigned>(i)] = (*prev_wp)[i];

              sub_prob->simple_setup->setStartState(start_state);

              // Get descrete contact manager for testing provided start and end position
              // This is required because collision checking happens in motion validators now
              // instead of the isValid function to avoid unnecessary collision checks.
              if (checkStateInCollision(*sub_prob, *prev_wp))
              {
                CONSOLE_BRIDGE_logError("In OMPLPlannerFreespaceConfig: Start state is in collision");
              }
            }
            else
            {
              assert(false);
//              sub_prob->simple_setup->addStartState(start_state);
            }

            prob.push_back(std::move(sub_prob));
            ++index;
          }
        }
        else if (isCartesianWaypoint(plan_instruction->getWaypoint().getType()))
        {
          const auto* cur_wp = plan_instruction->getWaypoint().cast_const<tesseract_planning::CartesianWaypoint>();
          if (!prev_plan_instruction)
          {
            assert(seed_composite->size()==1);
          }
          else
          {
            assert(false);
            OMPLProblem::UPtr sub_prob = createSubProblem();
            sub_prob->n_output_states = static_cast<int>(seed_composite->size());
            cur_plan_profile->apply(*sub_prob, *cur_wp, *plan_instruction, active_link_names_, index);

            ompl::base::ScopedState<> start_state(sub_prob->simple_setup->getStateSpace());
            if (isJointWaypoint(prev_plan_instruction->getWaypoint().getType()))
            {
              const auto* prev_wp = prev_plan_instruction->getWaypoint().cast_const<tesseract_planning::JointWaypoint>();
              for (long i = 0; i < prev_wp->size(); ++i)
                start_state[static_cast<unsigned>(i)] = (*prev_wp)[i];

              sub_prob->simple_setup->setStartState(start_state);

              // Get descrete contact manager for testing provided start and end position
              // This is required because collision checking happens in motion validators now
              // instead of the isValid function to avoid unnecessary collision checks.
              if (checkStateInCollision(*sub_prob, *prev_wp))
              {
                CONSOLE_BRIDGE_logError("In OMPLPlannerFreespaceConfig: Start state is in collision");
              }
            }
            else
            {
              assert(false);
//              sub_prob->simple_setup->addStartState(start_state);
            }

            prob.push_back(std::move(sub_prob));
            ++index;
          }
        }
        else
        {
          throw std::runtime_error("OMPLMotionPlannerDefaultConfig: unknown waypoint type");
        }
      }
      else
      {
        throw std::runtime_error("OMPLMotionPlannerDefaultConfig: Unsupported!");
      }

      prev_plan_instruction = plan_instruction;
    }
  }

  return OMPLMotionPlannerConfig::generate();
}
}
