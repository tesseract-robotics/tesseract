/**
 * @file descartes_motion_planner_default_config.hpp
 * @brief Tesseract ROS Descartes planner default config
 *
 * @author Levi Armstrong
 * @date June 18, 2020
 * @version TODO
 * @bug No known bugs
 *
 * @copyright Copyright (c) 2017, Southwest Research Institute
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
#ifndef TESSERACT_MOTION_PLANNERS_IMPL_DESCARTES_DESCARTES_MOTION_PLANNER_DEFAULT_CONFIG_HPP
#define TESSERACT_MOTION_PLANNERS_IMPL_DESCARTES_DESCARTES_MOTION_PLANNER_DEFAULT_CONFIG_HPP

#include <tesseract_motion_planners/descartes/descartes_motion_planner_default_config.h>

namespace tesseract_planning
{

template <typename FloatType>
DescartesMotionPlannerDefaultConfig<FloatType>::DescartesMotionPlannerDefaultConfig(tesseract::Tesseract::ConstPtr tesseract,
                                                                                    tesseract_environment::EnvState::ConstPtr env_state,
                                                                                    std::string manipulator,
                                                                                    double manipulator_reach)
  : manipulator(manipulator)
  , manipulator_reach(manipulator_reach)
{
  this->prob.tesseract = tesseract;
  this->prob.env_state = env_state;
}

template <typename FloatType>
bool DescartesMotionPlannerDefaultConfig<FloatType>::generate()
{
  // Clear descartes data
  this->prob.edge_evaluators.clear();
  this->prob.timing_constraints.clear();
  this->prob.samplers.clear();

  // Get Manipulator Information
  getManipulatorInfo();

  // Process instructions
  if (!tesseract_kinematics::checkKinematics(this->prob.manip_fwd_kin, this->prob.manip_inv_kin))
    CONSOLE_BRIDGE_logError("Check Kinematics failed. This means that Inverse Kinematics does not agree with KDL (TrajOpt). Did you change the URDF recently?");

  std::vector<std::string> joint_names = this->prob.manip_inv_kin->getJointNames();
  std::vector<std::string> active_link_names = this->prob.manip_inv_kin->getActiveLinkNames();

  auto adjacency_map = std::make_shared<tesseract_environment::AdjacencyMap>(this->prob.tesseract->getEnvironmentConst()->getSceneGraph(), active_link_names, this->prob.env_state->link_transforms);
  const std::vector<std::string>& active_links = adjacency_map->getActiveLinkNames();

  // Check and make sure it does not contain any composite instruction
  const PlanInstruction* start_instruction {nullptr};
  for (const auto& instruction : instructions)
  {
    if (instruction.isComposite())
      throw std::runtime_error("Descartes planner does not support child composite instructions.");

    if (start_instruction == nullptr && instruction.isPlan())
      start_instruction = instruction.cast_const<PlanInstruction>();
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

      typename DescartesPlanProfile<FloatType>::Ptr cur_plan_profile {nullptr};
      auto it = plan_profiles.find(profile);
      if (it == plan_profiles.end())
        cur_plan_profile = std::make_shared<DescartesDefaultPlanProfile<FloatType>>();
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

            Eigen::Isometry3d prev_pose = Eigen::Isometry3d::Identity();
            if (isCartesianWaypoint(prev_plan_instruction->getWaypoint().getType()))
            {
              prev_pose = (*prev_plan_instruction->getWaypoint().cast_const<Eigen::Isometry3d>());
            }
            else if (isJointWaypoint(prev_plan_instruction->getWaypoint().getType()))
            {
              // This currently only works for ROBOT_ONLY configuration need to update to use state solver
              assert(this->prob.configuration == DescartesProblem<FloatType>::ROBOT_ONLY);
              const auto* jwp = prev_plan_instruction->getWaypoint().cast_const<JointWaypoint>();
              if (!this->prob.manip_fwd_kin->calcFwdKin(prev_pose, *jwp))
                throw std::runtime_error("DescartesMotionPlannerConfig: failed to solve forward kinematics!");

              prev_pose = this->prob.env_state->link_transforms.at(this->prob.manip_fwd_kin->getBaseLinkName()) * prev_pose * plan_instruction->getTCP();
            }
            else
            {
              throw std::runtime_error("DescartesMotionPlannerConfig: uknown waypoint type.");
            }

            tesseract_common::VectorIsometry3d poses = interpolate(prev_pose, *cur_wp, static_cast<int>(seed_composite->size()));
            // Add intermediate points with path costs and constraints
            for (std::size_t p = 1; p < poses.size() - 1; ++p)
            {
              cur_plan_profile->apply(this->prob, poses[p], *plan_instruction, active_links, index);

              ++index;
            }
          }
          else
          {
            assert(seed_composite->size()==1);
          }

          // Add final point with waypoint
          cur_plan_profile->apply(this->prob, *cur_wp, *plan_instruction, active_links, index);

          ++index;
        }
        else if (isJointWaypoint(plan_instruction->getWaypoint().getType()))
        {
          // This currently only works for ROBOT_ONLY configuration Need to update to use state solver
          assert(this->prob.configuration == DescartesProblem<FloatType>::ROBOT_ONLY);
          const auto* cur_wp = plan_instruction->getWaypoint().cast_const<JointWaypoint>();
          Eigen::Isometry3d cur_pose = Eigen::Isometry3d::Identity();
          if (!this->prob.manip_fwd_kin->calcFwdKin(cur_pose, *cur_wp))
            throw std::runtime_error("DescartesMotionPlannerConfig: failed to solve forward kinematics!");

          cur_pose = this->prob.env_state->link_transforms.at(this->prob.manip_fwd_kin->getBaseLinkName()) * cur_pose * plan_instruction->getTCP();
          if (prev_plan_instruction)
          {
            assert(prev_plan_instruction->getTCP().isApprox(plan_instruction->getTCP(), 1e-5));

            /** @todo This should also handle if waypoint type is joint */
            Eigen::Isometry3d prev_pose = Eigen::Isometry3d::Identity();
            if (isCartesianWaypoint(prev_plan_instruction->getWaypoint().getType()))
            {
              prev_pose = (*prev_plan_instruction->getWaypoint().cast_const<Eigen::Isometry3d>());
            }
            else if (isJointWaypoint(prev_plan_instruction->getWaypoint().getType()))
            {
              // This currently only works for ROBOT_ONLY configuration need to update to use state solver
              assert(this->prob.configuration == DescartesProblem<FloatType>::ROBOT_ONLY);
              const auto* jwp = prev_plan_instruction->getWaypoint().cast_const<JointWaypoint>();
              if (!this->prob.manip_fwd_kin->calcFwdKin(prev_pose, *jwp))
                throw std::runtime_error("DescartesMotionPlannerConfig: failed to solve forward kinematics!");

              prev_pose = this->prob.env_state->link_transforms.at(this->prob.manip_fwd_kin->getBaseLinkName()) * prev_pose * plan_instruction->getTCP();
            }
            else
            {
              throw std::runtime_error("DescartesMotionPlannerConfig: uknown waypoint type.");
            }

            tesseract_common::VectorIsometry3d poses = interpolate(prev_pose, cur_pose, static_cast<int>(seed_composite->size()));
            // Add intermediate points with path costs and constraints
            for (std::size_t p = 1; p < poses.size() - 1; ++p)
            {
              cur_plan_profile->apply(this->prob, poses[p], *plan_instruction, active_links, index);

              ++index;
            }
          }
          else
          {
            assert(seed_composite->size()==1);
          }

          // Add final point with waypoint
          cur_plan_profile->apply(this->prob, *cur_wp, *plan_instruction, active_links, index);

          ++index;
        }
        else
        {
          throw std::runtime_error("DescartesMotionPlannerConfig: unknown waypoint type");
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

          // Descartes does not support freespace so it will only include the plan instruction state, then in
          // post processing function will perform interpolation to fill out the seed, but may be in collision.
          // Usually this is only requested when it is being provided as a seed to a planner like trajopt.

          // Add final point with waypoint costs and contraints
          /** @todo Should check that the joint names match the order of the manipulator */
          cur_plan_profile->apply(this->prob, *cur_wp, *plan_instruction, active_links, index);

          ++index;
        }
        else if (isCartesianWaypoint(plan_instruction->getWaypoint().getType()))
        {
          const auto* cur_wp = plan_instruction->getWaypoint().cast_const<tesseract_planning::CartesianWaypoint>();
          if (!prev_plan_instruction)
          {
            assert(seed_composite->size()==1);
          }

          // Descartes does not support freespace so it will only include the plan instruction state, then in
          // post processing function will perform interpolation to fill out the seed, but may be in collision.
          // Usually this is only requested when it is being provided as a seed to a planner like trajopt.

          // Add final point with waypoint costs and contraints
          /** @todo Should check that the joint names match the order of the manipulator */
          cur_plan_profile->apply(this->prob, *cur_wp, *plan_instruction, active_links, index);

          ++index;
        }
        else
        {
          throw std::runtime_error("DescartesMotionPlannerConfig: unknown waypoint type");
        }
      }
      else
      {
        throw std::runtime_error("DescartesMotionPlannerConfig: Unsupported!");
      }

      prev_plan_instruction = plan_instruction;
    }
  }

  // Call the base class generate which checks the problem to make sure everything is in order
  return DescartesMotionPlannerConfig<FloatType>::generate();
}

template <typename FloatType>
void DescartesMotionPlannerDefaultConfig<FloatType>::getManipulatorInfo()
{
  this->prob.manip_fwd_kin = this->prob.tesseract->getFwdKinematicsManagerConst()->getFwdKinematicSolver(manipulator);
  if (manipulator_ik_solver.empty())
    this->prob.manip_inv_kin = this->prob.tesseract->getInvKinematicsManagerConst()->getInvKinematicSolver(manipulator);
  else
    this->prob.manip_inv_kin = this->prob.tesseract->getInvKinematicsManagerConst()->getInvKinematicSolver(manipulator, manipulator_ik_solver);

  this->prob.manip_reach = manipulator_reach;

  const std::vector<std::string>& joint_names = this->prob.manip_fwd_kin->getJointNames();

  this->prob.dof = static_cast<int>(this->prob.manip_fwd_kin->numJoints());
  this->prob.joint_limits = this->prob.manip_fwd_kin->getLimits();
  this->prob.joint_names = joint_names;
  this->prob.configuration = configuration;
  if (this->prob.configuration == DescartesProblem<FloatType>::ROBOT_ON_POSITIONER || this->prob.configuration == DescartesProblem<FloatType>::ROBOT_WITH_EXTERNAL_POSITIONER)
  {
    this->prob.positioner_fwd_kin = this->prob.tesseract->getFwdKinematicsManagerConst()->getFwdKinematicSolver(positioner);
    this->prob.dof += static_cast<int>(this->prob.positioner_fwd_kin->numJoints());
    this->prob.joint_limits = Eigen::MatrixX2d(this->prob.dof, 2);
    this->prob.joint_limits << this->prob.positioner_fwd_kin->getLimits(), this->prob.manip_fwd_kin->getLimits();
    this->prob.joint_names = this->prob.positioner_fwd_kin->getJointNames();
    this->prob.joint_names.insert(this->prob.joint_names.end(), joint_names.begin(), joint_names.end());
  }
}
}

#endif // TESSERACT_MOTION_PLANNERS_IMPL_DESCARTES_DESCARTES_MOTION_PLANNER_DEFAULT_CONFIG_HPP
