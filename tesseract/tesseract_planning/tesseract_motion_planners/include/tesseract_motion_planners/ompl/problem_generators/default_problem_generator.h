/**
 * @file default_problem_generator.h
 * @brief Generates a OMPL problem from a planner request
 *
 * @author Levi Armstrong
 * @date April 18, 2018
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
#ifndef TESSERACT_MOTION_PLANNERS_OMPL_DEFAULT_PROBLEM_GENERATOR_H
#define TESSERACT_MOTION_PLANNERS_OMPL_DEFAULT_PROBLEM_GENERATOR_H

#include <tesseract_motion_planners/ompl/profile/ompl_default_plan_profile.h>
#include <tesseract_kinematics/core/validate.h>
#include <tesseract/tesseract.h>
#include <tesseract_motion_planners/core/types.h>
#include <vector>

namespace tesseract_planning
{
inline OMPLProblem::UPtr CreateOMPLSubProblem(const PlannerRequest& request,
                                              tesseract_kinematics::ForwardKinematics::Ptr manip_fwd_kin,
                                              tesseract_kinematics::InverseKinematics::Ptr manip_inv_kin,
                                              std::vector<std::string> active_link_names)
{
  auto sub_prob = std::make_unique<OMPLProblem>();
  sub_prob->tesseract = request.tesseract;
  sub_prob->env_state = request.env_state;
  sub_prob->state_solver = request.tesseract->getEnvironmentConst()->getStateSolver();
  sub_prob->state_solver->setState(request.env_state->joints);
  sub_prob->manip_fwd_kin = manip_fwd_kin;
  sub_prob->manip_inv_kin = manip_inv_kin;
  sub_prob->contact_checker = request.tesseract->getEnvironmentConst()->getDiscreteContactManager();
  sub_prob->contact_checker->setCollisionObjectsTransform(request.env_state->link_transforms);
  sub_prob->contact_checker->setActiveCollisionObjects(active_link_names);
  return sub_prob;
}

inline std::vector<OMPLProblem::UPtr> DefaultOMPLProblemGenerator(const PlannerRequest& request,
                                                                  const OMPLPlanProfileMap& plan_profiles)
{
  std::vector<OMPLProblem::UPtr> problem;
  std::vector<std::string> active_link_names_;
  tesseract_kinematics::ForwardKinematics::Ptr manip_fwd_kin_;
  tesseract_kinematics::InverseKinematics::Ptr manip_inv_kin_;

  manip_fwd_kin_ = request.tesseract->getFwdKinematicsManagerConst()->getFwdKinematicSolver(request.manipulator);
  if (request.manipulator_ik_solver.empty())
    manip_inv_kin_ = request.tesseract->getInvKinematicsManagerConst()->getInvKinematicSolver(request.manipulator);
  else
    manip_inv_kin_ = request.tesseract->getInvKinematicsManagerConst()->getInvKinematicSolver(
        request.manipulator, request.manipulator_ik_solver);
  if (!manip_fwd_kin_)
  {
    CONSOLE_BRIDGE_logError("No Forward Kinematics solver found");
    return problem;
  }
  if (!manip_inv_kin_)
  {
    CONSOLE_BRIDGE_logError("No Inverse Kinematics solver found");
    return problem;
  }
  // Process instructions
  if (!tesseract_kinematics::checkKinematics(manip_fwd_kin_, manip_inv_kin_))
    CONSOLE_BRIDGE_logError("Check Kinematics failed. This means that Inverse Kinematics does not agree with KDL "
                            "(TrajOpt). Did you change the URDF recently?");

  // Get Active Link Names
  {
    std::vector<std::string> active_link_names = manip_inv_kin_->getActiveLinkNames();
    auto adjacency_map =
        std::make_shared<tesseract_environment::AdjacencyMap>(request.tesseract->getEnvironmentConst()->getSceneGraph(),
                                                              active_link_names,
                                                              request.env_state->link_transforms);
    active_link_names_ = adjacency_map->getActiveLinkNames();
  }

  // Check and make sure it does not contain any composite instruction
  for (const auto& instruction : request.instructions)
  {
    if (instruction.isComposite())
      throw std::runtime_error("OMPL planner does not support child composite instructions.");
  }

  // Transform plan instructions into descartes samplers
  const PlanInstruction* prev_plan_instruction{ nullptr };
  int index = 0;
  for (std::size_t i = 0; i < request.instructions.size(); ++i)
  {
    const auto& instruction = request.instructions[i];
    if (instruction.isPlan())
    {
      assert(instruction.getType() == static_cast<int>(InstructionType::PLAN_INSTRUCTION));
      const auto* plan_instruction = instruction.cast_const<PlanInstruction>();
      //      const Waypoint& wp = plan_instruction->getWaypoint();
      //      const std::string& working_frame = plan_instruction->getWorkingFrame();
      //      const Eigen::Isometry3d& tcp = plan_instruction->getTCP();

      assert(request.seed[i].isComposite());
      const auto* seed_composite = request.seed[i].cast_const<tesseract_planning::CompositeInstruction>();

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

      if (plan_instruction->isLinear())
      {
        /** @todo Add support for linear motion to ompl planner */
        if (isCartesianWaypoint(plan_instruction->getWaypoint().getType()))
        {
          //          const auto* cur_wp =
          //          plan_instruction->getWaypoint().cast_const<tesseract_planning::CartesianWaypoint>();
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
            //              assert(this->problem.configuration == DescartesProblem<FloatType>::ROBOT_ONLY);
            //              const auto* jwp = prev_plan_instruction->getWaypoint().cast_const<JointWaypoint>();
            //              if (!this->problem.manip_fwd_kin->calcFwdKin(prev_pose, *jwp))
            //                throw std::runtime_error("DescartesMotionPlannerConfig: failed to solve forward
            //                kinematics!");

            //              prev_pose =
            //              this->problem.env_state->link_transforms.at(this->problem.manip_fwd_kin->getBaseLinkName())
            //              * prev_pose * plan_instruction->getTCP();
            //            }
            //            else
            //            {
            //              throw std::runtime_error("DescartesMotionPlannerConfig: uknown waypoint type.");
            //            }

            //            tesseract_common::VectorIsometry3d poses = interpolate(prev_pose, *cur_wp,
            //            static_cast<int>(seed_composite->size()));
            //            // Add intermediate points with path costs and constraints
            //            for (std::size_t p = 1; p < poses.size() - 1; ++p)
            //            {
            //              cur_plan_profile->apply(this->problem, poses[p], *plan_instruction, active_links, index);

            //              ++index;
            //            }
          }
          else
          {
            assert(seed_composite->size() == 1);
          }

          //          // Add final point with waypoint
          //          cur_plan_profile->apply(this->problem, *cur_wp, *plan_instruction, active_links, index);

          //          ++index;

          // TODO Currently skipping linear moves until SE3 motion planning is implemented.
          problem.push_back(nullptr);
          ++index;
        }
        else if (isJointWaypoint(plan_instruction->getWaypoint().getType()))
        {
          //          // This currently only works for ROBOT_ONLY configuration Need to update to use state solver
          //          assert(configuration == OMPLProblemConfiguration::REAL_STATE_SPACE || configuration ==
          //          OMPLProblemConfiguration::REAL_CONSTRAINTED_STATE_SPACE || configuration ==
          //          OMPLProblemConfiguration::SE3_STATE_SPACE_ROBOT_ONLY); const auto* cur_wp =
          //          plan_instruction->getWaypoint().cast_const<JointWaypoint>(); Eigen::Isometry3d cur_pose =
          //          Eigen::Isometry3d::Identity(); if (!manip_fwd_kin_->calcFwdKin(cur_pose, *cur_wp))
          //            throw std::runtime_error("DescartesMotionPlannerConfig: failed to solve forward kinematics!");

          //          cur_pose = env_state->link_transforms.at(manip_fwd_kin_->getBaseLinkName()) * cur_pose *
          //          plan_instruction->getTCP(); if (prev_plan_instruction)
          //          {
          //            assert(prev_plan_instruction->getTCP().isApprox(plan_instruction->getTCP(), 1e-5));

          //            Eigen::Isometry3d prev_pose = Eigen::Isometry3d::Identity();
          //            if (isCartesianWaypoint(prev_plan_instruction->getWaypoint().getType()))
          //            {
          //              prev_pose = (*prev_plan_instruction->getWaypoint().cast_const<Eigen::Isometry3d>());
          //            }
          //            else if (isJointWaypoint(prev_plan_instruction->getWaypoint().getType()))
          //            {
          //              // This currently only works for ROBOT_ONLY configuration need to update to use state solver
          //              assert(configuration == OMPLProblemConfiguration::REAL_STATE_SPACE || configuration ==
          //              OMPLProblemConfiguration::REAL_CONSTRAINTED_STATE_SPACE || configuration ==
          //              OMPLProblemConfiguration::SE3_STATE_SPACE_ROBOT_ONLY); const auto* jwp =
          //              prev_plan_instruction->getWaypoint().cast_const<JointWaypoint>(); if
          //              (!manip_fwd_kin_->calcFwdKin(prev_pose, *jwp))
          //                throw std::runtime_error("DescartesMotionPlannerConfig: failed to solve forward
          //                kinematics!");

          //              prev_pose = env_state->link_transforms.at(manip_fwd_kin_->getBaseLinkName()) * prev_pose *
          //              plan_instruction->getTCP();
          //            }
          //            else
          //            {
          //              throw std::runtime_error("DescartesMotionPlannerConfig: uknown waypoint type.");
          //            }

          //            tesseract_common::VectorIsometry3d poses = interpolate(prev_pose, cur_pose,
          //            static_cast<int>(seed_composite->size()));
          //            // Add intermediate points with path costs and constraints
          //            for (std::size_t p = 1; p < poses.size() - 1; ++p)
          //            {
          //              cur_plan_profile->apply(this->problem, poses[p], *plan_instruction, active_links, index);

          //              ++index;
          //            }
          //          }
          //          else
          //          {
          //            assert(seed_composite->size()==1);
          //          }

          //          // Add final point with waypoint
          //          cur_plan_profile->apply(this->problem, *cur_wp, *plan_instruction, active_links, index);

          //          ++index;

          // TODO Currently skipping linear moves until SE3 motion planning is implemented.
          problem.push_back(nullptr);
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
          if (!prev_plan_instruction)
          {
            assert(seed_composite->size() == 1);
          }
          else
          {
            /** @todo Should check that the joint names match the order of the manipulator */
            OMPLProblem::UPtr sub_prob =
                CreateOMPLSubProblem(request, manip_fwd_kin_, manip_inv_kin_, active_link_names_);
            cur_plan_profile->setup(*sub_prob);
            cur_plan_profile->applyGoalStates(*sub_prob, *cur_wp, *plan_instruction, active_link_names_, index);
            sub_prob->n_output_states = static_cast<int>(seed_composite->size());

            if (index == 0)
            {
              ompl::base::ScopedState<> start_state(sub_prob->simple_setup->getStateSpace());
              if (isJointWaypoint(prev_plan_instruction->getWaypoint().getType()))
              {
                const auto* prev_wp =
                    prev_plan_instruction->getWaypoint().cast_const<tesseract_planning::JointWaypoint>();
                cur_plan_profile->applyStartStates(*sub_prob, *prev_wp, *plan_instruction, active_link_names_, index);
              }
              else if (isCartesianWaypoint(prev_plan_instruction->getWaypoint().getType()))
              {
                const auto* prev_wp =
                    prev_plan_instruction->getWaypoint().cast_const<tesseract_planning::CartesianWaypoint>();
                cur_plan_profile->applyStartStates(*sub_prob, *prev_wp, *plan_instruction, active_link_names_, index);
              }
              else
              {
                throw std::runtime_error("OMPLMotionPlannerDefaultConfig: unknown waypoint type");
              }

              problem.push_back(std::move(sub_prob));
              ++index;
            }
            else
            {
              /** @todo Update. Extract the solution for the previous plan and set as the start */
              assert(false);
            }
          }
        }
        else if (isCartesianWaypoint(plan_instruction->getWaypoint().getType()))
        {
          const auto* cur_wp = plan_instruction->getWaypoint().cast_const<tesseract_planning::CartesianWaypoint>();
          if (!prev_plan_instruction)
          {
            assert(seed_composite->size() == 1);
          }
          else
          {
            OMPLProblem::UPtr sub_prob =
                CreateOMPLSubProblem(request, manip_fwd_kin_, manip_inv_kin_, active_link_names_);
            sub_prob->n_output_states = static_cast<int>(seed_composite->size());
            cur_plan_profile->setup(*sub_prob);
            cur_plan_profile->applyGoalStates(*sub_prob, *cur_wp, *plan_instruction, active_link_names_, index);

            if (index == 0)
            {
              ompl::base::ScopedState<> start_state(sub_prob->simple_setup->getStateSpace());
              if (isJointWaypoint(prev_plan_instruction->getWaypoint().getType()))
              {
                const auto* prev_wp =
                    prev_plan_instruction->getWaypoint().cast_const<tesseract_planning::JointWaypoint>();
                cur_plan_profile->applyStartStates(*sub_prob, *prev_wp, *plan_instruction, active_link_names_, index);
              }
              else if (isCartesianWaypoint(prev_plan_instruction->getWaypoint().getType()))
              {
                const auto* prev_wp =
                    prev_plan_instruction->getWaypoint().cast_const<tesseract_planning::CartesianWaypoint>();
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

            problem.push_back(std::move(sub_prob));
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

  return problem;
}
}  // namespace tesseract_planning
#endif
