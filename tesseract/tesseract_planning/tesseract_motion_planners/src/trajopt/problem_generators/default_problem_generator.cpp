/**
 * @file default_problem_generator.cpp
 * @brief Generates a trajopt problem from a planner request
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

#include <tesseract_motion_planners/trajopt/problem_generators/default_problem_generator.h>
#include <tesseract_motion_planners/core/utils.h>

namespace tesseract_planning
{
// TODO: Restructure this into several smaller functions that are testable and easier to understand
trajopt::TrajOptProb::Ptr DefaultTrajoptProblemGenerator(const PlannerRequest& request,
                                                         const TrajOptPlanProfileMap& plan_profiles,
                                                         const TrajOptCompositeProfileMap& composite_profiles)
{
  auto pci = std::make_shared<trajopt::ProblemConstructionInfo>(request.tesseract);

  // Store fixed steps
  std::vector<int> fixed_steps;

  // Assume all the plan instructions have the same manipulator as the composite
  const std::string manipulator = request.instructions.getManipulatorInfo().manipulator;
  const std::string manipulator_ik_solver = request.instructions.getManipulatorInfo().manipulator_ik_solver;

  // Assign Kinematics object
  pci->kin = pci->getManipulator(manipulator);
  std::string link = pci->kin->getTipLinkName();

  if (pci->kin == nullptr)
  {
    std::string error_msg = "In trajopt_array_planner: manipulator_ does not exist in kin_map_";
    CONSOLE_BRIDGE_logError(error_msg.c_str());
    throw std::runtime_error(error_msg);
  }

  // Flatten the input for planning
  auto instructions_flat = flatten(request.instructions);
  auto seed_flat = flattenToPattern(request.seed, request.instructions);

  // Get kinematics information
  tesseract_environment::Environment::ConstPtr env = request.tesseract->getEnvironmentConst();
  tesseract_environment::AdjacencyMap map(
      env->getSceneGraph(), pci->kin->getActiveLinkNames(), env->getCurrentState()->link_transforms);
  const std::vector<std::string>& active_links = map.getActiveLinkNames();

  // Create a temp seed storage.
  std::vector<Eigen::VectorXd> seed_states;
  seed_states.reserve(instructions_flat.size());

  int index = 0;
  std::string profile;
  Waypoint start_waypoint = NullWaypoint();
  Instruction placeholder_instruction = NullInstruction();
  const Instruction* start_instruction = nullptr;
  if (request.instructions.hasStartInstruction())
  {
    assert(isMoveInstruction(request.instructions.getStartInstruction()));
    start_instruction = &(request.instructions.getStartInstruction());
    if (isMoveInstruction(*start_instruction))
    {
      const auto* temp = start_instruction->cast_const<MoveInstruction>();
      assert(temp->isStart());
      start_waypoint = temp->getWaypoint();
      profile = temp->getProfile();
    }
    else
    {
      throw std::runtime_error("TrajOpt DefaultProblemGenerator: Unsupported start instruction type!");
    }
  }
  else
  {
    Eigen::VectorXd current_jv = request.env_state->getJointValues(pci->kin->getJointNames());
    JointWaypoint temp(current_jv);
    temp.joint_names = pci->kin->getJointNames();

    MoveInstruction temp_move(temp, MoveInstructionType::START);
    temp_move.setWaypoint(StateWaypoint(current_jv));
    placeholder_instruction = temp_move;
    start_instruction = &placeholder_instruction;
    start_waypoint = temp;
  }

  // Get Plan Profile
  if (profile.empty())
    profile = "DEFAULT";

  TrajOptPlanProfile::Ptr start_plan_profile{ nullptr };
  auto it = plan_profiles.find(profile);
  if (it == plan_profiles.end())
    start_plan_profile = std::make_shared<TrajOptDefaultPlanProfile>();
  else
    start_plan_profile = it->second;

  // Add start seed state
  assert(request.seed.hasStartInstruction());
  assert(isMoveInstruction(request.seed.getStartInstruction()));
  const auto* seed_instruction = request.seed.getStartInstruction().cast_const<MoveInstruction>();
  seed_states.push_back(getJointPosition(seed_instruction->getWaypoint()));

  // Add start waypoint
  if (isCartesianWaypoint(start_waypoint))
  {
    const auto* cwp = start_waypoint.cast_const<Eigen::Isometry3d>();
    start_plan_profile->apply(*pci, *cwp, *start_instruction, active_links, index);
  }
  else if (isJointWaypoint(start_waypoint))
  {
    const auto* jwp = start_waypoint.cast_const<JointWaypoint>();
    start_plan_profile->apply(*pci, *jwp, *start_instruction, active_links, index);
  }
  else
  {
    throw std::runtime_error("TrajOpt Problem Generator: unknown waypoint type.");
  }

  ++index;

  // ----------------
  // Translate TCL
  // ----------------

  // Transform plan instructions into trajopt cost and constraints
  for (std::size_t i = 0; i < instructions_flat.size(); ++i)
  {
    const auto& instruction = instructions_flat[i].get();
    if (isPlanInstruction(instruction))
    {
      assert(isPlanInstruction(instruction));
      const auto* plan_instruction = instruction.cast_const<PlanInstruction>();

      assert(isCompositeInstruction(seed_flat[i].get()));
      const auto* seed_composite = seed_flat[i].get().cast_const<tesseract_planning::CompositeInstruction>();
      auto interpolate_cnt = static_cast<int>(seed_composite->size());

      // Get Plan Profile
      std::string profile = plan_instruction->getProfile();
      if (profile.empty())
        profile = "DEFAULT";

      TrajOptPlanProfile::Ptr cur_plan_profile{ nullptr };
      auto it = plan_profiles.find(profile);
      if (it == plan_profiles.end())
        cur_plan_profile = std::make_shared<TrajOptDefaultPlanProfile>();
      else
        cur_plan_profile = it->second;

      if (plan_instruction->isLinear())
      {
        if (isCartesianWaypoint(plan_instruction->getWaypoint()))
        {
          const auto* cur_wp = plan_instruction->getWaypoint().cast_const<tesseract_planning::CartesianWaypoint>();

          Eigen::Isometry3d prev_pose = Eigen::Isometry3d::Identity();
          if (isCartesianWaypoint(start_waypoint))
          {
            prev_pose = *(start_waypoint.cast_const<Eigen::Isometry3d>());
          }
          else if (isJointWaypoint(start_waypoint))
          {
            const auto* jwp = start_waypoint.cast_const<JointWaypoint>();
            if (!pci->kin->calcFwdKin(prev_pose, *jwp))
              throw std::runtime_error("TrajOptPlannerUniversalConfig: failed to solve forward kinematics!");

            prev_pose = pci->env->getCurrentState()->link_transforms.at(pci->kin->getBaseLinkName()) * prev_pose *
                        plan_instruction->getManipulatorInfo().tcp;
          }
          else
          {
            throw std::runtime_error("TrajOptPlannerUniversalConfig: uknown waypoint type.");
          }

          tesseract_common::VectorIsometry3d poses = interpolate(prev_pose, *cur_wp, interpolate_cnt);
          // Add intermediate points with path costs and constraints
          for (std::size_t p = 1; p < poses.size() - 1; ++p)
          {
            cur_plan_profile->apply(*pci, poses[p], *plan_instruction, active_links, index);

            // Add seed state
            assert(isMoveInstruction(seed_composite->at(p)));
            const auto* seed_instruction = seed_composite->at(p).cast_const<MoveInstruction>();
            seed_states.push_back(getJointPosition(seed_instruction->getWaypoint()));

            ++index;
          }

          // Add final point with waypoint
          cur_plan_profile->apply(*pci, *cur_wp, *plan_instruction, active_links, index);

          // Add seed state
          assert(isMoveInstruction(seed_composite->back()));
          const auto* seed_instruction = seed_composite->back().cast_const<tesseract_planning::MoveInstruction>();
          seed_states.push_back(getJointPosition(seed_instruction->getWaypoint()));

          ++index;
        }
        else if (isJointWaypoint(plan_instruction->getWaypoint()))
        {
          const auto* cur_wp = plan_instruction->getWaypoint().cast_const<JointWaypoint>();
          Eigen::Isometry3d cur_pose = Eigen::Isometry3d::Identity();
          if (!pci->kin->calcFwdKin(cur_pose, *cur_wp))
            throw std::runtime_error("TrajOptPlannerUniversalConfig: failed to solve forward kinematics!");

          cur_pose = pci->env->getCurrentState()->link_transforms.at(pci->kin->getBaseLinkName()) * cur_pose *
                     plan_instruction->getManipulatorInfo().tcp;

          Eigen::Isometry3d prev_pose = Eigen::Isometry3d::Identity();
          if (isCartesianWaypoint(start_waypoint))
          {
            prev_pose = *(start_waypoint.cast_const<Eigen::Isometry3d>());
          }
          else if (isJointWaypoint(start_waypoint))
          {
            const auto* jwp = start_waypoint.cast_const<JointWaypoint>();
            if (!pci->kin->calcFwdKin(prev_pose, *jwp))
              throw std::runtime_error("TrajOptPlannerUniversalConfig: failed to solve forward kinematics!");

            prev_pose = pci->env->getCurrentState()->link_transforms.at(pci->kin->getBaseLinkName()) * prev_pose *
                        plan_instruction->getManipulatorInfo().tcp;
          }
          else
          {
            throw std::runtime_error("TrajOptPlannerUniversalConfig: uknown waypoint type.");
          }

          tesseract_common::VectorIsometry3d poses = interpolate(prev_pose, cur_pose, interpolate_cnt);
          // Add intermediate points with path costs and constraints
          for (std::size_t p = 1; p < poses.size() - 1; ++p)
          {
            cur_plan_profile->apply(*pci, poses[p], *plan_instruction, active_links, index);

            // Add seed state
            assert(isMoveInstruction(seed_composite->at(p)));
            const auto* seed_instruction = seed_composite->at(p).cast_const<MoveInstruction>();
            seed_states.push_back(getJointPosition(seed_instruction->getWaypoint()));

            ++index;
          }

          // Add final point with waypoint
          cur_plan_profile->apply(*pci, *cur_wp, *plan_instruction, active_links, index);

          // Add seed state
          assert(isMoveInstruction(seed_composite->back()));
          const auto* seed_instruction = seed_composite->back().cast_const<MoveInstruction>();
          seed_states.push_back(getJointPosition(seed_instruction->getWaypoint()));

          ++index;
        }
        else
        {
          throw std::runtime_error("TrajOptPlannerUniversalConfig: unknown waypoint type");
        }
      }
      else if (plan_instruction->isFreespace())
      {
        if (isJointWaypoint(plan_instruction->getWaypoint()))
        {
          const auto* cur_wp = plan_instruction->getWaypoint().cast_const<JointWaypoint>();

          // Add intermediate points with path costs and constraints
          for (std::size_t s = 0; s < seed_composite->size() - 1; ++s)
          {
            // Add seed state
            assert(isMoveInstruction(seed_composite->at(s)));
            const auto* seed_instruction = seed_composite->at(s).cast_const<MoveInstruction>();
            seed_states.push_back(getJointPosition(seed_instruction->getWaypoint()));

            ++index;
          }

          // Add final point with waypoint costs and contraints
          /** @todo Should check that the joint names match the order of the manipulator */
          cur_plan_profile->apply(*pci, *cur_wp, *plan_instruction, active_links, index);

          // Add to fixed indices
          fixed_steps.push_back(index);

          // Add seed state
          assert(isMoveInstruction(seed_composite->back()));
          const auto* seed_instruction = seed_composite->back().cast_const<MoveInstruction>();
          seed_states.push_back(getJointPosition(seed_instruction->getWaypoint()));
        }
        else if (isCartesianWaypoint(plan_instruction->getWaypoint()))
        {
          const auto* cur_wp = plan_instruction->getWaypoint().cast_const<CartesianWaypoint>();

          // Add intermediate points with path costs and constraints
          for (std::size_t s = 0; s < seed_composite->size() - 1; ++s)
          {
            // Add seed state
            assert(isMoveInstruction(seed_composite->at(s)));
            const auto* seed_instruction = seed_composite->at(s).cast_const<MoveInstruction>();
            seed_states.push_back(getJointPosition(seed_instruction->getWaypoint()));

            ++index;
          }

          // Add final point with waypoint costs and contraints
          /** @todo Should check that the joint names match the order of the manipulator */
          cur_plan_profile->apply(*pci, *cur_wp, *plan_instruction, active_links, index);

          // Add to fixed indices
          fixed_steps.push_back(index);

          // Add seed state
          assert(isMoveInstruction(seed_composite->back()));
          const auto* seed_instruction = seed_composite->back().cast_const<MoveInstruction>();
          seed_states.push_back(getJointPosition(seed_instruction->getWaypoint()));
        }
        else
        {
          throw std::runtime_error("TrajOptPlannerUniversalConfig: unknown waypoint type");
        }

        ++index;
      }
      else
      {
        throw std::runtime_error("Unsupported!");
      }

      start_waypoint = plan_instruction->getWaypoint();
      start_instruction = &instruction;
    }
  }

  // ----------------
  // Create Problem
  // ----------------

  // Setup Basic Info
  pci->basic_info.n_steps = index;
  pci->basic_info.manip = manipulator;
  pci->basic_info.start_fixed = false;
  pci->basic_info.use_time = false;
  //  pci->basic_info.convex_solver = optimizer;  // TODO: Fix this when port to trajopt_ifopt

  // Set trajopt seed
  assert(static_cast<long>(seed_states.size()) == pci->basic_info.n_steps);
  pci->init_info.type = trajopt::InitInfo::GIVEN_TRAJ;
  pci->init_info.data.resize(pci->basic_info.n_steps, pci->kin->numJoints());
  for (long i = 0; i < pci->basic_info.n_steps; ++i)
    pci->init_info.data.row(i) = seed_states[static_cast<std::size_t>(i)];

  // Apply Composite Profile
  profile = request.instructions.getProfile();
  if (profile.empty())
    profile = "DEFAULT";

  TrajOptCompositeProfile::Ptr cur_composite_profile{ nullptr };
  auto it_composite = composite_profiles.find(profile);
  if (it_composite == composite_profiles.end())
    cur_composite_profile = std::make_shared<TrajOptDefaultCompositeProfile>();
  else
    cur_composite_profile = it_composite->second;

  cur_composite_profile->apply(*pci, 0, pci->basic_info.n_steps - 1, active_links, fixed_steps);

  // Construct Problem
  trajopt::TrajOptProb::Ptr problem = trajopt::ConstructProblem(*pci);

  return problem;
}

}  // namespace tesseract_planning
