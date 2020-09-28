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
#include <tesseract_motion_planners/trajopt/profile/trajopt_default_composite_profile.h>
#include <tesseract_motion_planners/trajopt/profile/trajopt_default_plan_profile.h>
#include <tesseract_motion_planners/core/utils.h>

namespace tesseract_planning
{
/// @todo: Restructure this into several smaller functions that are testable and easier to understand
trajopt::TrajOptProb::Ptr DefaultTrajoptProblemGenerator(const std::string& name,
                                                         const PlannerRequest& request,
                                                         const TrajOptPlanProfileMap& plan_profiles,
                                                         const TrajOptCompositeProfileMap& composite_profiles)
{
  auto pci = std::make_shared<trajopt::ProblemConstructionInfo>(request.tesseract);

  // Store fixed steps
  std::vector<int> fixed_steps;

  // Assume all the plan instructions have the same manipulator as the composite
  assert(!request.instructions.getManipulatorInfo().empty());
  const ManipulatorInfo& composite_mi = request.instructions.getManipulatorInfo();

  // Assign Kinematics object
  pci->kin = pci->getManipulator(composite_mi.manipulator);
  std::string link = pci->kin->getTipLinkName();

  if (pci->kin == nullptr)
  {
    std::string error_msg = "In trajopt_array_planner: manipulator_ does not exist in kin_map_";
    CONSOLE_BRIDGE_logError(error_msg.c_str());
    throw std::runtime_error(error_msg);
  }

  // Flatten the input for planning
  auto instructions_flat = flattenProgram(request.instructions);
  auto seed_flat = flattenProgramToPattern(request.seed, request.instructions);

  // Get kinematics information
  tesseract_environment::Environment::ConstPtr env = request.tesseract->getEnvironment();
  tesseract_environment::AdjacencyMap map(
      env->getSceneGraph(), pci->kin->getActiveLinkNames(), env->getCurrentState()->link_transforms);
  const std::vector<std::string>& active_links = map.getActiveLinkNames();

  // Create a temp seed storage.
  std::vector<Eigen::VectorXd> seed_states;
  seed_states.reserve(instructions_flat.size());

  std::size_t start_index = 0;  // If it has a start instruction then skip first instruction in instructions_flat
  int index = 0;
  std::string profile;
  Waypoint start_waypoint = NullWaypoint();
  Instruction placeholder_instruction = NullInstruction();
  const Instruction* start_instruction = nullptr;
  if (request.instructions.hasStartInstruction())
  {
    assert(isPlanInstruction(request.instructions.getStartInstruction()));
    start_instruction = &(request.instructions.getStartInstruction());
    if (isPlanInstruction(*start_instruction))
    {
      const auto* temp = start_instruction->cast_const<PlanInstruction>();
      assert(temp->isStart());
      start_waypoint = temp->getWaypoint();
      profile = temp->getProfile();
    }
    else
    {
      throw std::runtime_error("TrajOpt DefaultProblemGenerator: Unsupported start instruction type!");
    }
    ++start_index;
  }
  else
  {
    Eigen::VectorXd current_jv = request.env_state->getJointValues(pci->kin->getJointNames());
    StateWaypoint swp(pci->kin->getJointNames(), current_jv);

    MoveInstruction temp_move(swp, MoveInstructionType::START);
    placeholder_instruction = temp_move;
    start_instruction = &placeholder_instruction;
    start_waypoint = swp;
  }

  // Get Plan Profile
  if (profile.empty())
    profile = "DEFAULT";

  // Check for remapping of profile
  auto remap = request.plan_profile_remapping.find(name);
  if (remap != request.plan_profile_remapping.end())
  {
    auto p = remap->second.find(profile);
    if (p != remap->second.end())
      profile = p->second;
  }

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
    start_plan_profile->apply(*pci, *cwp, *start_instruction, composite_mi, active_links, index);
  }
  else if (isJointWaypoint(start_waypoint) || isStateWaypoint(start_waypoint))
  {
    const Eigen::VectorXd& position = getJointPosition(start_waypoint);
    start_plan_profile->apply(*pci, position, *start_instruction, composite_mi, active_links, index);
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
  for (std::size_t i = start_index; i < instructions_flat.size(); ++i)
  {
    const auto& instruction = instructions_flat[i].get();
    if (isPlanInstruction(instruction))
    {
      assert(isPlanInstruction(instruction));
      const auto* plan_instruction = instruction.cast_const<PlanInstruction>();

      // If plan instruction has manipulator information then use it over the one provided by the composite.
      ManipulatorInfo manip_info = composite_mi.getCombined(plan_instruction->getManipulatorInfo());
      Eigen::Isometry3d tcp = request.tesseract->findTCP(manip_info);

      assert(isCompositeInstruction(seed_flat[i].get()));
      const auto* seed_composite = seed_flat[i].get().cast_const<tesseract_planning::CompositeInstruction>();
      auto interpolate_cnt = static_cast<int>(seed_composite->size());

      // Get Plan Profile
      std::string profile = plan_instruction->getProfile();
      if (profile.empty())
        profile = "DEFAULT";

      // Check for remapping of profile
      auto remap = request.plan_profile_remapping.find(name);
      if (remap != request.plan_profile_remapping.end())
      {
        auto p = remap->second.find(profile);
        if (p != remap->second.end())
          profile = p->second;
      }

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
          else if (isJointWaypoint(start_waypoint) || isStateWaypoint(start_waypoint))
          {
            const Eigen::VectorXd& position = getJointPosition(start_waypoint);
            if (!pci->kin->calcFwdKin(prev_pose, position))
              throw std::runtime_error("TrajOptPlannerUniversalConfig: failed to solve forward kinematics!");

            prev_pose = pci->env->getCurrentState()->link_transforms.at(pci->kin->getBaseLinkName()) * prev_pose * tcp;
          }
          else
          {
            throw std::runtime_error("TrajOptPlannerUniversalConfig: uknown waypoint type.");
          }

          tesseract_common::VectorIsometry3d poses = interpolate(prev_pose, *cur_wp, interpolate_cnt);
          // Add intermediate points with path costs and constraints
          for (std::size_t p = 1; p < poses.size() - 1; ++p)
          {
            cur_plan_profile->apply(*pci, poses[p], *plan_instruction, composite_mi, active_links, index);

            // Add seed state
            assert(isMoveInstruction(seed_composite->at(p)));
            const auto* seed_instruction = seed_composite->at(p).cast_const<MoveInstruction>();
            seed_states.push_back(getJointPosition(seed_instruction->getWaypoint()));

            ++index;
          }

          // Add final point with waypoint
          cur_plan_profile->apply(*pci, *cur_wp, *plan_instruction, composite_mi, active_links, index);

          // Add seed state
          assert(isMoveInstruction(seed_composite->back()));
          const auto* seed_instruction = seed_composite->back().cast_const<tesseract_planning::MoveInstruction>();
          seed_states.push_back(getJointPosition(seed_instruction->getWaypoint()));

          ++index;
        }
        else if (isJointWaypoint(plan_instruction->getWaypoint()) || isStateWaypoint(plan_instruction->getWaypoint()))
        {
          const Eigen::VectorXd& cur_position = getJointPosition(plan_instruction->getWaypoint());
          Eigen::Isometry3d cur_pose = Eigen::Isometry3d::Identity();
          if (!pci->kin->calcFwdKin(cur_pose, cur_position))
            throw std::runtime_error("TrajOptPlannerUniversalConfig: failed to solve forward kinematics!");

          cur_pose = pci->env->getCurrentState()->link_transforms.at(pci->kin->getBaseLinkName()) * cur_pose * tcp;

          Eigen::Isometry3d prev_pose = Eigen::Isometry3d::Identity();
          if (isCartesianWaypoint(start_waypoint))
          {
            prev_pose = *(start_waypoint.cast_const<Eigen::Isometry3d>());
          }
          else if (isJointWaypoint(start_waypoint) || isStateWaypoint(start_waypoint))
          {
            const Eigen::VectorXd& position = getJointPosition(start_waypoint);
            if (!pci->kin->calcFwdKin(prev_pose, position))
              throw std::runtime_error("TrajOptPlannerUniversalConfig: failed to solve forward kinematics!");

            prev_pose = pci->env->getCurrentState()->link_transforms.at(pci->kin->getBaseLinkName()) * prev_pose * tcp;
          }
          else
          {
            throw std::runtime_error("TrajOptPlannerUniversalConfig: uknown waypoint type.");
          }

          tesseract_common::VectorIsometry3d poses = interpolate(prev_pose, cur_pose, interpolate_cnt);
          // Add intermediate points with path costs and constraints
          for (std::size_t p = 1; p < poses.size() - 1; ++p)
          {
            cur_plan_profile->apply(*pci, poses[p], *plan_instruction, composite_mi, active_links, index);

            // Add seed state
            assert(isMoveInstruction(seed_composite->at(p)));
            const auto* seed_instruction = seed_composite->at(p).cast_const<MoveInstruction>();
            seed_states.push_back(getJointPosition(seed_instruction->getWaypoint()));

            ++index;
          }

          // Add final point with waypoint
          cur_plan_profile->apply(*pci, cur_position, *plan_instruction, composite_mi, active_links, index);

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
        if (isJointWaypoint(plan_instruction->getWaypoint()) || isStateWaypoint(plan_instruction->getWaypoint()))
        {
          const Eigen::VectorXd& cur_position = getJointPosition(plan_instruction->getWaypoint());

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
          cur_plan_profile->apply(*pci, cur_position, *plan_instruction, composite_mi, active_links, index);

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
          cur_plan_profile->apply(*pci, *cur_wp, *plan_instruction, composite_mi, active_links, index);

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
    }
  }

  // ----------------
  // Create Problem
  // ----------------

  // Setup Basic Info
  pci->basic_info.n_steps = index;
  pci->basic_info.manip = composite_mi.manipulator;
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

  // Check for remapping of profile
  remap = request.composite_profile_remapping.find(name);
  if (remap != request.composite_profile_remapping.end())
  {
    auto p = remap->second.find(profile);
    if (p != remap->second.end())
      profile = p->second;
  }

  TrajOptCompositeProfile::Ptr cur_composite_profile{ nullptr };
  auto it_composite = composite_profiles.find(profile);
  if (it_composite == composite_profiles.end())
  {
    CONSOLE_BRIDGE_logDebug("Trajopt profile not found. Setting default");
    cur_composite_profile = std::make_shared<TrajOptDefaultCompositeProfile>();
  }
  else
    cur_composite_profile = it_composite->second;

  cur_composite_profile->apply(*pci, 0, pci->basic_info.n_steps - 1, composite_mi, active_links, fixed_steps);

  // Construct Problem
  trajopt::TrajOptProb::Ptr problem = trajopt::ConstructProblem(*pci);

  return problem;
}

}  // namespace tesseract_planning
