/**
 * @file fix_state_collisions_process_generator.cpp
 * @brief Process generator for process that pushes plan instructions to be out of collision
 *
 * @author Matthew Powelson
 * @date August 31. 2020
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

#include <tesseract_common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <console_bridge/console.h>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <trajopt/problem_description.hpp>
#include <tesseract_environment/core/utils.h>

#include <tesseract_process_managers/process_generators/fix_state_collision_process_generator.h>
#include <tesseract_command_language/utils/utils.h>

namespace tesseract_planning
{
bool StateInCollision(const Eigen::Ref<Eigen::VectorXd>& start_pos,
                      const ProcessInput& input,
                      const FixStateCollisionProfile& profile)
{
  using namespace tesseract_collision;
  using namespace tesseract_environment;

  auto env = input.tesseract->getEnvironment();
  auto kin = input.tesseract->getManipulatorManager()->getFwdKinematicSolver(input.manip_info.manipulator);

  std::vector<ContactResultMap> collisions;
  tesseract_environment::StateSolver::Ptr state_solver = env->getStateSolver();
  DiscreteContactManager::Ptr manager = env->getDiscreteContactManager();
  AdjacencyMap::Ptr adjacency_map = std::make_shared<tesseract_environment::AdjacencyMap>(
      env->getSceneGraph(), kin->getActiveLinkNames(), env->getCurrentState()->link_transforms);

  manager->setActiveCollisionObjects(adjacency_map->getActiveLinkNames());
  manager->setContactDistanceThreshold(profile.safety_margin);
  collisions.clear();

  tesseract_common::TrajArray traj(1, start_pos.size());
  traj.row(0) = start_pos.transpose();

  // This never returns any collisions
  if (!checkTrajectory(collisions, *manager, *state_solver, kin->getJointNames(), traj))
  {
    CONSOLE_BRIDGE_logDebug("No collisions found");
    return false;
  }
  else
  {
    CONSOLE_BRIDGE_logDebug("Waypoint is not contact free!");
    for (std::size_t i = 0; i < collisions.size(); i++)
      for (const auto& contact_vec : collisions[i])
        for (const auto& contact : contact_vec.second)
          CONSOLE_BRIDGE_logDebug(("timestep: " + std::to_string(i) + " Links: " + contact.link_names[0] + ", " +
                                   contact.link_names[1] + " Dist: " + std::to_string(contact.distance))
                                      .c_str());
  }

  return true;
}

bool WaypointInCollision(const Waypoint& waypoint, const ProcessInput& input, const FixStateCollisionProfile& profile)
{
  // Get position associated with waypoint
  Eigen::VectorXd start_pos;
  try
  {
    start_pos = getJointPosition(waypoint);
  }
  catch (std::runtime_error& e)
  {
    CONSOLE_BRIDGE_logError("WaypointInCollision error: %s", e.what());
    return false;
  }
  return StateInCollision(start_pos, input, profile);
}

bool MoveWaypointFromCollisionTrajopt(Waypoint& waypoint,
                                      const ProcessInput& input,
                                      const FixStateCollisionProfile& profile)
{
  using namespace trajopt;

  // Get position associated with waypoint
  Eigen::VectorXd start_pos;
  try
  {
    start_pos = getJointPosition(waypoint);
  }
  catch (std::runtime_error& e)
  {
    CONSOLE_BRIDGE_logError("MoveWaypointFromCollision error: %s", e.what());
    return false;
  }
  std::size_t num_jnts = static_cast<std::size_t>(start_pos.size());

  // Setup trajopt problem with basic info
  ProblemConstructionInfo pci(input.tesseract);
  pci.basic_info.n_steps = 1;
  pci.basic_info.manip = input.manip_info.manipulator;
  pci.basic_info.start_fixed = false;
  pci.basic_info.use_time = false;

  // Create Kinematic Object
  pci.kin = pci.getManipulator(pci.basic_info.manip);

  // Initialize trajectory to waypoint position
  pci.init_info.type = InitInfo::GIVEN_TRAJ;
  pci.init_info.data = tesseract_common::TrajArray(1, start_pos.size());
  pci.init_info.data.row(0) = start_pos.transpose();

  // Add constraint that position is allowed to be within a tolerance of the original position
  {
    Eigen::MatrixX2d limits = pci.getManipulator(pci.basic_info.manip)->getLimits().joint_limits;
    Eigen::VectorXd range = limits.col(1).array() - limits.col(0).array();
    Eigen::VectorXd pos_tolerance = range * profile.jiggle_factor;
    Eigen::VectorXd neg_tolerance = range * -profile.jiggle_factor;

    auto jp = std::make_shared<JointPosTermInfo>();
    jp->coeffs = std::vector<double>(num_jnts, 1.0);
    jp->targets = std::vector<double>(start_pos.data(), start_pos.data() + start_pos.size());
    jp->upper_tols = std::vector<double>(pos_tolerance.data(), pos_tolerance.data() + pos_tolerance.size());
    jp->lower_tols = std::vector<double>(neg_tolerance.data(), neg_tolerance.data() + neg_tolerance.size());
    jp->first_step = 0;
    jp->last_step = 0;
    jp->name = "joint_pos";
    jp->term_type = TT_CNT;
    pci.cnt_infos.push_back(jp);
  }

  // Add a constraint that it must not be in collision
  {
    auto collision = std::make_shared<CollisionTermInfo>();
    collision->name = "collision";
    collision->term_type = TT_CNT;
    collision->evaluator_type = trajopt::CollisionEvaluatorType::SINGLE_TIMESTEP;
    collision->first_step = 0;
    collision->last_step = 0;
    collision->info = createSafetyMarginDataVector(pci.basic_info.n_steps, profile.safety_margin, 1);
    collision->use_weighted_sum = true;
    pci.cnt_infos.push_back(collision);
  }
  // Add an additional cost to collisions to help it converge
  {
    auto collision = std::make_shared<CollisionTermInfo>();
    collision->name = "collision";
    collision->term_type = TT_COST;
    collision->evaluator_type = trajopt::CollisionEvaluatorType::SINGLE_TIMESTEP;
    collision->first_step = 0;
    collision->last_step = 0;
    collision->info = createSafetyMarginDataVector(pci.basic_info.n_steps, profile.safety_margin, 20);
    collision->use_weighted_sum = true;
    pci.cost_infos.push_back(collision);
  }
  auto prob = ConstructProblem(pci);

  // Run trajopt optimization
  sco::BasicTrustRegionSQP opt(prob);
  opt.initialize(trajToDblVec(prob->GetInitTraj()));
  opt.optimize();
  if (opt.results().status != sco::OptStatus::OPT_CONVERGED)
  {
    CONSOLE_BRIDGE_logError("MoveWaypointFromCollision did not converge");
    return false;
  }
  Eigen::VectorXd results(start_pos.size());
  results = getTraj(opt.x(), prob->GetVars()).row(0);
  return setJointPosition(waypoint, results);
}

bool MoveWaypointFromCollisionRandomSampler(Waypoint& waypoint,
                                            const ProcessInput& input,
                                            const FixStateCollisionProfile& profile)
{
  // Get position associated with waypoint
  Eigen::VectorXd start_pos;
  try
  {
    start_pos = getJointPosition(waypoint);
  }
  catch (std::runtime_error& e)
  {
    CONSOLE_BRIDGE_logError("MoveWaypointFromCollision error: %s", e.what());
    return false;
  }

  const auto kin = input.tesseract->getManipulatorManager()->getFwdKinematicSolver(input.manip_info.manipulator);
  Eigen::MatrixXd limits = kin->getLimits().joint_limits;
  Eigen::VectorXd range = limits.col(1).array() - limits.col(0).array();
  Eigen::VectorXd pos_sampling_limits = range * profile.jiggle_factor;
  Eigen::VectorXd neg_sampline_limits = range * -profile.jiggle_factor;

  for (int i = 0; i < profile.sampling_attempts; i++)
  {
    Eigen::VectorXd start_sampled_pos =
        start_pos + Eigen::VectorXd::Random(start_pos.size()) * range * profile.jiggle_factor;

    // Make sure it doesn't violate joint limits
    Eigen::VectorXd sampled_pos = start_sampled_pos;
    sampled_pos = sampled_pos.cwiseMax(limits.col(0));
    sampled_pos = sampled_pos.cwiseMin(limits.col(1));

    if (!StateInCollision(sampled_pos, input, profile))
    {
      return setJointPosition(waypoint, sampled_pos);
    }
  }

  return false;
}

bool ApplyCorrectionWorkflow(Waypoint& waypoint, const ProcessInput& input, const FixStateCollisionProfile& profile)
{
  for (const auto& method : profile.correction_workflow)
  {
    switch (method)
    {
      case FixStateCollisionProfile::CorrectionMethod::NONE:
        return false;  // No correction and in collision, so return false
      case FixStateCollisionProfile::CorrectionMethod::TRAJOPT:
        if (MoveWaypointFromCollisionTrajopt(waypoint, input, profile))
          return true;
        break;
      case FixStateCollisionProfile::CorrectionMethod::RANDOM_SAMPLER:
        if (MoveWaypointFromCollisionRandomSampler(waypoint, input, profile))
          return true;
        break;
    }
  }
  // If all methods have tried without returning, then correction failed
  return false;
}

FixStateCollisionProcessGenerator::FixStateCollisionProcessGenerator(std::string name) : name_(std::move(name))
{
  // Register default profile
  composite_profiles["DEFAULT"] = std::make_shared<FixStateCollisionProfile>();
}

const std::string& FixStateCollisionProcessGenerator::getName() const { return name_; }

std::function<void()> FixStateCollisionProcessGenerator::generateTask(ProcessInput input)
{
  return [=]() { process(input); };
}

std::function<int()> FixStateCollisionProcessGenerator::generateConditionalTask(ProcessInput input)
{
  return [=]() { return conditionalProcess(input); };
}

int FixStateCollisionProcessGenerator::conditionalProcess(ProcessInput input) const
{
  if (abort_)
    return 0;

  // --------------------
  // Check that inputs are valid
  // --------------------
  const Instruction* input_intruction = input.getInstruction();
  if (!isCompositeInstruction(*(input_intruction)))
  {
    CONSOLE_BRIDGE_logError("Input instruction to FixStateCollision must be a composite instruction");
    return 0;
  }

  const auto* ci = input_intruction->cast_const<CompositeInstruction>();
  const ManipulatorInfo& manip_info = input.manip_info;
  const auto fwd_kin = input.tesseract->getManipulatorManager()->getFwdKinematicSolver(manip_info.manipulator);

  // Get Composite profile
  std::string profile = ci->getProfile();
  if (profile.empty())
    profile = "DEFAULT";

  // Check for remapping of composite profile
  {
    auto remap = input.composite_profile_remapping.find(name_);
    if (remap != input.composite_profile_remapping.end())
    {
      auto p = remap->second.find(profile);
      if (p != remap->second.end())
        profile = p->second;
    }
  }

  // Get the parameters associated with this profile
  typename FixStateCollisionProfile::Ptr cur_composite_profile{ nullptr };
  auto it = composite_profiles.find(profile);
  if (it == composite_profiles.end())
    cur_composite_profile = std::make_shared<FixStateCollisionProfile>();
  else
    cur_composite_profile = it->second;

  switch (cur_composite_profile->mode)
  {
    case FixStateCollisionProfile::Settings::START_ONLY:
    {
      const PlanInstruction* instr_const_ptr = getFirstPlanInstruction(*ci);
      if (instr_const_ptr)
      {
        PlanInstruction* mutable_instruction = const_cast<PlanInstruction*>(instr_const_ptr);
        if (WaypointInCollision(mutable_instruction->getWaypoint(), input, *cur_composite_profile))
        {
          CONSOLE_BRIDGE_logInform("FixStateCollisionProcessGenerator is modifying the const input instructions");
          if (!ApplyCorrectionWorkflow(mutable_instruction->getWaypoint(), input, *cur_composite_profile))
            return 0;
        }
      }
    }
    break;
    case FixStateCollisionProfile::Settings::END_ONLY:
    {
      const PlanInstruction* instr_const_ptr = getLastPlanInstruction(*ci);
      if (instr_const_ptr)
      {
        PlanInstruction* mutable_instruction = const_cast<PlanInstruction*>(instr_const_ptr);
        if (WaypointInCollision(mutable_instruction->getWaypoint(), input, *cur_composite_profile))
        {
          CONSOLE_BRIDGE_logInform("FixStateCollisionProcessGenerator is modifying the const input instructions");
          if (!ApplyCorrectionWorkflow(mutable_instruction->getWaypoint(), input, *cur_composite_profile))
            return 0;
        }
      }
    }
    break;
    case FixStateCollisionProfile::Settings::ALL:
    {
      auto flattened = flatten(*ci, planFilter);
      if (flattened.empty())
      {
        CONSOLE_BRIDGE_logWarn("FixStateCollisionProcessGenerator found no PlanInstructions to process");
        return 1;
      }

      bool in_collision = false;
      for (const auto& instruction : flattened)
      {
        in_collision |= WaypointInCollision(
            instruction.get().cast_const<PlanInstruction>()->getWaypoint(), input, *cur_composite_profile);
      }
      if (!in_collision)
        break;

      CONSOLE_BRIDGE_logInform("FixStateCollisionProcessGenerator is modifying the const input instructions");
      for (const auto& instruction : flattened)
      {
        const Instruction* instr_const_ptr = &instruction.get();
        Instruction* mutable_instruction = const_cast<Instruction*>(instr_const_ptr);
        PlanInstruction* plan = mutable_instruction->cast<PlanInstruction>();

        if (!ApplyCorrectionWorkflow(plan->getWaypoint(), input, *cur_composite_profile))
          return 0;
      }
    }
    break;
    case FixStateCollisionProfile::Settings::DISABLED:
      return 1;
  }

  CONSOLE_BRIDGE_logDebug("FixStateCollisionProcessGenerator succeeded");
  return 1;
}

void FixStateCollisionProcessGenerator::process(ProcessInput input) const { conditionalProcess(input); }

bool FixStateCollisionProcessGenerator::getAbort() const { return abort_; }
void FixStateCollisionProcessGenerator::setAbort(bool abort) { abort_ = abort; }

}  // namespace tesseract_planning
