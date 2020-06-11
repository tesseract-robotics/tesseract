#include <tesseract_command_language/planners/trajopt/trajopt_default_profile.h>
#include <tesseract_command_language/planners/trajopt/trajopt_planner_universal_config.h>
#include <tesseract_command_language/planners/trajopt/trajopt_utils.h>

#include <tesseract_command_language/core/instruction.h>
#include <tesseract_command_language/plan_instruction.h>
#include <tesseract_command_language/move_instruction.h>
#include <tesseract_command_language/cartesian_waypoint.h>
#include <tesseract_command_language/joint_waypoint.h>
#include <tesseract_motion_planners/core/utils.h>

static const double LONGEST_VALID_SEGMENT_FRACTION_DEFAULT = 0.01;

namespace tesseract_planning
{

TrajOptProfileResults TrajOptDefaultProfile::generate(const TrajOptPlannerUniversalConfig& config)
{
  // -------- Construct the problem ------------
  //  -------------------------------------------
  auto pci = std::make_shared<trajopt::ProblemConstructionInfo>(config.tesseract);
  auto pair = addInstructions(*pci, config);

  if (collision_constraint_config.enabled)
    addCollisionConstraint(*pci, pair.second, config);

  if (collision_cost_config.enabled)
    addCollisionCost(*pci, pair.second, config);

  if (smooth_velocities)
    addVelocitySmoothing(*pci, pair.second, config);

  if (smooth_accelerations)
    addAccelerationSmoothing(*pci, pair.second, config);

  if (smooth_jerks)
    addJerkSmoothing(*pci, pair.second, config);

  if (!constraint_error_functions.empty())
    addConstraintErrorFunctions(*pci, pair.second, config);

  if (avoid_singularity)
    addAvoidSingularity(*pci, pair.second, config);

  TrajOptProfileResults result;
  result.pci = pci;
  result.plan_instruction_indices = pair.first;
  return result;
}

std::pair<std::vector<std::size_t>, std::vector<int>>
TrajOptDefaultProfile::addInstructions(trajopt::ProblemConstructionInfo &pci,
                                       const TrajOptPlannerUniversalConfig& config) const
{
  // Store plan instruction indices
  std::vector<std::size_t> plan_instruction_indices;
  std::vector<int> fixed_steps;

  // Assign Kinematics object
  pci.kin = pci.getManipulator(config.manipulator);
  std::string link = pci.kin->getTipLinkName();

  if (pci.kin == nullptr)
  {
    std::string error_msg = "In trajopt_array_planner: manipulator_ does not exist in kin_map_";
    CONSOLE_BRIDGE_logError(error_msg.c_str());
    throw std::runtime_error(error_msg);
  }

  // Check and make sure it does not contain any composite instruction
  const PlanInstruction* start_instruction {nullptr};
  for (const auto& instruction : config.instructions)
  {
    if (instruction.isComposite())
      throw std::runtime_error("Trajopt planner does not support child composite instructions.");

    if (start_instruction == nullptr && instruction.isPlan())
      start_instruction = instruction.cast_const<PlanInstruction>();
  }

  // Get kinematics information
  tesseract_environment::Environment::ConstPtr env = config.tesseract->getEnvironmentConst();
  tesseract_environment::AdjacencyMap map(
      env->getSceneGraph(), pci.kin->getActiveLinkNames(), env->getCurrentState()->link_transforms);
  const std::vector<std::string>& adjacency_links = map.getActiveLinkNames();

  // Create a temp seed storage.
  std::vector<Eigen::VectorXd> seed_states;
  seed_states.reserve(config.instructions.size());

  // Transform plan instructions into trajopt cost and constraints
  const PlanInstruction* prev_plan_instruction {nullptr};
  int index = 0;
  for (std::size_t i = 0; i < config.instructions.size(); ++i)
  {
    const auto& instruction = config.instructions[i];
    if (instruction.isPlan())
    {
      // Save plan index for process trajectory
      plan_instruction_indices.push_back(i);

      assert(instruction.getType() == static_cast<int>(InstructionType::PLAN_INSTRUCTION));
      const auto* plan_instruction = instruction.cast_const<PlanInstruction>();
      const Waypoint& wp = plan_instruction->getWaypoint();
      const std::string& working_frame = plan_instruction->getWorkingFrame();
      const Eigen::Isometry3d& tcp = plan_instruction->getTCP();

      assert(config.seed[i].isComposite());
      const auto* seed_composite = config.seed[i].cast_const<tesseract_planning::CompositeInstruction>();

      if (plan_instruction->isLinear())
      {
        /** @todo This should also handle if waypoint type is joint */
        const auto* cur_wp = plan_instruction->getWaypoint().cast_const<tesseract_planning::CartesianWaypoint>();
        auto coeffs = Eigen::VectorXd::Constant(1, 1, 5);
        if (prev_plan_instruction)
        {
          /** @todo This should also handle if waypoint type is joint */
          const auto* pre_wp = prev_plan_instruction->getWaypoint().cast_const<tesseract_planning::CartesianWaypoint>();

          tesseract_common::VectorIsometry3d poses = tesseract_motion_planners::interpolate(*pre_wp, *cur_wp, static_cast<int>(seed_composite->size()));
          // Add intermediate points with path costs and constraints
          for (std::size_t p = 1; p < poses.size() - 1; ++p)
          {
            tesseract_planning::CartesianWaypoint p_cpw = poses[p];

            /* Check if this cartesian waypoint is dynamic
             * (i.e. defined relative to a frame that will move with the kinematic chain)
             */
            auto it = std::find(adjacency_links.begin(), adjacency_links.end(), working_frame);
            if (it != adjacency_links.end())
            {
              auto ti = createDynamicCartesianWaypointTermInfo(p_cpw, index, working_frame, tcp, coeffs, link, trajopt::TT_CNT);
              pci.cnt_infos.push_back(ti);
            }
            else
            {
              auto ti = createCartesianWaypointTermInfo(p_cpw, index, working_frame, tcp, coeffs, link, trajopt::TT_CNT);
              pci.cnt_infos.push_back(ti);
            }

            // Add seed state
            assert(seed_composite->at(p - 1).isMove());
            const auto* seed_instruction = seed_composite->at(p - 1).cast_const<tesseract_planning::MoveInstruction>();
            seed_states.push_back(seed_instruction->getPosition());

            ++index;
          }
        }

        // Add final point with waypoint costs and contraints

        /* Check if this cartesian waypoint is dynamic
         * (i.e. defined relative to a frame that will move with the kinematic chain)
         */
        auto it = std::find(adjacency_links.begin(), adjacency_links.end(), working_frame);
        if (it != adjacency_links.end())
        {
          auto ti = createDynamicCartesianWaypointTermInfo(*cur_wp, index, working_frame, tcp, coeffs, link, trajopt::TT_CNT);
          pci.cnt_infos.push_back(ti);
        }
        else
        {
          auto ti = createCartesianWaypointTermInfo(*cur_wp, index, working_frame, tcp, coeffs, link, trajopt::TT_CNT);
          pci.cnt_infos.push_back(ti);
        }

        // Add seed state
        assert(seed_composite->back().isMove());
        const auto* seed_instruction = seed_composite->back().cast_const<tesseract_planning::MoveInstruction>();
        seed_states.push_back(seed_instruction->getPosition());

        ++index;
        prev_plan_instruction = plan_instruction;
      }
      else if (plan_instruction->isFreespace())
      {
        /** @todo This should also handle if waypoint type is cartesian */
        const auto* cur_wp = plan_instruction->getWaypoint().cast_const<tesseract_planning::JointWaypoint>();
        auto coeffs = Eigen::VectorXd::Constant(1, 1, 5);
        if (prev_plan_instruction)
        {
          /** @todo This should also handle if waypoint type is cartesian */
          const auto* pre_wp = prev_plan_instruction->getWaypoint().cast_const<tesseract_planning::JointWaypoint>();

          Eigen::MatrixXd states = tesseract_motion_planners::interpolate(*pre_wp, *cur_wp, static_cast<int>(seed_composite->size()));
          // Add intermediate points with path costs and constraints
          for (long s = 1; s < states.cols() - 1; ++s)
          {
            // Add seed state
            tesseract_planning::JointWaypoint i_jpw = states.col(s);
            assert(seed_composite->at(static_cast<std::size_t>(s - 1)).isMove());
            const auto* seed_instruction = seed_composite->at(static_cast<std::size_t>(s - 1)).cast_const<tesseract_planning::MoveInstruction>();
            seed_states.push_back(seed_instruction->getPosition());

            ++index;
          }
        }

        // Add final point with waypoint costs and contraints
        /** @todo Should check that the joint names match the order of the manipulator */
        auto ti = createJointWaypointTermInfo(*cur_wp, index, coeffs, trajopt::TT_CNT);
        pci.cnt_infos.push_back(ti);

        // Add to fixed indices
        fixed_steps.push_back(index);

        // Add seed state
        assert(seed_composite->back().isMove());
        const auto* seed_instruction = seed_composite->back().cast_const<tesseract_planning::MoveInstruction>();
        seed_states.push_back(seed_instruction->getPosition());

        ++index;
      }
      else
      {
        throw std::runtime_error("Unsupported!");
      }

      prev_plan_instruction = plan_instruction;
    }
  }

  // Setup Basic Info
  pci.basic_info.n_steps = index;
  pci.basic_info.manip = config.manipulator;
  pci.basic_info.start_fixed = false;
  pci.basic_info.use_time = false;
  pci.basic_info.convex_solver = config.optimizer;

  // Set trajopt seed
  assert(static_cast<long>(seed_states.size()) == pci.basic_info.n_steps);
  pci.init_info.type = trajopt::InitInfo::GIVEN_TRAJ;
  pci.init_info.data.resize(pci.basic_info.n_steps, pci.kin->numJoints());
  for(long i = 0; i < pci.basic_info.n_steps; ++i)
    pci.init_info.data.row(i) = seed_states[static_cast<std::size_t>(i)];

  return std::make_pair(plan_instruction_indices, fixed_steps);
}

void TrajOptDefaultProfile::addCollisionCost(trajopt::ProblemConstructionInfo& pci,
                                             const std::vector<int>& fixed_steps,
                                             const TrajOptPlannerUniversalConfig& config) const
{
  // Calculate longest valid segment length
  const Eigen::MatrixX2d& limits = pci.kin->getLimits();
  double length = 0;
  double extent = (limits.col(1) - limits.col(0)).norm();
  if (config.longest_valid_segment_fraction > 0 && config.longest_valid_segment_length > 0)
  {
    length = std::min(config.longest_valid_segment_fraction * extent, config.longest_valid_segment_length);
  }
  else if (config.longest_valid_segment_fraction > 0)
  {
    length = config.longest_valid_segment_fraction * extent;
  }
  else if (config.longest_valid_segment_length > 0)
  {
    length = config.longest_valid_segment_length;
  }
  else
  {
    length = LONGEST_VALID_SEGMENT_FRACTION_DEFAULT * extent;
  }

  // Create a default collision term info
  trajopt::TermInfo::Ptr ti = createCollisionTermInfo(0,
                                                      pci.basic_info.n_steps - 1,
                                                      collision_cost_config.buffer_margin,
                                                      collision_constraint_config.safety_margin_buffer,
                                                      collision_cost_config.type,
                                                      collision_cost_config.use_weighted_sum,
                                                      collision_cost_config.coeff,
                                                      contact_test_type,
                                                      length,
                                                      trajopt::TermType::TT_COST);

  // Update the term info with the (possibly) new start and end state indices for which to apply this cost
  std::shared_ptr<trajopt::CollisionTermInfo> ct = std::static_pointer_cast<trajopt::CollisionTermInfo>(ti);
  if (config.special_collision_cost)
  {
    for (auto& info : ct->info)
    {
      info = config.special_collision_cost;
    }
  }
  ct->fixed_steps = fixed_steps;

  pci.cost_infos.push_back(ct);
}

void TrajOptDefaultProfile::addCollisionConstraint(trajopt::ProblemConstructionInfo& pci,
                                                   const std::vector<int>& fixed_steps,
                                                   const TrajOptPlannerUniversalConfig& config) const
{
  // Calculate longest valid segment length
  const Eigen::MatrixX2d& limits = pci.kin->getLimits();
  double length = 0;
  double extent = (limits.col(1) - limits.col(0)).norm();
  if (config.longest_valid_segment_fraction > 0 && config.longest_valid_segment_length > 0)
  {
    length = std::min(config.longest_valid_segment_fraction * extent, config.longest_valid_segment_length);
  }
  else if (config.longest_valid_segment_fraction > 0)
  {
    length = config.longest_valid_segment_fraction * extent;
  }
  else if (config.longest_valid_segment_length > 0)
  {
    length = config.longest_valid_segment_length;
  }
  else
  {
    length = LONGEST_VALID_SEGMENT_FRACTION_DEFAULT * extent;
  }

  // Create a default collision term info
  trajopt::TermInfo::Ptr ti = createCollisionTermInfo(0,
                                                      pci.basic_info.n_steps - 1,
                                                      collision_constraint_config.safety_margin,
                                                      collision_constraint_config.safety_margin_buffer,
                                                      collision_constraint_config.type,
                                                      collision_cost_config.use_weighted_sum,
                                                      collision_constraint_config.coeff,
                                                      contact_test_type,
                                                      length,
                                                      trajopt::TermType::TT_CNT);

  // Update the term info with the (possibly) new start and end state indices for which to apply this cost
  std::shared_ptr<trajopt::CollisionTermInfo> ct = std::static_pointer_cast<trajopt::CollisionTermInfo>(ti);
  if (config.special_collision_constraint)
  {
    for (auto& info : ct->info)
    {
      info = config.special_collision_constraint;
    }
  }
  ct->fixed_steps = fixed_steps;

  pci.cnt_infos.push_back(ct);
}

void TrajOptDefaultProfile::addVelocitySmoothing(trajopt::ProblemConstructionInfo& pci,
                                                 const std::vector<int>& /*fixed_steps*/,
                                                 const TrajOptPlannerUniversalConfig& /*config*/) const
{
  if (velocity_coeff.size() == 0)
    pci.cost_infos.push_back(createSmoothVelocityTermInfo(0, pci.basic_info.n_steps - 1, static_cast<int>(pci.kin->numJoints())));
  else
    pci.cost_infos.push_back(createSmoothVelocityTermInfo(0, pci.basic_info.n_steps - 1, velocity_coeff));
}

void TrajOptDefaultProfile::addAccelerationSmoothing(trajopt::ProblemConstructionInfo& pci,
                                                     const std::vector<int>& /*fixed_steps*/,
                                                     const TrajOptPlannerUniversalConfig& /*config*/) const
{
  if (acceleration_coeff.size() == 0)
    pci.cost_infos.push_back(createSmoothAccelerationTermInfo(0, pci.basic_info.n_steps - 1, static_cast<int>(pci.kin->numJoints())));
  else
    pci.cost_infos.push_back(createSmoothAccelerationTermInfo(0, pci.basic_info.n_steps - 1, acceleration_coeff));
}

void TrajOptDefaultProfile::addJerkSmoothing(trajopt::ProblemConstructionInfo& pci,
                                             const std::vector<int>& /*fixed_steps*/,
                                             const TrajOptPlannerUniversalConfig& /*config*/) const
{
  if (jerk_coeff.size() == 0)
    pci.cost_infos.push_back(createSmoothJerkTermInfo(0, pci.basic_info.n_steps - 1, static_cast<int>(pci.kin->numJoints())));
  else
    pci.cost_infos.push_back(createSmoothJerkTermInfo(0, pci.basic_info.n_steps - 1, jerk_coeff));
}

void TrajOptDefaultProfile::addConstraintErrorFunctions(trajopt::ProblemConstructionInfo& pci,
                                                        const std::vector<int>& fixed_steps,
                                                        const TrajOptPlannerUniversalConfig& /*config*/) const
{
  for (std::size_t i = 0; i < constraint_error_functions.size(); ++i)
  {
    auto& c = constraint_error_functions[i];
    trajopt::TermInfo::Ptr ti = createUserDefinedTermInfo(0,
                                                          pci.basic_info.n_steps - 1,
                                                          std::get<0>(c),
                                                          std::get<1>(c),
                                                          trajopt::TermType::TT_CNT);
    ti->name = "user_defined_" + std::to_string(i);

    // Update the term info with the (possibly) new start and end state indices for which to apply this cost
    std::shared_ptr<trajopt::UserDefinedTermInfo> ef = std::static_pointer_cast<trajopt::UserDefinedTermInfo>(ti);
    ef->term_type = trajopt::TT_CNT;
    ef->constraint_type = std::get<2>(c);
    ef->coeff = std::get<3>(c);
    ef->first_step = 0;
    ef->last_step = pci.basic_info.n_steps - 1;
    ef->fixed_steps = fixed_steps;

    pci.cnt_infos.push_back(ef);
  }
}

void TrajOptDefaultProfile::addAvoidSingularity(trajopt::ProblemConstructionInfo& pci,
                                                const std::vector<int>& /*fixed_steps*/,
                                                const TrajOptPlannerUniversalConfig& /*config*/) const
{
  trajopt::TermInfo::Ptr ti = createAvoidSingularityTermInfo(0, pci.basic_info.n_steps - 1, pci.kin->getTipLinkName(), avoid_singularity_coeff);
  pci.cost_infos.push_back(ti);
}

}
