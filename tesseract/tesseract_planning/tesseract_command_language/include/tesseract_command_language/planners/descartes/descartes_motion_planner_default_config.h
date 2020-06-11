#ifndef TESSERACT_COMMAND_LANGUAGE_DESCARTES_MOTION_PLANNER_DEFAULT_CONFIG_H
#define TESSERACT_COMMAND_LANGUAGE_DESCARTES_MOTION_PLANNER_DEFAULT_CONFIG_H

#include <tesseract/tesseract.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <descartes_light/interface/collision_interface.h>
#include <descartes_light/interface/kinematics_interface.h>
#include <descartes_light/interface/edge_evaluator.h>
#include <descartes_light/interface/position_sampler.h>
#include <descartes_light/descartes_light.h>
#include <Eigen/Geometry>
#include <vector>
#include <string>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract/tesseract.h>
#include <tesseract_kinematics/core/validate.h>
#include <tesseract_command_language/composite_instruction.h>
#include <tesseract_command_language/plan_instruction.h>
#include <tesseract_command_language/move_instruction.h>
#include <tesseract_command_language/cartesian_waypoint.h>
#include <tesseract_command_language/joint_waypoint.h>
#include <tesseract_command_language/planners/descartes/descartes_motion_planner_config.h>

#include <tesseract_kinematics/core/forward_kinematics.h>
#include <tesseract_kinematics/core/inverse_kinematics.h>

#include <tesseract_motion_planners/descartes/descartes_collision.h>
#include <tesseract_motion_planners/core/utils.h>

#include <tesseract_command_language/descartes/descartes_utils.h>

namespace tesseract_planning
{
template <typename FloatType>
struct DescartesMotionPlannerDefaultConfig : DescartesMotionPlannerConfig<FloatType>
{
  using Ptr = std::shared_ptr<DescartesMotionPlannerDefaultConfig<FloatType>>;
  using ConstPtr = std::shared_ptr<const DescartesMotionPlannerDefaultConfig<FloatType>>;

  DescartesMotionPlannerDefaultConfig(tesseract::Tesseract::ConstPtr tesseract_ptr)
    : tesseract(std::move(tesseract_ptr))
  {
  }

  virtual ~DescartesMotionPlannerDefaultConfig() = default;
  DescartesMotionPlannerDefaultConfig(const DescartesMotionPlannerDefaultConfig&) = default;
  DescartesMotionPlannerDefaultConfig& operator=(const DescartesMotionPlannerDefaultConfig&) = default;
  DescartesMotionPlannerDefaultConfig(DescartesMotionPlannerDefaultConfig&&) = default;             // NOLINT
  DescartesMotionPlannerDefaultConfig& operator=(DescartesMotionPlannerDefaultConfig&&) = default;  // NOLINT

  tesseract::Tesseract::ConstPtr tesseract;
  tesseract_environment::EnvState::ConstPtr env_state;
  std::string manipulator;
  std::string manipulator_ik_solver;
  std::string manipulator_reach;

  std::string positioner;

  enum Configuration
  {
    ROBOT_ONLY,
    ROBOT_ON_POSITIONER,
    ROBOT_WITH_EXTERNAL_POSITIONER
  };

  Configuration configuration;

//  std::vector<std::string> active_link_names;
//  std::vector<std::string> joint_names;

//  Eigen::Isometry3d tcp = Eigen::Isometry3d::Identity();
//  Eigen::Isometry3d world_to_base = Eigen::Isometry3d::Identity();

  /**
   * @brief The program instruction
   * This must containt a minimum of two move instruction the first move instruction is the start state
   */
  CompositeInstruction instructions;

  /**
   * @brief This should be a one to one match with the instructions where the PlanInstruction is replaced with a
   * composite instruction of MoveInstructions.
   */
  CompositeInstruction seed;

  bool generate() override
  {

    return DescartesMotionPlannerConfig<FloatType>::generate();
  }
private:
  std::vector<std::size_t> plan_instruction_indices_;

  void getManipulatorInfo()
  {
    manip_fwd_kin_ = tesseract->getFwdKinematicsManagerConst()->getFwdKinematicSolver(manipulator);
    manip_inv_kin_ = tesseract->getInvKinematicsManagerConst()->getInvKinematicSolver(manipulator, manipulator_ik_solver);
    this->dof = manip_fwd_kin_->numJoints();

    if (configuration == ROBOT_ON_POSITIONER || configuration == ROBOT_WITH_EXTERNAL_POSITIONER)
      positioner_fwd_kin_ = tesseract->getFwdKinematicsManagerConst()->getFwdKinematicSolver(positioner);
  }

//  template <typename FloatType>
//  std::vector<typename descartes_light::PositionSampler<FloatType>::Ptr>
//  makeRobotSamplers(const std::vector<Waypoint::Ptr>& path,
//                    const typename descartes_light::CollisionInterface<FloatType>::Ptr& collision_interface,
//                    const tesseract_environment::EnvState::ConstPtr current_state,
//                    const Eigen::Isometry3d& robot_tcp,
//                    const double robot_reach,
//                    const bool allow_collision,
//                    const DescartesIsValidFn<FloatType>& is_valid)
//  {
//    const std::vector<std::string>& joint_names = robot_kinematics->getJointNames();
//    std::vector<typename descartes_light::PositionSampler<FloatType>::Ptr> result;
//    result.reserve(path.size());

//    for (const auto& wp : path)
//    {
//      typename descartes_light::CollisionInterface<FloatType>::Ptr ci = nullptr;
//      if (collision_interface != nullptr)
//        ci = collision_interface->clone();

//      tesseract_motion_planners::PoseSamplerFn target_pose_sampler = getPoseSampler(wp,
//                                                                                    x_sample_resolution,
//                                                                                    y_sample_resolution,
//                                                                                    z_sample_resolution,
//                                                                                    x_axes_sample_resolution,
//                                                                                    y_axes_sample_resolution,
//                                                                                    z_axes_sample_resolution);
//      if (wp->getType() == WaypointType::CARTESIAN_WAYPOINT)
//      {
//        CartesianWaypoint::ConstPtr cwp = std::static_pointer_cast<const CartesianWaypoint>(wp);

//        // Check if the waypoint is not relative to the world coordinate system
//        Eigen::Isometry3d world_to_waypoint = Eigen::Isometry3d::Identity();
//        if (!cwp->getParentLinkName().empty())
//          world_to_waypoint = current_state->link_transforms.at(cwp->getParentLinkName());

//        auto sampler = std::make_shared<DescartesRobotSampler<FloatType>>(world_to_waypoint * cwp->getTransform(),
//                                                                          target_pose_sampler,
//                                                                          robot_kinematics,
//                                                                          ci,
//                                                                          current_state,
//                                                                          robot_tcp,
//                                                                          robot_reach,
//                                                                          allow_collision,
//                                                                          is_valid);
//        result.push_back(std::move(sampler));
//      }
//      else if (wp->getType() == WaypointType::JOINT_WAYPOINT)
//      {
//        JointWaypoint::ConstPtr jwp = std::static_pointer_cast<const JointWaypoint>(wp);
//        Eigen::Matrix<FloatType, 1, Eigen::Dynamic> jwp_positions = jwp->getPositions(joint_names).cast<FloatType>();
//        std::vector<FloatType> joint_pose(jwp_positions.data(),
//                                          jwp_positions.data() + jwp_positions.rows() * jwp_positions.cols());
//        auto sampler = std::make_shared<descartes_light::FixedJointPoseSampler<FloatType>>(joint_pose);
//        result.push_back(std::move(sampler));
//      }
//      else
//      {
//        CONSOLE_BRIDGE_logError("Tesseract Descartes planner does not currently support waypoint type: %d",
//                                wp->getType());
//        return std::vector<typename descartes_light::PositionSampler<FloatType>::Ptr>();
//      }
//    }

  descartes_light::PositionSampler<FloatType>::Ptr
  createCartesianComponents(const CartesianWaypoint& c_wp,
                            int index,
                            std::string working_frame,
                            Eigen::Isometry3d tcp,
                            const std::vector<ComponentInfo>& components,
                            std::string link)
  {
    typename descartes_light::CollisionInterface<FloatType>::Ptr ci = nullptr;
    if (collision_interface != nullptr)
      ci = collision_interface->clone();

    for (const auto& component : components)
    {
      tesseract_motion_planners::PoseSamplerFn sampler = getPoseSampler(component);
      /** @todo Need to figure out how to handle cases when there is more than one cartesian constraint or cost */
      if (sampler != nullptr)
      {
        // Check if the waypoint is not relative to the world coordinate system
        Eigen::Isometry3d world_to_waypoint = Eigen::Isometry3d::Identity();
        if (!working_frame.empty())
          world_to_waypoint = current_state->link_transforms.at(working_frame);

        auto sampler = std::make_shared<DescartesRobotSampler<FloatType>>(world_to_waypoint * c_wp
                                                                          target_pose_sampler,
                                                                          robot_kinematics,
                                                                          ci,
                                                                          current_state,
                                                                          robot_tcp,
                                                                          robot_reach,
                                                                          allow_collision,
                                                                          is_valid);
        break;
      }
    }
  }

  bool RobotOnlyGenerate()
  {
    if (!tesseract_kinematics::checkKinematics(manip_fwd_kin_, manip_inv_kin_))
      CONSOLE_BRIDGE_logError("Check Kinematics failed. This means that Inverse Kinematics does not agree with KDL (TrajOpt). Did you change the URDF recently?");

    std::vector<std::string> joint_names = manip_inv_kin_->getJointNames();
    std::vector<std::string> active_link_names = manip_inv_kin_->getActiveLinkNames();

    auto adjacency_map = std::make_shared<tesseract_environment::AdjacencyMap>(tesseract->getEnvironmentConst()->getSceneGraph(), active_link_names, env_state->link_transforms);
    const std::vector<std::string>& adjacency_links = adjacency_map->getActiveLinkNames();

    // Check and make sure it does not contain any composite instruction
    const PlanInstruction* start_instruction {nullptr};
    for (const auto& instruction : instructions)
    {
      if (instruction.isComposite())
        throw std::runtime_error("Descartes planner does not support child composite instructions.");

      if (start_instruction == nullptr && instruction.isPlan())
        start_instruction = instruction.cast_const<PlanInstruction>();
    }

    // Create a temp seed storage.
    std::vector<Eigen::VectorXd> seed_states;
    seed_states.reserve(instructions.size());

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
        const Waypoint& wp = plan_instruction->getWaypoint();
        const std::string& working_frame = plan_instruction->getWorkingFrame();
        const Eigen::Isometry3d& tcp = plan_instruction->getTCP();

        assert(seed[i].isComposite());
        const auto* seed_composite = seed[i].cast_const<tesseract_planning::CompositeInstruction>();

        if (plan_instruction->isLinear())
        {
          /** @todo This should also handle if waypoint type is joint */
          const auto* cur_wp = plan_instruction->getWaypoint().cast_const<tesseract_planning::CartesianWaypoint>();
          if (prev_plan_instruction)
          {
            /** @todo This should also handle if waypoint type is joint */
            const auto* pre_wp = prev_plan_instruction->getWaypoint().cast_const<tesseract_planning::CartesianWaypoint>();

            tesseract_common::VectorIsometry3d poses = tesseract_motion_planners::interpolate(*pre_wp, *cur_wp, static_cast<int>(seed_composite->size()));
            // Add intermediate points with path costs and constraints
            for (std::size_t p = 1; p < poses.size() - 1; ++p)
            {
              tesseract_planning::CartesianWaypoint p_cpw = poses[p];
//              createCartesianComponents(p_cpw, index, working_frame, tcp, plan_instruction->getPathCosts(), link);
              createCartesianComponents(p_cpw, index, working_frame, tcp, plan_instruction->getPathConstraints(), link);

              /** @todo Add createDynamicCartesianWaypointTermInfo */
              /* Check if this cartesian waypoint is dynamic
               * (i.e. defined relative to a frame that will move with the kinematic chain)
               */
              auto it = std::find(adjacency_links.begin(), adjacency_links.end(), working_frame);
              if (it != adjacency_links.end())
                throw std::runtime_error("Dynamic cartesian waypoint is currently not supported!");

              // Add seed state
              assert(seed_composite->at(p - 1).isMove());
              const auto* seed_instruction = seed_composite->at(p - 1).cast_const<tesseract_planning::MoveInstruction>();
              seed_states.push_back(seed_instruction->getPosition());

              ++index;
            }
          }

          // Add final point with waypoint costs and contraints
//          createCartesianComponents(pci, *cur_wp, index, working_frame, tcp, plan_instruction->getCosts(), link, trajopt::TT_COST);
//          createCartesianComponents(pci, *cur_wp, index, working_frame, tcp, plan_instruction->getConstraints(), link, trajopt::TT_CNT);

          /** @todo Add createDynamicCartesianWaypointTermInfo */
          /* Check if this cartesian waypoint is dynamic
           * (i.e. defined relative to a frame that will move with the kinematic chain)
           */
          auto it = std::find(adjacency_links.begin(), adjacency_links.end(), working_frame);
          if (it != adjacency_links.end())
            throw std::runtime_error("Dynamic cartesian waypoint is currently not supported!");

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
          if (prev_plan_instruction)
          {
            /** @todo This should also handle if waypoint type is cartesian */
            const auto* pre_wp = prev_plan_instruction->getWaypoint().cast_const<tesseract_planning::JointWaypoint>();

            Eigen::MatrixXd states = tesseract_motion_planners::interpolate(*pre_wp, *cur_wp, static_cast<int>(seed_composite->size()));
            // Add intermediate points with path costs and constraints
            for (long s = 1; s < states.cols() - 1; ++s)
            {
              tesseract_planning::JointWaypoint i_jpw = states.col(s);
//              createJointComponents(pci, i_jpw, index, plan_instruction->getPathCosts(), link, trajopt::TT_COST);
//              createJointComponents(pci, i_jpw, index, plan_instruction->getPathConstraints(), link, trajopt::TT_CNT);

              // Add seed state
              assert(seed_composite->at(static_cast<std::size_t>(s - 1)).isMove());
              const auto* seed_instruction = seed_composite->at(static_cast<std::size_t>(s - 1)).cast_const<tesseract_planning::MoveInstruction>();
              seed_states.push_back(seed_instruction->getPosition());

              ++index;
            }
          }

          // Add final point with waypoint costs and contraints
//          createJointComponents(pci, *cur_wp, index, plan_instruction->getCosts(), link, trajopt::TT_COST);
//          createJointComponents(pci, *cur_wp, index, plan_instruction->getConstraints(), link, trajopt::TT_CNT);

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

//    // Get Collision Interface
//    auto coll_interface = std::make_shared<tesseract_motion_planners::DescartesCollision<FloatType>>(tesseract->getEnvironmentConst(), adjacency_map->getActiveLinkNames(), joint_names, contact_distance);

//    // Create Timing Constraint
//    int s = 10;
//    std::vector<descartes_core::TimingConstraint<FloatType>> timing(s, std::numeric_limits<FloatType>::max());

//    // Create Edge Evaluator
//    auto edge_computer = std::make_shared<descartes_light::EuclideanDistanceEdgeEvaluator<FloatType>>(manip_inv_kin_->numJoints());

//    // Create isValid function pointer
//    auto is_valid = std::bind(&twc_motion_planning::isValidState<FloatType>, std::placeholders::_1, full_kin->getLimits().cast<FloatType>(), check_kin);

//    // Create Position Samplers
//    std::vector<typename descartes_light::PositionSampler<FloatType>::Ptr> position_samplers =
//        tesseract_motion_planners::makeRobotSamplers<FloatType>(waypoints,
//                                                                robot_inv_kin,
//                                                                coll_interface,
//                                                                current_state,
//                                                                robot_tcp,
//                                                                robot_reach,
//                                                                true,
//                                                                is_valid,
//                                                                0.001,
//                                                                0.001,
//                                                                0.001,
//                                                                10 * M_PI / 180,
//                                                                10 * M_PI / 180,
//                                                                10 * M_PI / 180);

  }

  bool RobotOnPositionerGenerate()
  {

  }

  bool RobotWithExternalPositionerGenerate()
  {

  }

  tesseract_kinematics::ForwardKinematics::ConstPtr manip_fwd_kin_;
  tesseract_kinematics::InverseKinematics::ConstPtr manip_inv_kin_;

  tesseract_kinematics::ForwardKinematics::ConstPtr positioner_fwd_kin_;

};

using DescartesMotionPlannerConfigD = DescartesMotionPlannerConfig<double>;
using DescartesMotionPlannerConfigF = DescartesMotionPlannerConfig<float>;
}
#endif // TESSERACT_COMMAND_LANGUAGE_DESCARTES_MOTION_PLANNER_DEFAULT_CONFIG_H
