#include <tesseract/common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <benchmark/benchmark.h>
#include <console_bridge/console.h>
TESSERACT_COMMON_IGNORE_WARNINGS_POP
#include <tesseract/environment/environment.h>
#include <tesseract/environment/utils.h>
#include <tesseract/environment/commands/add_link_command.h>
#include <tesseract/scene_graph/graph.h>
#include <tesseract/scene_graph/link.h>
#include <tesseract/scene_graph/joint.h>
#include <tesseract/state_solver/state_solver.h>
#include <tesseract/common/resource_locator.h>
#include <tesseract/srdf/srdf_model.h>
#include <tesseract/urdf/urdf_parser.h>
#include <tesseract/geometry/impl/sphere.h>
#include <tesseract/collision/continuous_contact_manager.h>
#include <tesseract/collision/discrete_contact_manager.h>
#include <tesseract/kinematics/joint_group.h>

using namespace tesseract::scene_graph;
using namespace tesseract::srdf;
using namespace tesseract::collision;
using namespace tesseract::environment;

SceneGraph::Ptr getSceneGraph()
{
  std::string url = "package://tesseract/support/urdf/lbr_iiwa_14_r820.urdf";

  tesseract::common::GeneralResourceLocator locator;
  return tesseract::urdf::parseURDFFile(locator.locateResource(url)->getFilePath(), locator);
}

SRDFModel::Ptr getSRDFModel(const SceneGraph& scene_graph)
{
  std::string url = "package://tesseract/support/urdf/lbr_iiwa_14_r820.srdf";
  tesseract::common::GeneralResourceLocator locator;

  auto srdf = std::make_shared<SRDFModel>();
  srdf->initFile(scene_graph, locator.locateResource(url)->getFilePath(), locator);

  return srdf;
}

static void BM_CHECK_TRAJECTORY_CONTINUOUS_SS(benchmark::State& state,
                                              std::vector<tesseract::collision::ContactResultMap> contacts,
                                              const tesseract::collision::ContinuousContactManager::Ptr& manager,
                                              const StateSolver::Ptr& state_solver,
                                              const std::vector<std::string>& joint_names,
                                              const tesseract::common::TrajArray& traj,
                                              const tesseract::collision::CollisionCheckConfig& config,
                                              bool log_level_debug)
{
  if (log_level_debug)
    console_bridge::setLogLevel(console_bridge::LogLevel::CONSOLE_BRIDGE_LOG_DEBUG);
  else
    console_bridge::setLogLevel(console_bridge::LogLevel::CONSOLE_BRIDGE_LOG_INFO);

  for (auto _ : state)  // NOLINT
  {
    benchmark::DoNotOptimize(checkTrajectory(contacts, *manager, *state_solver, joint_names, traj, config));
  }
}

static void BM_CHECK_TRAJECTORY_CONTINUOUS_MANIP(benchmark::State& state,
                                                 std::vector<tesseract::collision::ContactResultMap> contacts,
                                                 const tesseract::collision::ContinuousContactManager::Ptr& manager,
                                                 const tesseract::kinematics::JointGroup::ConstPtr& manip,
                                                 const tesseract::common::TrajArray& traj,
                                                 const tesseract::collision::CollisionCheckConfig& config,
                                                 bool log_level_debug)
{
  if (log_level_debug)
    console_bridge::setLogLevel(console_bridge::LogLevel::CONSOLE_BRIDGE_LOG_DEBUG);
  else
    console_bridge::setLogLevel(console_bridge::LogLevel::CONSOLE_BRIDGE_LOG_INFO);

  for (auto _ : state)  // NOLINT
  {
    benchmark::DoNotOptimize(checkTrajectory(contacts, *manager, *manip, traj, config));
  }
}

static void BM_CHECK_TRAJECTORY_DISCRETE_SS(benchmark::State& state,
                                            std::vector<tesseract::collision::ContactResultMap> contacts,
                                            const tesseract::collision::DiscreteContactManager::Ptr& manager,
                                            const StateSolver::Ptr& state_solver,
                                            const std::vector<std::string>& joint_names,
                                            const tesseract::common::TrajArray& traj,
                                            const tesseract::collision::CollisionCheckConfig& config,
                                            bool log_level_debug)
{
  if (log_level_debug)
    console_bridge::setLogLevel(console_bridge::LogLevel::CONSOLE_BRIDGE_LOG_DEBUG);
  else
    console_bridge::setLogLevel(console_bridge::LogLevel::CONSOLE_BRIDGE_LOG_INFO);

  for (auto _ : state)  // NOLINT
  {
    benchmark::DoNotOptimize(checkTrajectory(contacts, *manager, *state_solver, joint_names, traj, config));
  }
}

static void BM_CHECK_TRAJECTORY_DISCRETE_MANIP(benchmark::State& state,
                                               std::vector<tesseract::collision::ContactResultMap> contacts,
                                               const tesseract::collision::DiscreteContactManager::Ptr& manager,
                                               const tesseract::kinematics::JointGroup::ConstPtr& manip,
                                               const tesseract::common::TrajArray& traj,
                                               const tesseract::collision::CollisionCheckConfig& config,
                                               bool log_level_debug)
{
  if (log_level_debug)
    console_bridge::setLogLevel(console_bridge::LogLevel::CONSOLE_BRIDGE_LOG_DEBUG);
  else
    console_bridge::setLogLevel(console_bridge::LogLevel::CONSOLE_BRIDGE_LOG_INFO);

  for (auto _ : state)  // NOLINT
  {
    benchmark::DoNotOptimize(checkTrajectory(contacts, *manager, *manip, traj, config));
  }
}

int main(int argc, char** argv)
{
  auto env = std::make_shared<Environment>();
  SceneGraph::Ptr scene_graph = getSceneGraph();
  auto srdf = getSRDFModel(*scene_graph);
  env->init(*scene_graph, srdf);
  env->setResourceLocator(std::make_shared<tesseract::common::GeneralResourceLocator>());

  // Add sphere to environment
  Link link_sphere("sphere_attached");

  Visual::Ptr visual = std::make_shared<Visual>();
  visual->origin = Eigen::Isometry3d::Identity();
  visual->origin.translation() = Eigen::Vector3d(0.5, 0, 0.55);
  visual->geometry = std::make_shared<tesseract::geometry::Sphere>(0.15);
  link_sphere.visual.push_back(visual);

  Collision::Ptr collision = std::make_shared<Collision>();
  collision->origin = visual->origin;
  collision->geometry = visual->geometry;
  link_sphere.collision.push_back(collision);

  Joint joint_sphere("joint_sphere_attached");
  joint_sphere.parent_link_name = "base_link";
  joint_sphere.child_link_name = link_sphere.getName();
  joint_sphere.type = JointType::FIXED;

  auto cmd = std::make_shared<tesseract::environment::AddLinkCommand>(link_sphere, joint_sphere);

  env->applyCommand(cmd);

  // Set the robot initial state
  std::vector<std::string> joint_names;
  joint_names.emplace_back("joint_a1");
  joint_names.emplace_back("joint_a2");
  joint_names.emplace_back("joint_a3");
  joint_names.emplace_back("joint_a4");
  joint_names.emplace_back("joint_a5");
  joint_names.emplace_back("joint_a6");
  joint_names.emplace_back("joint_a7");

  Eigen::VectorXd joint_start_pos(7);
  joint_start_pos(0) = -0.4;
  joint_start_pos(1) = 0.2762;
  joint_start_pos(2) = 0.0;
  joint_start_pos(3) = -1.3348;
  joint_start_pos(4) = 0.0;
  joint_start_pos(5) = 1.4959;
  joint_start_pos(6) = 0.0;

  Eigen::VectorXd joint_end_pos(7);
  joint_end_pos(0) = 0.4;
  joint_end_pos(1) = 0.2762;
  joint_end_pos(2) = 0.0;
  joint_end_pos(3) = -1.3348;
  joint_end_pos(4) = 0.0;
  joint_end_pos(5) = 1.4959;
  joint_end_pos(6) = 0.0;

  Eigen::VectorXd joint_pos_collision(7);
  joint_pos_collision(0) = 0.0;
  joint_pos_collision(1) = 0.2762;
  joint_pos_collision(2) = 0.0;
  joint_pos_collision(3) = -1.3348;
  joint_pos_collision(4) = 0.0;
  joint_pos_collision(5) = 1.4959;
  joint_pos_collision(6) = 0.0;

  std::vector<tesseract::common::TrajArray> traj_arrays;

  // Only intermediat states are in collision
  tesseract::common::TrajArray traj(5, joint_start_pos.size());
  for (int i = 0; i < joint_start_pos.size(); ++i)
    traj.col(i) = Eigen::VectorXd::LinSpaced(5, joint_start_pos(i), joint_end_pos(i));

  // Only start state is not in collision
  tesseract::common::TrajArray traj2(3, joint_start_pos.size());
  for (int i = 0; i < joint_start_pos.size(); ++i)
    traj2.col(i) = Eigen::VectorXd::LinSpaced(3, joint_start_pos(i), joint_pos_collision(i));

  // Only start state is not in collision
  tesseract::common::TrajArray traj3(3, joint_start_pos.size());
  for (int i = 0; i < joint_start_pos.size(); ++i)
    traj3.col(i) = Eigen::VectorXd::LinSpaced(3, joint_pos_collision(i), joint_end_pos(i));

  // Only two states
  tesseract::common::TrajArray traj4(2, joint_start_pos.size());
  traj4.row(0) = joint_pos_collision;
  traj4.row(1) = joint_end_pos;

  tesseract::common::TrajArray traj5(2, joint_start_pos.size());
  traj5.row(0) = joint_start_pos;
  traj5.row(1) = joint_pos_collision;

  traj_arrays.push_back(traj);
  traj_arrays.push_back(traj2);
  traj_arrays.push_back(traj3);
  traj_arrays.push_back(traj4);
  traj_arrays.push_back(traj5);

  tesseract::collision::DiscreteContactManager::Ptr discrete_manager = env->getDiscreteContactManager();
  tesseract::collision::ContinuousContactManager::Ptr continuous_manager = env->getContinuousContactManager();
  StateSolver::Ptr state_solver = env->getStateSolver();
  tesseract::kinematics::JointGroup::ConstPtr joint_group = env->getJointGroup("manipulator");
  std::vector<tesseract::collision::ContactResultMap> contacts;
  tesseract::collision::CollisionCheckConfig discrete_config;
  discrete_config.type = CollisionEvaluatorType::DISCRETE;
  discrete_config.check_program_mode = CollisionCheckProgramType::ALL;
  discrete_config.exit_condition = CollisionCheckExitType::ALL;
  tesseract::collision::CollisionCheckConfig discrete_lvs_config;
  discrete_lvs_config.type = CollisionEvaluatorType::LVS_DISCRETE;
  discrete_lvs_config.check_program_mode = CollisionCheckProgramType::ALL;
  discrete_lvs_config.exit_condition = CollisionCheckExitType::ALL;
  tesseract::collision::CollisionCheckConfig continuous_config;
  continuous_config.type = CollisionEvaluatorType::CONTINUOUS;
  continuous_config.check_program_mode = CollisionCheckProgramType::ALL;
  continuous_config.exit_condition = CollisionCheckExitType::ALL;
  tesseract::collision::CollisionCheckConfig continuous_lvs_config;
  continuous_lvs_config.type = CollisionEvaluatorType::LVS_CONTINUOUS;
  continuous_lvs_config.check_program_mode = CollisionCheckProgramType::ALL;
  continuous_lvs_config.exit_condition = CollisionCheckExitType::ALL;

  //////////////////////////////////////
  // Clone
  //////////////////////////////////////

  {
    std::function<void(benchmark::State&,
                       std::vector<tesseract::collision::ContactResultMap>,
                       tesseract::collision::ContinuousContactManager::Ptr,
                       StateSolver::Ptr,
                       std::vector<std::string>,
                       tesseract::common::TrajArray,
                       tesseract::collision::CollisionCheckConfig,
                       bool)>
        BM_CHECK_TRAJ_CS = BM_CHECK_TRAJECTORY_CONTINUOUS_SS;
    std::function<void(benchmark::State&,
                       std::vector<tesseract::collision::ContactResultMap>,
                       tesseract::collision::ContinuousContactManager::Ptr,
                       tesseract::kinematics::JointGroup::ConstPtr,
                       tesseract::common::TrajArray,
                       tesseract::collision::CollisionCheckConfig,
                       bool)>
        BM_CHECK_TRAJ_CM = BM_CHECK_TRAJECTORY_CONTINUOUS_MANIP;
    std::function<void(benchmark::State&,
                       std::vector<tesseract::collision::ContactResultMap>,
                       tesseract::collision::DiscreteContactManager::Ptr,
                       StateSolver::Ptr,
                       std::vector<std::string>,
                       tesseract::common::TrajArray,
                       tesseract::collision::CollisionCheckConfig,
                       bool)>
        BM_CHECK_TRAJ_DS = BM_CHECK_TRAJECTORY_DISCRETE_SS;
    std::function<void(benchmark::State&,
                       std::vector<tesseract::collision::ContactResultMap>,
                       tesseract::collision::DiscreteContactManager::Ptr,
                       tesseract::kinematics::JointGroup::ConstPtr,
                       tesseract::common::TrajArray,
                       tesseract::collision::CollisionCheckConfig,
                       bool)>
        BM_CHECK_TRAJ_DM = BM_CHECK_TRAJECTORY_DISCRETE_MANIP;

    for (const bool log_level_debug : { false, true })
    {
      for (std::size_t i = 0; i < traj_arrays.size(); i++)
      {
        std::string debug_str;
        if (log_level_debug)
          debug_str = "-Debug";
        const auto& traj_array = traj_arrays[i];
        std::string BM_CHECK_TRAJ_CS_name =
            "BM_CHECK_TRAJ_CONTINUOUS_STATE_SOLVER-TRAJ" + std::to_string(i + 1) + debug_str;
        std::string BM_CHECK_TRAJ_CS_LVS_name =
            "BM_CHECK_TRAJ_CONTINUOUS_STATE_SOLVER-TRAJ" + std::to_string(i + 1) + "-LVS" + debug_str;
        std::string BM_CHECK_TRAJ_CM_name =
            "BM_CHECK_TRAJ_CONTINUOUS_JOINT_GROUP-TRAJ" + std::to_string(i + 1) + debug_str;
        std::string BM_CHECK_TRAJ_CM_LVS_name =
            "BM_CHECK_TRAJ_CONTINUOUS_JOINT_GROUP-TRAJ" + std::to_string(i + 1) + "-LVS" + debug_str;
        std::string BM_CHECK_TRAJ_DS_name =
            "BM_CHECK_TRAJ_DISCRETE_STATE_SOLVER-TRAJ" + std::to_string(i + 1) + debug_str;
        std::string BM_CHECK_TRAJ_DS_LVS_name =
            "BM_CHECK_TRAJ_DISCRETE_STATE_SOLVER-TRAJ" + std::to_string(i + 1) + "-LVS" + debug_str;
        std::string BM_CHECK_TRAJ_DM_name =
            "BM_CHECK_TRAJ_DISCRETE_JOINT_GROUP-TRAJ" + std::to_string(i + 1) + debug_str;
        std::string BM_CHECK_TRAJ_DM_LVS_name =
            "BM_CHECK_TRAJ_DISCRETE_JOINT_GROUP-TRAJ" + std::to_string(i + 1) + "-LVS" + debug_str;

        benchmark::RegisterBenchmark(BM_CHECK_TRAJ_CS_name.c_str(),
                                     BM_CHECK_TRAJ_CS,
                                     contacts,
                                     continuous_manager,
                                     state_solver,
                                     joint_names,
                                     traj_array,
                                     continuous_config,
                                     log_level_debug)
            ->UseRealTime()
            ->Unit(benchmark::TimeUnit::kMicrosecond);
        benchmark::RegisterBenchmark(BM_CHECK_TRAJ_CS_LVS_name.c_str(),
                                     BM_CHECK_TRAJ_CS,
                                     contacts,
                                     continuous_manager,
                                     state_solver,
                                     joint_names,
                                     traj_array,
                                     continuous_lvs_config,
                                     log_level_debug)
            ->UseRealTime()
            ->Unit(benchmark::TimeUnit::kMicrosecond);
        benchmark::RegisterBenchmark(BM_CHECK_TRAJ_CM_name.c_str(),
                                     BM_CHECK_TRAJ_CM,
                                     contacts,
                                     continuous_manager,
                                     joint_group,
                                     traj_array,
                                     continuous_config,
                                     log_level_debug)
            ->UseRealTime()
            ->Unit(benchmark::TimeUnit::kMicrosecond);
        benchmark::RegisterBenchmark(BM_CHECK_TRAJ_CM_LVS_name.c_str(),
                                     BM_CHECK_TRAJ_CM,
                                     contacts,
                                     continuous_manager,
                                     joint_group,
                                     traj_array,
                                     continuous_lvs_config,
                                     log_level_debug)
            ->UseRealTime()
            ->Unit(benchmark::TimeUnit::kMicrosecond);
        benchmark::RegisterBenchmark(BM_CHECK_TRAJ_DS_name.c_str(),
                                     BM_CHECK_TRAJ_DS,
                                     contacts,
                                     discrete_manager,
                                     state_solver,
                                     joint_names,
                                     traj_array,
                                     discrete_config,
                                     log_level_debug)
            ->UseRealTime()
            ->Unit(benchmark::TimeUnit::kMicrosecond);
        benchmark::RegisterBenchmark(BM_CHECK_TRAJ_DS_LVS_name.c_str(),
                                     BM_CHECK_TRAJ_DS,
                                     contacts,
                                     discrete_manager,
                                     state_solver,
                                     joint_names,
                                     traj_array,
                                     discrete_lvs_config,
                                     log_level_debug)
            ->UseRealTime()
            ->Unit(benchmark::TimeUnit::kMicrosecond);
        benchmark::RegisterBenchmark(BM_CHECK_TRAJ_DM_name.c_str(),
                                     BM_CHECK_TRAJ_DM,
                                     contacts,
                                     discrete_manager,
                                     joint_group,
                                     traj_array,
                                     discrete_config,
                                     log_level_debug)
            ->UseRealTime()
            ->Unit(benchmark::TimeUnit::kMicrosecond);
        benchmark::RegisterBenchmark(BM_CHECK_TRAJ_DM_LVS_name.c_str(),
                                     BM_CHECK_TRAJ_DM,
                                     contacts,
                                     discrete_manager,
                                     joint_group,
                                     traj_array,
                                     discrete_lvs_config,
                                     log_level_debug)
            ->UseRealTime()
            ->Unit(benchmark::TimeUnit::kMicrosecond);
      }
    }
  }

  //////////////////////////////////////
  // Discrete LVS Contact Request Type & Exit Condition Comparison
  //////////////////////////////////////

  {
    std::function<void(benchmark::State&,
                       std::vector<tesseract::collision::ContactResultMap>,
                       tesseract::collision::DiscreteContactManager::Ptr,
                       tesseract::kinematics::JointGroup::ConstPtr,
                       tesseract::common::TrajArray,
                       tesseract::collision::CollisionCheckConfig,
                       bool)>
        BM_CHECK_TRAJ_DM = BM_CHECK_TRAJECTORY_DISCRETE_MANIP;

    // Define all contact test type options
    std::vector<std::pair<ContactTestType, std::string>> contact_test_types = { { ContactTestType::FIRST, "FIRST" },
                                                                                { ContactTestType::CLOSEST, "CLOSEST" },
                                                                                { ContactTestType::ALL, "ALL" },
                                                                                { ContactTestType::LIMITED,
                                                                                  "LIMITED" } };

    // Define all exit condition options
    std::vector<std::pair<CollisionCheckExitType, std::string>> exit_conditions = {
      { CollisionCheckExitType::FIRST, "FIRST" },
      { CollisionCheckExitType::ONE_PER_STEP, "ONE_PER_STEP" },
      { CollisionCheckExitType::ALL, "ALL" }
    };

    for (std::size_t i = 0; i < traj_arrays.size(); i++)
    {
      const auto& traj_array = traj_arrays[i];

      for (const auto& contact_test_type : contact_test_types)
      {
        for (const auto& exit_condition : exit_conditions)
        {
          // Create discrete LVS config with specific contact request type and exit condition
          tesseract::collision::CollisionCheckConfig discrete_lvs_test_config;
          discrete_lvs_test_config.type = CollisionEvaluatorType::LVS_DISCRETE;
          discrete_lvs_test_config.contact_request.type = contact_test_type.first;
          discrete_lvs_test_config.check_program_mode = CollisionCheckProgramType::ALL;
          discrete_lvs_test_config.exit_condition = exit_condition.first;

          std::string config_suffix = "_" + contact_test_type.second + "_" + exit_condition.second;

          // Current implementation
          std::string BM_CHECK_TRAJ_DM_CURRENT_name =
              "BM_CHECK_TRAJ_DISCRETE_LVS_JOINT_GROUP_CURRENT-TRAJ" + std::to_string(i + 1) + config_suffix;

          // Register current implementation
          benchmark::RegisterBenchmark(BM_CHECK_TRAJ_DM_CURRENT_name.c_str(),
                                       BM_CHECK_TRAJ_DM,
                                       contacts,
                                       discrete_manager,
                                       joint_group,
                                       traj_array,
                                       discrete_lvs_test_config,
                                       false)  // log_level_debug = false
              ->UseRealTime()
              ->Unit(benchmark::TimeUnit::kMicrosecond);
        }
      }
    }
  }

  benchmark::Initialize(&argc, argv);
  benchmark::RunSpecifiedBenchmarks();
}
