#include <tesseract_common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <benchmark/benchmark.h>
#include <console_bridge/console.h>
TESSERACT_COMMON_IGNORE_WARNINGS_POP
#include <tesseract_environment/environment.h>
#include <tesseract_environment/utils.h>
#include <tesseract_environment/commands/add_link_command.h>
#include <tesseract_scene_graph/graph.h>
#include <tesseract_scene_graph/link.h>
#include <tesseract_scene_graph/joint.h>
#include <tesseract_state_solver/state_solver.h>
#include <tesseract_common/resource_locator.h>
#include <tesseract_srdf/srdf_model.h>
#include <tesseract_urdf/urdf_parser.h>
#include <tesseract_geometry/impl/sphere.h>
#include <tesseract_collision/core/continuous_contact_manager.h>
#include <tesseract_collision/core/discrete_contact_manager.h>
#include <tesseract_kinematics/core/joint_group.h>

using namespace tesseract_scene_graph;
using namespace tesseract_srdf;
using namespace tesseract_collision;
using namespace tesseract_environment;

SceneGraph::Ptr getSceneGraph()
{
  std::string url = "package://tesseract_support/urdf/lbr_iiwa_14_r820.urdf";

  tesseract_common::GeneralResourceLocator locator;
  return tesseract_urdf::parseURDFFile(locator.locateResource(url)->getFilePath(), locator);
}

SRDFModel::Ptr getSRDFModel(const SceneGraph& scene_graph)
{
  std::string url = "package://tesseract_support/urdf/lbr_iiwa_14_r820.srdf";
  tesseract_common::GeneralResourceLocator locator;

  auto srdf = std::make_shared<SRDFModel>();
  srdf->initFile(scene_graph, locator.locateResource(url)->getFilePath(), locator);

  return srdf;
}

static void BM_CHECK_TRAJECTORY_CONTINUOUS_SS(benchmark::State& state,
                                              std::vector<tesseract_collision::ContactResultMap> contacts,
                                              const tesseract_collision::ContinuousContactManager::Ptr& manager,
                                              const tesseract_scene_graph::StateSolver::Ptr& state_solver,
                                              const std::vector<std::string>& joint_names,
                                              const tesseract_common::TrajArray& traj,
                                              const tesseract_collision::CollisionCheckConfig& config,
                                              bool log_level_debug)
{
  if (log_level_debug)
    console_bridge::setLogLevel(console_bridge::LogLevel::CONSOLE_BRIDGE_LOG_DEBUG);
  else
    console_bridge::setLogLevel(console_bridge::LogLevel::CONSOLE_BRIDGE_LOG_INFO);

  for (auto _ : state)
  {
    benchmark::DoNotOptimize(checkTrajectory(contacts, *manager, *state_solver, joint_names, traj, config));
  }
}

static void BM_CHECK_TRAJECTORY_CONTINUOUS_MANIP(benchmark::State& state,
                                                 std::vector<tesseract_collision::ContactResultMap> contacts,
                                                 const tesseract_collision::ContinuousContactManager::Ptr& manager,
                                                 const tesseract_kinematics::JointGroup::ConstPtr& manip,
                                                 const tesseract_common::TrajArray& traj,
                                                 const tesseract_collision::CollisionCheckConfig& config,
                                                 bool log_level_debug)
{
  if (log_level_debug)
    console_bridge::setLogLevel(console_bridge::LogLevel::CONSOLE_BRIDGE_LOG_DEBUG);
  else
    console_bridge::setLogLevel(console_bridge::LogLevel::CONSOLE_BRIDGE_LOG_INFO);

  for (auto _ : state)
  {
    benchmark::DoNotOptimize(checkTrajectory(contacts, *manager, *manip, traj, config));
  }
}

static void BM_CHECK_TRAJECTORY_DISCRETE_SS(benchmark::State& state,
                                            std::vector<tesseract_collision::ContactResultMap> contacts,
                                            const tesseract_collision::DiscreteContactManager::Ptr& manager,
                                            const tesseract_scene_graph::StateSolver::Ptr& state_solver,
                                            const std::vector<std::string>& joint_names,
                                            const tesseract_common::TrajArray& traj,
                                            const tesseract_collision::CollisionCheckConfig& config,
                                            bool log_level_debug)
{
  if (log_level_debug)
    console_bridge::setLogLevel(console_bridge::LogLevel::CONSOLE_BRIDGE_LOG_DEBUG);
  else
    console_bridge::setLogLevel(console_bridge::LogLevel::CONSOLE_BRIDGE_LOG_INFO);

  for (auto _ : state)
  {
    benchmark::DoNotOptimize(checkTrajectory(contacts, *manager, *state_solver, joint_names, traj, config));
  }
}

static void BM_CHECK_TRAJECTORY_DISCRETE_MANIP(benchmark::State& state,
                                               std::vector<tesseract_collision::ContactResultMap> contacts,
                                               const tesseract_collision::DiscreteContactManager::Ptr& manager,
                                               const tesseract_kinematics::JointGroup::ConstPtr& manip,
                                               const tesseract_common::TrajArray& traj,
                                               const tesseract_collision::CollisionCheckConfig& config,
                                               bool log_level_debug)
{
  if (log_level_debug)
    console_bridge::setLogLevel(console_bridge::LogLevel::CONSOLE_BRIDGE_LOG_DEBUG);
  else
    console_bridge::setLogLevel(console_bridge::LogLevel::CONSOLE_BRIDGE_LOG_INFO);

  for (auto _ : state)
  {
    benchmark::DoNotOptimize(checkTrajectory(contacts, *manager, *manip, traj, config));
  }
}

int main(int argc, char** argv)
{
  auto env = std::make_shared<Environment>();
  tesseract_scene_graph::SceneGraph::Ptr scene_graph = getSceneGraph();
  auto srdf = getSRDFModel(*scene_graph);
  env->init(*scene_graph, srdf);
  env->setResourceLocator(std::make_shared<tesseract_common::GeneralResourceLocator>());

  // Add sphere to environment
  Link link_sphere("sphere_attached");

  Visual::Ptr visual = std::make_shared<Visual>();
  visual->origin = Eigen::Isometry3d::Identity();
  visual->origin.translation() = Eigen::Vector3d(0.5, 0, 0.55);
  visual->geometry = std::make_shared<tesseract_geometry::Sphere>(0.15);
  link_sphere.visual.push_back(visual);

  Collision::Ptr collision = std::make_shared<Collision>();
  collision->origin = visual->origin;
  collision->geometry = visual->geometry;
  link_sphere.collision.push_back(collision);

  Joint joint_sphere("joint_sphere_attached");
  joint_sphere.parent_link_name = "base_link";
  joint_sphere.child_link_name = link_sphere.getName();
  joint_sphere.type = JointType::FIXED;

  auto cmd = std::make_shared<tesseract_environment::AddLinkCommand>(link_sphere, joint_sphere);

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

  std::vector<tesseract_common::TrajArray> traj_arrays;

  // Only intermediat states are in collision
  tesseract_common::TrajArray traj(5, joint_start_pos.size());
  for (int i = 0; i < joint_start_pos.size(); ++i)
    traj.col(i) = Eigen::VectorXd::LinSpaced(5, joint_start_pos(i), joint_end_pos(i));

  // Only start state is not in collision
  tesseract_common::TrajArray traj2(3, joint_start_pos.size());
  for (int i = 0; i < joint_start_pos.size(); ++i)
    traj2.col(i) = Eigen::VectorXd::LinSpaced(3, joint_start_pos(i), joint_pos_collision(i));

  // Only start state is not in collision
  tesseract_common::TrajArray traj3(3, joint_start_pos.size());
  for (int i = 0; i < joint_start_pos.size(); ++i)
    traj3.col(i) = Eigen::VectorXd::LinSpaced(3, joint_pos_collision(i), joint_end_pos(i));

  // Only two states
  tesseract_common::TrajArray traj4(2, joint_start_pos.size());
  traj4.row(0) = joint_pos_collision;
  traj4.row(1) = joint_end_pos;

  tesseract_common::TrajArray traj5(2, joint_start_pos.size());
  traj5.row(0) = joint_start_pos;
  traj5.row(1) = joint_pos_collision;

  traj_arrays.push_back(traj);
  traj_arrays.push_back(traj2);
  traj_arrays.push_back(traj3);
  traj_arrays.push_back(traj4);
  traj_arrays.push_back(traj5);

  tesseract_collision::DiscreteContactManager::Ptr discrete_manager = env->getDiscreteContactManager();
  tesseract_collision::ContinuousContactManager::Ptr continuous_manager = env->getContinuousContactManager();
  tesseract_scene_graph::StateSolver::Ptr state_solver = env->getStateSolver();
  tesseract_kinematics::JointGroup::ConstPtr joint_group = env->getJointGroup("manipulator");
  std::vector<tesseract_collision::ContactResultMap> contacts;
  tesseract_collision::CollisionCheckConfig discrete_config;
  discrete_config.type = CollisionEvaluatorType::DISCRETE;
  discrete_config.check_program_mode = CollisionCheckProgramType::ALL;
  tesseract_collision::CollisionCheckConfig discrete_lvs_config;
  discrete_lvs_config.type = CollisionEvaluatorType::LVS_DISCRETE;
  discrete_lvs_config.check_program_mode = CollisionCheckProgramType::ALL;
  tesseract_collision::CollisionCheckConfig continuous_config;
  continuous_config.type = CollisionEvaluatorType::CONTINUOUS;
  continuous_config.check_program_mode = CollisionCheckProgramType::ALL;
  tesseract_collision::CollisionCheckConfig continuous_lvs_config;
  continuous_lvs_config.type = CollisionEvaluatorType::LVS_CONTINUOUS;
  continuous_lvs_config.check_program_mode = CollisionCheckProgramType::ALL;

  //////////////////////////////////////
  // Clone
  //////////////////////////////////////

  {
    std::function<void(benchmark::State&,
                       std::vector<tesseract_collision::ContactResultMap>,
                       tesseract_collision::ContinuousContactManager::Ptr,
                       tesseract_scene_graph::StateSolver::Ptr,
                       std::vector<std::string>,
                       tesseract_common::TrajArray,
                       tesseract_collision::CollisionCheckConfig,
                       bool)>
        BM_CHECK_TRAJ_CS = BM_CHECK_TRAJECTORY_CONTINUOUS_SS;
    std::function<void(benchmark::State&,
                       std::vector<tesseract_collision::ContactResultMap>,
                       tesseract_collision::ContinuousContactManager::Ptr,
                       tesseract_kinematics::JointGroup::ConstPtr,
                       tesseract_common::TrajArray,
                       tesseract_collision::CollisionCheckConfig,
                       bool)>
        BM_CHECK_TRAJ_CM = BM_CHECK_TRAJECTORY_CONTINUOUS_MANIP;
    std::function<void(benchmark::State&,
                       std::vector<tesseract_collision::ContactResultMap>,
                       tesseract_collision::DiscreteContactManager::Ptr,
                       tesseract_scene_graph::StateSolver::Ptr,
                       std::vector<std::string>,
                       tesseract_common::TrajArray,
                       tesseract_collision::CollisionCheckConfig,
                       bool)>
        BM_CHECK_TRAJ_DS = BM_CHECK_TRAJECTORY_DISCRETE_SS;
    std::function<void(benchmark::State&,
                       std::vector<tesseract_collision::ContactResultMap>,
                       tesseract_collision::DiscreteContactManager::Ptr,
                       tesseract_kinematics::JointGroup::ConstPtr,
                       tesseract_common::TrajArray,
                       tesseract_collision::CollisionCheckConfig,
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
                                     traj,
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
                                     traj,
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

  benchmark::Initialize(&argc, argv);
  benchmark::RunSpecifiedBenchmarks();
}
