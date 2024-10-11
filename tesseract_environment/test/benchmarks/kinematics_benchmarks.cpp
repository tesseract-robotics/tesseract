#include <tesseract_common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <benchmark/benchmark.h>
#include <algorithm>
TESSERACT_COMMON_IGNORE_WARNINGS_POP
#include <tesseract_urdf/urdf_parser.h>
#include <tesseract_srdf/srdf_model.h>
#include <tesseract_common/resource_locator.h>
#include <tesseract_kinematics/core/joint_group.h>
#include <tesseract_scene_graph/graph.h>
#include <tesseract_scene_graph/scene_state.h>
#include <tesseract_state_solver/state_solver.h>
#include <tesseract_environment/environment.h>
#include <tesseract_environment/utils.h>
#include <tesseract_geometry/impl/sphere.h>

using namespace tesseract_scene_graph;
using namespace tesseract_srdf;
using namespace tesseract_environment;

SceneGraph::Ptr getSceneGraph(const tesseract_common::ResourceLocator& locator)
{
  std::string path = "package://tesseract_support/urdf/lbr_iiwa_14_r820.urdf";
  return tesseract_urdf::parseURDFFile(locator.locateResource(path)->getFilePath(), locator);
}

SRDFModel::Ptr getSRDFModel(const SceneGraph& scene_graph, const tesseract_common::ResourceLocator& locator)
{
  std::string path = "package://tesseract_support/urdf/lbr_iiwa_14_r820.srdf";

  auto srdf = std::make_shared<SRDFModel>();
  srdf->initFile(scene_graph, locator.locateResource(path)->getFilePath(), locator);

  return srdf;
}

using CalcStateFn = std::function<tesseract_common::TransformMap(const Eigen::Ref<const Eigen::VectorXd>& state)>;

static void BM_GET_STATE_JOINT_NAMES_JOINT_VALUES_SS(benchmark::State& state,
                                                     CalcStateFn fn,
                                                     const tesseract_common::TrajArray& traj)
{
  tesseract_common::TransformMap transform_map;
  for (auto _ : state)
  {
    for (Eigen::Index i = 0; i < traj.rows(); i++)
    {
      benchmark::DoNotOptimize(transform_map = fn(traj.row(i)));
    }
  }
}

static void BM_SET_AND_GET_STATE_JOINT_NAMES_JOINT_VALUES_SS(benchmark::State& state,
                                                             CalcStateFn fn,
                                                             const tesseract_common::TrajArray& traj)
{
  tesseract_common::TransformMap transform_map;
  for (auto _ : state)
  {
    for (Eigen::Index i = 0; i < traj.rows(); i++)
    {
      benchmark::DoNotOptimize(transform_map = fn(traj.row(i)));
    }
  }
}

static void BM_GET_JACOBIAN_JOINT_NAMES_JOINT_VALUES_SS(benchmark::State& state,
                                                        tesseract_scene_graph::StateSolver::Ptr state_solver,
                                                        std::vector<std::string> joint_names,
                                                        const tesseract_common::TrajArray& traj,
                                                        std::string link_name)
{
  Eigen::MatrixXd jacobian;
  for (auto _ : state)
  {
    for (Eigen::Index i = 0; i < traj.rows(); i++)
    {
      benchmark::DoNotOptimize(jacobian = state_solver->getJacobian(joint_names, traj.row(i), link_name));
    }
  }
}

static void BM_CALC_FWD_KIN_MANIP(benchmark::State& state, CalcStateFn fn, const tesseract_common::TrajArray& traj)
{
  tesseract_common::TransformMap transforms;
  for (auto _ : state)
  {
    for (Eigen::Index i = 0; i < traj.rows(); i++)
    {
      benchmark::DoNotOptimize(transforms = fn(traj.row(i)));
    }
  }
}

static void BM_GET_JACOBIAN_MANIP(benchmark::State& state,
                                  tesseract_kinematics::JointGroup::Ptr manip,
                                  const tesseract_common::TrajArray& traj,
                                  std::string link_name)
{
  Eigen::MatrixXd jacobian;
  for (auto _ : state)
  {
    for (Eigen::Index i = 0; i < traj.rows(); i++)
    {
      benchmark::DoNotOptimize(jacobian = manip->calcJacobian(traj.row(i), link_name));
    }
  }
}

int main(int argc, char** argv)
{
  tesseract_common::GeneralResourceLocator locator;
  auto env = std::make_shared<Environment>();
  tesseract_scene_graph::SceneGraph::Ptr scene_graph = getSceneGraph(locator);
  auto srdf = getSRDFModel(*scene_graph, locator);
  env->init(*scene_graph, srdf);
  env->setResourceLocator(std::make_shared<tesseract_common::GeneralResourceLocator>());

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

  tesseract_common::TrajArray traj(5, joint_start_pos.size());
  for (int i = 0; i < joint_start_pos.size(); ++i)
    traj.col(i) = Eigen::VectorXd::LinSpaced(5, joint_start_pos(i), joint_end_pos(i));

  tesseract_scene_graph::StateSolver::Ptr state_solver = env->getStateSolver();
  tesseract_kinematics::JointGroup::Ptr joint_group = env->getJointGroup("manipulator");
  std::string tip_link{ "tool0" };

  //////////////////////////////////////
  // Benchmarks
  //////////////////////////////////////

  {
    tesseract_scene_graph::StateSolver::Ptr local_ss = state_solver->clone();
    CalcStateFn fn = [local_ss,
                      joint_names](const Eigen::Ref<const Eigen::VectorXd>& state) -> tesseract_common::TransformMap {
      return local_ss->getState(joint_names, state).link_transforms;
    };

    std::function<void(benchmark::State&, CalcStateFn, const tesseract_common::TrajArray&)> BM_GET_STATE_JN_JV_SS =
        BM_GET_STATE_JOINT_NAMES_JOINT_VALUES_SS;
    std::string name = "BM_GET_STATE_JOINT_NAMES_JOINT_VALUES_SS";
    benchmark::RegisterBenchmark(name.c_str(), BM_GET_STATE_JN_JV_SS, fn, traj)
        ->UseRealTime()
        ->Unit(benchmark::TimeUnit::kMicrosecond);
  }
  {
    tesseract_scene_graph::StateSolver::Ptr local_ss = state_solver->clone();
    CalcStateFn fn = [local_ss,
                      joint_names](const Eigen::Ref<const Eigen::VectorXd>& state) -> tesseract_common::TransformMap {
      local_ss->setState(joint_names, state);
      return local_ss->getState().link_transforms;
    };

    std::function<void(benchmark::State&, CalcStateFn, const tesseract_common::TrajArray&)>
        BM_SET_AND_GET_STATE_JN_JV_SS = BM_SET_AND_GET_STATE_JOINT_NAMES_JOINT_VALUES_SS;
    std::string name = "BM_SET_AND_GET_STATE_JOINT_NAMES_JOINT_VALUES_SS";
    benchmark::RegisterBenchmark(name.c_str(), BM_SET_AND_GET_STATE_JN_JV_SS, fn, traj)
        ->UseRealTime()
        ->Unit(benchmark::TimeUnit::kMicrosecond);
  }
  {
    std::function<void(benchmark::State&,
                       tesseract_scene_graph::StateSolver::Ptr,
                       std::vector<std::string>,
                       const tesseract_common::TrajArray&,
                       std::string)>
        BM_GET_JACOBIAN_JN_JV_SS = BM_GET_JACOBIAN_JOINT_NAMES_JOINT_VALUES_SS;
    std::string name = "BM_GET_JACOBIAN_JOINT_NAMES_JOINT_VALUES_SS";
    benchmark::RegisterBenchmark(name.c_str(), BM_GET_JACOBIAN_JN_JV_SS, state_solver, joint_names, traj, tip_link)
        ->UseRealTime()
        ->Unit(benchmark::TimeUnit::kMicrosecond);
  }
  {
    CalcStateFn fn = [joint_group](const Eigen::Ref<const Eigen::VectorXd>& state) -> tesseract_common::TransformMap {
      return joint_group->calcFwdKin(state);
    };

    std::function<void(benchmark::State&, CalcStateFn, const tesseract_common::TrajArray&)> BM_CFK_MANIP =
        BM_CALC_FWD_KIN_MANIP;
    std::string name = "BM_CALC_FWD_KIN_MANIP";
    benchmark::RegisterBenchmark(name.c_str(), BM_CFK_MANIP, fn, traj)
        ->UseRealTime()
        ->Unit(benchmark::TimeUnit::kMicrosecond);
  }
  {
    std::function<void(
        benchmark::State&, tesseract_kinematics::JointGroup::Ptr, const tesseract_common::TrajArray&, std::string)>
        BM_CJ_MANIP = BM_GET_JACOBIAN_MANIP;
    std::string name = "BM_GET_JACOBIAN_MANIP";
    benchmark::RegisterBenchmark(name.c_str(), BM_CJ_MANIP, joint_group, traj, tip_link)
        ->UseRealTime()
        ->Unit(benchmark::TimeUnit::kMicrosecond);
  }

  benchmark::Initialize(&argc, argv);
  benchmark::RunSpecifiedBenchmarks();
}
