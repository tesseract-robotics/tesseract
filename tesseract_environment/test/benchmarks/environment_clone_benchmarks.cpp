#include <tesseract_common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <benchmark/benchmark.h>
TESSERACT_COMMON_IGNORE_WARNINGS_POP
#include <tesseract_scene_graph/graph.h>
#include <tesseract_srdf/srdf_model.h>
#include <tesseract_state_solver/state_solver.h>
#include <tesseract_environment/environment.h>
#include <tesseract_kinematics/core/joint_group.h>
#include <tesseract_kinematics/core/kinematic_group.h>
#include <tesseract_common/resource_locator.h>
#include <tesseract_urdf/urdf_parser.h>

using namespace tesseract_scene_graph;
using namespace tesseract_collision;
using namespace tesseract_environment;
using namespace tesseract_kinematics;

SceneGraph::Ptr getSceneGraph(const tesseract_common::ResourceLocator& locator)
{
  std::string path = "package://tesseract_support/urdf/lbr_iiwa_14_r820.urdf";

  return tesseract_urdf::parseURDFFile(locator.locateResource(path)->getFilePath(), locator);
}

tesseract_srdf::SRDFModel::Ptr getSRDFModel(const SceneGraph& scene_graph,
                                            const tesseract_common::ResourceLocator& locator)
{
  std::string path = "package://tesseract_support/urdf/lbr_iiwa_14_r820.srdf";

  auto srdf = std::make_shared<tesseract_srdf::SRDFModel>();
  srdf->initFile(scene_graph, locator.locateResource(path)->getFilePath(), locator);

  return srdf;
}

/** @brief Benchmark that checks the Tesseract clone method*/
static void BM_ENVIRONMENT_CLONE(benchmark::State& state, const Environment::Ptr& env)
{
  Environment::Ptr clone;
  for (auto _ : state)  // NOLINT
  {
    benchmark::DoNotOptimize(clone = env->clone());
  }
}

/** @brief Benchmark that checks the Tesseract clone method*/
static void BM_STATE_SOLVER_CLONE(benchmark::State& state, const StateSolver::Ptr& state_solver)
{
  StateSolver::Ptr clone;
  for (auto _ : state)  // NOLINT
  {
    benchmark::DoNotOptimize(clone = state_solver->clone());
  }
}

/** @brief Benchmark that checks the Tesseract clone method*/
static void BM_SCENE_GRAPH_CLONE(benchmark::State& state, const SceneGraph::Ptr& sg)
{
  SceneGraph::Ptr clone;
  for (auto _ : state)  // NOLINT
  {
    benchmark::DoNotOptimize(clone = sg->clone());
  }
}

/** @brief Benchmark that checks the Tesseract clone method*/
static void BM_JOINT_GROUP_COPY(benchmark::State& state, const JointGroup::ConstPtr& jg)
{
  JointGroup::Ptr clone;
  for (auto _ : state)  // NOLINT
  {
    benchmark::DoNotOptimize(clone = std::make_unique<JointGroup>(*jg));
  }
}

static void BM_KINEMATIC_GROUP_COPY(benchmark::State& state, const KinematicGroup::ConstPtr& kg)
{
  KinematicGroup::Ptr clone;
  for (auto _ : state)  // NOLINT
  {
    benchmark::DoNotOptimize(clone = std::make_unique<KinematicGroup>(*kg));
  }
}

int main(int argc, char** argv)
{
  tesseract_common::GeneralResourceLocator locator;
  SceneGraph::Ptr scene_graph = getSceneGraph(locator);
  auto srdf = getSRDFModel(*scene_graph, locator);
  Environment::Ptr env = std::make_shared<Environment>();
  env->init(*scene_graph, srdf);
  StateSolver::Ptr state_solver = env->getStateSolver();
  JointGroup::ConstPtr joint_group = env->getJointGroup("manipulator");
  KinematicGroup::ConstPtr kinematic_group = env->getKinematicGroup("manipulator");
  //////////////////////////////////////
  // Clone
  //////////////////////////////////////

  {
    std::function<void(benchmark::State&, Environment::Ptr)> BM_CLONE_FUNC = BM_ENVIRONMENT_CLONE;
    std::string name = "BM_ENVIRONMENT_CLONE";
    benchmark::RegisterBenchmark(name.c_str(), BM_CLONE_FUNC, env)
        ->UseRealTime()
        ->Unit(benchmark::TimeUnit::kMicrosecond);
  }

  {
    std::function<void(benchmark::State&, StateSolver::Ptr)> BM_CLONE_FUNC = BM_STATE_SOLVER_CLONE;
    std::string name = "BM_STATE_SOLVER_CLONE";
    benchmark::RegisterBenchmark(name.c_str(), BM_CLONE_FUNC, state_solver)
        ->UseRealTime()
        ->Unit(benchmark::TimeUnit::kMicrosecond);
  }

  {
    std::function<void(benchmark::State&, SceneGraph::Ptr)> BM_CLONE_FUNC = BM_SCENE_GRAPH_CLONE;
    std::string name = "BM_SCENE_GRAPH_CLONE";
    benchmark::RegisterBenchmark(name.c_str(), BM_CLONE_FUNC, scene_graph)
        ->UseRealTime()
        ->Unit(benchmark::TimeUnit::kMicrosecond);
  }

  {
    std::function<void(benchmark::State&, JointGroup::ConstPtr)> BM_CLONE_FUNC = BM_JOINT_GROUP_COPY;
    std::string name = "BM_JOINT_GROUP_COPY";
    benchmark::RegisterBenchmark(name.c_str(), BM_CLONE_FUNC, joint_group)
        ->UseRealTime()
        ->Unit(benchmark::TimeUnit::kMicrosecond);
  }

  {
    std::function<void(benchmark::State&, KinematicGroup::ConstPtr)> BM_CLONE_FUNC = BM_KINEMATIC_GROUP_COPY;
    std::string name = "BM_KINEMATIC_GROUP_COPY";
    benchmark::RegisterBenchmark(name.c_str(), BM_CLONE_FUNC, kinematic_group)
        ->UseRealTime()
        ->Unit(benchmark::TimeUnit::kMicrosecond);
  }

  benchmark::Initialize(&argc, argv);
  benchmark::RunSpecifiedBenchmarks();
}
