#include <tesseract_common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <benchmark/benchmark.h>
#include <algorithm>
TESSERACT_COMMON_IGNORE_WARNINGS_POP
#include <tesseract_scene_graph/graph.h>
#include <tesseract_environment/environment.h>
#include <tesseract_common/resource_locator.h>
#include <tesseract_urdf/urdf_parser.h>
#include <tesseract_support/tesseract_support_resource_locator.h>

using namespace tesseract_scene_graph;
using namespace tesseract_collision;
using namespace tesseract_environment;

SceneGraph::Ptr getSceneGraph()
{
  std::string path = std::string(TESSERACT_SUPPORT_DIR) + "/urdf/lbr_iiwa_14_r820.urdf";

  tesseract_common::TesseractSupportResourceLocator locator;
  return tesseract_urdf::parseURDFFile(path, locator);
}

/** @brief Benchmark that checks the Tesseract clone method*/
static void BM_ENVIRONMENT_CLONE(benchmark::State& state, Environment::Ptr env)
{
  Environment::Ptr clone;
  for (auto _ : state)
  {
    benchmark::DoNotOptimize(clone = env->clone());
  }
}

int main(int argc, char** argv)
{
  Environment::Ptr env = std::make_shared<Environment>();
  env->init(*getSceneGraph());

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

  benchmark::Initialize(&argc, argv);
  benchmark::RunSpecifiedBenchmarks();
}
