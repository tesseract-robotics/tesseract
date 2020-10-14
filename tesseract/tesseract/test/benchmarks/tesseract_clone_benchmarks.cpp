#include <tesseract_common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <benchmark/benchmark.h>
#include <algorithm>
#include <tesseract_urdf/urdf_parser.h>
TESSERACT_COMMON_IGNORE_WARNINGS_POP
#include <tesseract/tesseract.h>

using namespace tesseract_scene_graph;
using namespace tesseract_collision;
using namespace tesseract;

std::string locateResource(const std::string& url)
{
  std::string mod_url = url;
  if (url.find("package://tesseract_support") == 0)
  {
    mod_url.erase(0, strlen("package://tesseract_support"));
    size_t pos = mod_url.find('/');
    if (pos == std::string::npos)
    {
      return std::string();
    }

    std::string package = mod_url.substr(0, pos);
    mod_url.erase(0, pos);
    std::string package_path = std::string(TESSERACT_SUPPORT_DIR);

    if (package_path.empty())
    {
      return std::string();
    }

    mod_url = package_path + mod_url;
  }

  return mod_url;
}

SceneGraph::Ptr getSceneGraph()
{
  std::string path = std::string(TESSERACT_SUPPORT_DIR) + "/urdf/lbr_iiwa_14_r820.urdf";

  tesseract_scene_graph::ResourceLocator::Ptr locator =
      std::make_shared<tesseract_scene_graph::SimpleResourceLocator>(locateResource);
  return tesseract_urdf::parseURDFFile(path, locator);
}

/** @brief Benchmark that checks the Tesseract clone method*/
static void BM_TESSERACT_CLONE(benchmark::State& state, tesseract::Tesseract::Ptr tesseract)
{
  Tesseract::Ptr clone;
  for (auto _ : state)
  {
    benchmark::DoNotOptimize(clone = tesseract->clone());
  }
};

int main(int argc, char** argv)
{
  tesseract::Tesseract::Ptr tesseract = std::make_shared<tesseract::Tesseract>();
  tesseract->init(getSceneGraph());

  //////////////////////////////////////
  // Clone
  //////////////////////////////////////

  {
    std::function<void(benchmark::State&, tesseract::Tesseract::Ptr)> BM_CLONE_FUNC = BM_TESSERACT_CLONE;
    std::string name = "BM_TESSERACT_CLONE";
    benchmark::RegisterBenchmark(name.c_str(), BM_CLONE_FUNC, tesseract)
        ->UseRealTime()
        ->Unit(benchmark::TimeUnit::kMicrosecond);
  }

  benchmark::Initialize(&argc, argv);
  benchmark::RunSpecifiedBenchmarks();
}
