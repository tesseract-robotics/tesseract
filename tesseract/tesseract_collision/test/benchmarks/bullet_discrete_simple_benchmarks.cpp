#include <benchmark/benchmark.h>
#include <Eigen/Eigen>

#include <tesseract_collision/test_suite/benchmarks/collision_benchmarks.hpp>
#include <tesseract_collision/test_suite/benchmarks/benchmark_utils.hpp>
#include <tesseract_collision/bullet/bullet_discrete_simple_manager.h>

using namespace tesseract_collision;
using namespace test_suite;
using namespace tesseract_geometry;

int main(int argc, char** argv)
{
  std::function<void(benchmark::State & state, DiscreteBenchmarkInfo info)> BM_CONTACT_TEST_FUNC = BM_CONTACT_TEST;

  // Make vector of all shapes to try
  std::vector<tesseract_geometry::GeometryType> geometry_types = {
    GeometryType::BOX, GeometryType::CONE, GeometryType::SPHERE, GeometryType::CAPSULE, GeometryType::CYLINDER
  };

  std::vector<ContactTestType> test_types = {
    ContactTestType::ALL, ContactTestType::FIRST, ContactTestType::CLOSEST, ContactTestType::LIMITED
  };

  // BulletDiscreteSimpleManager - In Collision
  {
    auto checker = std::make_shared<tesseract_collision_bullet::BulletDiscreteSimpleManager>();
    for (const auto& test_type : test_types)
    {
      // Loop over all primative combinations
      for (const auto& type1 : geometry_types)
      {
        for (const auto& type2 : geometry_types)
        {
          std::string name = "BM_CONTACT_TEST_0_" + checker->name() + "_" +
                             ContactTestTypeStrings[static_cast<std::size_t>(test_type)] + "_" +
                             GeometryTypeStrings[type1] + "_" + GeometryTypeStrings[type2];
          benchmark::RegisterBenchmark(name.c_str(),
                                       BM_CONTACT_TEST_FUNC,
                                       DiscreteBenchmarkInfo(checker,
                                                             CreateUnitPrimative(type1),
                                                             Eigen::Isometry3d::Identity(),
                                                             CreateUnitPrimative(type2),
                                                             Eigen::Isometry3d::Identity(),
                                                             test_type))
              ->UseRealTime()
              ->Unit(benchmark::TimeUnit::kMicrosecond);
        }
      }
    }
  }
  // BulletDiscreteSimpleManager - Not in collision. Within contact threshold
  {
    auto checker = std::make_shared<tesseract_collision_bullet::BulletDiscreteSimpleManager>();
    for (const auto& test_type : test_types)
    {
      // Loop over all primative combinations
      for (const auto& type1 : geometry_types)
      {
        for (const auto& type2 : geometry_types)
        {
          auto tf = Eigen::Isometry3d::Identity();
          std::string name = "BM_CONTACT_TEST_1_" + checker->name() + "_" +
                             ContactTestTypeStrings[static_cast<std::size_t>(test_type)] + "_" +
                             GeometryTypeStrings[type1] + "_" + GeometryTypeStrings[type2];
          benchmark::RegisterBenchmark(name.c_str(),
                                       BM_CONTACT_TEST_FUNC,
                                       DiscreteBenchmarkInfo(checker,
                                                             CreateUnitPrimative(type1),
                                                             Eigen::Isometry3d::Identity(),
                                                             CreateUnitPrimative(type2),
                                                             tf.translate(Eigen::Vector3d(0.6, 0, 0)),
                                                             test_type))
              ->UseRealTime()
              ->Unit(benchmark::TimeUnit::kMicrosecond);
        }
      }
    }
  }

  // BulletDiscreteSimpleManager - Not in collision. Outside contact threshold
  {
    auto checker = std::make_shared<tesseract_collision_bullet::BulletDiscreteSimpleManager>();
    for (const auto& test_type : test_types)
    {
      // Loop over all primative combinations
      for (const auto& type1 : geometry_types)
      {
        for (const auto& type2 : geometry_types)
        {
          auto tf = Eigen::Isometry3d::Identity();
          std::string name = "BM_CONTACT_TEST_2_" + checker->name() + "_" +
                             ContactTestTypeStrings[static_cast<std::size_t>(test_type)] + "_" +
                             GeometryTypeStrings[type1] + "_" + GeometryTypeStrings[type2];
          benchmark::RegisterBenchmark(name.c_str(),
                                       BM_CONTACT_TEST_FUNC,
                                       DiscreteBenchmarkInfo(checker,
                                                             CreateUnitPrimative(type1),
                                                             Eigen::Isometry3d::Identity(),
                                                             CreateUnitPrimative(type2),
                                                             tf.translate(Eigen::Vector3d(2, 0, 0)),
                                                             test_type))
              ->UseRealTime()
              ->Unit(benchmark::TimeUnit::kMicrosecond);
        }
      }
    }
  }

  benchmark::Initialize(&argc, argv);
  benchmark::RunSpecifiedBenchmarks();
}
