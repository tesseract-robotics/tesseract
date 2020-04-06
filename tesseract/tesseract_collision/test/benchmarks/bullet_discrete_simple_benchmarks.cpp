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
  const tesseract_collision_bullet::BulletDiscreteSimpleManager::ConstPtr checker =
      std::make_shared<tesseract_collision_bullet::BulletDiscreteSimpleManager>();

  //////////////////////////////////////
  // Clone
  //////////////////////////////////////

  {
    std::vector<int> num_links = { 0, 2, 4, 8, 16, 32, 64, 128, 256, 512 };
    std::function<void(benchmark::State&, DiscreteBenchmarkInfo, int)> BM_CLONE_FUNC = BM_CLONE;
    for (const auto& num_link : num_links)
    {
      std::string name = "BM_CLONE_" + checker->name() + "_ACTIVE_OBJ_" + std::to_string(num_link);
      benchmark::RegisterBenchmark(name.c_str(),
                                   BM_CLONE_FUNC,
                                   DiscreteBenchmarkInfo(checker,
                                                         CreateUnitPrimative(GeometryType::BOX),
                                                         Eigen::Isometry3d::Identity(),
                                                         CreateUnitPrimative(GeometryType::BOX),
                                                         Eigen::Isometry3d::Identity(),
                                                         ContactTestType::ALL),
                                   num_link)
          ->UseRealTime()
          ->Unit(benchmark::TimeUnit::kMicrosecond);
    }
  }

  //////////////////////////////////////
  // contactTest
  //////////////////////////////////////
  std::function<void(benchmark::State&, DiscreteBenchmarkInfo)> BM_CONTACT_TEST_FUNC = BM_CONTACT_TEST;

  // Make vector of all shapes to try
  std::vector<tesseract_geometry::GeometryType> geometry_types = {
    GeometryType::BOX, GeometryType::CONE, GeometryType::SPHERE, GeometryType::CAPSULE, GeometryType::CYLINDER
  };

  std::vector<ContactTestType> test_types = {
    ContactTestType::ALL, ContactTestType::FIRST, ContactTestType::CLOSEST, ContactTestType::LIMITED
  };

  // BulletDiscreteSimpleManager - In Collision
  {
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

  //////////////////////////////////////
  // setCollisionObjectTransform
  //////////////////////////////////////
  {
    std::function<void(benchmark::State&, int)> BM_SELECT_RANDOM_OBJECT_FUNC = BM_SELECT_RANDOM_OBJECT;
    benchmark::RegisterBenchmark("BM_SELECT_RANDOM_OBJECT", BM_SELECT_RANDOM_OBJECT_FUNC, 256)
        ->UseRealTime()
        ->Unit(benchmark::TimeUnit::kNanosecond);
  }
  {
    std::function<void(benchmark::State&, DiscreteBenchmarkInfo, int)> BM_SET_COLLISION_OBJECTS_TRANSFORM_SINGLE_FUNC =
        BM_SET_COLLISION_OBJECTS_TRANSFORM_SINGLE;
    std::vector<int> num_links = { 2, 4, 8, 16, 32, 64, 128, 256, 512 };

    for (const auto& num_link : num_links)
    {
      auto tf = Eigen::Isometry3d::Identity();
      std::string name =
          "BM_SET_COLLISION_OBJECTS_TRANSFORM_SINGLE_" + checker->name() + "_ACTIVE_OBJ_" + std::to_string(num_link);
      benchmark::RegisterBenchmark(name.c_str(),
                                   BM_SET_COLLISION_OBJECTS_TRANSFORM_SINGLE_FUNC,
                                   DiscreteBenchmarkInfo(checker,
                                                         CreateUnitPrimative(GeometryType::BOX),
                                                         Eigen::Isometry3d::Identity(),
                                                         CreateUnitPrimative(GeometryType::BOX),
                                                         tf.translate(Eigen::Vector3d(2, 0, 0)),
                                                         ContactTestType::ALL),
                                   num_link)
          ->UseRealTime()
          ->Unit(benchmark::TimeUnit::kNanosecond);
    }
  }
  {
    std::function<void(benchmark::State&, DiscreteBenchmarkInfo, int)> BM_SET_COLLISION_OBJECTS_TRANSFORM_VECTOR_FUNC =
        BM_SET_COLLISION_OBJECTS_TRANSFORM_VECTOR;
    std::vector<std::size_t> num_links = { 2, 4, 8, 16, 32, 64, 128, 256, 512 };

    for (const auto& num_link : num_links)
    {
      auto tf = Eigen::Isometry3d::Identity();
      std::string name =
          "BM_SET_COLLISION_OBJECTS_TRANSFORM_VECTOR_" + checker->name() + "_ACTIVE_OBJ_" + std::to_string(num_link);
      benchmark::RegisterBenchmark(name.c_str(),
                                   BM_SET_COLLISION_OBJECTS_TRANSFORM_VECTOR_FUNC,
                                   DiscreteBenchmarkInfo(checker,
                                                         CreateUnitPrimative(GeometryType::BOX),
                                                         Eigen::Isometry3d::Identity(),
                                                         CreateUnitPrimative(GeometryType::BOX),
                                                         tf.translate(Eigen::Vector3d(2, 0, 0)),
                                                         ContactTestType::ALL),
                                   num_link)
          ->UseRealTime()
          ->Unit(benchmark::TimeUnit::kNanosecond);
    }
  }
  {
    std::function<void(benchmark::State&, DiscreteBenchmarkInfo, int)> BM_SET_COLLISION_OBJECTS_TRANSFORM_MAP_FUNC =
        BM_SET_COLLISION_OBJECTS_TRANSFORM_MAP;
    std::vector<std::size_t> num_links = { 2, 4, 8, 16, 32, 64, 128, 256, 512 };

    for (const auto& num_link : num_links)
    {
      auto tf = Eigen::Isometry3d::Identity();
      std::string name =
          "BM_SET_COLLISION_OBJECTS_TRANSFORM_MAP_" + checker->name() + "_ACTIVE_OBJ_" + std::to_string(num_link);
      benchmark::RegisterBenchmark(name.c_str(),
                                   BM_SET_COLLISION_OBJECTS_TRANSFORM_MAP_FUNC,
                                   DiscreteBenchmarkInfo(checker,
                                                         CreateUnitPrimative(GeometryType::BOX),
                                                         Eigen::Isometry3d::Identity(),
                                                         CreateUnitPrimative(GeometryType::BOX),
                                                         tf.translate(Eigen::Vector3d(2, 0, 0)),
                                                         ContactTestType::ALL),
                                   num_link)
          ->UseRealTime()
          ->Unit(benchmark::TimeUnit::kNanosecond);
    }
  }

  benchmark::Initialize(&argc, argv);
  benchmark::RunSpecifiedBenchmarks();
}
