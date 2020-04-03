#ifndef TESSERACT_COLLISION_BENCHMARKS_HPP
#define TESSERACT_COLLISION_BENCHMARKS_HPP

#include <tesseract_collision/core/discrete_contact_manager.h>
#include <tesseract_geometry/geometries.h>

#include <Eigen/Eigen>

namespace tesseract_collision
{
namespace test_suite
{
/**
 * @brief Contains the information necessary to run the benchmarks for discrete collision checking
 */
struct DiscreteBenchmarkInfo
{
  DiscreteBenchmarkInfo(const DiscreteContactManager::ConstPtr& contact_manager,
                        const tesseract_geometry::Geometry::ConstPtr& geom1,
                        const Eigen::Isometry3d& pose1,
                        const tesseract_geometry::Geometry::ConstPtr& geom2,
                        const Eigen::Isometry3d& pose2,
                        ContactTestType contact_test_type)

  {
    contact_manager_ = contact_manager->clone();
    geom1_.push_back(geom1->clone());
    geom2_.push_back(geom2->clone());
    obj1_poses.push_back(pose1);
    obj2_poses.push_back(pose2);
    contact_test_type_ = contact_test_type;
  }
  DiscreteContactManager::Ptr contact_manager_;
  CollisionShapesConst geom1_;
  tesseract_common::VectorIsometry3d obj1_poses;
  CollisionShapesConst geom2_;
  tesseract_common::VectorIsometry3d obj2_poses;
  ContactTestType contact_test_type_;
};

/** @brief Benchmark that checks the clone method in discrete contact managers*/
static void BM_CLONE(benchmark::State& state, DiscreteBenchmarkInfo info, int num_obj)
{
  std::vector<std::string> active_obj(num_obj);
  for (int ind = 0; ind < num_obj; ind++)
  {
    std::string name = "geom_" + std::to_string(ind);
    active_obj.push_back(name);
    info.contact_manager_->addCollisionObject(name, 0, info.geom1_, info.obj1_poses);
  }
  info.contact_manager_->setActiveCollisionObjects(active_obj);
  info.contact_manager_->setContactDistanceThreshold(0.5);

  DiscreteContactManager::Ptr clone;
  for (auto _ : state)
  {
    benchmark::DoNotOptimize(clone = info.contact_manager_->clone());
  }
};

/** @brief Benchmark that checks the contactTest function in discrete contact managers*/
static void BM_CONTACT_TEST(benchmark::State& state, DiscreteBenchmarkInfo info)
{
  info.contact_manager_->addCollisionObject(std::string("geom1"), 0, info.geom1_, info.obj1_poses);
  info.contact_manager_->addCollisionObject(std::string("geom2"), 0, info.geom2_, info.obj2_poses);

  info.contact_manager_->setActiveCollisionObjects({ "geom1", "geom2" });
  info.contact_manager_->setContactDistanceThreshold(0.5);

  ContactResultMap result;
  for (auto _ : state)
  {
    info.contact_manager_->contactTest(result, info.contact_test_type_);
  }
};

}  // namespace test_suite
}  // namespace tesseract_collision

#endif
