#ifndef TESSERACT_COLLISION_PRIMATIVES_BENCHMARKS_HPP
#define TESSERACT_COLLISION_PRIMATIVES_BENCHMARKS_HPP

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
static void BM_CLONE(benchmark::State& state, DiscreteBenchmarkInfo info, std::size_t num_obj)  // NOLINT
{
  std::vector<std::string> active_obj(num_obj);
  for (std::size_t ind = 0; ind < num_obj; ind++)
  {
    std::string name = "geom_" + std::to_string(ind);
    active_obj.push_back(name);
    info.contact_manager_->addCollisionObject(name, 0, info.geom1_, info.obj1_poses);
  }
  info.contact_manager_->setActiveCollisionObjects(active_obj);
  info.contact_manager_->setCollisionMarginData(CollisionMarginData(0.5));

  DiscreteContactManager::Ptr clone;
  for (auto _ : state)  // NOLINT
  {
    benchmark::DoNotOptimize(clone = info.contact_manager_->clone());
  }
};

/** @brief Benchmark that checks the contactTest function in discrete contact managers*/
static void BM_CONTACT_TEST(benchmark::State& state, DiscreteBenchmarkInfo info)  // NOLINT
{
  info.contact_manager_->addCollisionObject(std::string("geom1"), 0, info.geom1_, info.obj1_poses);
  info.contact_manager_->addCollisionObject(std::string("geom2"), 0, info.geom2_, info.obj2_poses);

  info.contact_manager_->setActiveCollisionObjects({ "geom1", "geom2" });
  info.contact_manager_->setCollisionMarginData(CollisionMarginData(0.5));

  ContactResultMap result;
  for (auto _ : state)  // NOLINT
  {
    result.clear();
    info.contact_manager_->contactTest(result, ContactRequest(info.contact_test_type_));
  }
};

/** @brief Benchmark that checks how long it takes to select a random object so that number can be subtracted from other
 * benchmarks if that is important*/
static void BM_SELECT_RANDOM_OBJECT(benchmark::State& state, int num_obj)
{
  int selected_link{ 0 };
  for (auto _ : state)  // NOLINT
  {
    benchmark::DoNotOptimize(selected_link = rand() % num_obj);  // NOLINT
  }
};

/** @brief Benchmark that checks the setCollisionObjectsTransform(const std::string& name, const Eigen::Isometry3d&
 * pose) method in discrete contact managers*/
static void BM_SET_COLLISION_OBJECTS_TRANSFORM_SINGLE(benchmark::State& state,
                                                      DiscreteBenchmarkInfo info,
                                                      std::size_t num_obj)
{
  // Setting up collision objects
  std::vector<std::string> active_obj(num_obj);
  for (std::size_t ind = 0; ind < num_obj; ind++)
  {
    std::string name = "geom_" + std::to_string(ind);
    active_obj[ind] = name;
    info.contact_manager_->addCollisionObject(name, 0, info.geom1_, info.obj1_poses);
  }
  info.contact_manager_->setActiveCollisionObjects(active_obj);
  info.contact_manager_->setCollisionMarginData(CollisionMarginData(0.5));

  for (auto _ : state)  // NOLINT
  {
    // Including this seems necessary to insure that a distribution of links is used rather than always searching for
    // the same one. Subtract off approximately BM_SELECT_RANDOM_OBJECT if you need absolute numbers rather than
    // relative.
    std::size_t selected_link = static_cast<std::size_t>(rand()) % num_obj;
    info.contact_manager_->setCollisionObjectsTransform(active_obj[selected_link], info.obj2_poses[0]);
  }
};

/** @brief Benchmark that checks the setCollisionObjectsTransform(const std::vector<std::string>& names, const
   tesseract_common::VectorIsometry3d& poses) method in discrete contact managers. Moves only a single random link*/
static void BM_SET_COLLISION_OBJECTS_TRANSFORM_VECTOR(benchmark::State& state,
                                                      DiscreteBenchmarkInfo info,  // NOLINT
                                                      std::size_t num_obj)
{
  // Setting up collision objects
  std::vector<std::string> active_obj(num_obj);
  for (std::size_t ind = 0; ind < num_obj; ind++)
  {
    std::string name = "geom_" + std::to_string(ind);
    active_obj[ind] = name;
    info.contact_manager_->addCollisionObject(name, 0, info.geom1_, info.obj1_poses);
  }
  info.contact_manager_->setActiveCollisionObjects(active_obj);
  info.contact_manager_->setCollisionMarginData(CollisionMarginData(0.5));

  std::vector<std::string> selected_links(1);
  for (auto _ : state)  // NOLINT
  {
    // Including this seems necessary to insure that a distribution of links is used rather than always searching for
    // the same one. Subtract off approximately BM_SELECT_RANDOM_OBJECT if you need absolute numbers rather than
    // relative.
    selected_links[0] = active_obj[static_cast<std::size_t>(rand()) % num_obj];
    info.contact_manager_->setCollisionObjectsTransform(selected_links, info.obj2_poses);
  }
};

/** @brief Benchmark that checks the setCollisionObjectsTransform(const tesseract_common::TransformMap& transforms)
 * method in discrete contact managers. Moves only a single random link*/
static void BM_SET_COLLISION_OBJECTS_TRANSFORM_MAP(benchmark::State& state,
                                                   DiscreteBenchmarkInfo info,
                                                   std::size_t num_obj)
{
  // Setting up collision objects
  std::vector<std::string> active_obj(num_obj);
  for (std::size_t ind = 0; ind < num_obj; ind++)
  {
    std::string name = "geom_" + std::to_string(ind);
    active_obj[ind] = name;
    info.contact_manager_->addCollisionObject(name, 0, info.geom1_, info.obj1_poses);
  }
  info.contact_manager_->setActiveCollisionObjects(active_obj);
  info.contact_manager_->setCollisionMarginData(CollisionMarginData(0.5));

  tesseract_common::TransformMap selected_link;
  for (auto _ : state)  // NOLINT
  {
    // Including this seems necessary to insure that a distribution of links is used rather than always searching for
    // the same one. It might be worth it to manually time these as well if it's really important
    selected_link[active_obj[static_cast<std::size_t>(rand()) % num_obj]] = info.obj2_poses[0];
    info.contact_manager_->setCollisionObjectsTransform(selected_link);
  }
};

}  // namespace test_suite
}  // namespace tesseract_collision

#endif
