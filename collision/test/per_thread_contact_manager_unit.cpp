#include <tesseract/common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <gtest/gtest.h>
#include <atomic>
#include <memory>
#include <thread>
#include <unordered_set>
#include <vector>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract/collision/per_thread_contact_manager.h>
#include <tesseract/collision/bullet/bullet_discrete_bvh_manager.h>
#include <tesseract/collision/bullet/bullet_cast_bvh_manager.h>
#include <tesseract/geometry/impl/sphere.h>

using namespace tesseract::collision;
using tesseract::common::LinkId;

namespace
{
/** @brief Threads alive at once in the multi-threaded tests */
constexpr std::size_t NUM_THREADS{ 4 };

/** @brief Radius of both spheres (m) */
constexpr double SPHERE_RADIUS{ 0.25 };

/** @brief Default collision margin of the prototype (m); larger than every separation the tests use */
constexpr double MARGIN{ 1.0 };

/** @brief Contact tests each thread runs in the concurrent test */
constexpr int NUM_CONTACT_TESTS{ 200 };

/** @brief Distance tolerance (m) for Bullet's sphere-sphere distance */
constexpr double DISTANCE_TOL{ 1e-6 };

const LinkId MOVING("moving_link");
const LinkId STATIC("static_link");

/** @brief A manager with two spheres at the origin, only MOVING active, and the default margin set to MARGIN */
template <typename ManagerT>
std::unique_ptr<ManagerT> makePrototype()
{
  auto manager = std::make_unique<ManagerT>();
  const CollisionShapesConst shapes{ std::make_shared<tesseract::geometry::Sphere>(SPHERE_RADIUS) };
  const tesseract::common::VectorIsometry3d poses{ Eigen::Isometry3d::Identity() };
  manager->addCollisionObject(MOVING, 0, shapes, poses);
  manager->addCollisionObject(STATIC, 0, shapes, poses);
  manager->setActiveCollisionObjects(std::vector<LinkId>{ MOVING });
  manager->setDefaultCollisionMargin(MARGIN);
  return manager;
}

/** @brief Run fn(thread_index) on NUM_THREADS threads that are all alive until every one has finished fn */
template <typename Fn>
void runOnConcurrentThreads(Fn fn)
{
  std::atomic<std::size_t> finished{ 0 };
  std::vector<std::thread> threads;
  threads.reserve(NUM_THREADS);
  for (std::size_t i = 0; i < NUM_THREADS; ++i)
  {
    threads.emplace_back([&fn, &finished, i] {
      fn(i);
      ++finished;
      // Stay alive so no later thread can be handed this thread's id
      while (finished.load() < NUM_THREADS)
        std::this_thread::yield();
    });
  }
  for (auto& thread : threads)
    thread.join();
}

template <typename ManagerT, typename ConcreteT>
void runSameThreadSameManager()
{
  auto prototype = makePrototype<ConcreteT>();
  const ManagerT* prototype_ptr = prototype.get();
  const PerThreadContactManager<ManagerT> managers(std::move(prototype));
  EXPECT_EQ(managers.size(), 0);

  ManagerT& first = managers.get();
  ManagerT& second = managers.get();
  EXPECT_EQ(&first, &second);
  EXPECT_NE(&first, prototype_ptr);
  EXPECT_EQ(managers.size(), 1);
}

template <typename ManagerT, typename ConcreteT>
void runOwnManagerPerThread()
{
  const PerThreadContactManager<ManagerT> managers(makePrototype<ConcreteT>());

  std::vector<const ManagerT*> first(NUM_THREADS, nullptr);
  std::vector<const ManagerT*> second(NUM_THREADS, nullptr);
  runOnConcurrentThreads([&](std::size_t i) {
    first[i] = &managers.get();
    second[i] = &managers.get();
  });

  EXPECT_EQ(first, second);
  EXPECT_EQ(std::unordered_set<const ManagerT*>(first.begin(), first.end()).size(), NUM_THREADS);
  EXPECT_EQ(managers.size(), NUM_THREADS);
}

template <typename ManagerT, typename ConcreteT>
void runClonesKeepPrototypeConfiguration()
{
  const PerThreadContactManager<ManagerT> managers(makePrototype<ConcreteT>());

  // From another thread, so the manager is cloned there
  std::thread([&managers] {
    const ManagerT& manager = managers.get();
    EXPECT_EQ(manager.getCollisionObjects(), (std::vector<LinkId>{ MOVING, STATIC }));
    EXPECT_EQ(manager.getActiveCollisionObjects(), (std::unordered_set<LinkId>{ MOVING }));
    EXPECT_DOUBLE_EQ(manager.getCollisionMarginData().getDefaultCollisionMargin(), MARGIN);
  }).join();
}
}  // namespace

TEST(TesseractCollisionUnit, PerThreadContactManagerNullPrototypeThrowsUnit)  // NOLINT
{
  EXPECT_THROW(PerThreadDiscreteContactManager(nullptr), std::invalid_argument);    // NOLINT
  EXPECT_THROW(PerThreadContinuousContactManager(nullptr), std::invalid_argument);  // NOLINT
}

TEST(TesseractCollisionUnit, PerThreadDiscreteContactManagerSameThreadUnit)  // NOLINT
{
  runSameThreadSameManager<DiscreteContactManager, BulletDiscreteBVHManager>();
}

TEST(TesseractCollisionUnit, PerThreadContinuousContactManagerSameThreadUnit)  // NOLINT
{
  runSameThreadSameManager<ContinuousContactManager, BulletCastBVHManager>();
}

TEST(TesseractCollisionUnit, PerThreadDiscreteContactManagerOwnManagerPerThreadUnit)  // NOLINT
{
  runOwnManagerPerThread<DiscreteContactManager, BulletDiscreteBVHManager>();
}

TEST(TesseractCollisionUnit, PerThreadContinuousContactManagerOwnManagerPerThreadUnit)  // NOLINT
{
  runOwnManagerPerThread<ContinuousContactManager, BulletCastBVHManager>();
}

TEST(TesseractCollisionUnit, PerThreadDiscreteContactManagerClonesKeepConfigurationUnit)  // NOLINT
{
  runClonesKeepPrototypeConfiguration<DiscreteContactManager, BulletDiscreteBVHManager>();
}

TEST(TesseractCollisionUnit, PerThreadContinuousContactManagerClonesKeepConfigurationUnit)  // NOLINT
{
  runClonesKeepPrototypeConfiguration<ContinuousContactManager, BulletCastBVHManager>();
}

/**
 * Every thread moves the same link to its own position and checks the distance it gets back, many times over. With a
 * manager shared between threads, one thread's transform update would land between another's update and its contact
 * test.
 */
TEST(TesseractCollisionUnit, PerThreadDiscreteContactManagerConcurrentContactTestsUnit)  // NOLINT
{
  const PerThreadDiscreteContactManager managers(makePrototype<BulletDiscreteBVHManager>());

  std::vector<int> wrong_results(NUM_THREADS, 0);
  runOnConcurrentThreads([&](std::size_t i) {
    // Thread i keeps the spheres 0.1 * (i + 1) m apart, surface to surface
    const double separation = 0.1 * static_cast<double>(i + 1);
    Eigen::Isometry3d pose = Eigen::Isometry3d::Identity();
    pose.translation().x() = (2 * SPHERE_RADIUS) + separation;

    DiscreteContactManager& manager = managers.get();
    for (int k = 0; k < NUM_CONTACT_TESTS; ++k)
    {
      manager.setCollisionObjectsTransform(MOVING, pose);
      ContactResultMap results;
      manager.contactTest(results, ContactRequest(ContactTestType::CLOSEST));

      ContactResultVector flat;
      results.flattenMoveResults(flat);
      if (flat.size() != 1 || std::abs(flat.front().distance - separation) > DISTANCE_TOL)
        ++wrong_results[i];
    }
  });

  EXPECT_EQ(wrong_results, std::vector<int>(NUM_THREADS, 0));
  EXPECT_EQ(managers.size(), NUM_THREADS);
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);

  return RUN_ALL_TESTS();
}
