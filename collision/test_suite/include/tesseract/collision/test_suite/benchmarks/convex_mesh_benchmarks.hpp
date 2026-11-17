#ifndef TESSERACT_COLLISION_CONVEX_MESH_BENCHMARKS_HPP
#define TESSERACT_COLLISION_CONVEX_MESH_BENCHMARKS_HPP

#include <tesseract/common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <benchmark/benchmark.h>
#include <chrono>
#include <cmath>
#include <memory>
#include <random>
#include <string>
#include <vector>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract/collision/discrete_contact_manager.h>
#include <tesseract/geometry/impl/convex_mesh.h>

#include <Eigen/Eigen>

namespace tesseract::collision::test_suite
{
/**
 * @brief Closed latitude-longitude sphere as a ConvexMesh: rings * segments vertices plus two poles.
 * @details Every vertex lies on the sphere, so every vertex is a hull vertex, and the faces are watertight
 * (FCL's Convex checks this to enable edge-walking support queries).
 */
inline tesseract::geometry::ConvexMesh::Ptr makeConvexSphere(int rings, int segments, double radius)
{
  auto vertices = std::make_shared<tesseract::common::VectorVector3d>();
  vertices->reserve((static_cast<std::size_t>(rings) * static_cast<std::size_t>(segments)) + 2U);
  vertices->emplace_back(0, 0, radius);  // north pole, index 0
  for (int r = 1; r <= rings; ++r)
  {
    const double polar = static_cast<double>(EIGEN_PI) * r / (rings + 1);
    for (int s = 0; s < segments; ++s)
    {
      const double azimuth = 2.0 * static_cast<double>(EIGEN_PI) * s / segments;
      vertices->emplace_back(radius * std::sin(polar) * std::cos(azimuth),
                             radius * std::sin(polar) * std::sin(azimuth),
                             radius * std::cos(polar));
    }
  }
  vertices->emplace_back(0, 0, -radius);  // south pole
  const int south = static_cast<int>(vertices->size()) - 1;
  auto ring = [segments](int r, int s) { return 1 + ((r - 1) * segments) + (s % segments); };

  // Caps are triangles, the bands between rings are quads; all counter-clockwise seen from outside.
  const int face_count = (2 * segments) + ((rings - 1) * segments);
  auto faces = std::make_shared<Eigen::VectorXi>((2 * segments * 4) + ((rings - 1) * segments * 5));
  int i = 0;
  for (int s = 0; s < segments; ++s)
  {
    (*faces)[i++] = 3;
    (*faces)[i++] = 0;
    (*faces)[i++] = ring(1, s);
    (*faces)[i++] = ring(1, s + 1);
  }
  for (int r = 1; r < rings; ++r)
    for (int s = 0; s < segments; ++s)
    {
      (*faces)[i++] = 4;
      (*faces)[i++] = ring(r, s);
      (*faces)[i++] = ring(r + 1, s);
      (*faces)[i++] = ring(r + 1, s + 1);
      (*faces)[i++] = ring(r, s + 1);
    }
  for (int s = 0; s < segments; ++s)
  {
    (*faces)[i++] = 3;
    (*faces)[i++] = south;
    (*faces)[i++] = ring(rings, s + 1);
    (*faces)[i++] = ring(rings, s);
  }
  return std::make_shared<tesseract::geometry::ConvexMesh>(vertices, faces, face_count);
}

/** @brief Box [-x/2, x/2] x [-y/2, y/2] x [-z, 0] as a ConvexMesh (top face at z = 0) */
inline tesseract::geometry::ConvexMesh::Ptr makeConvexTile(double x, double y, double z)
{
  auto vertices = std::make_shared<tesseract::common::VectorVector3d>();
  for (int k = 0; k < 8; ++k)
    vertices->emplace_back(((k & 1) != 0 ? 0.5 : -0.5) * x, ((k & 2) != 0 ? 0.5 : -0.5) * y, ((k & 4) != 0 ? 0.0 : -z));
  auto faces = std::make_shared<Eigen::VectorXi>(30);
  *faces << 4, 0, 2, 3, 1,  // bottom
      4, 4, 5, 7, 6,        // top
      4, 0, 1, 5, 4,        // -y
      4, 2, 6, 7, 3,        // +y
      4, 0, 4, 6, 2,        // -x
      4, 1, 3, 7, 5;        // +x
  return std::make_shared<tesseract::geometry::ConvexMesh>(vertices, faces, 6);
}

/**
 * @brief Time to add one link holding a single ConvexMesh of state.range(0) vertices (approximately).
 * @details This is where the shape is built. A new geometry every iteration (new UUID) means the shape cache
 * never returns a shape built earlier. Only addCollisionObject() is timed.
 */
static void BM_CONVEX_MESH_ADD(benchmark::State& state, const DiscreteContactManager::ConstPtr& checker)  // NOLINT
{
  const int rings = static_cast<int>(std::lround(std::sqrt(static_cast<double>(state.range(0)) / 2.0)));
  const auto proto = makeConvexSphere(rings, 2 * rings, 1.0);
  const tesseract::common::VectorIsometry3d poses{ Eigen::Isometry3d::Identity() };
  const tesseract::common::LinkId id("hull");
  DiscreteContactManager::Ptr manager = checker->clone();

  for (auto _ : state)  // NOLINT
  {
    const CollisionShapesConst shapes{ proto->clone() };
    const auto start = std::chrono::steady_clock::now();
    manager->addCollisionObject(id, 0, shapes, poses);
    const auto stop = std::chrono::steady_clock::now();
    state.SetIterationTime(std::chrono::duration<double>(stop - start).count());
    manager->removeCollisionObject(id);
  }
  state.SetComplexityN(proto->getVertexCount());
}

/**
 * @brief contactTest() of a 62-vertex convex probe against one link of state.range(0) ConvexMesh box tiles
 * (20 x 20 x 10 mm on a 25 mm grid). One iteration is a sweep over 100 fixed probe poses. `gap` is the probe
 * clearance above the tiles. The "contacts" counter is the contact count of one sweep, to compare engines on the
 * same scene.
 */
static void BM_CONVEX_MESH_TILES_CONTACT_TEST(benchmark::State& state,  // NOLINT
                                              const DiscreteContactManager::ConstPtr& checker,
                                              double gap,
                                              double margin)
{
  constexpr double tile_xy = 0.020;
  constexpr double tile_z = 0.010;
  constexpr double pitch = 0.025;
  constexpr double probe_radius = 0.050;

  const int side = static_cast<int>(std::lround(std::sqrt(static_cast<double>(state.range(0)))));
  CollisionShapesConst tiles;
  tesseract::common::VectorIsometry3d tile_poses;
  for (int i = 0; i < side; ++i)
    for (int j = 0; j < side; ++j)
    {
      tiles.push_back(makeConvexTile(tile_xy, tile_xy, tile_z));
      Eigen::Isometry3d p = Eigen::Isometry3d::Identity();
      p.translation() = Eigen::Vector3d(i * pitch, j * pitch, 0);
      tile_poses.push_back(p);
    }

  DiscreteContactManager::Ptr manager = checker->clone();
  const tesseract::common::LinkId tiles_id("tiles");
  const tesseract::common::LinkId probe_id("probe");
  manager->addCollisionObject(tiles_id, 0, tiles, tile_poses);
  manager->addCollisionObject(
      probe_id, 0, { makeConvexSphere(5, 12, probe_radius) }, { Eigen::Isometry3d::Identity() });
  manager->setActiveCollisionObjects(std::vector<tesseract::common::LinkId>{ probe_id });
  manager->setCollisionMarginData(CollisionMarginData(margin));

  std::mt19937 rng(42);  // NOLINT
  std::uniform_real_distribution<double> xy(0.0, (side - 1) * pitch);
  tesseract::common::VectorIsometry3d probe_poses;
  for (int k = 0; k < 100; ++k)
  {
    Eigen::Isometry3d p = Eigen::Isometry3d::Identity();
    p.translation() = Eigen::Vector3d(xy(rng), xy(rng), probe_radius + gap);
    probe_poses.push_back(p);
  }

  const ContactRequest request(ContactTestType::ALL);
  ContactResultMap result;
  long contacts = 0;
  for (auto _ : state)  // NOLINT
  {
    contacts = 0;
    for (const auto& pose : probe_poses)
    {
      manager->setCollisionObjectsTransform(probe_id, pose);
      result.clear();
      manager->contactTest(result, request);
      contacts += result.count();
    }
  }
  state.counters["contacts"] = benchmark::Counter(static_cast<double>(contacts));
}

/** @brief Register the ConvexMesh benchmarks for one contact manager */
inline void registerConvexMeshBenchmarks(const DiscreteContactManager::ConstPtr& checker)
{
  const std::string prefix = "BM_CONVEX_MESH_ADD_" + checker->getName();
  benchmark::RegisterBenchmark(prefix.c_str(), BM_CONVEX_MESH_ADD, checker)  // NOLINT
      ->Arg(1000)
      ->Arg(4000)
      ->Arg(16000)
      ->UseManualTime()
      ->Complexity()
      ->Unit(benchmark::TimeUnit::kMicrosecond);

  struct Case
  {
    const char* name;
    double gap;
    double margin;
  };
  for (const Case& c :
       { Case{ "FREE", 0.020, 0.0 }, Case{ "PENETRATION", -0.002, 0.0 }, Case{ "DISTANCE", 0.010, 0.030 } })
  {
    const std::string name = "BM_CONVEX_MESH_TILES_CONTACT_TEST_" + std::string(c.name) + "_" + checker->getName();
    benchmark::RegisterBenchmark(name.c_str(), BM_CONVEX_MESH_TILES_CONTACT_TEST, checker, c.gap, c.margin)  // NOLINT
        ->Arg(256)
        ->Arg(4096)
        ->Arg(16384)
        ->Unit(benchmark::TimeUnit::kMillisecond);
  }
}

}  // namespace tesseract::collision::test_suite

#endif  // TESSERACT_COLLISION_CONVEX_MESH_BENCHMARKS_HPP
