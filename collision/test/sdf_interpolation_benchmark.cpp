/**
 * @file sdf_interpolation_benchmark.cpp
 * @brief Speed and accuracy of the Bullet signed distance field paths.
 *
 * Compares the ways TesseractSdfCollisionShape can read a field in place (trilinear, tricubic
 * Catmull-Rom, and split: trilinear distance with tricubic direction), for dense and sparse
 * (NanoVDB) fields. Every query goes through the same helpers Bullet's SDF algorithms use
 * (querySdfShapePoint, makeImplicitSdfShape + collideImplicitSDF), so only the field
 * representation differs.
 *
 * Accuracy is measured against exact analytic distances on curved surfaces (a sphere, a cylindrical
 * bore) and sharp features (a box's edges and creases, and a 90 degree inside corner).
 * The inside corner is the hardest of these: where two surfaces meet at 90 degrees, the distance has a
 * crease along their bisector and its gradient flips across it. It is also where a tool often works,
 * for example along the seam between two plates in welding.
 *
 * Usage: sdf_interpolation_benchmark [spacing_m] (default 0.001)
 *        sdf_interpolation_benchmark field.nvdb  (a real field, mm)
 *
 * @copyright Copyright (c) 2026, Tesseract Robotics
 * @par License
 * Software License Agreement (Apache License)
 * @par
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 * http://www.apache.org/licenses/LICENSE-2.0
 * @par
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include <tesseract/common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <algorithm>
#include <chrono>
#include <array>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <fstream>
#include <functional>
#include <iterator>
#include <memory>
#include <random>
#include <string>
#include <vector>
#ifdef __linux__
#include <unistd.h>
#endif
#include <BulletCollision/CollisionShapes/btCollisionShape.h>

extern "C" {
#include <tinyvdb_io.h>
#include <tinyvdb_nanovdb.h>
#include <tinyvdb_sparse_tree.h>
}
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract/collision/bullet/bullet_utils.h>
#include <tesseract/collision/bullet/tesseract_sdf_collision_shape.h>
#include <tesseract/collision/implicit_sdf_collision_solver.h>
#include <tesseract/geometry/geometries.h>
#include <tesseract/geometry/impl/signed_distance_field_utils.h>

using tesseract::geometry::SignedDistanceField;
using tesseract::geometry::SignedDistanceFieldInterpolation;
namespace bi = tesseract::collision::bullet_internal;

namespace
{
using Clock = std::chrono::steady_clock;

double seconds(Clock::time_point start) { return std::chrono::duration<double>(Clock::now() - start).count(); }

/** @brief Resident memory of this process (bytes); 0 where it isn't measured (off Linux). */
double residentBytes()
{
#ifdef __linux__
  std::ifstream statm("/proc/self/statm");
  double pages_total{ 0 };
  double pages_resident{ 0 };
  statm >> pages_total >> pages_resident;
  return pages_resident * static_cast<double>(sysconf(_SC_PAGESIZE));
#else
  return 0;
#endif
}

/** @brief A test geometry with its exact signed distance and gradient. */
struct Shape
{
  std::string name;
  std::function<double(const Eigen::Vector3d&)> distance;
  std::function<Eigen::Vector3d(const Eigen::Vector3d&)> gradient;
  /** @brief Accuracy regions: label for a point, or empty to skip it. */
  std::function<std::string(const Eigen::Vector3d&, double)> region;
};

constexpr double RADIUS = 0.03;     // sphere and bore radius
constexpr double HALF_BOX = 0.025;  // box half extent

std::vector<Shape> makeShapes(double spacing)
{
  std::vector<Shape> shapes;
  shapes.push_back({ "sphere",
                     [](const Eigen::Vector3d& p) { return p.norm() - RADIUS; },
                     [](const Eigen::Vector3d& p) { return Eigen::Vector3d(p.normalized()); },
                     [](const Eigen::Vector3d&, double) { return std::string("convex curve (sphere R30)"); } });

  // Material outside a cylindrical bore along z: free space is inside the bore, the wall is concave.
  shapes.push_back({ "bore",
                     [](const Eigen::Vector3d& p) { return RADIUS - p.head<2>().norm(); },
                     [](const Eigen::Vector3d& p) {
                       const Eigen::Vector2d r = p.head<2>().normalized();
                       return Eigen::Vector3d(-r.x(), -r.y(), 0.0);
                     },
                     [](const Eigen::Vector3d&, double) { return std::string("concave curve (bore R30)"); } });

  shapes.push_back({ "box",
                     [](const Eigen::Vector3d& p) {
                       const Eigen::Vector3d q = p.cwiseAbs() - Eigen::Vector3d::Constant(HALF_BOX);
                       return q.cwiseMax(0.0).norm() + std::min(q.maxCoeff(), 0.0);
                     },
                     [](const Eigen::Vector3d& p) {
                       const Eigen::Vector3d q = p.cwiseAbs() - Eigen::Vector3d::Constant(HALF_BOX);
                       Eigen::Vector3d g;
                       if (q.maxCoeff() > 0.0)
                         g = q.cwiseMax(0.0).normalized();
                       else
                       {
                         Eigen::Index axis{ 0 };
                         q.maxCoeff(&axis);
                         g = Eigen::Vector3d::Zero();
                         g[axis] = 1.0;
                       }
                       return Eigen::Vector3d(g.cwiseProduct(p.cwiseSign()));
                     },
                     [spacing](const Eigen::Vector3d& p, double d) {
                       const Eigen::Vector3d q = p.cwiseAbs() - Eigen::Vector3d::Constant(HALF_BOX);
                       if (d > 0.0)
                       {
                         const int beyond = (q.x() > 0) + (q.y() > 0) + (q.z() > 0);
                         return beyond >= 2 ? std::string("box outside edges/corners (convex)") : std::string();
                       }
                       // Inside: near the crease where two faces are equally near (the interior medial surface).
                       std::array<double, 3> f{ -q.x(), -q.y(), -q.z() };
                       std::sort(f.begin(), f.end());
                       return (f[1] - f[0] < spacing) ? std::string("box inside creases") : std::string();
                     } });

  // Two plates meeting at 90 degrees: material is {z < 0} or {x < 0}; free space is x > 0, z > 0,
  // and its distance is min(x, z), with a crease on the bisector x = z.
  shapes.push_back({ "inside corner",
                     [](const Eigen::Vector3d& p) {
                       if (p.x() <= 0.0 && p.z() <= 0.0)
                         return -std::hypot(p.x(), p.z());
                       if (p.x() > 0.0 && p.z() > 0.0)
                         return std::min(p.x(), p.z());
                       return std::min(p.x(), p.z());
                     },
                     [](const Eigen::Vector3d& p) {
                       if (p.x() <= 0.0 && p.z() <= 0.0)
                         return Eigen::Vector3d(Eigen::Vector3d(p.x(), 0, p.z()).normalized());
                       return (p.x() < p.z()) ? Eigen::Vector3d::UnitX().eval() : Eigen::Vector3d::UnitZ().eval();
                     },
                     [spacing](const Eigen::Vector3d& p, double d) {
                       if (d <= 0.0)
                         return std::string();
                       const double off_crease = std::abs(p.x() - p.z()) / std::sqrt(2.0);
                       if (off_crease < 1.5 * spacing)
                         return std::string("inside corner, at crease");
                       if (off_crease > 4.0 * spacing)
                         return std::string("inside corner, flat faces");
                       return std::string();
                     } });
  return shapes;
}

constexpr double EXTENT = 0.05;  // domain [-EXTENT, EXTENT]^3

/** @brief A grid-backed field of @p shape on the lattice (values stored as float, as fusion stores them). */
std::shared_ptr<SignedDistanceField> makeDenseField(const Shape& shape, double spacing)
{
  const int n = static_cast<int>(std::lround(2 * EXTENT / spacing)) + 1;
  std::vector<double> distances;
  distances.reserve(static_cast<std::size_t>(n) * static_cast<std::size_t>(n) * static_cast<std::size_t>(n));
  for (int k = 0; k < n; ++k)
    for (int j = 0; j < n; ++j)
      for (int i = 0; i < n; ++i)
        distances.push_back(static_cast<float>(shape.distance(
            Eigen::Vector3d(-EXTENT + (i * spacing), -EXTENT + (j * spacing), -EXTENT + (k * spacing)))));
  return std::make_shared<SignedDistanceField>(
      Eigen::AlignedBox3d(Eigen::Vector3d::Constant(-EXTENT), Eigen::Vector3d::Constant(EXTENT)),
      Eigen::Vector3i::Constant(n),
      std::move(distances));
}

/** @brief A bare NanoVDB grid of @p shape holding only voxels with |distance| <= @p band. */
std::vector<std::uint8_t> makeNarrowBandGrid(const Shape& shape, double spacing, double band)
{
  const int half = static_cast<int>(std::lround(EXTENT / spacing));
  std::vector<tvdb_vec3i> coordinates;
  std::vector<float> values;
  for (int k = -half; k <= half; ++k)
    for (int j = -half; j <= half; ++j)
      for (int i = -half; i <= half; ++i)
      {
        const double d = shape.distance(spacing * Eigen::Vector3d(i, j, k));
        if (std::abs(d) > band)
          continue;
        coordinates.push_back({ i, j, k });
        values.push_back(static_cast<float>(d));
      }
  std::string grid_type = "Tree_float_5_4_3";
  tvdb_grid_t template_grid{};
  template_grid.descriptor.grid_type = grid_type.data();
  template_grid.tree.layout.num_levels = 4;
  template_grid.tree.layout.levels[0] = { TVDB_NODE_ROOT, TVDB_VALUE_FLOAT, 0 };
  template_grid.tree.layout.levels[1] = { TVDB_NODE_INTERNAL, TVDB_VALUE_FLOAT, 5 };
  template_grid.tree.layout.levels[2] = { TVDB_NODE_INTERNAL, TVDB_VALUE_FLOAT, 4 };
  template_grid.tree.layout.levels[3] = { TVDB_NODE_LEAF, TVDB_VALUE_FLOAT, 3 };
  template_grid.transform.type = TVDB_TRANSFORM_UNIFORM_SCALE;
  for (int axis = 0; axis < 3; ++axis)
  {
    template_grid.transform.scale_values[axis] = spacing;
    template_grid.transform.voxel_size[axis] = spacing;
  }
  const auto background = static_cast<float>(band);
  tvdb_grid_t grid{};
  if (!tvdb_grid_from_sparse_typed_using_template(&template_grid,
                                                  coordinates.data(),
                                                  values.data(),
                                                  values.size(),
                                                  TVDB_VALUE_FLOAT,
                                                  &background,
                                                  "band",
                                                  &grid))
    throw std::runtime_error("failed to build the narrow band grid");
  std::uint8_t* data = nullptr;
  std::size_t size = 0;
  tvdb_error_t error{};
  const tvdb_status_t status = tvdb_grid_to_nanovdb_float(&grid, &data, &size, &error);
  tvdb_grid_destroy_owned(&grid);
  if (status != TVDB_OK)
    throw std::runtime_error("failed to convert the narrow band grid");
  std::vector<std::uint8_t> result(data, data + size);
  std::free(data);  // NOLINT(cppcoreguidelines-no-malloc, cppcoreguidelines-owning-memory)
  return result;
}

/** @brief One way Bullet can see a field. */
struct Variant
{
  std::string name;
  std::shared_ptr<btCollisionShape> shape;
  double build_seconds{ 0 };
  double memory_bytes{ 0 };
};

/** @brief A way of reading the field: distance method and direction method. */
struct Method
{
  const char* label;
  SignedDistanceFieldInterpolation distance;
  SignedDistanceFieldInterpolation gradient;
};

const std::array<Method, 3> METHODS{
  { { "trilinear", SignedDistanceFieldInterpolation::TRILINEAR, SignedDistanceFieldInterpolation::TRILINEAR },
    { "tricubic", SignedDistanceFieldInterpolation::TRICUBIC, SignedDistanceFieldInterpolation::TRICUBIC },
    { "split (trilinear distance, tricubic direction)",
      SignedDistanceFieldInterpolation::TRILINEAR,
      SignedDistanceFieldInterpolation::TRICUBIC } }
};

std::shared_ptr<SignedDistanceField> withMethod(const SignedDistanceField& field, const Method& m)
{
  auto copy = std::make_shared<SignedDistanceField>(field);
  copy->setInterpolation(m.distance);
  copy->setGradientInterpolation(m.gradient);
  return copy;
}

double percentile(std::vector<double> v, double q)
{
  if (v.empty())
    return std::nan("");
  const auto at = static_cast<std::size_t>(
      std::clamp(q * static_cast<double>(v.size() - 1), 0.0, static_cast<double>(v.size() - 1)));
  std::nth_element(v.begin(), v.begin() + static_cast<std::ptrdiff_t>(at), v.end());
  return v[at];
}

double angleDegrees(const Eigen::Vector3d& a, const Eigen::Vector3d& b)
{
  if (a.squaredNorm() == 0.0 || b.squaredNorm() == 0.0)
    return 180.0;
  return std::acos(std::clamp(a.normalized().dot(b.normalized()), -1.0, 1.0)) * 180.0 / M_PI;
}

/** @brief Points spread over the domain (away from its faces) whose true distance is in [lo, hi]. */
std::vector<Eigen::Vector3d> bandPoints(const Shape& shape, double lo, double hi, std::size_t count, unsigned seed)
{
  std::mt19937 rng(seed);
  std::uniform_real_distribution<double> u(-EXTENT + 0.006, EXTENT - 0.006);
  std::vector<Eigen::Vector3d> points;
  while (points.size() < count)
  {
    const Eigen::Vector3d p(u(rng), u(rng), u(rng));
    const double d = shape.distance(p);
    if (d >= lo && d <= hi)
      points.push_back(p);
  }
  return points;
}

std::string decimals(double v, int digits = 2)
{
  std::array<char, 64> buffer{};
  std::snprintf(buffer.data(), buffer.size(), "%.*f", digits, v);
  return buffer.data();
}

std::string mm(double metres, int digits = 3) { return decimals(metres * 1e3, digits); }

/** @brief One field of a shape: where its samples came from, and the Bullet views of it. */
struct Source
{
  std::string name;
  std::shared_ptr<SignedDistanceField> dense;
  std::vector<Variant> variants;
};

struct Stats
{
  std::vector<double> abs_error;
  std::vector<double> angle;
  double over{ 0 };
  double under{ 0 };
  std::size_t missed{ 0 };
  void add(double e)
  {
    abs_error.push_back(std::abs(e));
    over = std::max(over, e);
    under = std::max(under, -e);
  }
  double mean() const
  {
    double m = 0;
    for (double e : abs_error)
      m += e;
    return m / static_cast<double>(std::max<std::size_t>(abs_error.size(), 1));
  }
};

/** @brief The bytes of a file; empty if it cannot be read. */
std::vector<std::uint8_t> readFile(const std::string& path)
{
  std::ifstream in(path, std::ios::binary);
  return { std::istreambuf_iterator<char>(in), std::istreambuf_iterator<char>() };
}

/**
 * @brief Benchmark a real fusion field (a millimetre NanoVDB file), read at scale 1e-3 (millimetres to metres).
 * @details No reference geometry exists for a real part, so this measures what can be measured:
 * load time and memory, dense-vs-sparse agreement (a fully active field, so both read the same
 * samples), query / hull / probe timings per method, and direction smoothness along the surface.
 * @param whole_path The field's .nvdb (read dense and sparse)
 */
int runRealFields(const std::string& whole_path)
{
  const Eigen::Vector3d scale = Eigen::Vector3d::Constant(1e-3);  // fields in mm, used in m
  std::printf("# SDF benchmark on a real fusion field\n\nWHOLE: `%s`\n\nDistances in mm.\n\n", whole_path.c_str());

  const std::vector<std::uint8_t> whole_bytes = readFile(whole_path);
  if (whole_bytes.empty())
    throw std::runtime_error("cannot read " + whole_path);

  // WHOLE dense (the reader expands it to its box) and sparse (read in place).
  auto start = Clock::now();
  const double rss_before = residentBytes();
  const auto dense = tesseract::geometry::readSignedDistanceFieldNVDB(whole_bytes, scale);
  const double dense_load = seconds(start);
  const double dense_rss = residentBytes() - rss_before;
  start = Clock::now();
  const auto sparse = std::make_shared<SignedDistanceField>(whole_bytes, 0.0, scale);
  const double sparse_load = seconds(start);
  const auto sparse_bytes = static_cast<double>(sparse->getNVDB().size());

  const Eigen::Vector3i dims = dense->getDimensions();
  std::printf("## Load and memory\n\n| Field | load (s) | memory (MB) | notes |\n| -- | --: | --: | -- |\n");
  std::printf("| WHOLE dense | %s | %s | %d x %d x %d samples as double |\n",
              decimals(dense_load, 2).c_str(),
              decimals(std::max(dense_rss, 8.0 * dims.cast<double>().prod()) / 1e6, 0).c_str(),
              dims.x(),
              dims.y(),
              dims.z());
  std::printf("| WHOLE sparse | %s | %s | NanoVDB grid, read in place |\n",
              decimals(sparse_load, 2).c_str(),
              decimals(sparse_bytes / 1e6, 0).c_str());

  // Points near WHOLE's surface (metres, in the field's frame): true distance unknown, so read the
  // dense field, which is the same samples as the sparse one.
  const Eigen::AlignedBox3d box(dense->getDomain().min().cwiseProduct(scale),
                                dense->getDomain().max().cwiseProduct(scale));
  const auto near_surface = [&box](const SignedDistanceField& field,
                                   const Eigen::Vector3d& field_scale,
                                   double lo,
                                   double hi,
                                   std::size_t count,
                                   unsigned seed) {
    std::mt19937 rng(seed);
    const Eigen::Vector3d inset = Eigen::Vector3d::Constant(0.005);
    std::uniform_real_distribution<double> ux(box.min().x() + inset.x(), box.max().x() - inset.x());
    std::uniform_real_distribution<double> uy(box.min().y() + inset.y(), box.max().y() - inset.y());
    std::uniform_real_distribution<double> uz(box.min().z() + inset.z(), box.max().z() - inset.z());
    std::vector<Eigen::Vector3d> points;
    for (std::size_t tries = 0; points.size() < count && tries < 200 * count; ++tries)
    {
      const Eigen::Vector3d p(ux(rng), uy(rng), uz(rng));
      const double d = field.getDistance(p.cwiseQuotient(field_scale)) * field_scale.x();
      if (d >= lo && d <= hi)
        points.push_back(p);
    }
    return points;
  };
  const std::vector<Eigen::Vector3d> points = near_surface(*dense, scale, -0.002, 0.008, 200000, 7);
  std::printf("\n%zu query points within [-2, +8] mm of WHOLE's surface.\n", points.size());

  // Dense vs sparse WHOLE: same samples, two readers.
  double max_gap = 0;
  for (const auto& p : points)
  {
    const Eigen::Vector3d q = p.cwiseQuotient(scale);
    max_gap = std::max(max_gap, std::abs(dense->getDistance(q) - sparse->getDistance(q)));
  }
  std::printf("\nDense vs sparse WHOLE, trilinear, over those points: max |difference| = %.2e mm.\n", max_gap);

  // Timings and smoothness, per field and method.
  tesseract::collision::ImplicitSDFCollisionConfig config;
  config.contact_margin = 0.01;
  constexpr double PROBE = 0.005;
  constexpr double HULL_THRESHOLD = 0.008;  // contact distance for the hull check
  std::string speed_rows;
  std::string smooth_rows;
  const auto run =
      [&](const std::string& label, const SignedDistanceField& base, const std::vector<Eigen::Vector3d>& query_points) {
        for (const Method& method : METHODS)
        {
          const bi::TesseractSdfCollisionShape shape_obj(withMethod(base, method));
          volatile double sink = 0;
          auto t = Clock::now();
          for (const auto& p : query_points)
          {
            btScalar d{ 0 };
            btVector3 normal;
            if (bi::querySdfShapePoint(shape_obj, bi::convertEigenToBt(p), d, normal))
              sink = sink + static_cast<double>(d) + static_cast<double>(normal.x());
          }
          const double query_ns = seconds(t) * 1e9 / static_cast<double>(query_points.size());

          // Surface frames at 200 of the points: foot point and outward normal.
          std::vector<std::pair<Eigen::Vector3d, Eigen::Vector3d>> frames;
          for (std::size_t i = 0; i < query_points.size() && frames.size() < 200; i += query_points.size() / 200 + 1)
          {
            btScalar d{ 0 };
            btVector3 normal;
            if (!bi::querySdfShapePoint(shape_obj, bi::convertEigenToBt(query_points[i]), d, normal) ||
                normal.length2() == 0)
              continue;
            const Eigen::Vector3d n = bi::convertBtToEigen(normal).normalized();
            frames.emplace_back(query_points[i] - (static_cast<double>(d) * n), n);
          }

          // Hull check: a 1477-vertex, 10 mm hull 3 mm off the surface, read lazily at 8 mm.
          t = Clock::now();
          for (const auto& [foot, n] : frames)
          {
            const Eigen::Vector3d centre = foot + (0.013 * n);
            for (int i = 0; i < 1477; ++i)
            {
              const double z = 1.0 - (2.0 * (i + 0.5) / 1477.0);
              const double r = std::sqrt(1.0 - (z * z));
              const double phi = i * M_PI * (3.0 - std::sqrt(5.0));
              const Eigen::Vector3d v = centre + (0.01 * Eigen::Vector3d(r * std::cos(phi), r * std::sin(phi), z));
              btScalar d{ 0 };
              btVector3 normal;
              if (bi::querySdfShapeContact(
                      shape_obj, bi::convertEigenToBt(v), static_cast<btScalar>(HULL_THRESHOLD), d, normal))
                sink = sink + static_cast<double>(d);
            }
          }
          const double hull_us = seconds(t) * 1e6 / static_cast<double>(std::max<std::size_t>(frames.size(), 1));

          // Probe contact: a 5 mm sphere 2 mm off the surface.
          const auto field_shape = bi::makeImplicitSdfShape(shape_obj, btTransform::getIdentity());
          t = Clock::now();
          for (const auto& [foot, n] : frames)
          {
            const tesseract::geometry::Sphere probe(PROBE);
            const auto probe_shape = tesseract::collision::makeImplicitSDFShape(
                probe, Eigen::Isometry3d(Eigen::Translation3d(foot + ((PROBE + 0.002) * n))));
            sink = sink + static_cast<double>(
                              tesseract::collision::collideImplicitSDF(probe_shape, field_shape, config).size());
          }
          const double probe_us = seconds(t) * 1e6 / static_cast<double>(std::max<std::size_t>(frames.size(), 1));
          (void)sink;
          speed_rows += "| " + label + " | " + method.label + " | " + decimals(query_ns, 0) + " | " +
                        decimals(hull_us, 0) + " | " + decimals(probe_us, 0) + " |\n";

          // Smoothness: from 2 mm off the surface, walk 10 mm along it in 1/64 mm steps. A step that
          // turns more than 0.5 deg is bisected down to ~0.1 nm towards its larger half: a turn that
          // survives there is a discontinuity, classed by whether the distance jumps with it.
          const double step = 1e-3 / 64.0;
          const auto read = [&shape_obj](const Eigen::Vector3d& p, double& d, Eigen::Vector3d& g) {
            btScalar bd{ 0 };
            btVector3 normal;
            if (!bi::querySdfShapePoint(shape_obj, bi::convertEigenToBt(p), bd, normal))
              return false;
            d = static_cast<double>(bd);
            g = bi::convertBtToEigen(normal);
            return g.squaredNorm() > 0;
          };
          std::size_t interpolation_jumps = 0;  // direction jumps, distance continuous
          std::size_t data_jumps = 0;           // direction and distance jump: the edge of the data
          std::size_t no_direction = 0;         // no data or a zero gradient
          std::size_t steps = 0;
          std::vector<double> smooth_turns;
          std::vector<double> jump_sizes;  // interpolation jumps, deg
          for (const auto& [foot, n] : frames)
          {
            const Eigen::Vector3d tangent = n.unitOrthogonal();
            for (int k = 0; k < 64 * 10; ++k)
            {
              Eigen::Vector3d a = foot + (0.002 * n) + (tangent * (k * step));
              Eigen::Vector3d b = a + (tangent * step);
              double da{ 0 };
              double db{ 0 };
              Eigen::Vector3d ga;
              Eigen::Vector3d gb;
              ++steps;
              if (!read(a, da, ga) || !read(b, db, gb))
              {
                ++no_direction;
                continue;
              }
              const double turn = angleDegrees(ga, gb);
              if (turn <= 0.5)
              {
                smooth_turns.push_back(turn);
                continue;
              }
              bool lost = false;
              for (int halving = 0; halving < 18 && !lost; ++halving)
              {
                const Eigen::Vector3d m = 0.5 * (a + b);
                double dm{ 0 };
                Eigen::Vector3d gm;
                if (!read(m, dm, gm))
                {
                  lost = true;
                  break;
                }
                if (angleDegrees(ga, gm) >= angleDegrees(gm, gb))
                {
                  b = m;
                  db = dm;
                  gb = gm;
                }
                else
                {
                  a = m;
                  da = dm;
                  ga = gm;
                }
              }
              if (lost)
                ++no_direction;
              else if (angleDegrees(ga, gb) <= 0.1)
                smooth_turns.push_back(turn);  // turned fast but continuously
              else if (std::abs(db - da) > 1e-9 + (10.0 * (b - a).norm()))
                ++data_jumps;
              else
              {
                ++interpolation_jumps;
                jump_sizes.push_back(angleDegrees(ga, gb));
              }
            }
          }
          const double per_walk = 1.0 / static_cast<double>(std::max<std::size_t>(frames.size(), 1));
          smooth_rows +=
              "| " + label + " | " + method.label + " | " +
              decimals(static_cast<double>(interpolation_jumps) * per_walk) + " | " +
              (jump_sizes.empty() ?
                   std::string("-") :
                   decimals(percentile(jump_sizes, 0.5)) + " / " + decimals(percentile(jump_sizes, 0.99))) +
              " | " + decimals(static_cast<double>(data_jumps) * per_walk) + " | " +
              decimals(100.0 * static_cast<double>(no_direction) / static_cast<double>(std::max<std::size_t>(steps, 1)),
                       2) +
              " | " + decimals(percentile(smooth_turns, 0.99), 3) + " | " +
              decimals(percentile(smooth_turns, 0.999), 3) + " |\n";
        }
      };
  run("WHOLE dense", *dense, points);
  run("WHOLE sparse", *sparse, points);

  std::printf("\n## Speed\n\n*Query* = one distance + normal at a point near the surface. *Hull* = one vertex-query "
              "check of a 1477-vertex, 10 mm hull whose nearest vertex is 3 mm off the surface, the normal read only "
              "within 8 mm (Bullet's vertex loop). *Probe* = one implicit-solver contact check of a 5 mm sphere 2 mm "
              "off the surface. Hull and probe averaged over 200 places on the surface.\n\n"
              "| Field | Method | query (ns) | hull (us) | probe (us) |\n| -- | -- | --: | --: | --: |\n%s",
              speed_rows.c_str());
  std::printf("\n## Direction smoothness along the surface\n\nFrom 200 places, 2 mm off the surface, a 10 mm walk "
              "along it in 1/64 mm steps. Any step turning more than 0.5 deg is bisected to ~0.1 nm: a turn that "
              "survives is a discontinuity. *Interpolation jumps* = the direction jumps while the distance stays "
              "continuous (a trilinear cell-face jump, or a stencil switching method). *Data jumps* = the distance "
              "jumps too (the edge of the grid's data). *No direction* = no data or a zero "
              "gradient on either side. Turn percentiles are over the steps with no discontinuity (deg per 1/64 "
              "mm). *Jump size* = the direction change across an interpolation jump, median / p99.\n\n| Field | Method "
              "| "
              "interpolation jumps per walk | jump size (deg) | data jumps per walk | no direction (%% of steps) | "
              "turn p99 | turn p99.9 |\n| -- | -- | --: | --: | --: | --: | --: | --: |\n%s",
              smooth_rows.c_str());
  return 0;
}

}  // namespace

int main(int argc, char** argv)
{
  const std::string first = (argc > 1) ? argv[1] : "";
  if (first.size() > 5 && first.compare(first.size() - 5, 5, ".nvdb") == 0)
    return runRealFields(first);
  const double spacing = (argc > 1) ? std::atof(argv[1]) : 0.001;
  const std::vector<Shape> shapes = makeShapes(spacing);
  const int n = static_cast<int>(std::lround(2 * EXTENT / spacing)) + 1;
  const double samples = std::pow(n, 3);
  std::printf("# SDF interpolation benchmark\n\nGrid spacing %s mm over a %s mm cube: %d^3 = %.2f M samples. "
              "Distances in mm.\n\n",
              mm(spacing, 2).c_str(),
              mm(2 * EXTENT, 0).c_str(),
              n,
              samples / 1e6);

  tesseract::collision::ImplicitSDFCollisionConfig config;
  config.contact_margin = 0.01;
  constexpr double PROBE = 0.005;

  std::string accuracy_rows;
  std::string smooth_rows;
  std::string contact_rows;
  std::string speed_rows;
  std::string hull_note;

  for (std::size_t s = 0; s < shapes.size(); ++s)
  {
    const Shape& shape = shapes[s];
    std::vector<Source> sources;
    sources.push_back({ "exact", makeDenseField(shape, spacing), {} });

    const std::vector<Eigen::Vector3d> points =
        bandPoints(shape, -0.003, 0.010, 400000, 17U + static_cast<unsigned>(s));
    const std::vector<Eigen::Vector3d> centres =
        bandPoints(shape, PROBE - 0.002, PROBE + 0.008, 3000, 101U + static_cast<unsigned>(s));
    std::vector<std::string> labels;
    for (const auto& p : points)
    {
      const std::string label = shape.region(p, shape.distance(p));
      if (!label.empty() && std::find(labels.begin(), labels.end(), label) == labels.end())
        labels.push_back(label);
    }

    for (Source& source : sources)
    {
      std::fprintf(stderr, "%s / %s\n", shape.name.c_str(), source.name.c_str());
      for (const Method& method : METHODS)
        source.variants.push_back({ std::string("field, ") + method.label,
                                    std::make_shared<bi::TesseractSdfCollisionShape>(withMethod(*source.dense, method)),
                                    0.0,
                                    0.0 });

      // Point queries per region.
      for (const std::string& label : labels)
        for (const Variant& v : source.variants)
        {
          Stats st;
          for (const auto& p : points)
          {
            const double truth = shape.distance(p);
            if (shape.region(p, truth) != label)
              continue;
            btScalar d{ 0 };
            btVector3 normal;
            if (!bi::querySdfShapePoint(*v.shape, bi::convertEigenToBt(p), d, normal))
              continue;
            st.add(static_cast<double>(d) - truth);
            st.angle.push_back(angleDegrees(bi::convertBtToEigen(normal), shape.gradient(p)));
          }
          accuracy_rows += "| " + label + " | " + source.name + " | " + v.name + " | " + mm(st.mean()) + " | " +
                           mm(percentile(st.abs_error, 0.99)) + " | " + mm(st.over) + " | " + mm(st.under) + " | " +
                           decimals(percentile(st.angle, 0.5)) + " | " + decimals(percentile(st.angle, 0.99)) + " |\n";
        }

      // Direction smoothness along paths over the sphere.
      if (s == 0)
      {
        std::uniform_real_distribution<double> u(-1.0, 1.0);
        for (const Variant& v : source.variants)
        {
          double max_turn = 0;
          std::size_t big = 0;
          std::size_t steps = 0;
          std::mt19937 path_rng(9);
          for (int path = 0; path < 200; ++path)
          {
            Eigen::Vector3d dir(u(path_rng), u(path_rng), u(path_rng));
            dir.normalize();
            Eigen::Vector3d start(u(path_rng), u(path_rng), u(path_rng));
            start = start.normalized() * (RADIUS + 0.002);
            Eigen::Vector3d tangent = dir - (dir.dot(start.normalized()) * start.normalized());
            tangent.normalize();
            Eigen::Vector3d previous = Eigen::Vector3d::Zero();
            for (int k = 0; k < 64 * 10; ++k)
            {
              const Eigen::Vector3d p = start + (tangent * (k * spacing / 64.0));
              btScalar d{ 0 };
              btVector3 normal;
              if (!bi::querySdfShapePoint(*v.shape, bi::convertEigenToBt(p), d, normal))
                continue;
              const Eigen::Vector3d g = bi::convertBtToEigen(normal);
              if (k > 0 && previous.squaredNorm() > 0)
              {
                const double turn = angleDegrees(previous, g);
                max_turn = std::max(max_turn, turn);
                big += (turn > 1.0) ? 1U : 0U;
                ++steps;
              }
              previous = g;
            }
          }
          smooth_rows +=
              "| " + source.name + " | " + v.name + " | " + decimals(max_turn) + " | " +
              decimals(100.0 * static_cast<double>(big) / static_cast<double>(std::max<std::size_t>(steps, 1)), 3) +
              " |\n";
        }
      }

      // Probe contacts per region.
      for (const std::string& label : labels)
        for (const Variant& v : source.variants)
        {
          const auto field_shape = bi::makeImplicitSdfShape(*v.shape, btTransform::getIdentity());
          Stats st;
          for (const auto& c : centres)
          {
            const double truth_centre = shape.distance(c);
            if (shape.region(c, truth_centre) != label)
              continue;
            const tesseract::geometry::Sphere probe(PROBE);
            const auto probe_shape =
                tesseract::collision::makeImplicitSDFShape(probe, Eigen::Isometry3d(Eigen::Translation3d(c)));
            const auto contacts = tesseract::collision::collideImplicitSDF(probe_shape, field_shape, config);
            if (contacts.empty())
            {
              ++st.missed;
              continue;
            }
            double d = contacts.front().distance;
            for (const auto& contact : contacts)
              d = std::min(d, contact.distance);
            st.add(d - (truth_centre - PROBE));
          }
          contact_rows += "| " + label + " | " + source.name + " | " + v.name + " | " + mm(st.mean()) + " | " +
                          mm(percentile(st.abs_error, 0.99)) + " | " + mm(st.over) + " | " + mm(st.under) + " | " +
                          std::to_string(st.missed) + " |\n";
        }

      // Speed and memory, sphere / exact only.
      if (s == 0 && source.name == "exact")
      {
        const std::vector<Eigen::Vector3d> speed_points = bandPoints(shape, -0.002, 0.010, 1000000, 7);
        const std::vector<Eigen::Vector3d> narrow_points = bandPoints(shape, -0.002, 0.002, 1000000, 8);
        std::vector<btVector3> hull;
        const Eigen::Vector3d hull_centre(RADIUS + 0.013, 0.0, 0.0);
        for (int i = 0; i < 1477; ++i)
        {
          const double z = 1.0 - (2.0 * (i + 0.5) / 1477.0);
          const double r = std::sqrt(1.0 - (z * z));
          const double phi = i * M_PI * (3.0 - std::sqrt(5.0));
          hull.push_back(bi::convertEigenToBt(
              Eigen::Vector3d(hull_centre + (0.01 * Eigen::Vector3d(r * std::cos(phi), r * std::sin(phi), z)))));
        }
        const tesseract::geometry::Sphere probe(PROBE);
        const auto probe_shape = tesseract::collision::makeImplicitSDFShape(
            probe, Eigen::Isometry3d(Eigen::Translation3d(RADIUS + PROBE + 0.002, 0.001, 0.002)));
        {
          std::size_t within8 = 0;
          std::size_t within2 = 0;
          for (const auto& vertex : hull)
          {
            const double d = shape.distance(bi::convertBtToEigen(vertex));
            within8 += (d <= 0.008) ? 1U : 0U;
            within2 += (d <= 0.002) ? 1U : 0U;
          }
          hull_note = "The hull's vertices are 3-23 mm from the surface: " + std::to_string(within8) + " of " +
                      std::to_string(hull.size()) + " are within 8 mm and " + std::to_string(within2) +
                      " within 2 mm, so those are the vertices whose normal is read.";
        }

        const auto measure = [&](const std::string& representation,
                                 const std::string& method,
                                 const btCollisionShape& shape_ref,
                                 double build,
                                 double memory,
                                 const std::vector<Eigen::Vector3d>& query_points) {
          volatile double sink = 0;
          auto start = Clock::now();
          for (const auto& p : query_points)
          {
            btScalar d{ 0 };
            btVector3 normal;
            if (bi::querySdfShapePoint(shape_ref, bi::convertEigenToBt(p), d, normal))
              sink = sink + static_cast<double>(d) + static_cast<double>(normal.x());
          }
          const double query_ns = seconds(start) * 1e9 / static_cast<double>(query_points.size());
          constexpr int HULL_REPEATS = 200;
          // Eager: distance and normal at every vertex (how the vertex loop worked before).
          start = Clock::now();
          for (int r = 0; r < HULL_REPEATS; ++r)
            for (const auto& vertex : hull)
            {
              btScalar d{ 0 };
              btVector3 normal;
              if (bi::querySdfShapePoint(shape_ref, vertex, d, normal) && d < 0.008)
                sink = sink + static_cast<double>(d);
            }
          const double hull_eager_us = seconds(start) * 1e6 / HULL_REPEATS;
          // Lazy: Bullet's vertex loop now, the normal only within the contact threshold.
          std::array<double, 2> hull_lazy_us{};
          const std::array<double, 2> thresholds{ 0.008, 0.002 };
          for (std::size_t t = 0; t < thresholds.size(); ++t)
          {
            start = Clock::now();
            for (int r = 0; r < HULL_REPEATS; ++r)
              for (const auto& vertex : hull)
              {
                btScalar d{ 0 };
                btVector3 normal;
                if (bi::querySdfShapeContact(shape_ref, vertex, static_cast<btScalar>(thresholds[t]), d, normal))
                  sink = sink + static_cast<double>(d) + static_cast<double>(normal.x());
              }
            hull_lazy_us[t] = seconds(start) * 1e6 / HULL_REPEATS;
          }
          const auto field_shape = bi::makeImplicitSdfShape(shape_ref, btTransform::getIdentity());
          constexpr int PROBE_REPEATS = 2000;
          start = Clock::now();
          for (int r = 0; r < PROBE_REPEATS; ++r)
            sink = sink + static_cast<double>(
                              tesseract::collision::collideImplicitSDF(probe_shape, field_shape, config).size());
          const double probe_us = seconds(start) * 1e6 / PROBE_REPEATS;
          (void)sink;
          // NOLINTNEXTLINE(performance-inefficient-string-concatenation): one table row per configuration
          speed_rows += "| " + representation + " | " + method + " | " + (build > 0 ? decimals(build, 3) : "0") +
                        " | " + decimals(memory / 1e6, 1) + " | " + decimals(query_ns, 0) + " | " +
                        decimals(hull_eager_us, 0) + " | " + decimals(hull_lazy_us[0], 0) + " | " +
                        decimals(hull_lazy_us[1], 0) + " | " + decimals(probe_us, 0) + " |\n";
        };

        for (std::size_t m = 0; m < METHODS.size(); ++m)
          measure(
              "dense, field in place", METHODS[m].label, *source.variants[m].shape, 0.0, 8.0 * samples, speed_points);
        {
          const auto start = Clock::now();
          std::vector<std::uint8_t> nvdb = tesseract::geometry::writeSignedDistanceFieldNVDB(*source.dense);
          const double encode = seconds(start);
          const auto parse_start = Clock::now();
          const SignedDistanceField sparse(std::move(nvdb), 0.0);
          const double parse = seconds(parse_start);
          const auto bytes = static_cast<double>(sparse.getNVDB().size());
          speed_rows += "| (sparse full box: NanoVDB encode " + decimals(encode, 3) + " s, parse " +
                        decimals(parse, 3) + " s) | | | | | | | | |\n";
          for (const Method& method : METHODS)
          {
            bi::TesseractSdfCollisionShape shape_obj(withMethod(sparse, method));
            measure("sparse, full box", method.label, shape_obj, parse, bytes, speed_points);
          }
        }
        {
          std::vector<std::uint8_t> nvdb = makeNarrowBandGrid(shape, spacing, 0.003);
          const auto parse_start = Clock::now();
          const SignedDistanceField sparse(std::move(nvdb), 0.003);
          const double parse = seconds(parse_start);
          const auto bytes = static_cast<double>(sparse.getNVDB().size());
          for (const Method& method : METHODS)
          {
            bi::TesseractSdfCollisionShape shape_obj(withMethod(sparse, method));
            measure("sparse, 3 mm band", method.label, shape_obj, parse, bytes, narrow_points);
          }
        }
      }
      source.variants.clear();  // frees this source's fields before the next
    }
  }

  std::printf("## Point queries: distance and direction error vs the exact geometry\n\n"
              "Points with true distance in [-3, +10] mm. *Over-read* = the field reports more clearance than "
              "there is (unsafe); *under-read* = less (conservative). Direction error is the angle between the "
              "reported and true normal.\n\n"
              "| Region | Field | Method | mean abs | p99 abs | max over-read | max under-read | direction p50 "
              "(deg) | direction p99 (deg) |\n| -- | -- | -- | --: | --: | --: | --: | --: | --: |\n%s",
              accuracy_rows.c_str());
  std::printf("\n## Direction smoothness along a path (sphere)\n\n"
              "200 straight paths over the sphere just outside its surface, sampled every 1/64 of a cell. The "
              "exact direction turns by at most ~0.03 deg per step; a jump at a cell face shows as a large step.\n\n"
              "| Field | Method | max turn per step (deg) | steps turning > 1 deg (%%) |\n| -- | -- | --: | --: |\n%s",
              smooth_rows.c_str());
  std::printf("\n## Contacts: a 5 mm probe sphere, as Bullet's convex-vs-SDF algorithm computes them\n\n"
              "True contact distance = exact distance of the probe centre - 5 mm, for probes with true contact "
              "distance in [-2, +8] mm and a 10 mm contact margin. Missed = no contact reported.\n\n"
              "| Region | Field | Method | mean abs | p99 abs | max over-read | max under-read | missed |\n"
              "| -- | -- | -- | --: | --: | --: | --: | --: |\n%s",
              contact_rows.c_str());
  std::printf("\n## Build, memory and query speed (sphere, exact field)\n\n"
              "*Build* = time to parse a sparse field. *Memory* = what the field holds. *Query* = one "
              "distance + normal, as Bullet's queryPoint asks it, averaged over 1 M points within 10 mm of the "
              "surface. *Hull* = one vertex-query collision check of a 1477-vertex hull: "
              "*eager* reads distance and normal at every vertex (the old vertex loop); *lazy* reads the normal "
              "only within the contact threshold (Bullet's vertex loop now), at 8 mm "
              "and 2 mm. %s *Probe* = one implicit-solver contact check of a sphere probe. Narrow-band rows "
              "query points within 2 mm of the surface.\n\n"
              "| Representation | Method | build (s) | memory (MB) | query (ns) | hull eager (us) | hull lazy 8 mm "
              "(us) "
              "| hull lazy 2 mm (us) | probe (us) |\n"
              "| -- | -- | --: | --: | --: | --: | --: | --: | --: |\n%s",
              hull_note.c_str(),
              speed_rows.c_str());
  return 0;
}
