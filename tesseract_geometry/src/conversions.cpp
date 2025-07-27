/**
 * @file conversions.h
 * @brief Tesseract Geometry Conversion Functions
 *
 * @author Levi Armstrong
 * @date July 27, 2025
 * @version TODO
 * @bug No known bugs
 *
 * @copyright Copyright (c) 2025, Levi Armstrong
 *
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

#include <tesseract_geometry/conversions.h>
#include <tesseract_geometry/geometry.h>
#include <tesseract_geometry/impl/mesh.h>
#include <tesseract_geometry/impl/box.h>
#include <tesseract_geometry/impl/sphere.h>
#include <tesseract_geometry/impl/cylinder.h>
#include <tesseract_geometry/impl/cone.h>
#include <tesseract_geometry/impl/capsule.h>

#include <Eigen/Core>
#include <Eigen/Geometry>

namespace tesseract_geometry
{
/**
 * @brief Generates a sphere mesh using a UV sphere approximation.
 *
 * The sphere is approximated using a grid of latitude and longitude segments, where
 * each segment forms part of a triangle mesh that covers the sphere's surface.
 *
 * Rather than specifying the number of segments directly, this function estimates the
 * number of segments required to ensure the maximum deviation between the mesh surface
 * and the ideal sphere surface does not exceed the given tolerance.
 *
 * ## Tolerance-Based Segmentation
 *
 * The sphere is sampled with:
 * - **Latitude segments** (horizontal rings)
 * - **Longitude segments** (vertical slices)
 *
 * The maximum edge length or surface deviation is bounded by the specified `tolerance`.
 *
 * For small angular steps \( \theta \), the arc chord distance from the center is approximately:
 *
 * \f[
 *   \text{chord deviation} \approx R \left(1 - \cos\left(\frac{\theta}{2}\right)\right)
 *                              \approx \frac{R \theta^2}{8}
 * \f]
 *
 * To achieve a maximum surface deviation \( \delta \), solve for \( \theta \):
 *
 * \f[
 *   \delta \approx \frac{R \theta^2}{8} \Rightarrow \theta \approx \sqrt{\frac{8\delta}{R}}
 * \f]
 *
 * Then estimate:
 * - \f$ \text{Latitude segments} \approx \frac{\pi}{\theta} \f$
 * - \f$ \text{Longitude segments} \approx \frac{2\pi}{\theta} \f$
 *
 */
std::unique_ptr<Mesh> toTriangleMesh(const Sphere& geom, double tolerance, const Eigen::Isometry3d& origin)
{
  auto vertices = std::make_shared<tesseract_common::VectorVector3d>();
  auto normals = std::make_shared<tesseract_common::VectorVector3d>();
  std::vector<int> triangles;

  // Estimate angular step size to satisfy tolerance
  const double radius{ geom.getRadius() };
  const double theta_step = std::sqrt(8.0 * tolerance / radius);
  const int latitude_segments = std::max(3, static_cast<int>(std::ceil(M_PI / theta_step)));
  const int longitude_segments = std::max(3, static_cast<int>(std::ceil(2.0 * M_PI / theta_step)));

  // Top vertex
  {
    const Eigen::Vector3d pos(0, 0, geom.getRadius());
    vertices->push_back(origin * pos);
    normals->push_back(origin * pos.normalized());
  }

  // Latitude rings (excluding poles)
  for (int lat = 1; lat < latitude_segments; ++lat)
  {
    const double theta = M_PI * lat / latitude_segments;
    const double sin_theta = std::sin(theta);
    const double cos_theta = std::cos(theta);

    for (int lon = 0; lon < longitude_segments; ++lon)
    {
      const double phi = 2.0 * M_PI * lon / longitude_segments;
      const double sin_phi = std::sin(phi);
      const double cos_phi = std::cos(phi);

      Eigen::Vector3d pos;
      pos.x() = radius * sin_theta * cos_phi;
      pos.y() = radius * sin_theta * sin_phi;
      pos.z() = radius * cos_theta;

      vertices->push_back(origin * pos);
      normals->push_back(origin * pos.normalized());
    }
  }

  // Bottom vertex
  {
    const Eigen::Vector3d pos(0, 0, -radius);
    vertices->push_back(origin * pos);
    normals->push_back(origin * pos.normalized());
  }

  // Indices
  const int top_index = 0;
  const int bottom_index = static_cast<int>(vertices->size() - 1);

  // Top cap triangles
  for (int lon = 0; lon < longitude_segments; ++lon)
  {
    const int next = (lon + 1) % longitude_segments;
    triangles.push_back(3);
    triangles.push_back(top_index);
    triangles.push_back(lon + 1);
    triangles.push_back(next + 1);
  }

  // Middle bands
  for (int lat = 1; lat < latitude_segments - 1; ++lat)
  {
    for (int lon = 0; lon < longitude_segments; ++lon)
    {
      const int current = (lat - 1) * longitude_segments + lon + 1;
      const int next = current + longitude_segments;

      const int next_lon = (lon + 1) % longitude_segments;
      const int current_right = (lat - 1) * longitude_segments + next_lon + 1;
      const int next_right = current_right + longitude_segments;

      triangles.push_back(3);
      triangles.push_back(current);
      triangles.push_back(next);
      triangles.push_back(current_right);

      triangles.push_back(3);
      triangles.push_back(current_right);
      triangles.push_back(next);
      triangles.push_back(next_right);
    }
  }

  // Bottom cap triangles
  const int base = (latitude_segments - 2) * longitude_segments + 1;
  for (int lon = 0; lon < longitude_segments; ++lon)
  {
    const int next = (lon + 1) % longitude_segments;
    triangles.push_back(3);
    triangles.push_back(bottom_index);
    triangles.push_back(base + next);
    triangles.push_back(base + lon);
  }

  auto faces = std::make_shared<Eigen::VectorXi>(
      Eigen::Map<Eigen::VectorXi>(triangles.data(), static_cast<int>(triangles.size())));
  return std::make_unique<Mesh>(vertices, faces, nullptr, Eigen::Vector3d(1, 1, 1), normals);
}

std::unique_ptr<Mesh> toTriangleMesh(const Box& geom, double /*tolerance*/, const Eigen::Isometry3d& origin)
{
  const double l = geom.getX() / 2.0;
  const double w = geom.getY() / 2.0;
  const double h = geom.getZ() / 2.0;

  // Shared 8 corners
  auto vertices = std::make_shared<tesseract_common::VectorVector3d>(
      tesseract_common::VectorVector3d{ origin * Eigen::Vector3d{ -l, -w, -h },   // 0
                                        origin * Eigen::Vector3d{ +l, -w, -h },   // 1
                                        origin * Eigen::Vector3d{ +l, +w, -h },   // 2
                                        origin * Eigen::Vector3d{ -l, +w, -h },   // 3
                                        origin * Eigen::Vector3d{ -l, -w, +h },   // 4
                                        origin * Eigen::Vector3d{ +l, -w, +h },   // 5
                                        origin * Eigen::Vector3d{ +l, +w, +h },   // 6
                                        origin * Eigen::Vector3d{ -l, +w, +h } }  // 7
  );

  // Triangle indices (2 per face)
  const std::vector<int> triangles = { // Bottom face (-Z)
                                       3,
                                       0,
                                       2,
                                       1,
                                       3,
                                       0,
                                       3,
                                       2,
                                       // Top face (+Z)
                                       3,
                                       4,
                                       5,
                                       6,
                                       3,
                                       4,
                                       6,
                                       7,
                                       // Front face (-Y)
                                       3,
                                       0,
                                       1,
                                       5,
                                       3,
                                       0,
                                       5,
                                       4,
                                       // Back face (+Y)
                                       3,
                                       2,
                                       3,
                                       7,
                                       3,
                                       2,
                                       7,
                                       6,
                                       // Left face (-X)
                                       3,
                                       0,
                                       4,
                                       7,
                                       3,
                                       0,
                                       7,
                                       3,
                                       // Right face (+X)
                                       3,
                                       1,
                                       2,
                                       6,
                                       3,
                                       1,
                                       6,
                                       5
  };

  // Initialize empty normals
  auto normals = std::make_shared<tesseract_common::VectorVector3d>(vertices->size(), Eigen::Vector3d::Zero());

  // Accumulate face normals per vertex
  for (std::size_t i = 0; i < triangles.size(); i += 4)
  {
    // triangles layout: [3, v1, v2, v3]
    const int v1 = triangles[i + 1];
    const int v2 = triangles[i + 2];
    const int v3 = triangles[i + 3];

    const Eigen::Vector3d& a = (*vertices)[v1];
    const Eigen::Vector3d& b = (*vertices)[v2];
    const Eigen::Vector3d& c = (*vertices)[v3];
    const Eigen::Vector3d face_normal = (b - a).cross(c - a).normalized();

    (*normals)[v1] += face_normal;
    (*normals)[v2] += face_normal;
    (*normals)[v3] += face_normal;
  }

  // Normalize each accumulated normal
  for (auto& n : *normals)
    n.normalize();

  auto faces = std::make_shared<Eigen::VectorXi>(
      Eigen::Map<const Eigen::VectorXi>(triangles.data(), static_cast<int>(triangles.size())));
  return std::make_unique<Mesh>(vertices, faces, nullptr, Eigen::Vector3d(1, 1, 1), normals);
}

std::unique_ptr<Mesh> toTriangleMesh(const Cylinder& geom, double tolerance, const Eigen::Isometry3d& origin)
{
  auto vertices = std::make_shared<tesseract_common::VectorVector3d>();
  auto normals = std::make_shared<tesseract_common::VectorVector3d>();

  // Compute angle step size for tolerance
  const double radius = geom.getRadius();
  const double theta = std::sqrt(8.0 * tolerance / radius);  // similar to sphere derivation
  const int radial_segments = std::max(3, static_cast<int>(std::ceil(2.0 * M_PI / theta)));

  const double half_length = geom.getLength() / 2.0;

  const int center_top_index = 0;
  const int center_bottom_index = 1;

  // Add center vertices for caps (Z up and down)
  vertices->emplace_back(origin * Eigen::Vector3d{ 0, 0, +half_length });  // top center
  normals->emplace_back(origin * Eigen::Vector3d{ 0, 0, 1 });
  vertices->emplace_back(origin * Eigen::Vector3d{ 0, 0, -half_length });  // bottom center
  normals->emplace_back(origin * Eigen::Vector3d{ 0, 0, -1 });

  // Store indices for circular rim
  std::vector<int> top_rim_indices;
  std::vector<int> bottom_rim_indices;

  for (int i = 0; i < radial_segments; ++i)
  {
    const double angle = 2.0 * M_PI * i / radial_segments;
    const double x = radius * std::cos(angle);
    const double y = radius * std::sin(angle);

    // Top rim vertex
    vertices->emplace_back(origin * Eigen::Vector3d{ x, y, +half_length });
    normals->emplace_back(origin * Eigen::Vector3d{ 0, 0, 1 });
    top_rim_indices.push_back(static_cast<int>(vertices->size() - 1));

    // Bottom rim vertex
    vertices->emplace_back(origin * Eigen::Vector3d{ x, y, -half_length });
    normals->emplace_back(origin * Eigen::Vector3d{ 0, 0, -1 });
    bottom_rim_indices.push_back(static_cast<int>(vertices->size() - 1));
  }

  std::vector<int> triangles;

  // Top cap triangles (fan)
  for (int i = 0; i < radial_segments; ++i)
  {
    const int next = (i + 1) % radial_segments;
    triangles.push_back(3);
    triangles.push_back(center_top_index);
    triangles.push_back(top_rim_indices[i]);
    triangles.push_back(top_rim_indices[next]);
  }

  // Bottom cap triangles (fan)
  for (int i = 0; i < radial_segments; ++i)
  {
    const int next = (i + 1) % radial_segments;
    triangles.push_back(3);
    triangles.push_back(center_bottom_index);
    triangles.push_back(bottom_rim_indices[next]);
    triangles.push_back(bottom_rim_indices[i]);
  }

  // Side surface (quads split into triangles)
  for (int i = 0; i < radial_segments; ++i)
  {
    const int next = (i + 1) % radial_segments;

    const int top1 = top_rim_indices[i];
    const int top2 = top_rim_indices[next];
    const int bot1 = bottom_rim_indices[i];
    const int bot2 = bottom_rim_indices[next];

    // Compute shared normal for side vertices (drop Z, then normalize)
    Eigen::Vector3d norm1(origin * Eigen::Vector3d{ (*vertices)[top1].x(), (*vertices)[top1].y(), 0.0 });
    norm1.normalize();
    Eigen::Vector3d norm2(origin * Eigen::Vector3d{ (*vertices)[top2].x(), (*vertices)[top2].y(), 0.0 });
    norm2.normalize();

    // Update normals for side smoothing
    (*normals)[top1] = norm1;
    (*normals)[bot1] = norm1;
    (*normals)[top2] = norm2;
    (*normals)[bot2] = norm2;

    // Two triangles per quad
    triangles.push_back(3);
    triangles.push_back(top1);
    triangles.push_back(bot1);
    triangles.push_back(top2);

    triangles.push_back(3);
    triangles.push_back(top2);
    triangles.push_back(bot1);
    triangles.push_back(bot2);
  }

  auto faces = std::make_shared<Eigen::VectorXi>(
      Eigen::Map<Eigen::VectorXi>(triangles.data(), static_cast<int>(triangles.size())));
  return std::make_unique<Mesh>(vertices, faces, nullptr, Eigen::Vector3d(1, 1, 1), normals);
}

std::unique_ptr<Mesh> toTriangleMesh(const Cone& geom, double tolerance, const Eigen::Isometry3d& origin)
{
  auto vertices = std::make_shared<tesseract_common::VectorVector3d>();
  auto normals = std::make_shared<tesseract_common::VectorVector3d>();

  // Compute angular step size for tolerance-based deviation
  const double radius{ geom.getRadius() };
  const double theta = std::sqrt(8.0 * tolerance / radius);
  const int radial_segments = std::max(3, static_cast<int>(std::ceil(2.0 * M_PI / theta)));

  const double half_length = geom.getLength() / 2.0;

  // Vertex indices:
  const int apex_index = 0;
  const int base_center_index = 1;

  // Add apex at +Z and base center at -Z
  vertices->emplace_back(origin * Eigen::Vector3d{ 0, 0, +half_length });  // Apex
  normals->emplace_back(origin * Eigen::Vector3d{ 0, 0, 1 });
  vertices->emplace_back(origin * Eigen::Vector3d{ 0, 0, -half_length });  // Base center
  normals->emplace_back(origin * Eigen::Vector3d{ 0, 0, -1 });

  // Store base rim indices
  std::vector<int> base_indices;
  for (int i = 0; i < radial_segments; ++i)
  {
    const double angle = 2.0 * M_PI * i / radial_segments;
    const double x = radius * std::cos(angle);
    const double y = radius * std::sin(angle);
    const double z = -half_length;

    // Compute normal as direction from midpoint of edge to apex
    vertices->emplace_back(origin * Eigen::Vector3d{ x, y, z });
    const Eigen::Vector3d side_dir =
        origin * Eigen::Vector3d(x, y, half_length).normalized();  // pointing away from cone surface
    normals->push_back(side_dir);
    base_indices.push_back(static_cast<int>(vertices->size() - 1));
  }

  std::vector<int> triangles;

  // Side surface: triangle fans from apex to base rim
  for (int i = 0; i < radial_segments; ++i)
  {
    const int next = (i + 1) % radial_segments;
    triangles.push_back(3);
    triangles.push_back(apex_index);
    triangles.push_back(base_indices[i]);
    triangles.push_back(base_indices[next]);
  }

  // Base cap: triangle fan from base center to base rim (with flat normals)
  for (int i = 0; i < radial_segments; ++i)
  {
    const int next = (i + 1) % radial_segments;
    triangles.push_back(3);
    triangles.push_back(base_center_index);
    triangles.push_back(base_indices[next]);
    triangles.push_back(base_indices[i]);
  }

  auto faces = std::make_shared<Eigen::VectorXi>(
      Eigen::Map<Eigen::VectorXi>(triangles.data(), static_cast<int>(triangles.size())));
  return std::make_unique<Mesh>(vertices, faces, nullptr, Eigen::Vector3d(1, 1, 1), normals);
}

std::unique_ptr<Mesh> toTriangleMesh(const Capsule& geom, double tolerance, const Eigen::Isometry3d& origin)
{
  auto vertices = std::make_shared<tesseract_common::VectorVector3d>();
  auto normals = std::make_shared<tesseract_common::VectorVector3d>();

  // Estimate angular step size
  const double radius{ geom.getRadius() };
  const double length{ geom.getLength() };
  const double theta = std::sqrt(8.0 * tolerance / radius);
  const int lat_segments = std::max(4, static_cast<int>(std::ceil(M_PI / theta)));
  const int cyl_segments = std::max(1, static_cast<int>(std::ceil(length / (2 * radius * theta))));
  const int lon_segments = std::max(6, static_cast<int>(std::ceil(2.0 * M_PI / theta)));

  const int hemi_rows = lat_segments / 2;
  const int total_rows = hemi_rows * 2 + cyl_segments + 2;

  const double half_length = length / 2.0;
  const int cols = lon_segments + 1;

  // Generate vertices
  for (int row = 0; row < total_rows; ++row)
  {
    double z{ NAN };
    double r{ NAN };

    if (row == 0)
    {
      // Top pole
      r = 0;
      z = radius + half_length;
    }
    else if (row < hemi_rows + 1)
    {
      // Top hemisphere rings (row=1..hemi_rows)
      double phi = M_PI_2 * (static_cast<double>(row) / hemi_rows);
      r = radius * std::sin(phi);
      z = radius * std::cos(phi) + half_length;
    }
    else if (row < hemi_rows + cyl_segments + 1)
    {
      // Cylinder rings
      double t = static_cast<double>(row - hemi_rows) / cyl_segments;  // [0..1]
      r = radius;
      z = half_length - t * length;
    }
    else if (row < 2 * hemi_rows + cyl_segments + 1)
    {
      // Bottom hemisphere rings
      double phi = M_PI_2 * (static_cast<double>(row - hemi_rows - cyl_segments) / hemi_rows);
      // we want phi from 0→π/2
      r = radius * std::sin(M_PI_2 - phi);
      z = -radius * std::cos(M_PI_2 - phi) - half_length;
    }
    else
    {
      // Bottom pole row
      r = 0;
      z = -radius - half_length;
    }

    // For each column, duplicate the ring (even poles) so that
    // our quad→triangulation loop still works unmodified.
    for (int col = 0; col < cols; ++col)
    {
      double ang = 2.0 * M_PI * static_cast<double>(col) / lon_segments;
      double x = r * std::cos(ang);
      double y = r * std::sin(ang);
      Eigen::Vector3d pos(x, y, z);

      // Smooth normal: radial for cylinder/hemispheres, ±Z at poles
      Eigen::Vector3d n;
      if (r == 0)  // pole
        n = Eigen::Vector3d(0, 0, z > 0 ? 1 : -1);
      else if (std::abs(z - half_length) < 1e-8)  // top‐hemisphere base ring
        n = Eigen::Vector3d(0, 0, 1);
      else if (std::abs(z + half_length) < 1e-8)  // bottom‐hemisphere base ring
        n = Eigen::Vector3d(0, 0, -1);
      else
      {
        // slope normal for hemispheres/cylinder
        Eigen::Vector3d axis_offset(0, 0, (z > 0 ? +half_length : -half_length));
        n = (pos - axis_offset).normalized();
      }

      vertices->push_back(origin * pos);
      normals->push_back(origin * n);
    }
  }

  // Generate triangles
  std::vector<int> triangles;
  for (int row = 0; row < total_rows - 1; ++row)
  {
    for (int col = 0; col < lon_segments; ++col)
    {
      const int i0 = row * cols + col;
      const int i1 = i0 + 1;
      const int i2 = (row + 1) * cols + col;
      const int i3 = i2 + 1;

      triangles.push_back(3);
      triangles.push_back(i0);
      triangles.push_back(i2);
      triangles.push_back(i1);

      triangles.push_back(3);
      triangles.push_back(i1);
      triangles.push_back(i2);
      triangles.push_back(i3);
    }
  }

  auto faces = std::make_shared<Eigen::VectorXi>(
      Eigen::Map<Eigen::VectorXi>(triangles.data(), static_cast<int>(triangles.size())));
  return std::make_unique<Mesh>(vertices, faces, nullptr, Eigen::Vector3d(1, 1, 1), normals);
}

std::unique_ptr<Mesh> toTriangleMesh(const Geometry& geom, double tolerance, const Eigen::Isometry3d& origin)
{
  switch (geom.getType())
  {
    case GeometryType::BOX:
    {
      return toTriangleMesh(static_cast<const Box&>(geom), tolerance, origin);
    }
    case GeometryType::SPHERE:
    {
      return toTriangleMesh(static_cast<const Sphere&>(geom), tolerance, origin);
    }
    case GeometryType::CYLINDER:
    {
      return toTriangleMesh(static_cast<const Cylinder&>(geom), tolerance, origin);
    }
    case GeometryType::CONE:
    {
      return toTriangleMesh(static_cast<const Cone&>(geom), tolerance, origin);
    }
    case GeometryType::CAPSULE:
    {
      return toTriangleMesh(static_cast<const Capsule&>(geom), tolerance, origin);
    }
    default:
    {
      const std::string type = std::to_string(static_cast<int>(geom.getType()));
      throw std::runtime_error("toTriangleMesh, this geometric shape type (" + type + ") is not supported");
    }
  }
}
}  // namespace tesseract_geometry
