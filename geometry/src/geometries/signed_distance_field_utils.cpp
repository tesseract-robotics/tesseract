/**
 * @file signed_distance_field_utils.cpp
 * @brief Utilities for discretizing and (de)serializing a SignedDistanceField grid
 *
 * @author Joel Kang
 * @date June 15, 2026
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

#include <tesseract/common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <array>
#include <cmath>
#include <cstdlib>
#include <cstring>
#include <limits>
#include <memory>
#include <stdexcept>
#include <string>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract/common/utils.h>
#include <tesseract/geometry/impl/signed_distance_field_utils.h>

extern "C" {
#include <tinyvdb_io.h>
#include <tinyvdb_nanovdb.h>
#include <tinyvdb_sparse_tree.h>
}

namespace tesseract::geometry
{
namespace
{
constexpr std::size_t MAX_SDF_VOXELS = std::size_t{ 256 } * std::size_t{ 1024 } * std::size_t{ 1024 };

struct ActiveBounds
{
  Eigen::Vector3i min{ Eigen::Vector3i::Constant(std::numeric_limits<int>::max()) };
  Eigen::Vector3i max{ Eigen::Vector3i::Constant(std::numeric_limits<int>::min()) };
  bool valid{ false };
};

int accumulateActiveBounds(const tvdb_leaf_view_t* leaf, void* user)
{
  auto& bounds = *static_cast<ActiveBounds*>(user);
  const int dim = 1 << leaf->log2dim;
  int linear_index = 0;
  for (int i = 0; i < dim; ++i)
    for (int j = 0; j < dim; ++j)
      for (int k = 0; k < dim; ++k, ++linear_index)
      {
        if (tvdb_nodemask_is_on(leaf->value_mask, linear_index) == 0)
          continue;
        const Eigen::Vector3i index(leaf->origin[0] + i, leaf->origin[1] + j, leaf->origin[2] + k);
        bounds.min = bounds.min.cwiseMin(index);
        bounds.max = bounds.max.cwiseMax(index);
        bounds.valid = true;
      }
  return 0;
}

// TinyVDB's allocator interface requires C allocation callbacks.
// NOLINTBEGIN(cppcoreguidelines-owning-memory,cppcoreguidelines-no-malloc)
void* systemMalloc(std::size_t size, void*) { return std::malloc(size); }

void* systemRealloc(void* pointer, std::size_t, std::size_t size, void*) { return std::realloc(pointer, size); }

void systemFree(void* pointer, std::size_t, void*) { std::free(pointer); }
// NOLINTEND(cppcoreguidelines-owning-memory,cppcoreguidelines-no-malloc)

tvdb_allocator_t systemAllocator() { return { &systemMalloc, &systemRealloc, &systemFree, nullptr }; }

void throwTinyVDBError(const char* operation, const tvdb_error_t& error)
{
  throw std::runtime_error(std::string(operation) + ": " +
                           (error.message[0] == '\0' ? tvdb_status_string(error.status) : error.message));
}
}  // namespace

/** @brief Adapt a per-point distance function to the batched interface. */
static BatchedSignedDistanceFunction toBatched(const SignedDistanceFunction& sdf)
{
  return [sdf](const std::vector<Eigen::Vector3d>& points) {
    std::vector<double> distances;
    distances.reserve(points.size());
    for (const auto& point : points)
      distances.push_back(sdf(point));
    return distances;
  };
}

SignedDistanceField::Ptr createDiscreteSignedDistanceField(const SignedDistanceFunction& sdf,
                                                           const Eigen::Vector3d& domain_min,
                                                           const Eigen::Vector3d& domain_max,
                                                           const Eigen::Vector3i& dimensions,
                                                           const Eigen::Vector3d& scale)
{
  return createDiscreteSignedDistanceField(toBatched(sdf), domain_min, domain_max, dimensions, scale);
}

SignedDistanceField::Ptr createDiscreteSignedDistanceField(const BatchedSignedDistanceFunction& sdf,
                                                           const Eigen::Vector3d& domain_min,
                                                           const Eigen::Vector3d& domain_max,
                                                           const Eigen::Vector3i& dimensions,
                                                           const Eigen::Vector3d& scale)
{
  // Eager: a pre-discretized lazy field, i.e. sample the grid up front so it is immediately serializable.
  auto field = createSignedDistanceField(sdf, domain_min, domain_max, dimensions, scale);
  field->discretize();
  return field;
}

SignedDistanceField::Ptr createSignedDistanceField(const SignedDistanceFunction& sdf,
                                                   const Eigen::Vector3d& domain_min,
                                                   const Eigen::Vector3d& domain_max,
                                                   const Eigen::Vector3i& dimensions,
                                                   const Eigen::Vector3d& scale)
{
  return createSignedDistanceField(toBatched(sdf), domain_min, domain_max, dimensions, scale);
}

SignedDistanceField::Ptr createSignedDistanceField(const BatchedSignedDistanceFunction& sdf,
                                                   const Eigen::Vector3d& domain_min,
                                                   const Eigen::Vector3d& domain_max,
                                                   const Eigen::Vector3i& dimensions,
                                                   const Eigen::Vector3d& scale)
{
  return std::make_shared<SignedDistanceField>(Eigen::AlignedBox3d(domain_min, domain_max), dimensions, sdf, scale);
}

std::vector<std::uint8_t> writeSignedDistanceFieldVDB(const SignedDistanceField& sdf)
{
  const std::vector<double>& distances = sdf.getDistances();
  const Eigen::Vector3i& dimensions = sdf.getDimensions();
  const Eigen::Vector3d voxel_size =
      (sdf.getDomain().max() - sdf.getDomain().min()).cwiseQuotient((dimensions.array() - 1).cast<double>().matrix());
  if (!voxel_size.allFinite() || voxel_size.minCoeff() <= 0.0 ||
      !voxel_size.isApprox(Eigen::Vector3d::Constant(voxel_size.x()), 1e-12))
    throw std::runtime_error("writeSignedDistanceFieldVDB: only uniformly spaced grids are supported");
  if (distances.size() > MAX_SDF_VOXELS)
    throw std::runtime_error("writeSignedDistanceFieldVDB: field exceeds the maximum supported voxel count");

  std::vector<float> values;
  values.reserve(distances.size());
  for (double distance : distances)
  {
    if (!std::isfinite(distance))
      throw std::runtime_error("writeSignedDistanceFieldVDB: distances must be finite");
    values.push_back(static_cast<float>(distance));
  }

  std::vector<tvdb_vec3i> coordinates;
  coordinates.reserve(values.size());
  for (int k = 0; k < dimensions.z(); ++k)
    for (int j = 0; j < dimensions.y(); ++j)
      for (int i = 0; i < dimensions.x(); ++i)
        coordinates.push_back({ i, j, k });

  std::string grid_type = "Tree_float_5_4_3";
  tvdb_grid_t template_grid{};
  template_grid.descriptor.grid_type = grid_type.data();  // TinyVDB duplicates this value.
  template_grid.tree.layout.num_levels = 4;
  template_grid.tree.layout.levels[0] = { TVDB_NODE_ROOT, TVDB_VALUE_FLOAT, 0 };
  template_grid.tree.layout.levels[1] = { TVDB_NODE_INTERNAL, TVDB_VALUE_FLOAT, 5 };
  template_grid.tree.layout.levels[2] = { TVDB_NODE_INTERNAL, TVDB_VALUE_FLOAT, 4 };
  template_grid.tree.layout.levels[3] = { TVDB_NODE_LEAF, TVDB_VALUE_FLOAT, 3 };
  template_grid.transform.type = TVDB_TRANSFORM_UNIFORM_SCALE_TRANSLATE;
  for (int axis = 0; axis < 3; ++axis)
  {
    template_grid.transform.scale_values[axis] = voxel_size.x();
    template_grid.transform.voxel_size[axis] = voxel_size.x();
    template_grid.transform.translation[axis] = sdf.getDomain().min()[axis];
  }

  const float background = values.empty() ? 0.0F : values.front();
  tvdb_grid_t grid{};
  if (!tvdb_grid_from_sparse_typed_using_template(&template_grid,
                                                  coordinates.data(),
                                                  values.data(),
                                                  values.size(),
                                                  TVDB_VALUE_FLOAT,
                                                  &background,
                                                  "signed_distance",
                                                  &grid))
    throw std::runtime_error("writeSignedDistanceFieldVDB: failed to build VDB grid");

  tvdb_file_t file{};
  file.alloc = systemAllocator();
  file.num_grids = 1;
  file.grids = &grid;
  tvdb_error_t error{};
  std::uint8_t* data = nullptr;
  std::size_t size = 0;
  const tvdb_status_t status =
      tvdb_write_to_memory(&file, TVDB_COMPRESS_BLOSC | TVDB_COMPRESS_ACTIVE_MASK, 5, &data, &size, &error);
  tvdb_grid_destroy_owned(&grid);
  if (status != TVDB_OK)
    throwTinyVDBError("writeSignedDistanceFieldVDB", error);

  const std::unique_ptr<std::uint8_t, decltype(&std::free)> data_owner(data, &std::free);
  std::vector<std::uint8_t> result(data, data + size);
  return result;
}

SignedDistanceField::Ptr readSignedDistanceFieldVDB(const std::uint8_t* data,
                                                    std::size_t size,
                                                    const Eigen::Vector3d& scale)
{
  if (data == nullptr || size == 0)
    throw std::runtime_error("readSignedDistanceFieldVDB: input is empty");

  tvdb_file_t file{};
  tvdb_error_t error{};
  tvdb_status_t status = tvdb_file_open_memory(&file, data, size, nullptr, &error);
  if (status != TVDB_OK)
    throwTinyVDBError("readSignedDistanceFieldVDB", error);

  status = tvdb_read_all_grids(&file, &error);
  if (status != TVDB_OK)
  {
    tvdb_file_close(&file);
    throwTinyVDBError("readSignedDistanceFieldVDB", error);
  }

  if (file.num_grids != 1 || file.grids[0].tree.layout.levels[0].value_type != TVDB_VALUE_FLOAT)
  {
    tvdb_file_close(&file);
    throw std::runtime_error("readSignedDistanceFieldVDB: exactly one FloatGrid is required");
  }

  const tvdb_grid_t& grid = file.grids[0];
  if (grid.transform.type != TVDB_TRANSFORM_UNIFORM_SCALE &&
      grid.transform.type != TVDB_TRANSFORM_UNIFORM_SCALE_TRANSLATE)
  {
    tvdb_file_close(&file);
    throw std::runtime_error("readSignedDistanceFieldVDB: only axis-aligned uniform transforms are supported");
  }

  const double voxel_size = grid.transform.voxel_size[0];
  if (!std::isfinite(voxel_size) || voxel_size <= 0.0)
  {
    tvdb_file_close(&file);
    throw std::runtime_error("readSignedDistanceFieldVDB: voxel size must be finite and positive");
  }

  ActiveBounds bounds;
  tvdb_grid_visit_leaves_float(&grid, &accumulateActiveBounds, &bounds);
  if (!bounds.valid)
  {
    tvdb_file_close(&file);
    throw std::runtime_error("readSignedDistanceFieldVDB: grid has no active voxels");
  }

  const Eigen::Vector3i dimensions = bounds.max - bounds.min + Eigen::Vector3i::Ones();
  const std::size_t voxel_count = static_cast<std::size_t>(dimensions.x()) * static_cast<std::size_t>(dimensions.y()) *
                                  static_cast<std::size_t>(dimensions.z());
  if (dimensions.minCoeff() < 2 || voxel_count > MAX_SDF_VOXELS)
  {
    tvdb_file_close(&file);
    throw std::runtime_error("readSignedDistanceFieldVDB: active bounds are unsupported or exceed the voxel limit");
  }

  const std::array<int32_t, 3> bounds_min = { bounds.min.x(), bounds.min.y(), bounds.min.z() };
  const std::array<int32_t, 3> bounds_max = { bounds.max.x() + 1, bounds.max.y() + 1, bounds.max.z() + 1 };
  tvdb_dense_grid dense{};
  if (!tvdb_grid_materialize_dense(
          &grid, bounds_min.data(), bounds_max.data(), tvdb_grid_float_background(&grid), &dense))
  {
    tvdb_file_close(&file);
    throw std::runtime_error("readSignedDistanceFieldVDB: failed to materialize grid data");
  }

  std::vector<double> distances(dense.data, dense.data + voxel_count);
  tvdb_dense_grid_free(&dense);
  const Eigen::Vector3d translation(
      grid.transform.translation[0], grid.transform.translation[1], grid.transform.translation[2]);
  const Eigen::Vector3d domain_min = translation + bounds.min.cast<double>() * voxel_size;
  const Eigen::Vector3d domain_max = translation + bounds.max.cast<double>() * voxel_size;
  tvdb_file_close(&file);
  return std::make_shared<SignedDistanceField>(
      Eigen::AlignedBox3d(domain_min, domain_max), dimensions, std::move(distances), scale);
}

SignedDistanceField::Ptr readSignedDistanceFieldVDB(const std::vector<std::uint8_t>& data, const Eigen::Vector3d& scale)
{
  return readSignedDistanceFieldVDB(data.data(), data.size(), scale);
}

std::vector<std::uint8_t> writeSignedDistanceFieldNVDB(const SignedDistanceField& sdf)
{
  const std::vector<std::uint8_t> vdb_data = writeSignedDistanceFieldVDB(sdf);
  tvdb_file_t vdb_file{};
  tvdb_error_t error{};
  tvdb_status_t status = tvdb_file_open_memory(&vdb_file, vdb_data.data(), vdb_data.size(), nullptr, &error);
  if (status != TVDB_OK)
    throwTinyVDBError("writeSignedDistanceFieldNVDB", error);

  status = tvdb_read_all_grids(&vdb_file, &error);
  if (status != TVDB_OK)
  {
    tvdb_file_close(&vdb_file);
    throwTinyVDBError("writeSignedDistanceFieldNVDB", error);
  }

  std::uint8_t* grid_data = nullptr;
  std::size_t grid_size = 0;
  status = tvdb_grid_to_nanovdb_float(&vdb_file.grids[0], &grid_data, &grid_size, &error);
  tvdb_file_close(&vdb_file);
  if (status != TVDB_OK)
    throwTinyVDBError("writeSignedDistanceFieldNVDB", error);

  const Eigen::Vector3i& dimensions = sdf.getDimensions();
  const Eigen::Vector3d voxel_size =
      (sdf.getDomain().max() - sdf.getDomain().min()).cwiseQuotient((dimensions.array() - 1).cast<double>().matrix());
  std::string grid_name = "signed_distance";
  tvdb_nanovdb_grid_t grid{};
  grid.name = grid_name.data();
  grid.size = grid_size;
  grid.grid_type = TVDB_NANOVDB_GRID_TYPE_FLOAT;
  grid.grid_class = TVDB_NANOVDB_GRID_CLASS_LEVEL_SET;
  grid.active_voxel_count = sdf.getDistances().size();
  for (int axis = 0; axis < 3; ++axis)
  {
    grid.voxel_size[axis] = voxel_size.x();
    grid.world_bbox_min[axis] = sdf.getDomain().min()[axis];
    grid.world_bbox_max[axis] = sdf.getDomain().max()[axis];
    grid.index_bbox_min[axis] = 0;
    grid.index_bbox_max[axis] = dimensions[axis] - 1;
    grid.map[4 * axis + axis] = voxel_size.x();
    grid.map[4 * axis + 3] = sdf.getDomain().min()[axis];
  }
  grid.data = grid_data;

  tvdb_nanovdb_file_t file{};
  file.grid_count = 1;
  file.num_grids = 1;
  file.grids = &grid;
  std::uint8_t* data = nullptr;
  std::size_t size = 0;
  status = tvdb_nanovdb_write_to_memory(&file, TVDB_NANOVDB_CODEC_NONE, &data, &size, &error);
  const std::unique_ptr<std::uint8_t, decltype(&std::free)> grid_data_owner(grid_data, &std::free);
  if (status != TVDB_OK)
    throwTinyVDBError("writeSignedDistanceFieldNVDB", error);

  const std::unique_ptr<std::uint8_t, decltype(&std::free)> data_owner(data, &std::free);
  std::vector<std::uint8_t> result(data, data + size);
  return result;
}

SignedDistanceField::Ptr readSignedDistanceFieldNVDB(const std::uint8_t* data,
                                                     std::size_t size,
                                                     const Eigen::Vector3d& scale)
{
  if (data == nullptr || size == 0)
    throw std::runtime_error("readSignedDistanceFieldNVDB: input is empty");

  tvdb_nanovdb_file_t file{};
  tvdb_error_t error{};
  const tvdb_status_t status = tvdb_nanovdb_file_open_memory(&file, data, size, nullptr, &error);
  if (status != TVDB_OK)
    throwTinyVDBError("readSignedDistanceFieldNVDB", error);

  if (file.num_grids != 1 || file.grids[0].grid_type != TVDB_NANOVDB_GRID_TYPE_FLOAT)
  {
    tvdb_nanovdb_file_close(&file);
    throw std::runtime_error("readSignedDistanceFieldNVDB: exactly one FloatGrid is required");
  }

  const tvdb_nanovdb_grid_t& grid = file.grids[0];
  const Eigen::Vector3d voxel_size(grid.voxel_size[0], grid.voxel_size[1], grid.voxel_size[2]);
  if (!voxel_size.allFinite() || voxel_size.minCoeff() <= 0.0 ||
      !voxel_size.isApprox(Eigen::Vector3d::Constant(voxel_size.x()), 1e-12))
  {
    tvdb_nanovdb_file_close(&file);
    throw std::runtime_error("readSignedDistanceFieldNVDB: only axis-aligned uniform transforms are supported");
  }

  const Eigen::Vector3i bounds_min(grid.index_bbox_min[0], grid.index_bbox_min[1], grid.index_bbox_min[2]);
  const Eigen::Vector3i bounds_max(grid.index_bbox_max[0], grid.index_bbox_max[1], grid.index_bbox_max[2]);
  const Eigen::Vector3i dimensions = bounds_max - bounds_min + Eigen::Vector3i::Ones();
  const std::size_t voxel_count = static_cast<std::size_t>(dimensions.x()) * static_cast<std::size_t>(dimensions.y()) *
                                  static_cast<std::size_t>(dimensions.z());
  if (dimensions.minCoeff() < 2 || voxel_count > MAX_SDF_VOXELS)
  {
    tvdb_nanovdb_file_close(&file);
    throw std::runtime_error("readSignedDistanceFieldNVDB: active bounds are unsupported or exceed the voxel limit");
  }

  std::vector<double> distances;
  distances.reserve(voxel_count);
  for (int k = bounds_min.z(); k <= bounds_max.z(); ++k)
    for (int j = bounds_min.y(); j <= bounds_max.y(); ++j)
      for (int i = bounds_min.x(); i <= bounds_max.x(); ++i)
        distances.push_back(tvdb_nanovdb_get_voxel_f(&grid, i, j, k));

  const Eigen::Vector3d translation(grid.map[3], grid.map[7], grid.map[11]);
  const Eigen::Vector3d domain_min = translation + bounds_min.cast<double>() * voxel_size.x();
  const Eigen::Vector3d domain_max = translation + bounds_max.cast<double>() * voxel_size.x();
  tvdb_nanovdb_file_close(&file);
  return std::make_shared<SignedDistanceField>(
      Eigen::AlignedBox3d(domain_min, domain_max), dimensions, std::move(distances), scale);
}

SignedDistanceField::Ptr readSignedDistanceFieldNVDB(const std::vector<std::uint8_t>& data,
                                                     const Eigen::Vector3d& scale)
{
  return readSignedDistanceFieldNVDB(data.data(), data.size(), scale);
}

}  // namespace tesseract::geometry
