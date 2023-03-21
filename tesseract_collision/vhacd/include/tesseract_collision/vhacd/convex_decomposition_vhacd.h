/**
 * @file convex_decomposition_vhacd.h
 * @brief Convex decomposition VHACD implementation
 *
 * @author Levi Armstrong
 * @date June 2, 2021
 * @version TODO
 * @bug No known bugs
 *
 * @copyright Copyright (c) 2021, Southwest Research Institute
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
#ifndef TESSERACT_COLLISION_CONVEX_DECOMPOSITION_VHACD_H
#define TESSERACT_COLLISION_CONVEX_DECOMPOSITION_VHACD_H

#define ENABLE_VHACD_IMPLEMENTATION 1

#include <tesseract_common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <tesseract_collision/vhacd/VHACD.h>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract_collision/core/convex_decomposition.h>

namespace tesseract_collision
{
struct VHACDParameters
{
  /// The maximum number of convex hulls to produce
  uint32_t max_convex_hulls{ 64 };
  /// The voxel resolution to use
  uint32_t resolution{ 400000 };
  /// if the voxels are within 1% of the volume of the hull, we consider this a close enough approximation
  double minimum_volume_percent_error_allowed{ 1 };
  /// The maximum recursion depth
  uint32_t max_recursion_depth{ 10 };
  /// Whether or not to shrinkwrap the voxel positions to the source mesh on output
  bool shrinkwrap{ true };
  /// How to fill the interior of the voxelized mesh
  VHACD::FillMode fill_mode{ VHACD::FillMode::FLOOD_FILL };
  /// The maximum number of vertices allowed in any output convex hull
  uint32_t max_num_vertices_per_ch{ 64 };
  /// Whether or not to run asynchronously, taking advantage of additional cores
  bool async_ACD{ true };
  /// Once a voxel patch has an edge length of less than 4 on all 3 sides, we don't keep recursing
  uint32_t min_edge_length{ 2 };
  /// Whether or not to attempt to split planes along the best location. Experimental feature. False by default.
  bool find_best_plane{ false };

  void print() const;
};

class ConvexDecompositionVHACD : public ConvexDecomposition
{
public:
  using Ptr = std::shared_ptr<ConvexDecompositionVHACD>;
  using ConstPtr = std::shared_ptr<const ConvexDecompositionVHACD>;

  ConvexDecompositionVHACD() = default;
  ConvexDecompositionVHACD(const VHACDParameters& params);

  std::vector<tesseract_geometry::ConvexMesh::Ptr> compute(const tesseract_common::VectorVector3d& vertices,
                                                           const Eigen::VectorXi& faces) const override;

private:
  VHACDParameters params_;
};

}  // namespace tesseract_collision
#endif  // TESSERACT_COLLISION_CONVEX_DECOMPOSITION_VHACD_H
