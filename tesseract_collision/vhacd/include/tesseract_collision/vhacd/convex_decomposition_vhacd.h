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

#include <tesseract_collision/core/convex_decomposition.h>

namespace tesseract_collision
{
struct VHACDParameters
{
  double concavity{ 0.001 };
  double alpha{ 0.05 };
  double beta{ 0.05 };
  double min_volume_per_ch{ 0.0001 };
  uint32_t resolution{ 1000 };
  uint32_t max_num_vertices_per_ch{ 256 };
  uint32_t plane_downsampling{ 4 };
  uint32_t convexhull_downsampling{ 4 };
  uint32_t pca{ 0 };
  uint32_t mode{ 0 };  // 0: voxel-based (recommended), 1: tetrahedron-based
  uint32_t convexhull_approximation{ 1U };
  uint32_t ocl_acceleration{ 1U };
  uint32_t max_convehulls{ 1024 };
  /**
   * @brief This will project the output convex hull vertices onto the original source mesh
   *  to increase the floating point accuracy of the results
   */
  bool project_hull_vertices{ true };

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
