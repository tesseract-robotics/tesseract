/**
 * @file convex_decomposition_hacd.h
 * @brief Convex decomposition HACD implementation
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
#ifndef TESSERACT_COLLISION_CONVEX_DECOMPOSITION_HACD_H
#define TESSERACT_COLLISION_CONVEX_DECOMPOSITION_HACD_H

#include <tesseract_collision/core/convex_decomposition.h>

namespace tesseract_collision
{
struct HACDParameters
{
  double compacity_weight{ 0.1 };
  double volume_weight{ 0.0 };
  double concavity{ 0.001 };
  uint32_t max_num_vertices_per_ch{ 256 };
  uint32_t min_num_clusters{ 2 };
  bool add_extra_dist_points{ false };
  bool add_neighbours_dist_points{ false };
  bool add_faces_points{ false };

  void print() const;
};

class ConvexDecompositionHACD : public ConvexDecomposition
{
public:
  using Ptr = std::shared_ptr<ConvexDecompositionHACD>;
  using ConstPtr = std::shared_ptr<const ConvexDecompositionHACD>;

  ConvexDecompositionHACD() = default;
  ConvexDecompositionHACD(const HACDParameters& params);

  std::vector<tesseract_geometry::ConvexMesh::Ptr> compute(const tesseract_common::VectorVector3d& vertices,
                                                           const Eigen::VectorXi& faces) const override;

private:
  HACDParameters params_;
};

}  // namespace tesseract_collision

#endif  // TESSERACT_COLLISION_CONVEX_DECOMPOSITION_HACD_H
