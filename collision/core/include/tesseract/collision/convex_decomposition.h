/**
 * @file convex_decomposition.h
 * @brief Convex decomposition interface
 *
 * @author Levi Armstrong
 * @date June 2, 2021
 *
 * @copyright Copyright (c) 2020, Southwest Research Institute
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
#ifndef TESSERACT_COLLISION_CONVEX_DECOMPOSITION_H
#define TESSERACT_COLLISION_CONVEX_DECOMPOSITION_H

#include <memory>
#include <tesseract/common/eigen_types.h>
#include <tesseract/geometry/fwd.h>

namespace tesseract::collision
{
class ConvexDecomposition
{
public:
  using Ptr = std::shared_ptr<ConvexDecomposition>;
  using ConstPtr = std::shared_ptr<const ConvexDecomposition>;

  ConvexDecomposition() = default;
  virtual ~ConvexDecomposition() = default;
  ConvexDecomposition(const ConvexDecomposition&) = default;
  ConvexDecomposition& operator=(const ConvexDecomposition&) = default;
  ConvexDecomposition(ConvexDecomposition&&) = default;
  ConvexDecomposition& operator=(ConvexDecomposition&&) = default;

  /**
   * @brief Run convex decomposition algorithm
   * @param vertices The vertices
   * @param faces A vector of triangle indicies. Every face starts with the number of vertices followed the the vertice
   * index
   * @return
   */
  virtual std::vector<std::shared_ptr<tesseract::geometry::ConvexMesh>>
  compute(const tesseract::common::VectorVector3d& vertices,
          const Eigen::VectorXi& faces,
          bool verbose = true) const = 0;
};

}  // namespace tesseract::collision

#endif  // TESSERACT_COLLISION_CONVEX_DECOMPOSITION_H
