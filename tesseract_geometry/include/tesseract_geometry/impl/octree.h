/**
 * @file octree.h
 * @brief Tesseract Octree Geometry
 *
 * @author Levi Armstrong
 * @date January 18, 2018
 *
 * @copyright Copyright (c) 2017, Southwest Research Institute
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
#ifndef TESSERACT_GEOMETRY_OCTREE_H
#define TESSERACT_GEOMETRY_OCTREE_H

#include <tesseract_common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <memory>
#include <cassert>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract_geometry/geometry.h>

namespace octomap
{
class OcTree;
class OcTreeNode;
}  // namespace octomap

namespace tesseract::geometry
{
enum class OctreeSubType : std::uint8_t
{
  BOX,
  SPHERE_INSIDE,
  SPHERE_OUTSIDE
};

class Octree;

template <class Archive>
void serialize(Archive& ar, Octree& obj);

class Octree : public Geometry
{
public:
  using Ptr = std::shared_ptr<Octree>;
  using ConstPtr = std::shared_ptr<const Octree>;

  Octree(std::shared_ptr<const octomap::OcTree> octree,
         OctreeSubType sub_type,
         bool pruned = false,
         bool binary_octree = false);
  Octree() = default;
  ~Octree() override = default;

  const std::shared_ptr<const octomap::OcTree>& getOctree() const;

  OctreeSubType getSubType() const;

  bool getPruned() const;

  Geometry::Ptr clone() const override final;

  bool operator==(const Octree& rhs) const;
  bool operator!=(const Octree& rhs) const;

  /**
   * @brief Octrees are typically generated from 3D sensor data so this method
   * should be used to efficiently update the collision shape.
   */
  void update() { assert(false); }  // NOLINT

  /**
   * @brief Calculate the number of sub shapes that would get generated for this octree
   *
   * This is expensive and should not be called multiple times
   *
   * @return number of sub shapes
   */
  long calcNumSubShapes() const;

private:
  std::shared_ptr<const octomap::OcTree> octree_;
  OctreeSubType sub_type_{ OctreeSubType::BOX };
  double resolution_{ 0.01 };
  bool pruned_{ false };
  bool binary_octree_{ false };

  static bool isNodeCollapsible(octomap::OcTree& octree, octomap::OcTreeNode* node);

  static bool pruneNode(octomap::OcTree& octree, octomap::OcTreeNode* node);

  // NOLINTNEXTLINE(misc-no-recursion)
  static void pruneRecurs(octomap::OcTree& octree,
                          octomap::OcTreeNode* node,
                          unsigned int depth,
                          unsigned int max_depth,
                          unsigned int& num_pruned);

  template <class Archive>
  friend void ::tesseract::geometry::serialize(Archive& ar, Octree& obj);

public:
  /**
   * @brief A custom octree prune which will prune if all children are above the occupancy threshold.
   *
   * This is different from the octomap::OcTree::prune which requires all children to have the same
   * occupancy to be collapsed.
   *
   * @param octree The octree to be pruned.
   */
  static void prune(octomap::OcTree& octree);
};
}  // namespace tesseract::geometry

#endif
