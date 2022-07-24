/**
 * @file octree.h
 * @brief Tesseract Octree Geometry
 *
 * @author Levi Armstrong
 * @date January 18, 2018
 * @version TODO
 * @bug No known bugs
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
#include <boost/serialization/access.hpp>
#include <boost/serialization/export.hpp>
#include <Eigen/Geometry>
#include <memory>
#include <octomap/octomap.h>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract_geometry/geometry.h>

namespace tesseract_geometry
{
class Octree : public Geometry
{
public:
  using Ptr = std::shared_ptr<Octree>;
  using ConstPtr = std::shared_ptr<const Octree>;

  enum SubType
  {
    BOX,
    SPHERE_INSIDE,
    SPHERE_OUTSIDE
  };

  Octree(std::shared_ptr<const octomap::OcTree> octree, const SubType sub_type)
    : Geometry(GeometryType::OCTREE), octree_(std::move(octree)), sub_type_(sub_type)
  {
  }

  template <typename PointT>
  Octree(const PointT& point_cloud,
         const double resolution,
         const SubType sub_type,
         const bool prune,
         const bool binary = true)
    : Geometry(GeometryType::OCTREE), sub_type_(sub_type), resolution_(resolution)
  {
    auto ot = std::make_shared<octomap::OcTree>(resolution);

    for (auto& point : point_cloud.points)
      ot->updateNode(point.x, point.y, point.z, true, true);

    // Per the documentation for overload updateNode above with lazy_eval enabled this must be called after all points
    // are added
    ot->updateInnerOccupancy();
    if (binary)
    {
      ot->toMaxLikelihood();
      binary_octree_ = binary;
    }

    if (prune)
    {
      tesseract_geometry::Octree::prune(*ot);
      pruned_ = prune;
    }

    octree_ = ot;
  }

  Octree() = default;
  ~Octree() override = default;

  const std::shared_ptr<const octomap::OcTree>& getOctree() const { return octree_; }

  SubType getSubType() const { return sub_type_; }

  bool getPruned() const { return pruned_; }

  Geometry::Ptr clone() const override final { return std::make_shared<Octree>(octree_, sub_type_); }
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
  long calcNumSubShapes() const
  {
    long cnt = 0;
    double occupancy_threshold = octree_->getOccupancyThres();
    for (auto it = octree_->begin(static_cast<unsigned char>(octree_->getTreeDepth())), end = octree_->end(); it != end;
         ++it)
      if (it->getOccupancy() >= occupancy_threshold)
        ++cnt;

    return cnt;
  }

private:
  std::shared_ptr<const octomap::OcTree> octree_;
  SubType sub_type_{ SubType::BOX };
  double resolution_{ 0.01 };
  bool pruned_{ false };
  bool binary_octree_{ false };

  static bool isNodeCollapsible(octomap::OcTree& octree, octomap::OcTreeNode* node)
  {
    if (!octree.nodeChildExists(node, 0))
      return false;

    double occupancy_threshold = octree.getOccupancyThres();

    const octomap::OcTreeNode* firstChild = octree.getNodeChild(node, 0);
    if (octree.nodeHasChildren(firstChild) || firstChild->getOccupancy() < occupancy_threshold)
      return false;

    for (unsigned int i = 1; i < 8; i++)
    {
      // comparison via getChild so that casts of derived classes ensure
      // that the right == operator gets called
      if (!octree.nodeChildExists(node, i))
        return false;

      if (octree.nodeHasChildren(octree.getNodeChild(node, i)))
        return false;

      if (octree.getNodeChild(node, i)->getOccupancy() < occupancy_threshold)
        return false;
    }

    return true;
  }

  static bool pruneNode(octomap::OcTree& octree, octomap::OcTreeNode* node)
  {
    if (!isNodeCollapsible(octree, node))
      return false;

    // set value to children's values (all assumed equal)
    node->copyData(*(octree.getNodeChild(node, 0)));

    // delete children (known to be leafs at this point!)
    for (unsigned int i = 0; i < 8; i++)
    {
      octree.deleteNodeChild(node, i);
    }

    return true;
  }

  // NOLINTNEXTLINE(misc-no-recursion)
  static void pruneRecurs(octomap::OcTree& octree,
                          octomap::OcTreeNode* node,
                          unsigned int depth,
                          unsigned int max_depth,
                          unsigned int& num_pruned)
  {
    assert(node);

    if (depth < max_depth)
    {
      for (unsigned int i = 0; i < 8; i++)
      {
        if (octree.nodeChildExists(node, i))
        {
          pruneRecurs(octree, octree.getNodeChild(node, i), depth + 1, max_depth, num_pruned);
        }
      }
    }  // end if depth

    else
    {
      // max level reached
      if (pruneNode(octree, node))
      {
        num_pruned++;
      }
    }
  }

  friend class boost::serialization::access;
  template <class Archive>
  void save(Archive& ar, const unsigned int version) const;  // NOLINT

  template <class Archive>
  void load(Archive& ar, const unsigned int version);  // NOLINT

  template <class Archive>
  void serialize(Archive& ar, const unsigned int version);  // NOLINT

public:
  /**
   * @brief A custom octree prune which will prune if all children are above the occupancy threshold.
   *
   * This is different from the octomap::OcTree::prune which requires all children to have the same
   * occupancy to be collapsed.
   *
   * @param octree The octree to be pruned.
   */
  static void prune(octomap::OcTree& octree)
  {
    if (octree.getRoot() == nullptr)
      return;

    for (unsigned int depth = octree.getTreeDepth() - 1; depth > 0; --depth)
    {
      unsigned int num_pruned = 0;
      pruneRecurs(octree, octree.getRoot(), 0, depth, num_pruned);
      if (num_pruned == 0)
        break;
    }
  }
};
}  // namespace tesseract_geometry

#include <boost/serialization/tracking.hpp>
BOOST_CLASS_EXPORT_KEY2(tesseract_geometry::Octree, "Octree")
BOOST_CLASS_TRACKING(tesseract_geometry::Octree, boost::serialization::track_never)
#endif
