/**
 * @file Octree.cpp
 * @brief Tesseract Octree Geometry
 *
 * @author Levi Armstrong
 * @date March 16, 2022
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
#include <octomap/octomap.h>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract/common/utils.h>
#include <tesseract/geometry/impl/octree.h>

namespace tesseract::geometry
{
Octree::Octree(std::shared_ptr<const octomap::OcTree> octree, OctreeSubType sub_type, bool pruned, bool binary_octree)
  : Geometry(GeometryType::OCTREE)
  , octree_(std::move(octree))
  , sub_type_(sub_type)
  , pruned_(pruned)
  , binary_octree_(binary_octree)
{
}

const std::shared_ptr<const octomap::OcTree>& Octree::getOctree() const { return octree_; }

OctreeSubType Octree::getSubType() const { return sub_type_; }

bool Octree::getPruned() const { return pruned_; }

Geometry::Ptr Octree::clone() const { return std::make_shared<Octree>(octree_, sub_type_); }

long Octree::calcNumSubShapes() const
{
  long cnt = 0;
  double occupancy_threshold = octree_->getOccupancyThres();
  for (auto it = octree_->begin(static_cast<unsigned char>(octree_->getTreeDepth())), end = octree_->end(); it != end;
       ++it)
    if (it->getOccupancy() >= occupancy_threshold)
      ++cnt;

  return cnt;
}

bool Octree::isNodeCollapsible(octomap::OcTree& octree, octomap::OcTreeNode* node)
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

bool Octree::pruneNode(octomap::OcTree& octree, octomap::OcTreeNode* node)
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
void Octree::pruneRecurs(octomap::OcTree& octree,
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

void Octree::prune(octomap::OcTree& octree)
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

bool Octree::operator==(const Octree& rhs) const
{
  using namespace tesseract::common;

  bool equal = true;
  equal &= Geometry::operator==(rhs);
  equal &= sub_type_ == rhs.sub_type_;
  equal &= pruned_ == rhs.pruned_;
  equal &= resolution_ == rhs.resolution_;

  // octree_ == rhs.octree_ looks for exact double equality
  equal &= octree_->getTreeDepth() == rhs.octree_->getTreeDepth();                             // tree_depth
  equal &= almostEqualRelativeAndAbs(octree_->getResolution(), rhs.octree_->getResolution());  // resolution
  equal &= octree_->size() == rhs.octree_->size();                                             // tree_size
  equal &= octree_->getNumLeafNodes() == rhs.octree_->getNumLeafNodes();
  equal &= calcNumSubShapes() == rhs.calcNumSubShapes();

  for (auto it = octree_->begin(static_cast<unsigned char>(octree_->getTreeDepth())), end = octree_->end(); it != end;
       ++it)
  {
    const auto coord = it.getCoordinate();
    const auto* node = rhs.octree_->search(coord);

    if (node != nullptr)
    {
      equal &= almostEqualRelativeAndAbs(it->getValue(), node->getValue());
      equal &= almostEqualRelativeAndAbs(it->getOccupancy(), node->getOccupancy());
    }
    else
      return false;
  }

  return equal;
}
bool Octree::operator!=(const Octree& rhs) const { return !operator==(rhs); }

}  // namespace tesseract::geometry
