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

#include <tesseract_geometry/macros.h>
TESSERACT_GEOMETRY_IGNORE_WARNINGS_PUSH
#include <Eigen/Geometry>
#include <memory>
#include <octomap/octomap.h>
TESSERACT_GEOMETRY_IGNORE_WARNINGS_POP

#include <tesseract_geometry/geometry.h>

namespace tesseract_geometry
{
  class Octree;
  typedef std::shared_ptr<Octree> OctreePtr;
  typedef std::shared_ptr<const Octree> OctreeConstPtr;

  class Octree : public Geometry
  {
  public:
    enum SubType
    {
      BOX,
      SPHERE_INSIDE,
      SPHERE_OUTSIDE
    };

    Octree(const std::shared_ptr<const octomap::OcTree>& octree, SubType sub_type)  : Geometry(GeometryType::OCTREE), octree_(octree), sub_type_(sub_type) {}
    ~Octree() override = default;

    const std::shared_ptr<const octomap::OcTree>& getOctree() const { return octree_; }
    SubType getSubType() const { return sub_type_; }

    GeometryPtr clone() const override { return OctreePtr(new Octree(octree_, sub_type_)); }

    /**
     * @brief Octrees are typically generated from 3D sensor data so this method
     * should be used to efficiently update the collision shape.
     */
    void update() { assert(false); }

  private:
    std::shared_ptr<const octomap::OcTree> octree_;
    SubType sub_type_;
  };
}
#endif
