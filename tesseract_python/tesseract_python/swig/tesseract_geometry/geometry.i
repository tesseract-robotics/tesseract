/**
 * @file geometry.i
 * @brief SWIG interface file for tesseract_environment/geometry.h
 *
 * @author John Wason
 * @date December 10, 2019
 * @version TODO
 * @bug No known bugs
 *
 * @copyright Copyright (c) 2019, Wason Technology, LLC
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

%{
#include <tesseract_geometry/geometry.h>
%}

%shared_ptr(tesseract_geometry::Geometry)

namespace tesseract_geometry
{
enum GeometryType
{
  SPHERE,
  CYLINDER,
  CAPSULE,
  CONE,
  BOX,
  PLANE,
  MESH,
  CONVEX_MESH,
  SDF_MESH,
  OCTREE
};

%nodefaultctor Geometry;
class Geometry
{
public:
  using Ptr = std::shared_ptr<Geometry>;
  using ConstPtr = std::shared_ptr<const Geometry>;

  virtual ~Geometry();

  virtual Geometry::Ptr clone();

  GeometryType getType() const { return type_; }

};

using Geometrys = std::vector<Geometry::Ptr>;
using GeometrysConst = std::vector<Geometry::ConstPtr>;
}  // namespace tesseract_geometry

%template(tesseract_geometry_Geometries) std::vector<std::shared_ptr<tesseract_geometry::Geometry> >;
%template(tesseract_geometry_GeometriesConst) std::vector<std::shared_ptr<const tesseract_geometry::Geometry> >;


