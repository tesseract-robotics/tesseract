/**
 * @file geometries.h
 * @brief Tesseract Geometries
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
#ifndef TESSERACT_GEOMETRY_GEOMETRIES_H
#define TESSERACT_GEOMETRY_GEOMETRIES_H

#include <tesseract_geometry/impl/box.h>
#include <tesseract_geometry/impl/capsule.h>
#include <tesseract_geometry/impl/cone.h>
#include <tesseract_geometry/impl/convex_mesh.h>
#include <tesseract_geometry/impl/cylinder.h>
#include <tesseract_geometry/impl/mesh.h>
#include <tesseract_geometry/impl/octree.h>
#include <tesseract_geometry/impl/plane.h>
#include <tesseract_geometry/impl/polygon_mesh.h>
#include <tesseract_geometry/impl/sdf_mesh.h>
#include <tesseract_geometry/impl/sphere.h>

#ifdef SWIG

%shared_factory(
    tesseract_geometry::Geometry,
    tesseract_geometry::Box,
    tesseract_geometry::Capsule,
    tesseract_geometry::Cone,
    tesseract_geometry::ConvexMesh,
    tesseract_geometry::Cylinder,
    tesseract_geometry::Mesh,
    tesseract_geometry::Octree,
    tesseract_geometry::Plane,
    tesseract_geometry::PolygonMesh,
    tesseract_geometry::SDFMesh,
    tesseract_geometry::Sphere
)

%include <tesseract_geometry/impl/box.h>
%include <tesseract_geometry/impl/capsule.h>
%include <tesseract_geometry/impl/cone.h>
%include <tesseract_geometry/impl/convex_mesh.h>
%include <tesseract_geometry/impl/cylinder.h>
%include <tesseract_geometry/impl/mesh.h>
%include <tesseract_geometry/impl/mesh_material.h>
%include <tesseract_geometry/impl/octree.h>
%include <tesseract_geometry/impl/plane.h>
%include <tesseract_geometry/impl/polygon_mesh.h>
%include <tesseract_geometry/impl/sdf_mesh.h>
%include <tesseract_geometry/impl/sphere.h>

#endif  // SWIG

#endif
