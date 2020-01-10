/**
 * @file geometries.i
 * @brief SWIG interface file for tesseract_environment/geometries.h
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

//Automatic downcasting on return

%include "geometry.i"
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
	tesseract_geometry::SDFMesh,
	tesseract_geometry::Sphere
)

%include "impl/box.i"
%include "impl/capsule.i"
%include "impl/cone.i"
%include "impl/convex_mesh.i"
%include "impl/cylinder.i"
%include "impl/mesh.i"
%include "impl/octree.i"
%include "impl/plane.i"
%include "impl/sdf_mesh.i"
%include "impl/sphere.i"
