/**
 * @file tesseract_scene_graph_python.i
 * @brief The tesseract_scene_graph_python SWIG master file.
 *
 * @author John Wason
 * @date December 8, 2020
 * @version TODO
 * @bug No known bugs
 *
 * @copyright Copyright (c) 2020, Wason Technology, LLC
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

%module(directors="1", package="tesseract.tesseract_urdf") tesseract_urdf_python

#pragma SWIG nowarn=473

%include "tesseract_swig_include.i"

//%import "tesseract_common_python.i"
%import "tesseract_scene_graph_python.i"

%{

#include <boost/graph/adjacency_list.hpp>
#include <boost/graph/directed_graph.hpp>
#include <boost/graph/properties.hpp>
#include <boost/graph/depth_first_search.hpp>
#include <boost/graph/breadth_first_search.hpp>

#include <tesseract_common/status_code.h>
#include <tesseract_geometry/geometries.h>

// tesseract_urdf
#include <tesseract_urdf/urdf_parser.h>

%}

%ignore parse(Eigen::Isometry3d& origin,const tinyxml2::XMLElement* xml_element,const int /*version*/);

%include "tesseract_urdf/origin.h"
%include "tesseract_urdf/sphere.h"
%include "tesseract_urdf/box.h"
%include "tesseract_urdf/cylinder.h"
%include "tesseract_urdf/cone.h"
%include "tesseract_urdf/capsule.h"
%include "tesseract_urdf/mesh.h"
%include "tesseract_urdf/convex_mesh.h"
%include "tesseract_urdf/sdf_mesh.h"
%include "tesseract_urdf/octomap.h"
%include "tesseract_urdf/calibration.h"
%include "tesseract_urdf/dynamics.h"
%include "tesseract_urdf/geometry.h"
%include "tesseract_urdf/inertial.h"
%include "tesseract_urdf/joint.h"
%include "tesseract_urdf/limits.h"
%include "tesseract_urdf/material.h"
%include "tesseract_urdf/mimic.h"
%include "tesseract_urdf/octree.h"
%include "tesseract_urdf/point_cloud.h"
%include "tesseract_urdf/safety_controller.h"
%include "tesseract_urdf/collision.h"
%include "tesseract_urdf/visual.h"
%include "tesseract_urdf/link.h"
%include "tesseract_urdf/urdf_parser.h"


%rename("%s") parse;


