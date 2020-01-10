/**
 * @file tesseract_ros_python.i
 * @brief The tesseract_ros_python SWIG master file. Contains 
 *        Tesseract functionality that requires ROS.
 *
 * @author John Wason
 * @date November 26, 2019
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

%module(directors="1", package="tesseract_ros") tesseract_ros_python

// Include all headers from primary module
%{
//#include <ros/console.h>
#include <tesseract/tesseract.h>
#include <tesseract_motion_planners/trajopt/trajopt_motion_planner.h>
#include <tesseract_motion_planners/trajopt/config/trajopt_planner_default_config.h>
#include <tesseract_geometry/geometry_loaders.h>
%}


%include "../eigen.i"
%include "../shared_factory.i"
%include "../json_typemaps.i"
%include "../eigen_types.i"

%import "../tesseract_python.i"

%include "rosmsg_typemaps.i"

%include "tesseract_rosutils/utils.i"