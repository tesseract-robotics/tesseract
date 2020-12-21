/**
 * @file tesseract_visualization_python.i
 * @brief The tesseract_visualization_python SWIG master file.
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

%module(directors="1", package="tesseract.tesseract_visualization") tesseract_visualization_python

#pragma SWIG nowarn=473

%include "tesseract_swig_include.i"

%import "tesseract_environment_python.i"
%import "tesseract_command_language_python.i"

%{
// tesseract_visualization
#include <tesseract_visualization/visualization.h>

#include <tesseract_common/status_code.h>
#include <tesseract_geometry/geometries.h>

#include <tesseract_scene_graph/resource_locator.h>

#include "tesseract_environment_python_std_functions.h"
#include <tesseract_kinematics/core/rep_inverse_kinematics.h>
#include <tesseract_kinematics/core/rop_inverse_kinematics.h>
%}

// tesseract_visualization
#define TESSERACT_VISUALIZATION_PUBLIC
%include "tesseract_visualization/visualization.h"


