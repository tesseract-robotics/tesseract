/**
 * @file tesseract_collision_python.i
 * @brief The tesseract_collision_python SWIG master file.
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

%module(directors="1", package="tesseract.tesseract_collision_bullet") tesseract_collision_bullet_python

#pragma SWIG nowarn=473

%include "tesseract_swig_include.i"

//%import "tesseract_common_python.i"
//%import "tesseract_geometry_python.i"
%import "tesseract_collision_python.i"

%{
#include <tesseract_common/status_code.h>
// tesseract_collision
#include <tesseract_collision/bullet/bullet_cast_bvh_manager.h>
#include <tesseract_collision/bullet/bullet_cast_simple_manager.h>
#include <tesseract_collision/bullet/bullet_discrete_bvh_manager.h>
#include <tesseract_collision/bullet/bullet_discrete_simple_manager.h>

#include "tesseract_collisions_python_std_functions.h"
%}

// tesseract_collision
#define TESSERACT_COLLISION_BULLET_PUBLIC
%include "tesseract_collision/bullet/bullet_cast_bvh_manager.h"
%include "tesseract_collision/bullet/bullet_cast_simple_manager.h"
%include "tesseract_collision/bullet/bullet_discrete_bvh_manager.h"
%include "tesseract_collision/bullet/bullet_discrete_simple_manager.h"
