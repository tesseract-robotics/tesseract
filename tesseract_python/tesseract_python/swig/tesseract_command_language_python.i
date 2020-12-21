/**
 * @file tesseract_command_language_python.i
 * @brief The tesseract_command_language_python SWIG master file.
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

%module(directors="1", package="tesseract.tesseract_command_language") tesseract_command_language_python

#pragma SWIG nowarn=473

%include "tesseract_swig_include.i"
%include "tesseract_std_function.i"

%import "tesseract_common_python.i"

%{
// tesseract_command_language
#include <tesseract_command_language/core/waypoint.h>
#include <tesseract_command_language/core/instruction.h>
#include <tesseract_command_language/command_language.h>
#include <tesseract_command_language/serialize.h>
#include <tesseract_command_language/deserialize.h>

#include <tesseract_command_language/utils/utils.h>
#include <tesseract_command_language/utils/get_instruction_utils.h>
#include <tesseract_command_language/utils/filter_functions.h>
#include <tesseract_command_language/utils/flatten_utils.h>

#include <tesseract_common/status_code.h>
#include <tesseract_geometry/geometries.h>
#include <tesseract_common/resource.h>

#include "tesseract_command_language_python_std_functions.h"
%}

%include "tesseract_vector_reference_wrapper_instruction_typemaps.i"

// tesseract_command_language
#define TESSERACT_COMMAND_LANGUAGE_PUBLIC

%define %tesseract_erasure_ctor(class_type,inner_type)
%extend tesseract_planning::class_type {
  class_type (tesseract_planning::inner_type && inner_waypoint)
  {
     return new tesseract_planning::class_type (inner_waypoint);
  }
}
%enddef

%define %tesseract_command_language_add_waypoint_type(TYPE)
%template(cast_##TYPE) tesseract_planning::Waypoint::cast<tesseract_planning::TYPE>;
%template(cast_const_##TYPE) tesseract_planning::Waypoint::cast_const<tesseract_planning::TYPE>;
%tesseract_erasure_ctor(Waypoint,TYPE);
%enddef

%define %tesseract_command_language_add_instruction_type(TYPE)
%template(cast_##TYPE) tesseract_planning::Instruction::cast<tesseract_planning::TYPE>;
%template(cast_const_##TYPE) tesseract_planning::Instruction::cast_const<tesseract_planning::TYPE>;
%tesseract_erasure_ctor(Instruction,TYPE);
%enddef

%tesseract_std_function(flattenFilterFn,tesseract_planning,bool,const tesseract_planning::Instruction&,a,const tesseract_planning::CompositeInstruction&,b,bool,c);
%tesseract_std_function(locateFilterFn,tesseract_planning,bool,const tesseract_planning::Instruction&,a,const tesseract_planning::CompositeInstruction&,b,bool,c);

%include "tesseract_command_language/types.h"
%include "tesseract_command_language/core/waypoint.h"
%include "tesseract_command_language/core/instruction.h"
%include "tesseract_command_language/command_language.h"
%include "tesseract_command_language/serialize.h"
%include "tesseract_command_language/deserialize.h"
%include "tesseract_command_language/utils/filter_functions.h"
%include "tesseract_command_language/utils/utils.h"
%include "tesseract_command_language/utils/get_instruction_utils.h"
%include "tesseract_command_language/utils/flatten_utils.h"