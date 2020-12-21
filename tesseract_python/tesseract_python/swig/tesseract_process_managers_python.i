/**
 * @file tesseract_motion_planners_trajopt_python.i
 * @brief The tesseract_motion_planners_trajopt_python SWIG master file.
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

%module(directors="1", package="tesseract.tesseract_process_managers") tesseract_process_managers_python

#pragma SWIG nowarn=473

%include "tesseract_swig_include.i"

%import "tesseract_motion_planners_simple_python.i"
%import "tesseract_motion_planners_trajopt_python.i"
%import "tesseract_motion_planners_ompl_python.i"
%import "tesseract_motion_planners_descartes_python.i"
%import "tesseract_time_parameterization_python.i"

%{

// tesseract_motion_planners
#include <tesseract_motion_planners/core/profile_dictionary.h>

// tesseract_motion_planners_simple
#include <tesseract_motion_planners/simple/profile/simple_planner_profile.h>
#include <tesseract_motion_planners/simple/profile/simple_planner_default_plan_profile.h>
#include <tesseract_motion_planners/simple/profile/simple_planner_default_lvs_plan_profile.h>
#include <tesseract_motion_planners/simple/profile/simple_planner_interpolation_plan_profile.h>
#include <tesseract_motion_planners/simple/step_generators/fixed_size_assign_position.h>
#include <tesseract_motion_planners/simple/step_generators/fixed_size_interpolation.h>
#include <tesseract_motion_planners/simple/step_generators/lvs_interpolation.h>
#include <tesseract_motion_planners/simple/simple_motion_planner.h>

// tesseract_motion_planners_trajopt
#include <tesseract_motion_planners/trajopt/trajopt_motion_planner.h>
#include <tesseract_motion_planners/trajopt/trajopt_collision_config.h>
#include <tesseract_motion_planners/trajopt/trajopt_utils.h>
#include <tesseract_motion_planners/trajopt/profile/trajopt_profile.h>
#include <tesseract_motion_planners/trajopt/profile/trajopt_default_composite_profile.h>
#include <tesseract_motion_planners/trajopt/profile/trajopt_default_plan_profile.h>
#include <tesseract_motion_planners/trajopt/profile/trajopt_default_solver_profile.h>
#include <tesseract_motion_planners/trajopt/problem_generators/default_problem_generator.h>
#include <tesseract_motion_planners/trajopt/serialize.h>
#include <tesseract_motion_planners/trajopt/deserialize.h>

// tesseract_motion_planners_ompl
#include <tesseract_motion_planners/ompl/ompl_planner_configurator.h>
#include <tesseract_motion_planners/ompl/ompl_problem.h>
#include <tesseract_motion_planners/ompl/ompl_motion_planner_status_category.h>
#include <tesseract_motion_planners/ompl/profile/ompl_profile.h>
#include <tesseract_motion_planners/ompl/profile/ompl_default_plan_profile.h>
#include <tesseract_motion_planners/ompl/ompl_motion_planner.h>
#include <tesseract_motion_planners/ompl/problem_generators/default_problem_generator.h>
#include <tesseract_motion_planners/ompl/serialize.h>
#include <tesseract_motion_planners/ompl/deserialize.h>

// tesseract_motion_planner_descartes
#include <tesseract_motion_planners/descartes/descartes_problem.h>
#include <tesseract_motion_planners/descartes/descartes_motion_planner_status_category.h>
#include <tesseract_motion_planners/descartes/profile/descartes_profile.h>
#include <tesseract_motion_planners/descartes/profile/descartes_default_plan_profile.h>
#include <tesseract_motion_planners/descartes/descartes_motion_planner.h>
#include <tesseract_motion_planners/descartes/problem_generators/default_problem_generator.h>
#include <tesseract_motion_planners/descartes/serialize.h>
#include <tesseract_motion_planners/descartes/deserialize.h>

// tesseract_process_managers

#include <tesseract_process_managers/core/process_interface.h>
#include <tesseract_process_managers/core/process_info.h>
#include <tesseract_process_managers/core/process_planning_request.h>
#include <tesseract_process_managers/core/process_planning_future.h>
#include <tesseract_process_managers/core/process_planning_server.h>

#include <tesseract_process_managers/process_generators/profile_switch_process_generator.h>
#include <tesseract_process_managers/process_generators/iterative_spline_parameterization_process_generator.h>

#include <tesseract_common/status_code.h>
#include <tesseract_geometry/geometries.h>
#include <tesseract_common/resource.h>

#include "tesseract_command_language_python_std_functions.h"

#include "tesseract_environment_python_std_functions.h"
#include <tesseract_kinematics/core/rep_inverse_kinematics.h>
#include <tesseract_kinematics/core/rop_inverse_kinematics.h>

%}

%define %tesseract_process_managers_add_profile_type( TYPE )
%template(hasProfileEntry_##TYPE) tesseract_planning::ProfileDictionary::hasProfileEntry<tesseract_planning::TYPE>;
%template(removeProfileEntry_##TYPE) tesseract_planning::ProfileDictionary::removeProfileEntry<tesseract_planning::TYPE>;
%template(getProfileEntry_##TYPE) tesseract_planning::ProfileDictionary::getProfileEntry<tesseract_planning::TYPE>;
%template(addProfile_##TYPE) tesseract_planning::ProfileDictionary::addProfile<tesseract_planning::TYPE>;
%template(getProfile_##TYPE) tesseract_planning::ProfileDictionary::getProfile<tesseract_planning::TYPE>;
%template(hasProfile_##TYPE) tesseract_planning::ProfileDictionary::hasProfile<tesseract_planning::TYPE>;
%template(removeProfile_##TYPE) tesseract_planning::ProfileDictionary::removeProfile<tesseract_planning::TYPE>;
%enddef

%define %tesseract_process_managers_add_profile_type2(NAME, TYPE )
%template(hasProfileEntry_##NAME) tesseract_planning::ProfileDictionary::hasProfileEntry<tesseract_planning::TYPE>;
%template(removeProfileEntry_##NAME) tesseract_planning::ProfileDictionary::removeProfileEntry<tesseract_planning::TYPE>;
%template(getProfileEntry_##NAME) tesseract_planning::ProfileDictionary::getProfileEntry<tesseract_planning::TYPE>;
%template(addProfile_##NAME) tesseract_planning::ProfileDictionary::addProfile<tesseract_planning::TYPE>;
%template(getProfile_##NAME) tesseract_planning::ProfileDictionary::getProfile<tesseract_planning::TYPE>;
%template(hasProfile_##NAME) tesseract_planning::ProfileDictionary::hasProfile<tesseract_planning::TYPE>;
%template(removeProfile_##NAME) tesseract_planning::ProfileDictionary::removeProfile<tesseract_planning::TYPE>;
%enddef

%shared_ptr(tesseract_planning::IterativeSplineParameterizationProfile)
%shared_ptr(tesseract_planning::ProfileSwitchProfile);

%include "tesseract_motion_planners/core/profile_dictionary.h"

%include "tesseract_process_managers/core/process_interface.h"
%include "tesseract_process_managers/core/process_info.h"
%include "tesseract_process_managers/core/process_planning_request.h"
%include "tesseract_process_managers/core/process_planning_future.h"
%include "tesseract_process_managers/core/process_planning_server.h"

//%include "tesseract_process_managers/process_generators/profile_switch_process_generator.h"
//%include "tesseract_process_managers/process_generators/iterative_spline_parameterization_process_generator.h"


