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

%module(directors="1", package="tesseract.tesseract_motion_planners_trajopt") tesseract_motion_planners_trajopt_python

#pragma SWIG nowarn=473

%include "tesseract_swig_include.i"
%include "tesseract_std_function.i"

%import "tesseract_motion_planners_python.i"

%{
// trajopt
#include <trajopt/problem_description.hpp>

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

#include <tesseract_common/status_code.h>
#include <tesseract_geometry/geometries.h>
#include <tesseract_common/resource.h>

#include "tesseract_command_language_python_std_functions.h"

#include "tesseract_environment_python_std_functions.h"
#include <tesseract_kinematics/core/rep_inverse_kinematics.h>
#include <tesseract_kinematics/core/rop_inverse_kinematics.h>
%}

// trajopt

// Including trajopt headers is too noisy, use *.i file instead
%include "trajopt/problem_description.i"

%tesseract_std_function_base(TrajOptProblemGeneratorFn,tesseract_planning,std::shared_ptr<trajopt::ProblemConstructionInfo>,const std::string&,a,const tesseract_planning::PlannerRequest&,b,const tesseract_planning::TrajOptPlanProfileMap&,c,const tesseract_planning::TrajOptCompositeProfileMap&,d,const tesseract_planning::TrajOptSolverProfileMap&,e);
%tesseract_std_function(TrajOptProblemGeneratorFn,tesseract_planning,std::shared_ptr<trajopt::ProblemConstructionInfo>,const std::string&,a,const tesseract_planning::PlannerRequest&,b,const tesseract_planning::TrajOptPlanProfileMap&,c,const tesseract_planning::TrajOptCompositeProfileMap&,d,const tesseract_planning::TrajOptSolverProfileMap&,e);

// tesseract_motion_planners_trajopt
#define TESSERACT_MOTION_PLANNERS_TRAJOPT_PUBLIC

%include "tesseract_motion_planners/trajopt/trajopt_collision_config.h"
%include "tesseract_motion_planners/trajopt/profile/trajopt_profile.h"
%include "tesseract_motion_planners/trajopt/profile/trajopt_default_plan_profile.h"
%include "tesseract_motion_planners/trajopt/profile/trajopt_default_composite_profile.h"

%include "tesseract_motion_planners/trajopt/trajopt_utils.h"
%include "tesseract_motion_planners/trajopt/trajopt_motion_planner.h"
%include "tesseract_motion_planners/trajopt/serialize.h"
%include "tesseract_motion_planners/trajopt/deserialize.h"
%include "tesseract_motion_planners/trajopt/problem_generators/default_problem_generator.h"
