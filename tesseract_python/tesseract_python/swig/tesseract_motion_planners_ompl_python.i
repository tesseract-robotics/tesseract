/**
 * @file tesseract_motion_planners_ompl_python.i
 * @brief The tesseract_motion_planners_ompl_python SWIG master file.
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

%module(directors="1", package="tesseract.tesseract_motion_planners_ompl") tesseract_motion_planners_ompl_python

#pragma SWIG nowarn=473

%include "tesseract_swig_include.i"
%include "tesseract_std_function.i"

%import "tesseract_motion_planners_python.i"

%{
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

#include <tesseract_common/status_code.h>
#include <tesseract_geometry/geometries.h>
#include <tesseract_common/resource.h>

#include "tesseract_command_language_python_std_functions.h"

#include "tesseract_environment_python_std_functions.h"
#include <tesseract_kinematics/core/rep_inverse_kinematics.h>
#include <tesseract_kinematics/core/rop_inverse_kinematics.h>
%}

// tesseract_motion_planners_ompl
#define TESSERACT_MOTION_PLANNERS_OMPL_PUBLIC
%include "tesseract_motion_planners/ompl/ompl_planner_configurator.h"
%include "tesseract_motion_planners/ompl/ompl_problem.h"
%include "tesseract_motion_planners/ompl/ompl_motion_planner_status_category.h"
%include "tesseract_motion_planners/ompl/profile/ompl_profile.h"
%include "tesseract_motion_planners/ompl/profile/ompl_default_plan_profile.h"

%tesseract_std_function_base(OMPLProblemGeneratorFn,tesseract_planning,std::vector<std::shared_ptr<tesseract_planning::OMPLProblem>>,const std::string&,a,const tesseract_planning::PlannerRequest&,b,const tesseract_planning::OMPLPlanProfileMap&,c);
%tesseract_std_function(OMPLProblemGeneratorFn,tesseract_planning,std::vector<std::shared_ptr<tesseract_planning::OMPLProblem>>,const std::string&,a,const tesseract_planning::PlannerRequest&,b,const tesseract_planning::OMPLPlanProfileMap&,c);

%include "tesseract_motion_planners/ompl/ompl_motion_planner.h"
%include "tesseract_motion_planners/ompl/problem_generators/default_problem_generator.h"
%include "tesseract_motion_planners/ompl/serialize.h"
%include "tesseract_motion_planners/ompl/deserialize.h"
