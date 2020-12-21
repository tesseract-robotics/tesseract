/**
 * @file tesseract_motion_planners_descartes_python.i
 * @brief The tesseract_motion_planners_descartes_python SWIG master file.
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

%module(directors="1", package="tesseract.tesseract_motion_planners_descartes") tesseract_motion_planners_descartes_python

#pragma SWIG nowarn=473

%include "tesseract_swig_include.i"
%include "tesseract_std_function.i"

%import "tesseract_motion_planners_python.i"

%{
// tesseract_motion_planner_descartes
#include <tesseract_motion_planners/descartes/descartes_problem.h>
#include <tesseract_motion_planners/descartes/descartes_motion_planner_status_category.h>
#include <tesseract_motion_planners/descartes/profile/descartes_profile.h>
#include <tesseract_motion_planners/descartes/profile/descartes_default_plan_profile.h>
#include <tesseract_motion_planners/descartes/descartes_motion_planner.h>
#include <tesseract_motion_planners/descartes/problem_generators/default_problem_generator.h>
#include <tesseract_motion_planners/descartes/serialize.h>
#include <tesseract_motion_planners/descartes/deserialize.h>

#include <tesseract_common/status_code.h>
#include <tesseract_geometry/geometries.h>
#include <tesseract_common/resource.h>

#include "tesseract_command_language_python_std_functions.h"

#include "tesseract_environment_python_std_functions.h"
#include <tesseract_kinematics/core/rep_inverse_kinematics.h>
#include <tesseract_kinematics/core/rop_inverse_kinematics.h>
%}

// tesseract_motion_planner_descartes
#define TESSERACT_MOTION_PLANNERS_DESCARTES_PUBLIC

%include "tesseract_motion_planners/descartes/descartes_utils.h"
%tesseract_std_function_base(PoseSamplerFn,tesseract_planning,tesseract_common::VectorIsometry3d,const Eigen::Isometry3d&,a);
%tesseract_std_function(PoseSamplerFn,tesseract_planning,tesseract_common::VectorIsometry3d,const Eigen::Isometry3d&,a);
%include "tesseract_motion_planners/descartes/descartes_problem.h"
%include "tesseract_motion_planners/descartes/descartes_motion_planner_status_category.h"
%include "tesseract_motion_planners/descartes/profile/descartes_profile.h"
%include "tesseract_motion_planners/descartes/profile/descartes_default_plan_profile.h"

%tesseract_std_function_base(DescartesProblemGeneratorFnD,tesseract_planning,std::shared_ptr<tesseract_planning::DescartesProblem<double>>,const std::string&,a,const tesseract_planning::PlannerRequest&,b,const tesseract_planning::DescartesPlanProfileMapD&,c);
%tesseract_std_function(DescartesProblemGeneratorFnD,tesseract_planning,std::shared_ptr<tesseract_planning::DescartesProblem<double>>,const std::string&,a,const tesseract_planning::PlannerRequest&,b,const tesseract_planning::DescartesPlanProfileMapD&,c);

%include "tesseract_motion_planners/descartes/descartes_motion_planner.h"
//%include "tesseract_motion_planners/descartes/problem_generators/default_problem_generator.h"

%inline
{
std::shared_ptr<tesseract_planning::DescartesProblem<double>>
DefaultDescartesProblemGeneratorD(const std::string& name,
                                 const tesseract_planning::PlannerRequest& request,
                                 const tesseract_planning::DescartesPlanProfileMapD& plan_profiles)
{
    return tesseract_planning::DefaultDescartesProblemGenerator<double>(name,request,plan_profiles);
}
}

%include "tesseract_motion_planners/descartes/serialize.h"
%include "tesseract_motion_planners/descartes/deserialize.h"
