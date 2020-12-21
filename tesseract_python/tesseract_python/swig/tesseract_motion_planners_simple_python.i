/**
 * @file tesseract_motion_planners_simple_python.i
 * @brief The tesseract_motion_planners_simple_python SWIG master file.
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

%module(directors="1", package="tesseract.tesseract_motion_planners_simple") tesseract_motion_planners_simple_python

#pragma SWIG nowarn=473

%include "tesseract_swig_include.i"
%include "tesseract_std_function.i"

%import "tesseract_motion_planners_python.i"

%{
// tesseract_motion_planners_simple
#include <tesseract_motion_planners/simple/profile/simple_planner_profile.h>
#include <tesseract_motion_planners/simple/profile/simple_planner_default_plan_profile.h>
#include <tesseract_motion_planners/simple/profile/simple_planner_default_lvs_plan_profile.h>
#include <tesseract_motion_planners/simple/profile/simple_planner_interpolation_plan_profile.h>
#include <tesseract_motion_planners/simple/step_generators/fixed_size_assign_position.h>
#include <tesseract_motion_planners/simple/step_generators/fixed_size_interpolation.h>
#include <tesseract_motion_planners/simple/step_generators/lvs_interpolation.h>
#include <tesseract_motion_planners/simple/simple_motion_planner.h>

#include <tesseract_common/status_code.h>
#include <tesseract_geometry/geometries.h>

#include <tesseract_scene_graph/resource_locator.h>

#include "tesseract_environment_python_std_functions.h"
#include <tesseract_kinematics/core/rep_inverse_kinematics.h>
#include <tesseract_kinematics/core/rop_inverse_kinematics.h>
%}

%tesseract_std_function_base(JointJointStepGenerator,tesseract_planning,tesseract_planning::CompositeInstruction,const tesseract_planning::JointWaypoint&,a,const tesseract_planning::JointWaypoint&,b,const tesseract_planning::PlanInstruction&,c,const tesseract_planning::PlannerRequest&,d,const tesseract_planning::ManipulatorInfo&,e);
%tesseract_std_function(JointJointStepGenerator,tesseract_planning,tesseract_planning::CompositeInstruction,const tesseract_planning::JointWaypoint&,a,const tesseract_planning::JointWaypoint&,b,const tesseract_planning::PlanInstruction&,c,const tesseract_planning::PlannerRequest&,d,const tesseract_planning::ManipulatorInfo&,e);
%tesseract_std_function_base(JointCartStepGenerator,tesseract_planning,tesseract_planning::CompositeInstruction,const tesseract_planning::JointWaypoint&,a,const tesseract_planning::CartesianWaypoint&,b,const tesseract_planning::PlanInstruction&,c,const tesseract_planning::PlannerRequest&,d,const tesseract_planning::ManipulatorInfo&,e);
%tesseract_std_function(JointCartStepGenerator,tesseract_planning,tesseract_planning::CompositeInstruction,const tesseract_planning::JointWaypoint&,a,const tesseract_planning::CartesianWaypoint&,b,const tesseract_planning::PlanInstruction&,c,const tesseract_planning::PlannerRequest&,d,const tesseract_planning::ManipulatorInfo&,e);
%tesseract_std_function_base(CartJointStepGenerator,tesseract_planning,tesseract_planning::CompositeInstruction,const tesseract_planning::CartesianWaypoint&,a,const tesseract_planning::JointWaypoint&,b,const tesseract_planning::PlanInstruction&,c,const tesseract_planning::PlannerRequest&,d,const tesseract_planning::ManipulatorInfo&,e);
%tesseract_std_function(CartJointStepGenerator,tesseract_planning,tesseract_planning::CompositeInstruction,const tesseract_planning::CartesianWaypoint&,a,const tesseract_planning::JointWaypoint&,b,const tesseract_planning::PlanInstruction&,c,const tesseract_planning::PlannerRequest&,d,const tesseract_planning::ManipulatorInfo&,e);
%tesseract_std_function_base(CartCartStepGenerator,tesseract_planning,tesseract_planning::CompositeInstruction,const tesseract_planning::CartesianWaypoint&,a,const tesseract_planning::CartesianWaypoint&,b,const tesseract_planning::PlanInstruction&,c,const tesseract_planning::PlannerRequest&,d,const tesseract_planning::ManipulatorInfo&,e);
%tesseract_std_function(CartCartStepGenerator,tesseract_planning,tesseract_planning::CompositeInstruction,const tesseract_planning::CartesianWaypoint&,a,const tesseract_planning::CartesianWaypoint&,b,const tesseract_planning::PlanInstruction&,c,const tesseract_planning::PlannerRequest&,d,const tesseract_planning::ManipulatorInfo&,e);

// tesseract_motion_planners_simple
#define TESSERACT_MOTION_PLANNERS_SIMPLE_PUBLIC
%include "tesseract_motion_planners/simple/profile/simple_planner_profile.h"
%include "tesseract_motion_planners/simple/profile/simple_planner_default_lvs_plan_profile.h"
%include "tesseract_motion_planners/simple/profile/simple_planner_interpolation_plan_profile.h"
%include "tesseract_motion_planners/simple/profile/simple_planner_default_plan_profile.h"
%include "tesseract_motion_planners/simple/step_generators/fixed_size_assign_position.h"
%include "tesseract_motion_planners/simple/step_generators/fixed_size_interpolation.h"
%include "tesseract_motion_planners/simple/step_generators/lvs_interpolation.h"
%include "tesseract_motion_planners/simple/simple_motion_planner.h"
