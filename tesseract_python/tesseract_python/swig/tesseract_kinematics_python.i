/**
 * @file tesseract_kinematics_python.i
 * @brief The tesseract_kinematics_python SWIG master file.
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

%module(directors="1", package="tesseract.tesseract_kinematics") tesseract_kinematics_python

#pragma SWIG nowarn=473

%include "tesseract_swig_include.i"

//%import "tesseract_common_python.i"
//%import "tesseract_geometry_python.i"
%import "tesseract_scene_graph_python.i"

%{

#include <boost/graph/adjacency_list.hpp>
#include <boost/graph/directed_graph.hpp>
#include <boost/graph/properties.hpp>
#include <boost/graph/depth_first_search.hpp>
#include <boost/graph/breadth_first_search.hpp>

#include <tesseract_common/status_code.h>
#include <tesseract_geometry/geometries.h>
#include <tesseract_scene_graph/resource_locator.h>

// tesseract_kinematics
#include <tesseract_kinematics/core/forward_kinematics.h>
#include <tesseract_kinematics/core/forward_kinematics_factory.h>
#include <tesseract_kinematics/core/inverse_kinematics.h>
#include <tesseract_kinematics/core/inverse_kinematics_factory.h>
#include <tesseract_kinematics/core/rep_inverse_kinematics.h>
#include <tesseract_kinematics/core/rop_inverse_kinematics.h>

%}

%template(pair_bool_matrix) std::pair<bool,Eigen::MatrixXd>;

%extend tesseract_kinematics::ForwardKinematics {

  bool calcJacobian(Eigen::MatrixXd& jacobian, const Eigen::Ref<const Eigen::VectorXd>& joint_angles) const
  {
    jacobian = Eigen::MatrixXd(6, joint_angles.rows());
    bool res = $self->calcJacobian(jacobian, joint_angles);
    
    return res;
  }

  bool calcJacobian(Eigen::MatrixXd& jacobian, const Eigen::Ref<const Eigen::VectorXd>& joint_angles, const std::string& link_name) const
  {
    jacobian = Eigen::MatrixXd(6, joint_angles.rows());
    bool res = $self->calcJacobian(jacobian, joint_angles, link_name);
    return res;
  }
}

%ignore calcJacobian;

// tesseract_kinematics
#define TESSERACT_KINEMATICS_CORE_PUBLIC
#define TESSERACT_KINEMATICS_IKFAST_PUBLIC
#define TESSERACT_KINEMATICS_KDL_PUBLIC
#define TESSERACT_KINEMATICS_OPW_PUBLIC
%include "tesseract_kinematics/core/forward_kinematics.h"
%include "tesseract_kinematics/core/forward_kinematics_factory.h"
%include "tesseract_kinematics/core/inverse_kinematics.h"
%include "tesseract_kinematics/core/inverse_kinematics_factory.h"
%include "tesseract_kinematics/core/rop_inverse_kinematics.h"
%include "tesseract_kinematics/core/rep_inverse_kinematics.h"


