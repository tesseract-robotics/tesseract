/**
 * @file tesseract_kinematics_kdl_python.i
 * @brief The tesseract_kinematics_kdl_python SWIG master file.
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

%module(directors="1", package="tesseract.tesseract_kinematic_kdl") tesseract_kinematics_kdl_python

#pragma SWIG nowarn=473

%include "tesseract_swig_include.i"

//%import "tesseract_common_python.i"
//%import "tesseract_geometry_python.i"
%import "tesseract_kinematics_python.i"

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
//#include <tesseract_kinematics/ikfast/ikfast_inv_kin.h>
#include <tesseract_kinematics/kdl/kdl_fwd_kin_chain.h>
#include <tesseract_kinematics/kdl/kdl_fwd_kin_chain_factory.h>
#include <tesseract_kinematics/kdl/kdl_fwd_kin_tree.h>
#include <tesseract_kinematics/kdl/kdl_fwd_kin_tree_factory.h>
#include <tesseract_kinematics/kdl/kdl_inv_kin_chain_lma.h>
#include <tesseract_kinematics/kdl/kdl_inv_kin_chain_lma_factory.h>
#include <tesseract_kinematics/kdl/kdl_inv_kin_chain_nr.h>
#include <tesseract_kinematics/kdl/kdl_inv_kin_chain_nr_factory.h>

%}

%ignore calcJacobian;

// tesseract_kinematics
#define TESSERACT_KINEMATICS_CORE_PUBLIC
#define TESSERACT_KINEMATICS_IKFAST_PUBLIC
#define TESSERACT_KINEMATICS_KDL_PUBLIC
#define TESSERACT_KINEMATICS_OPW_PUBLIC

%include "tesseract_kinematics/kdl/kdl_fwd_kin_chain.h"
%include "tesseract_kinematics/kdl/kdl_fwd_kin_chain_factory.h"
%include "tesseract_kinematics/kdl/kdl_fwd_kin_tree.h"
%include "tesseract_kinematics/kdl/kdl_fwd_kin_tree_factory.h"
%include "tesseract_kinematics/kdl/kdl_inv_kin_chain_lma.h"
%include "tesseract_kinematics/kdl/kdl_inv_kin_chain_lma_factory.h"
%include "tesseract_kinematics/kdl/kdl_inv_kin_chain_nr.h"
%include "tesseract_kinematics/kdl/kdl_inv_kin_chain_nr_factory.h"
