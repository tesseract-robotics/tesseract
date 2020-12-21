/**
 * @file tesseract_common_python.i
 * @brief The tesseract_common_python SWIG master file.
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

%module(directors="1", package="tesseract.tesseract_common") tesseract_common_python

#pragma SWIG nowarn=473

%{
//#define SWIG_PYTHON_EXTRA_NATIVE_CONTAINERS
%}

%include "tesseract_swig_include.i"

%{

// tesseract_common
#include <tesseract_common/types.h>
#include <tesseract_common/status_code.h>
#include <tesseract_common/resource.h>
#include <tesseract_common/manipulator_info.h>

%}

%include "tinyxml2.i"
%include "boost_filesystem_path.i"
%include "eigen_geometry.i"

%pythondynamic sco::ModelType;

%template(vector_string) std::vector<std::string>;
%template(pair_string) std::pair<std::string, std::string>;
%template(vector_pair_string) std::vector<std::pair<std::string, std::string> >;
%template(map_string_vector_pair_string) std::unordered_map<std::string, std::vector<std::pair<std::string, std::string>>>;
%template(pair_vector_string) std::pair<std::vector<std::string>, std::vector<std::string>>;

%template(vector_double) std::vector<double>;
%template(map_string_vector_double) std::unordered_map<std::string, std::vector<double> >;
%template(map_string_double) std::unordered_map<std::string, double>;
%template(map_string_map_string_double) std::unordered_map<std::string, std::unordered_map<std::string, double> >;
%template(map_string_map_string_string) std::unordered_map<std::string, std::unordered_map<std::string, std::string> >;
%template(map_string_map_string_map_string_double) std::unordered_map<std::string, std::unordered_map<std::string, std::unordered_map<std::string, double> > >;

%template(array2_int) std::array<int,2>;
%template(array2_string) std::array<std::string,2>;
%template(array2_Vector3d) std::array<Eigen::Vector3d,2>;
%template(array2_Isometry3d) std::array<Eigen::Isometry3d,2>;
%template(array2_double) std::array<double,2>;

%define %tesseract_aligned_vector(name,T)
%template(name) std::vector<T , Eigen::aligned_allocator<T >>;
%enddef

%define %tesseract_aligned_map(name,Key,Value)
%template(name) std::map<Key, Value, std::less<Key>, Eigen::aligned_allocator<std::pair<const Key, Value>>>;
%enddef

%define %tesseract_aligned_map_of_aligned_vector(name,Key,Value)
%tesseract_aligned_map(name, %arg(Key), %arg(std::vector<Value , Eigen::aligned_allocator<Value >>));
%enddef

%define %tesseract_aligned_unordered_map(name,Key,Value)
%template(name) std::unordered_map<Key,Value,std::hash<Key>,std::equal_to<Key>,Eigen::aligned_allocator<std::pair<const Key, Value>>>;
%enddef

%tesseract_aligned_vector(VectorIsometry3d, Eigen::Isometry3d);
%template(VectorVector3d) std::vector<Eigen::Vector3d>;
%tesseract_aligned_vector(VectorVector4d, Eigen::Vector4d);
%tesseract_aligned_map(TransformMap, std::string, Eigen::Isometry3d);

// SWIG is not smart enough to expand templated using, override the behavior
%define %tesseract_aligned_vector_using(name,T)
using name = std::vector<T , Eigen::aligned_allocator<T >>;
%enddef

%define %tesseract_aligned_map_using(name,Key,Value)
using name = std::map<Key, Value, std::less<Key>, Eigen::aligned_allocator<std::pair<const Key, Value>>>;
%enddef

%define %tesseract_aligned_map_of_aligned_vector_using(name,Key,Value)
%tesseract_aligned_map_using(name, %arg(Key), %arg(std::vector<Value , Eigen::aligned_allocator<Value >>));
%enddef

namespace tesseract_common
{
%tesseract_aligned_vector_using(VectorIsometry3d, Eigen::Isometry3d);
%tesseract_aligned_vector_using(VectorVector4d, Eigen::Vector4d);
%tesseract_aligned_map_using(TransformMap, std::string, Eigen::Isometry3d);
}

%ignore toXML(tinyxml2::XMLDocument& doc) const;

%typemap(out, fragment="SWIG_From_std_string") std::string& {
  $result = SWIG_From_std_string(*$1);
}

%typemap(out, fragment="SWIG_From_std_string") const std::string& {
  $result = SWIG_From_std_string(*$1);
}

#define EIGEN_MAKE_ALIGNED_OPERATOR_NEW
#define TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#define TESSERACT_COMMON_IGNORE_WARNINGS_POP
#define DEPRECATED(msg)

// tesseract_common
#define TESSERACT_COMMON_PUBLIC
%include "tesseract_common/types.h"
%include "tesseract_common/status_code.h"
%include "tesseract_common/resource.h"
%include "tesseract_common/manipulator_info.h"




