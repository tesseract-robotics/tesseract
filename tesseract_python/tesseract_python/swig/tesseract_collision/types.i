/**
 * @file types.i
 * @brief SWIG interface file for tesseract_collision/types.h
 *
 * @author John Wason
 * @date December 10, 2019
 * @version TODO
 * @bug No known bugs
 *
 * @copyright Copyright (c) 2019, Wason Technology, LLC
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

%{
#include <tesseract_collision/core/types.h>
%}

// Custom typemaps for ContactResult

// std::string[2]
%typemap(out) std::string [2]
{
  // Convert std::string [2] to tuple

  $result = Py_BuildValue("(s#s#)", $1[0].c_str(), $1[0].size(), $1[1].c_str(), $1[1].size());
  if (!$result)
    SWIG_fail;
}

%typemap(in) std::string [2] (std::array<std::string,2> temp)
{
  // Convert tuple to std::string [2]
  const char* str1;
  size_t str1_len;
  const char* str2;
  size_t str2_len;
  if (!PyArg_ParseTuple($input,"(#s,#s)", &str1, &str1_len, &str2, &str2_len))
    SWIG_fail;

  temp[0] = std::string(str1, str1_len);
  temp[1] = std::string(str2, str2_len);
  $1 = &temp[0];
}

// int[2]
%typemap(out) int [2]
{
  // Convert int [2] to tuple

  $result = Py_BuildValue("(ii)", $1[0], $1[1]);
  if (!$result)
    SWIG_fail;
}

%typemap(in) int [2] (std::array<int,2> temp)
{
  // Convert tuple to int [2]
  
  if (!PyArg_ParseTuple($input,"(i,i)", &temp[0], &temp[1]))
    SWIG_fail;

  $1[0] = temp[0];
  $1[1] = temp[1];
}

//Eigen::Vector3d[2]
%typemap(out, fragment="Eigen_Fragments") Eigen::Vector3d [2]  (PyObject* v1, PyObject* v2, PyObject* res)
{
  // Convert Eigen::Vector3d[2] to tuple
  res = PyTuple_New(2);
  if (!ConvertFromEigenToNumPyMatrix<Eigen::Vector3d >(&v1, &$1[0]))
  {
    Py_XDECREF(res);
    SWIG_fail;
  }
  PyTuple_SetItem(res, 0, v1);
  if (!ConvertFromEigenToNumPyMatrix<Eigen::Vector3d >(&v2, &$1[1]))
  {
    Py_XDECREF(res);
    SWIG_fail;  
  }
  
  PyTuple_SetItem(res, 1, v2);
  $result = res;
}

%typemap(in, fragment="Eigen_Fragments") Eigen::Vector3d [2]  (std::array<Eigen::Vector3d,2> temp, PyObject* v1, PyObject* v2)
{
  // Convert from tuple to Eigen::Vector3d[2]
  if (!PyTuple_Check($input))
    SWIG_fail;
  if (PyTuple_Size($input) != 2)
    SWIG_fail;
  v1 = PyTuple_GetItem($input, 0);
  v2 = PyTuple_GetItem($input, 1);

  if (!ConvertFromNumpyToEigenMatrix<Eigen::Vector3d>(&temp[0], v1))
    SWIG_fail;
  if (!ConvertFromNumpyToEigenMatrix<Eigen::Vector3d>(&temp[1], v2))
    SWIG_fail;  
  $1[0] = temp[0];
  $1[1] = temp[1];
}

//Eigen::Isometry3d[2]
%typemap(out, fragment="Eigen_Fragments") Eigen::Eigen::Matrix4d [2]  (PyObject* v1, PyObject* v2, PyObject* res)
{
  // Convert Eigen::Isometry3d[2] to tuple
  res = PyTuple_New(2);
  if (!ConvertFromEigenToNumPyMatrix<Eigen::Matrix4d >(&v1, &$1[0]))
  {
    Py_XDECREF(res);
    SWIG_fail;
  }
  PyTuple_SetItem(res, 0, v1);
  if (!ConvertFromEigenToNumPyMatrix<Eigen::Matrix4d >(&v2, &$1[1]))
  {
    Py_XDECREF(res);
    SWIG_fail;  
  }
  
  PyTuple_SetItem(res, 1, v2);
  $result = res;
}

%typemap(in, fragment="Eigen_Fragments") Eigen::Isometry3d [2]  (std::array<Eigen::Matrix4d,2> temp, PyObject* v1, PyObject* v2)
{
  // Convert from tuple to Eigen::Isometry3d[2]
  if (!PyTuple_Check($input))
    SWIG_fail;
  if (PyTuple_Size($input) != 2)
    SWIG_fail;
  v1 = PyTuple_GetItem($input, 0);
  v2 = PyTuple_GetItem($input, 1);

  if (!ConvertFromNumpyToEigenMatrix<Eigen::Matrix4d>(&temp[0], v1))
    SWIG_fail;
  if (!ConvertFromNumpyToEigenMatrix<Eigen::Matrix4d>(&temp[1], v2))
    SWIG_fail;  
  $1[0] = temp[0];
  $1[1] = temp[1];
}

namespace tesseract_collision
{
using CollisionShapesConst = std::vector<tesseract_geometry::Geometry::ConstPtr>;
using CollisionShapeConstPtr = tesseract_geometry::Geometry::ConstPtr;
using CollisionShapePtr = tesseract_geometry::Geometry::Ptr;

enum class ContinuousCollisionType
{
  CCType_None,
  CCType_Time0,
  CCType_Time1,
  CCType_Between
};

enum class ContactTestType
{
  FIRST = 0,
  CLOSEST = 1,
  ALL = 2,
  LIMITED = 3
};

struct ContactResult
{
  double distance;
  int type_id[2];
  std::string link_names[2];
  int shape_id[2];
  int subshape_id[2];
  Eigen::Vector3d nearest_points[2];
  Eigen::Vector3d nearest_points_local[2];
  Eigen::Isometry3d transform[2];
  Eigen::Vector3d normal;  
  double cc_time[2];
  //std::array<ContinuousCollisionType, 2> cc_type;
  Eigen::Isometry3d cc_transform[2];

  ContactResult();
  void clear();
};

// TODO: These using statements are not working
//using ContactResultVector = tesseract_common::AlignedVector<ContactResult>;
//using ContactResultMap = tesseract_common::AlignedMap<std::pair<std::string, std::string>, ContactResultVector>;
using ContactResultVector = std::vector< tesseract_collision::ContactResult,Eigen::aligned_allocator< tesseract_collision::ContactResult > >;
using ContactResultMap = std::map< std::pair< std::string,std::string >,std::vector< tesseract_collision::ContactResult,Eigen::aligned_allocator< tesseract_collision::ContactResult > >,std::less< std::pair< std::string,std::string > >,Eigen::aligned_allocator< std::pair< std::pair< std::string,std::string > const,std::vector< tesseract_collision::ContactResult,Eigen::aligned_allocator< tesseract_collision::ContactResult > > > > >;
}

%inline
{
//std::size_t flattenResults(ContactResultMap&& m, ContactResultVector& v);
//TODO: this function has lousy performance
tesseract_collision::ContactResultVector flattenResults(tesseract_collision::ContactResultMap m)
{
    tesseract_collision::ContactResultVector v;
    tesseract_collision::flattenResults(std::move(m),v);
    return v;
}
}

tesseract_aligned_vector(ContactResultVector, tesseract_collision::ContactResult);
tesseract_aligned_map_of_aligned_vector(ContactResultMap, %arg(std::pair<std::string,std::string>), tesseract_collision::ContactResult);

