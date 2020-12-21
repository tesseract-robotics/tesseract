/**
 * @file tesseract_swig_include.i
 * @brief Common include for tesseract swig modules
 *
 * @author John Wason
 * @date December 18, 2020
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

%{
#define SWIG_PYTHON_EXTRA_NATIVE_CONTAINERS
%}


%include <std_shared_ptr.i>
%include <std_string.i>
%include <std_vector.i>
%include <std_pair.i>
%include <std_map.i>
%include <std_unordered_map.i>
%include <std_array.i>
%include <stdint.i>
%include <attribute.i>
%include <exception.i>
%include <pybuffer.i>

namespace std {
    typedef ::size_t size_t;
}

%exception {
  try {
    $action
  }
  SWIG_CATCH_STDEXCEPT
}

%feature("director:except") {
    if ($error != NULL) {
        throw Swig::DirectorMethodException();
    }
}

#ifndef SWIGPYTHON2
%pythonnondynamic;
#endif

%include "eigen.i"
%include "shared_factory.i"
%include "json_typemaps.i"
%include "eigen_types.i"

%{
namespace std
{
  template<typename T> struct remove_reference<swig::SwigPySequence_Ref<T>>
  {
    typedef T type;
  };

  template<typename T> struct remove_reference<const swig::SwigPySequence_Ref<T>>
  {
    typedef const T type;
  };

  template<typename T> struct remove_reference<SwigValueWrapper<T>>
  {
    typedef T type;
  };

  template<typename T> struct remove_reference<const SwigValueWrapper<T>>
  {
    typedef const T type;
  };

  template<typename T> struct remove_reference<SwigValueWrapper<T>&>
  {
    typedef T type;
  };

  template<typename T> struct remove_reference<const SwigValueWrapper<T>&>
  {
    typedef const T type;
  };

}
%}