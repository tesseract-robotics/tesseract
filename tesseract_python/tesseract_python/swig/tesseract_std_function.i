/**
 * @file tesseract_std_function.i
 * @brief std_function typemap macros to provide write only callback functions
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

// Inspired by https://stackoverflow.com/questions/32644268/how-to-use-swig-to-wrap-stdfunction-objects

%define %_formacro_2n(macro, arg1, arg2,...)macro(arg1, arg2)
#if #__VA_ARGS__ != "__fordone__"
,%_formacro_2n(macro, __VA_ARGS__)
#endif
%enddef

%define %formacro_2n(macro,...)%_formacro_2n(macro,__VA_ARGS__,__fordone__)%enddef

%define _tesseract_std_function_call_args(arg_type, arg_name) arg_type arg_name  %enddef
%define _tesseract_std_function_call_vars(arg_type, arg_name) arg_name  %enddef

%define %tesseract_std_function(Name, Namespace, Ret, ...)

%shared_ptr(Name##Base)
%feature("director") Name##Base;
%pythondynamic Name##Base;

class Name##Base
{
public:
    virtual Ret call( %formacro_2n(_tesseract_std_function_call_args,__VA_ARGS__) ) = 0;
    virtual ~Name##Base() {}     
};

%typemap(in) Name##Base (void *argp, int res = 0, std::shared_ptr< Name##Base > temp1) {
    // tesseract_std_function %typemap(in)
  int newmem = 0;
  res = SWIG_ConvertPtrAndOwn($input, &argp, $descriptor(std::shared_ptr< Name##Base > *), %convertptr_flags, &newmem);
  if (!SWIG_IsOK(res)) {
    %argument_fail(res, "$type", $symname, $argnum);
  }
  if (!argp) {
    %argument_nullref("$type", $symname, $argnum);
  } else {
    temp1 = *(%reinterpret_cast(argp, std::shared_ptr< Name##Base > *));
    $1 = [temp1]( %formacro_2n(_tesseract_std_function_call_args,__VA_ARGS__) ) { return temp1->call(  %formacro_2n(_tesseract_std_function_call_vars,__VA_ARGS__)  ); };
    if (newmem & SWIG_CAST_NEW_MEMORY) delete %reinterpret_cast(argp, std::shared_ptr< Name##Base > *);
  }
}

%typemap(in) Name##Base const & (void *argp, int res = 0, std::shared_ptr< Name##Base > temp1, Namespace::Name temp2) {
    // const tesseract_std_function& %typemap(in)
  int newmem = 0;
  res = SWIG_ConvertPtrAndOwn($input, &argp, $descriptor(std::shared_ptr< Name##Base > *), %convertptr_flags, &newmem);
  if (!SWIG_IsOK(res)) {
    %argument_fail(res, "$type", $symname, $argnum);
  }
  if (!argp) {
    %argument_nullref("$type", $symname, $argnum);
  } else {
    temp1 = *(%reinterpret_cast(argp, std::shared_ptr< Name##Base > *));
    temp2 = [temp1]( %formacro_2n(_tesseract_std_function_call_args,__VA_ARGS__) ) { return temp1->call(  %formacro_2n(_tesseract_std_function_call_vars,__VA_ARGS__)  ); };
    $1 = &temp2;
    if (newmem & SWIG_CAST_NEW_MEMORY) delete %reinterpret_cast(argp, std::shared_ptr< Name##Base > *);
  }
}

%typemap(out) Name##Base const &  {
    // tesseract_std_function & %typemap(out)
    Py_INCREF(Py_None);
    %set_output(Py_None);
}

namespace Namespace
{
using Name = ::Name##Base;
}

%pythoncode %{

class Name(Name##Base):
  def __init__(self,fn):
    super(Name,self).__init__()
    self._fn = fn

  def call(self,*args):
    return self._fn(*args)
%}

%enddef

%define %tesseract_std_function_noargs(Name, Namespace, Ret)

%shared_ptr(Name##Base)
%feature("director") Name##Base;
%pythondynamic Name##Base;


class Name##Base
{
public:
    virtual Ret call() = 0;
    virtual ~Name##Base() {}     
};


%typemap(in) Name##Base (void *argp, int res = 0, std::shared_ptr< Name##Base > temp1) {
    // tesseract_std_function %typemap(in)
  int newmem = 0;
  res = SWIG_ConvertPtrAndOwn($input, &argp, $descriptor(std::shared_ptr< Name##Base > *), %convertptr_flags, &newmem);
  if (!SWIG_IsOK(res)) {
    %argument_fail(res, "$type", $symname, $argnum);
  }
  if (!argp) {
    %argument_nullref("$type", $symname, $argnum);
  } else {
    temp1 = *(%reinterpret_cast(argp, std::shared_ptr< Name##Base > *));
    $1 = [temp1]() { return temp1->call( ); };
    if (newmem & SWIG_CAST_NEW_MEMORY) delete %reinterpret_cast(argp, std::shared_ptr< Name##Base > *);
  }
}

%typemap(out) Name##Base*  {
    // tesseract_std_function* %typemap(out)
    Py_INCREF(Py_None);
    %set_output(Py_None);
}

namespace Namespace
{
using Name = ::Name##Base;
}

%pythoncode %{

class Name(Name##Base):
  def __init__(self,fn):
    super(Name,self).__init__()
    self._fn = fn

  def call(self):
    return self._fn()
%}

%enddef

%define %tesseract_std_function_base(Name, Namespace, Ret, ...)

%{
class Name##Base
{
public:
    virtual Ret call( %formacro_2n(_tesseract_std_function_call_args,__VA_ARGS__) ) = 0;        
    virtual ~Name##Base() {}
};
%}

%enddef

%define %tesseract_std_function_noargs_base(Name, Namespace, Ret)

%{
class Name##Base
{
public:
    virtual Ret call() = 0;
    virtual ~Name##Base() {}    
};
%}



%enddef