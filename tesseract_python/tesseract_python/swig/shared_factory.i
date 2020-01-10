/**
 * @file shared_factory.i
 * @brief Provide downcasting for shared_ptr types
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

// Based on https://stackoverflow.com/questions/27392602/swig-downcasting-from-base-to-derived?rq=1

%define %_shared_factory_dispatch(Type)
if (!dcast) {
  SWIG_SHARED_PTR_QNAMESPACE::shared_ptr<Type> dobj
          = SWIG_SHARED_PTR_QNAMESPACE::dynamic_pointer_cast<Type>($1);
  if (dobj) {
    dcast = 1;
    SWIG_SHARED_PTR_QNAMESPACE::shared_ptr<Type> *smartresult
            = dobj ? new SWIG_SHARED_PTR_QNAMESPACE::shared_ptr<Type>(dobj) : 0;
    %set_output(SWIG_NewPointerObj(%as_voidptr(smartresult),
                                   $descriptor(SWIG_SHARED_PTR_QNAMESPACE::shared_ptr<Type> *),
                                   SWIG_POINTER_OWN));
  }
}%enddef

%define %_shared_factory_dispatch_ref(Type)
if (!dcast) {
  SWIG_SHARED_PTR_QNAMESPACE::shared_ptr<Type> dobj
          = SWIG_SHARED_PTR_QNAMESPACE::dynamic_pointer_cast<Type>(*$1);
  if (dobj) {
    dcast = 1;
    SWIG_SHARED_PTR_QNAMESPACE::shared_ptr<Type> *smartresult
            = dobj ? new SWIG_SHARED_PTR_QNAMESPACE::shared_ptr<Type>(dobj) : 0;
    %set_output(SWIG_NewPointerObj(%as_voidptr(smartresult),
                                   $descriptor(SWIG_SHARED_PTR_QNAMESPACE::shared_ptr<Type> *),
                                   SWIG_POINTER_OWN));
  }
}%enddef

%define %_shared_factory_dispatch_pointer(Type)
if (!dcast) {
  Type* temp = dynamic_cast<Type*>($1);
  if (temp) {
    dcast = 1;
    SWIG_SHARED_PTR_QNAMESPACE::shared_ptr< Type > *smartresult = temp ? new SWIG_SHARED_PTR_QNAMESPACE::shared_ptr< Type >(temp SWIG_NO_NULL_DELETER_$owner) : 0;
    %set_output(SWIG_NewPointerObj(%as_voidptr(smartresult), $descriptor(SWIG_SHARED_PTR_QNAMESPACE::shared_ptr<Type > *), $owner | SWIG_POINTER_OWN));
  }
}%enddef

%define %shared_factory(BaseType,Types...)
%typemap(out) SWIG_SHARED_PTR_QNAMESPACE::shared_ptr<BaseType> {
  int dcast = 0;
  %formacro(%_shared_factory_dispatch, Types)
  if (!dcast) {
      SWIG_SHARED_PTR_QNAMESPACE::shared_ptr<BaseType> *smartresult
              = $1 ? new SWIG_SHARED_PTR_QNAMESPACE::shared_ptr<BaseType>($1) : 0;
      %set_output(SWIG_NewPointerObj(%as_voidptr(smartresult),
                                     $descriptor(SWIG_SHARED_PTR_QNAMESPACE::shared_ptr<BaseType> *),
                                     SWIG_POINTER_OWN));
  }
}

%typemap(out) SWIG_SHARED_PTR_QNAMESPACE::shared_ptr<BaseType>&, const SWIG_SHARED_PTR_QNAMESPACE::shared_ptr<BaseType>& {
  int dcast = 0;
  %formacro(%_shared_factory_dispatch_ref, Types)
  if (!dcast) {
      SWIG_SHARED_PTR_QNAMESPACE::shared_ptr<BaseType> *smartresult
              = *$1 ? new SWIG_SHARED_PTR_QNAMESPACE::shared_ptr<BaseType>(*$1) : 0;
      %set_output(SWIG_NewPointerObj(%as_voidptr(smartresult),
                                     $descriptor(SWIG_SHARED_PTR_QNAMESPACE::shared_ptr<BaseType> *),
                                     SWIG_POINTER_OWN));
  }
}

%typemap(out, fragment="SWIG_null_deleter_python") BaseType* {
  int dcast = 0;
  %formacro(%_shared_factory_dispatch_pointer, Types)
  if (!dcast) {
  SWIG_SHARED_PTR_QNAMESPACE::shared_ptr< BaseType > *smartresult = $1 ? new SWIG_SHARED_PTR_QNAMESPACE::shared_ptr< BaseType >($1 SWIG_NO_NULL_DELETER_$owner) : 0;
  %set_output(SWIG_NewPointerObj(%as_voidptr(smartresult), $descriptor(SWIG_SHARED_PTR_QNAMESPACE::shared_ptr<BaseType > *), $owner | SWIG_POINTER_OWN));
  }
}


%enddef