/**
 * @file tesseract_vector_reference_wrapper_instruction_typemaps.i
 * @brief typemaps for std::vector<std::reference_wrapper<tesseract_planning::Instruction>> template
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

%template() std::vector<std::reference_wrapper<tesseract_planning::Instruction>>;

%typemap(in, noblock=0) std::vector<std::reference_wrapper<tesseract_planning::Instruction>> *(void  *argp = 0, int res = 0, std::vector<std::reference_wrapper<tesseract_planning::Instruction>> temp, std::vector<tesseract_planning::Instruction>* temp1) {

  // custom std::vector<std::reference_wrapper<tesseract_planning::Instruction>>* in typemap
  res = SWIG_ConvertPtr($input, &argp,$descriptor(std::vector<tesseract_planning::Instruction>*), $disown | %convertptr_flags);
  if (!SWIG_IsOK(res)) { 
    %argument_fail(res, "$type", $symname, $argnum); 
  }
   temp1 = %reinterpret_cast(argp, std::vector<tesseract_planning::Instruction>*);
   for (auto& a: *temp1)
   {
       temp.push_back(a);
   }
   $1 = &temp;
}

%typemap(in, noblock=0) std::vector<std::reference_wrapper<tesseract_planning::Instruction>> & (void  *argp = 0, int res = 0, std::vector<std::reference_wrapper<tesseract_planning::Instruction>> temp, std::vector<tesseract_planning::Instruction>* temp1) {

  // custom std::vector<std::reference_wrapper<tesseract_planning::Instruction>>& in typemap
  res = SWIG_ConvertPtr($input, &argp,$descriptor(std::vector<tesseract_planning::Instruction>*), $disown | %convertptr_flags);
  if (!SWIG_IsOK(res)) { 
    %argument_fail(res, "$type", $symname, $argnum); 
  }
   temp1 = %reinterpret_cast(argp, std::vector<tesseract_planning::Instruction>*);
   for (auto& a: *temp1)
   {
       temp.push_back(a);
   }
   $1 = &temp;
}

%typemap(out) std::vector<std::reference_wrapper<tesseract_planning::Instruction>>* {

  // custom std::vector<std::reference_wrapper<tesseract_planning::Instruction>>* out typemap

  std::vector<tesseract_planning::Instruction> temp_out;
  for(auto& a: *$1)
  {
      temp_out.push_back(a.get());
  }
  
  %set_output(SWIG_NewPointerObj(%new_copy(temp_out, std::vector<tesseract_planning::Instruction>), $descriptor(std::vector<tesseract_planning::Instruction>&), SWIG_POINTER_OWN | %newpointer_flags));
}

%typemap(out) std::vector<std::reference_wrapper<tesseract_planning::Instruction>> {

  // custom std::vector<std::reference_wrapper<tesseract_planning::Instruction>> out typemap

  std::vector<tesseract_planning::Instruction> temp_out;
  for(auto& a: $1)
  {
      temp_out.push_back(a.get());
  }
  
  %set_output(SWIG_NewPointerObj(%new_copy(temp_out, std::vector<tesseract_planning::Instruction>), $descriptor(std::vector<tesseract_planning::Instruction>&), SWIG_POINTER_OWN | %newpointer_flags));
}

%typemap(out) std::vector<std::reference_wrapper<tesseract_planning::Instruction const>> {

  // custom std::vector<std::reference_wrapper<tesseract_planning::Instruction>> out typemap

  std::vector<tesseract_planning::Instruction> temp_out;
  for(auto& a: *(%reinterpret_cast(&$1, $&ltype)))
  {
      temp_out.push_back(a.get());
  }
  
  %set_output(SWIG_NewPointerObj(%new_copy(temp_out, std::vector<tesseract_planning::Instruction>), $descriptor(std::vector<tesseract_planning::Instruction>&), SWIG_POINTER_OWN | %newpointer_flags));
}

%typemap(typecheck,precedence=SWIG_TYPECHECK_POINTER,noblock=1) std::vector<std::reference_wrapper<tesseract_planning::Instruction>> & {
  void *vptr = 0;
  int res = SWIG_ConvertPtr($input, &vptr, $descriptor(std::vector<tesseract_planning::Instruction>&), SWIG_POINTER_NO_NULL);
  $1 = SWIG_CheckState(res);
}