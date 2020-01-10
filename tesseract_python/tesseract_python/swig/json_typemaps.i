/**
 * @file json_typemaps.i
 * @brief Python Typemaps for JsonCPP
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
#include <json/json.h>
%}

%typemap(in) const Json::Value& (Json::Value temp){
	std::string *ptr = (std::string *)0;
	int res = SWIG_AsPtr_std_string($input, &ptr);
	if (!SWIG_IsOK(res) || !ptr) { 
	  %argument_fail((ptr ? res : SWIG_TypeError), "$type", $symname, $argnum); 
	}
	std::string ptr_dat;
	ptr_dat.swap(*ptr);	
	if (SWIG_IsNewObj(res)) %delete(ptr);
	Json::Reader reader;
	bool parse_successful = reader.parse(ptr_dat, temp);
	if (!parse_successful)
	{
		PyErr_SetString(PyExc_ValueError, "Could not parse Json.");
		SWIG_fail;
	}
	$1 = &temp;
	
}

