/**
 * @file discrete_contact_manager_factory.i
 * @brief SWIG interface file for tesseract_collision/discrete_contact_manager_factory.h
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
#include <tesseract_collision/core/discrete_contact_manager_factory.h>
%}

%include "discrete_contact_manager.i"

//%shared_ptr(DiscreteContactManagerFactory)

namespace tesseract_collision
{

class DiscreteContactManagerFactory
{
public:
  //using CreateMethod = std::function<DiscreteContactManager::Ptr()>;
  DiscreteContactManagerFactory();

  //bool registar(const std::string name, CreateMethod create_function);

  DiscreteContactManager::Ptr create(const std::string& name) const;

};
}  // namespace tesseract_collision