/**
 * @file continuous_contact_manager_factory.h
 * @brief This is the continuous contact manager factory
 *
 * @author Levi Armstrong
 * @date Dec 18, 2017
 * @version TODO
 * @bug No known bugs
 *
 * @copyright Copyright (c) 2017, Southwest Research Institute
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

#ifndef TESSERACT_COLLISION_CONTINUOUS_CONTACT_MANAGER_FACTORY_H
#define TESSERACT_COLLISION_CONTINUOUS_CONTACT_MANAGER_FACTORY_H

#include <tesseract_collision/core/continuous_contact_manager.h>

#ifdef SWIG
%shared_ptr(tesseract_collision::ContinuousContactManagerFactory)
#endif  // SWIG

namespace tesseract_collision
{
using ContinuousContactManagerFactoryCreateMethod = std::function<ContinuousContactManager::Ptr()>;
class ContinuousContactManagerFactory
{
public:
  ContinuousContactManagerFactory() = default;

  bool registar(const std::string& name, ContinuousContactManagerFactoryCreateMethod create_function)
  {
    auto it = continuous_types.find(name);
    if (it == continuous_types.end())
    {
      continuous_types[name] = std::move(create_function);
      keys_.push_back(name);
      return true;
    }
    return false;
  }

  ContinuousContactManager::Ptr create(const std::string& name) const
  {
    auto it = continuous_types.find(name);
    if (it != continuous_types.end())
      return it->second();  // call the createFunc

    return nullptr;
  }

  const std::vector<std::string>& getAvailableManagers() const { return keys_; }

private:
  std::unordered_map<std::string, ContinuousContactManagerFactoryCreateMethod> continuous_types;
  std::vector<std::string> keys_;
};
}  // namespace tesseract_collision

#endif  // TESSERACT_COLLISION_CONTINUOUS_CONTACT_MANAGER_FACTORY_H
