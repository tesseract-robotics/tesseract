/**
 * @file discrete_contact_manager_factory.h
 * @brief This is the discrete contact manager factory
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
#ifndef TESSERACT_COLLISION_DISCRETE_CONTACT_MANAGER_FACTORY_H
#define TESSERACT_COLLISION_DISCRETE_CONTACT_MANAGER_FACTORY_H

#include <tesseract_collision/core/discrete_contact_manager.h>

#ifdef SWIG
%shared_ptr(tesseract_collision::DiscreteContactManagerFactory)
#endif  // SWIG

namespace tesseract_collision
{
using DiscreteContactManagerFactoryCreateMethod = std::function<DiscreteContactManager::Ptr()>;
class DiscreteContactManagerFactory
{
public:
  DiscreteContactManagerFactory() = default;

  bool registar(const std::string& name, DiscreteContactManagerFactoryCreateMethod create_function)
  {
    auto it = discrete_types.find(name);
    if (it == discrete_types.end())
    {
      discrete_types[name] = std::move(create_function);
      keys_.push_back(name);
      return true;
    }
    return false;
  }

  DiscreteContactManager::Ptr create(const std::string& name) const
  {
    auto it = discrete_types.find(name);
    if (it != discrete_types.end())
      return it->second();  // call the createFunc

    return nullptr;
  }

  const std::vector<std::string>& getAvailableManagers() const { return keys_; }

private:
  std::unordered_map<std::string, DiscreteContactManagerFactoryCreateMethod> discrete_types;
  std::vector<std::string> keys_;
};
}  // namespace tesseract_collision
#endif  // TESSERACT_COLLISION_DISCRETE_CONTACT_MANAGER_FACTORY_H
