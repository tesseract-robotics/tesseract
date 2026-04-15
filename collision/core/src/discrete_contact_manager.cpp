/**
 * @file discrete_contact_manager.h
 * @brief This is the discrete contact manager base class
 *
 * It should be used to perform discrete contact checking.
 *
 * @author Levi Armstrong
 * @date Dec 1, 2021
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

#include <tesseract/collision/discrete_contact_manager.h>
#include <tesseract/collision/utils.h>
#include <tesseract/common/types.h>

namespace tesseract::collision
{
void DiscreteContactManager::setCollisionObjectsTransform(const std::vector<std::string>& names,
                                                          const tesseract::common::VectorIsometry3d& poses)
{
  assert(names.size() == poses.size());
  for (auto i = 0U; i < names.size(); ++i)
    setCollisionObjectsTransform(tesseract::common::LinkId(names[i]), poses[i]);
}

void DiscreteContactManager::setActiveCollisionObjects(const std::vector<std::string>& names)
{
  std::vector<tesseract::common::LinkId> ids;
  ids.reserve(names.size());
  for (const auto& name : names)
    ids.push_back(tesseract::common::LinkId(name));
  setActiveCollisionObjects(ids);
}

std::vector<std::string> DiscreteContactManager::getActiveCollisionObjectNames() const
{
  const auto& ids = getActiveCollisionObjectIds();
  std::vector<std::string> result;
  result.reserve(ids.size());
  for (const auto& id : ids)
    result.push_back(id.name());
  return result;
}

void DiscreteContactManager::applyContactManagerConfig(const ContactManagerConfig& config)
{
  config.validate();

  if (config.default_margin.has_value())
    setDefaultCollisionMargin(config.default_margin.value());

  setCollisionMarginPairData(config.pair_margin_data, config.pair_margin_override_type);
  applyContactAllowedValidatorOverride(*this, config.acm, config.acm_override_type);
  applyModifyObjectEnabled(*this, config.modify_object_enabled);
}
}  // namespace tesseract::collision
