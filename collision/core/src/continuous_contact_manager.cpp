/**
 * @file continuous_contact_manager.h
 * @brief This is the continuous contact manager base class
 *
 * It should be used to perform continuous contact checking.
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

#include <tesseract/collision/continuous_contact_manager.h>
#include <tesseract/collision/utils.h>

namespace tesseract::collision
{
void ContinuousContactManager::setCollisionObjectsTransform(const tesseract::common::LinkIdTransformMap& transforms)
{
  for (const auto& id : getCollisionObjects())
  {
    auto it = transforms.find(id);
    if (it != transforms.end())
      setCollisionObjectsTransform(id.name(), it->second);
  }
}

void ContinuousContactManager::setCollisionObjectsTransform(tesseract::common::LinkId id,
                                                            const Eigen::Isometry3d& pose)
{
  for (const auto& obj_id : getCollisionObjects())
  {
    if (obj_id == id)
    {
      setCollisionObjectsTransform(obj_id.name(), pose);
      return;
    }
  }
}

void ContinuousContactManager::setCollisionObjectsTransform(const tesseract::common::LinkIdTransformMap& pose1,
                                                            const tesseract::common::LinkIdTransformMap& pose2)
{
  for (const auto& id : getCollisionObjects())
  {
    auto it1 = pose1.find(id);
    auto it2 = pose2.find(id);
    if (it1 != pose1.end() && it2 != pose2.end())
      setCollisionObjectsTransform(id.name(), it1->second, it2->second);
  }
}

void ContinuousContactManager::setCollisionObjectsTransform(tesseract::common::LinkId id,
                                                            const Eigen::Isometry3d& pose1,
                                                            const Eigen::Isometry3d& pose2)
{
  for (const auto& obj_id : getCollisionObjects())
  {
    if (obj_id == id)
    {
      setCollisionObjectsTransform(obj_id.name(), pose1, pose2);
      return;
    }
  }
}

void ContinuousContactManager::setActiveCollisionObjects(const std::vector<tesseract::common::LinkId>& ids)
{
  std::vector<std::string> names;
  names.reserve(ids.size());
  for (const auto& id : ids)
    names.push_back(id.name());
  setActiveCollisionObjects(names);
}

void ContinuousContactManager::applyContactManagerConfig(const ContactManagerConfig& config)
{
  config.validate();

  if (config.default_margin.has_value())
    setDefaultCollisionMargin(config.default_margin.value());

  setCollisionMarginPairData(config.pair_margin_data, config.pair_margin_override_type);
  applyContactAllowedValidatorOverride(*this, config.acm, config.acm_override_type);
  applyModifyObjectEnabled(*this, config.modify_object_enabled);
}
}  // namespace tesseract::collision
