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
#include <tesseract/common/types.h>

#include <stdexcept>
#include <string>

namespace tesseract::collision
{
void ContinuousContactManager::setCollisionObjectsTransform(const std::vector<tesseract::common::LinkId>& ids,
                                                            const tesseract::common::VectorIsometry3d& poses)
{
  if (ids.size() != poses.size())
    throw std::runtime_error("ContinuousContactManager, setCollisionObjectsTransform received " +
                             std::to_string(ids.size()) + " ids but " + std::to_string(poses.size()) + " poses!");

  for (std::size_t i = 0; i < ids.size(); ++i)
    setCollisionObjectsTransform(ids[i], poses[i]);
}

void ContinuousContactManager::setCollisionObjectsTransform(const std::vector<tesseract::common::LinkId>& ids,
                                                            const tesseract::common::VectorIsometry3d& pose1,
                                                            const tesseract::common::VectorIsometry3d& pose2)
{
  if (ids.size() != pose1.size() || ids.size() != pose2.size())
    throw std::runtime_error("ContinuousContactManager, setCollisionObjectsTransform received " +
                             std::to_string(ids.size()) + " ids but " + std::to_string(pose1.size()) +
                             " start poses and " + std::to_string(pose2.size()) + " end poses!");

  for (std::size_t i = 0; i < ids.size(); ++i)
    setCollisionObjectsTransform(ids[i], pose1[i], pose2[i]);
}

void ContinuousContactManager::setActiveCollisionObjects(const std::vector<tesseract::common::LinkId>& ids)
{
  setActiveCollisionObjects(std::unordered_set<tesseract::common::LinkId>(ids.begin(), ids.end()));
}

void ContinuousContactManager::setActiveCollisionObjects(std::initializer_list<tesseract::common::LinkId> ids)
{
  setActiveCollisionObjects(std::unordered_set<tesseract::common::LinkId>(ids.begin(), ids.end()));
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
