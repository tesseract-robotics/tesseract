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
#include <tesseract/common/macros.h>
#include <tesseract/common/types.h>

#include <stdexcept>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <vector>

namespace
{
/**
 * @brief Look up @p ids against @p state, writing one pose per id in the order given
 * @throws std::out_of_range naming the link, if any of @p ids is absent from @p state
 */
void gatherPoses(const std::vector<tesseract::common::LinkId>& ids,
                 const tesseract::common::LinkIdTransformMap& state,
                 tesseract::common::VectorIsometry3d& out_poses)
{
  out_poses.clear();
  out_poses.reserve(ids.size());
  for (const auto& id : ids)
  {
    auto it = state.find(id);
    if (it == state.end())
      throw std::out_of_range("setCollisionObjectsTransform, link '" + id.name() + "' is absent from the state");

    out_poses.push_back(it->second);
  }
}
}  // namespace

namespace tesseract::collision
{
bool DiscreteContactManager::addCollisionObjects(const std::vector<CollisionObjectSpec>& objects)
{
  bool success{ true };
  for (const auto& obj : objects)
    success &= addCollisionObject(obj.id, obj.mask_id, obj.shapes, obj.shape_poses, obj.enabled);

  return success;
}

bool DiscreteContactManager::removeCollisionObjects(const std::vector<tesseract::common::LinkId>& ids)
{
  bool success{ true };
  for (const auto& id : ids)
    success &= removeCollisionObject(id);

  return success;
}

bool DiscreteContactManager::setCollisionObjectsEnabled(
    const std::unordered_map<tesseract::common::LinkId, bool>& enabled)
{
  bool success{ true };
  for (const auto& entry : enabled)
    success &= entry.second ? enableCollisionObject(entry.first) : disableCollisionObject(entry.first);

  return success;
}

bool DiscreteContactManager::setCollisionObjectsEnabled(const std::vector<tesseract::common::LinkId>& ids, bool enabled)
{
  std::unordered_map<tesseract::common::LinkId, bool> entries;
  entries.reserve(ids.size());
  for (const auto& id : ids)
    entries[id] = enabled;

  return setCollisionObjectsEnabled(entries);
}

void DiscreteContactManager::setCollisionObjectsTransform(const std::vector<tesseract::common::LinkId>& ids,
                                                          const tesseract::common::VectorIsometry3d& poses)
{
  if (ids.size() != poses.size())
    throw std::runtime_error("DiscreteContactManager, setCollisionObjectsTransform received " +
                             std::to_string(ids.size()) + " ids but " + std::to_string(poses.size()) + " poses!");

  for (std::size_t i = 0; i < ids.size(); ++i)
    setCollisionObjectsTransform(ids[i], poses[i]);
}

// The scratch buffers below are live only for the duration of the forwarded call, which must not re-enter: a
// backend override that called back into one of these (ids, map) overloads on the same thread would clear the
// buffers out from under the call in progress.
void DiscreteContactManager::setCollisionObjectsTransform(const std::unordered_set<tesseract::common::LinkId>& ids,
                                                          const tesseract::common::LinkIdTransformMap& state)
{
  TESSERACT_THREAD_LOCAL std::vector<tesseract::common::LinkId> scratch_ids;
  TESSERACT_THREAD_LOCAL tesseract::common::VectorIsometry3d scratch_poses;
  scratch_ids.assign(ids.begin(), ids.end());
  gatherPoses(scratch_ids, state, scratch_poses);
  setCollisionObjectsTransform(scratch_ids, scratch_poses);
}

void DiscreteContactManager::setCollisionObjectsTransform(const std::vector<tesseract::common::LinkId>& ids,
                                                          const tesseract::common::LinkIdTransformMap& state)
{
  TESSERACT_THREAD_LOCAL tesseract::common::VectorIsometry3d scratch_poses;
  gatherPoses(ids, state, scratch_poses);
  setCollisionObjectsTransform(ids, scratch_poses);
}

void DiscreteContactManager::setActiveCollisionObjects(const std::vector<tesseract::common::LinkId>& ids)
{
  setActiveCollisionObjects(std::unordered_set<tesseract::common::LinkId>(ids.begin(), ids.end()));
}

void DiscreteContactManager::setActiveCollisionObjects(std::initializer_list<tesseract::common::LinkId> ids)
{
  setActiveCollisionObjects(std::unordered_set<tesseract::common::LinkId>(ids.begin(), ids.end()));
}

void DiscreteContactManager::applyContactManagerConfig(const ContactManagerConfig& config)
{
  config.validate();

  if (config.default_margin.has_value())
    setDefaultCollisionMargin(config.default_margin.value());

  setCollisionMarginPairData(config.pair_margin_data, config.pair_margin_override_type);
  applyContactAllowedValidatorOverride(*this, config.acm, config.acm_override_type);
  setCollisionObjectsEnabled(config.modify_object_enabled);
}
}  // namespace tesseract::collision
