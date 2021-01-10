/**
 * @file entity_manager.cpp
 * @brief A entity manager for Tesseract components that get added to Ignition Scene
 *
 * @author Levi Armstrong
 * @date May 14, 2020
 * @version TODO
 * @bug No known bugs
 *
 * @copyright Copyright (c) 2020, Southwest Research Institute
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
#include <tesseract_visualization/ignition/entity_manager.h>

namespace tesseract_visualization
{
EntityID EntityManager::addModel(const std::string& name)
{
  model_id_map_[name] = ++entity_counter_;
  return model_id_map_[name];
}

EntityID EntityManager::getModel(const std::string& name) const
{
  auto it = model_id_map_.find(name);
  if (it == model_id_map_.end())
    return NULL_ENTITY_ID;

  return it->second;
}

const EntityMap& EntityManager::getModels() const { return model_id_map_; }

EntityID EntityManager::addLink(const std::string& name)
{
  link_id_map_[name] = ++entity_counter_;
  return link_id_map_[name];
}

EntityID EntityManager::getLink(const std::string& name) const
{
  auto it = link_id_map_.find(name);
  if (it == link_id_map_.end())
    return NULL_ENTITY_ID;

  return it->second;
}

const EntityMap& EntityManager::getLinks() const { return link_id_map_; }

EntityID EntityManager::addVisual(const std::string& name)
{
  visual_id_map_[name] = ++entity_counter_;
  return visual_id_map_[name];
}

EntityID EntityManager::getVisual(const std::string& name) const
{
  auto it = visual_id_map_.find(name);
  if (it == visual_id_map_.end())
    return NULL_ENTITY_ID;

  return it->second;
}

const EntityMap& EntityManager::getVisuals() const { return visual_id_map_; }

EntityID EntityManager::addSensor(const std::string& name)
{
  sensor_id_map_[name] = ++entity_counter_;
  return sensor_id_map_[name];
}

EntityID EntityManager::getSensor(const std::string& name) const
{
  auto it = sensor_id_map_.find(name);
  if (it == sensor_id_map_.end())
    return NULL_ENTITY_ID;

  return it->second;
}

const EntityMap& EntityManager::getSensors() const { return sensor_id_map_; }

bool EntityManager::empty() const { return (entity_counter_ == 1000); }

void EntityManager::clear()
{
  model_id_map_.clear();
  link_id_map_.clear();
  visual_id_map_.clear();
  sensor_id_map_.clear();
  entity_counter_ = 1000;
}
}  // namespace tesseract_visualization
