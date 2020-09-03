/**
 * @file entity_manager.h
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
#ifndef TESSERACT_VISUALIZATION_IGNITION_ENTITY_MANAGER_H
#define TESSERACT_VISUALIZATION_IGNITION_ENTITY_MANAGER_H

#include <tesseract_common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <string>
#include <unordered_map>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

namespace tesseract_visualization
{
static const int NULL_ENTITY_ID = -1;
using EntityID = int;
using EntityMap = std::unordered_map<std::string, EntityID>;

class EntityManager
{
public:
  /**
   * @brief Add model name to manager and return id for model
   * @param name Name given to the model
   * @return The Entity ID
   */
  EntityID addModel(const std::string& name);

  /**
   * @brief Given the model name return the ID
   * @param name Name of the model
   * @return The ID of the model (if < 1000 it was not found)
   */
  EntityID getModel(const std::string& name) const;

  /**
   * @brief Get all models being managed
   * @return A map of names to entity id's
   */
  const EntityMap& getModels() const;

  /**
   * @brief Add link name to manager and return id for link
   * @param name Name given to the link
   * @return The Entity ID
   */
  EntityID addLink(const std::string& name);

  /**
   * @brief Given the link name return the ID
   * @param name Name of the link
   * @return The ID of the link (if < 1000 it was not found)
   */
  EntityID getLink(const std::string& name) const;

  /**
   * @brief Get all links being managed
   * @return A map of names to entity id's
   */
  const EntityMap& getLinks() const;

  /**
   * @brief Add visual name to manager and return id for visual
   * @param name Name given to the visual
   * @return The Entity ID
   */
  EntityID addVisual(const std::string& name);

  /**
   * @brief Given the visual name return the ID
   * @param name Name of the visual
   * @return The ID of the visual (if < 1000 it was not found)
   */
  EntityID getVisual(const std::string& name) const;

  /**
   * @brief Get all visuals being managed
   * @return A map of names to entity id's
   */
  const EntityMap& getVisuals() const;

  /**
   * @brief Add sensor name to manager and return id for sensor
   * @param name Name given to the sensor
   * @return The Entity ID
   */
  EntityID addSensor(const std::string& name);

  /**
   * @brief Given the sensor name return the ID
   * @param name Name of the sensor
   * @return The ID of the visual (if < 1000 it was not found)
   */
  EntityID getSensor(const std::string& name) const;

  /**
   * @brief Get all sensors being managed
   * @return A map of names to entity id's
   */
  const EntityMap& getSensors() const;

  /** @brief Check if empty */
  bool empty() const;

  /** @brief Clear interanl data */
  void clear();

private:
  EntityMap link_id_map_;           /**< Stores entity id for each link */
  EntityMap model_id_map_;          /**< Stores entity id for each model */
  EntityMap visual_id_map_;         /**< Stores entity id for each visual */
  EntityMap sensor_id_map_;         /**< Stores entity id for each sensor */
  EntityID entity_counter_{ 1000 }; /**< Start entity counter to avoid clashing with gazebo */
};
}  // namespace tesseract_visualization

#endif  // TESSERACT_VISUALIZATION_IGNITION_ENTITY_MANAGER_H
