/**
 * @file events.h
 * @brief Tesseract Events.
 *
 * @author Levi Armstrong
 * @date March 28, 2022
 * @version TODO
 * @bug No known bugs
 *
 * @copyright Copyright (c) 2022, Levi Armstrong
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
#ifndef TESSERACT_ENVIRONMENT_EVENTS_H
#define TESSERACT_ENVIRONMENT_EVENTS_H

#include <tesseract_common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <memory>
#include <vector>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract_scene_graph/fwd.h>

namespace tesseract_environment
{
class Command;

enum class Events
{
  COMMAND_APPLIED = 0,
  SCENE_STATE_CHANGED = 1
};

/** @brief The event base class */
struct Event
{
  Event(Events type);
  virtual ~Event() = default;
  Event(const Event&) = default;
  Event& operator=(const Event&) = delete;
  Event(Event&&) = default;
  Event& operator=(Event&&) = delete;

  const Events type;
};

/**
 * @brief The command applied event
 * @note Do not store the const& of command in your code make a copy instead
 */
struct CommandAppliedEvent : public Event
{
  CommandAppliedEvent(const std::vector<std::shared_ptr<const Command>>& commands, int revision);

  const std::vector<std::shared_ptr<const Command>>& commands;
  int revision;
};

/**
 * @brief The scene state changed event
 * @note Do not store the const& of state in your code make a copy instead
 */
struct SceneStateChangedEvent : public Event
{
  SceneStateChangedEvent(const tesseract_scene_graph::SceneState& state);

  const tesseract_scene_graph::SceneState& state;
};

}  // namespace tesseract_environment

#endif  // TESSERACT_ENVIRONMENT_EVENTS_H
