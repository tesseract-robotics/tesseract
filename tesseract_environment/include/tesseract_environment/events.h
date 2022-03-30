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
#include <tesseract_environment/command.h>
#include <tesseract_scene_graph/scene_state.h>

namespace tesseract_environment
{
enum class Events
{
  COMMAND_APPLIED = 0,
  SCENE_STATE_CHANGED = 1
};

/** @brief The event base class */
struct Event
{
  Event(Events type) : type(type) {}
  virtual ~Event() = default;

  const Events type;
};

/**
 * @brief The command applied event
 * @note Do not store the const& of command in your code make a copy instead
 */
struct CommandAppliedEvent : public Event
{
  CommandAppliedEvent(const Commands& commands, int revision)
    : Event(Events::COMMAND_APPLIED), commands(commands), revision(revision)
  {
  }

  const Commands& commands;
  int revision;
};

/**
 * @brief The scene state changed event
 * @note Do not store the const& of state in your code make a copy instead
 */
struct SceneStateChangedEvent : public Event
{
  SceneStateChangedEvent(const tesseract_scene_graph::SceneState& state)
    : Event(Events::SCENE_STATE_CHANGED), state(state)
  {
  }

  const tesseract_scene_graph::SceneState& state;
};

using EventCallbackFn = std::function<void(const Event& event)>;
}  // namespace tesseract_environment

#endif  // TESSERACT_ENVIRONMENT_EVENTS_H
