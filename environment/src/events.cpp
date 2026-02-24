/**
 * @file events.cpp
 * @brief Tesseract Events.
 *
 * @author Levi Armstrong
 * @date March 28, 2022
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

#include <tesseract/environment/events.h>
#include <tesseract/environment/command.h>
#include <tesseract/scene_graph/scene_state.h>

namespace tesseract::environment
{
Event::Event(Events type) : type(type) {}

CommandAppliedEvent::CommandAppliedEvent(const std::vector<std::shared_ptr<const Command> >& commands, int revision)
  : Event(Events::COMMAND_APPLIED), commands(commands), revision(revision)
{
}

SceneStateChangedEvent::SceneStateChangedEvent(const tesseract::scene_graph::SceneState& state)
  : Event(Events::SCENE_STATE_CHANGED), state(state)
{
}
}  // namespace tesseract::environment
