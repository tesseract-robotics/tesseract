/**
 * @file add_scene_graph_command.cpp
 * @brief Used to add a scene graph to the environment
 *
 * @author Levi Armstrong
 * @author Matthew Powelson
 * @date March 18, 2022
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

#include <tesseract/environment/commands/add_scene_graph_command.h>
#include <tesseract/common/utils.h>
#include <tesseract/scene_graph/graph.h>
#include <tesseract/scene_graph/joint.h>

#include <memory>
#include <string>

namespace tesseract::environment
{
AddSceneGraphCommand::AddSceneGraphCommand() : Command(CommandType::ADD_SCENE_GRAPH) {}

AddSceneGraphCommand::AddSceneGraphCommand(const tesseract::scene_graph::SceneGraph& scene_graph, std::string prefix)
  : Command(CommandType::ADD_SCENE_GRAPH)
  , scene_graph_(scene_graph.clone())
  , joint_(nullptr)
  , prefix_(std::move(prefix))
{
}

AddSceneGraphCommand::AddSceneGraphCommand(const tesseract::scene_graph::SceneGraph& scene_graph,
                                           const tesseract::scene_graph::Joint& joint,
                                           std::string prefix)
  : Command(CommandType::ADD_SCENE_GRAPH)
  , scene_graph_(scene_graph.clone())
  , joint_(std::make_shared<tesseract::scene_graph::Joint>(joint.clone()))
  , prefix_(std::move(prefix))
{
}

const std::shared_ptr<const tesseract::scene_graph::SceneGraph>& AddSceneGraphCommand::getSceneGraph() const
{
  return scene_graph_;
}
const std::shared_ptr<const tesseract::scene_graph::Joint>& AddSceneGraphCommand::getJoint() const { return joint_; }
const std::string& AddSceneGraphCommand::getPrefix() const { return prefix_; }

bool AddSceneGraphCommand::operator==(const AddSceneGraphCommand& rhs) const
{
  bool equal = true;
  equal &= Command::operator==(rhs);
  equal &= tesseract::common::pointersEqual(scene_graph_, rhs.scene_graph_);
  equal &= tesseract::common::pointersEqual(joint_, rhs.joint_);
  equal &= prefix_ == rhs.prefix_;
  return equal;
}
bool AddSceneGraphCommand::operator!=(const AddSceneGraphCommand& rhs) const { return !operator==(rhs); }

}  // namespace tesseract::environment
