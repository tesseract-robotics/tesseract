/**
 * @file add_scene_graph_command.cpp
 * @brief Used to add a scene graph to the environment
 *
 * @author Levi Armstrong
 * @author Matthew Powelson
 * @date March 18, 2022
 * @version TODO
 * @bug No known bugs
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

#include <tesseract_common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <boost/serialization/access.hpp>
#include <boost/serialization/nvp.hpp>
#include <boost/serialization/shared_ptr.hpp>
#include <memory>
#include <string>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract_common/utils.h>
#include <tesseract_environment/commands/add_scene_graph_command.h>

namespace tesseract_environment
{
AddSceneGraphCommand::AddSceneGraphCommand(const tesseract_scene_graph::SceneGraph& scene_graph, std::string prefix)
  : Command(CommandType::ADD_SCENE_GRAPH)
  , scene_graph_(scene_graph.clone())
  , joint_(nullptr)
  , prefix_(std::move(prefix))
{
}

AddSceneGraphCommand::AddSceneGraphCommand(const tesseract_scene_graph::SceneGraph& scene_graph,
                                           const tesseract_scene_graph::Joint& joint,
                                           std::string prefix)
  : Command(CommandType::ADD_SCENE_GRAPH)
  , scene_graph_(scene_graph.clone())
  , joint_(std::make_shared<tesseract_scene_graph::Joint>(joint.clone()))
  , prefix_(std::move(prefix))
{
}

bool AddSceneGraphCommand::operator==(const AddSceneGraphCommand& rhs) const
{
  bool equal = true;
  equal &= Command::operator==(rhs);
  equal &= tesseract_common::pointersEqual(scene_graph_, rhs.scene_graph_);
  equal &= tesseract_common::pointersEqual(joint_, rhs.joint_);
  equal &= prefix_ == rhs.prefix_;
  return equal;
}
bool AddSceneGraphCommand::operator!=(const AddSceneGraphCommand& rhs) const { return !operator==(rhs); }

template <class Archive>
void AddSceneGraphCommand::serialize(Archive& ar, const unsigned int /*version*/)
{
  ar& BOOST_SERIALIZATION_BASE_OBJECT_NVP(Command);
  ar& BOOST_SERIALIZATION_NVP(scene_graph_);
  ar& BOOST_SERIALIZATION_NVP(joint_);
  ar& BOOST_SERIALIZATION_NVP(prefix_);
}
}  // namespace tesseract_environment

#include <tesseract_common/serialization.h>
TESSERACT_SERIALIZE_ARCHIVES_INSTANTIATE(tesseract_environment::AddSceneGraphCommand)
BOOST_CLASS_EXPORT_IMPLEMENT(tesseract_environment::AddSceneGraphCommand)
