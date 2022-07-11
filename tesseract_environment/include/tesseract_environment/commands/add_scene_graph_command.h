/**
 * @file add_scene_graph_command.h
 * @brief Used to add scene graph to the environment
 *
 * @author Levi Armstrong
 * @date Dec 18, 2017
 * @version TODO
 * @bug No known bugs
 *
 * @copyright Copyright (c) 2017, Southwest Research Institute
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
#ifndef TESSERACT_ENVIRONMENT_ADD_SCENE_GRAPH_COMMAND_H
#define TESSERACT_ENVIRONMENT_ADD_SCENE_GRAPH_COMMAND_H

#include <tesseract_common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <boost/serialization/access.hpp>
#include <memory>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract_environment/command.h>
#include <tesseract_scene_graph/graph.h>

namespace tesseract_environment
{
class AddSceneGraphCommand : public Command
{
public:
  using Ptr = std::shared_ptr<AddSceneGraphCommand>;
  using ConstPtr = std::shared_ptr<const AddSceneGraphCommand>;

  AddSceneGraphCommand() : Command(CommandType::ADD_SCENE_GRAPH){};
  /**
   * @brief Merge a graph into the current environment
   * @param scene_graph Const ref to the graph to be merged (said graph will be copied)
   * @param prefix string Will be prepended to every link and joint of the merged graph
   * @return Return False if any link or joint name collides with current environment, otherwise True
   * Merge a subgraph into the current environment, considering that the root of the merged graph is attached to the
   * root of the environment by a fixed joint and no displacement. Every joint and link of the subgraph will be copied
   * into the environment graph. The prefix argument is meant to allow adding multiple copies of the same subgraph with
   * different names
   */
  AddSceneGraphCommand(const tesseract_scene_graph::SceneGraph& scene_graph, std::string prefix = "");

  /**
   * @brief Merge a graph into the current environment
   * @param scene_graph Const ref to the graph to be merged (said graph will be copied)
   * @param root_joint Const ptr to the joint that connects current environment with root of the merged graph
   * @param prefix string Will be prepended to every link and joint of the merged graph
   * @return Return False if any link or joint name collides with current environment, otherwise True
   * Merge a subgraph into the current environment. Every joint and link of the subgraph will be copied into the
   * environment graph. The prefix argument is meant to allow adding multiple copies of the same subgraph with different
   * names
   */
  AddSceneGraphCommand(const tesseract_scene_graph::SceneGraph& scene_graph,
                       const tesseract_scene_graph::Joint& joint,
                       std::string prefix = "");

  const tesseract_scene_graph::SceneGraph::ConstPtr& getSceneGraph() const { return scene_graph_; }
  const tesseract_scene_graph::Joint::ConstPtr& getJoint() const { return joint_; }
  const std::string& getPrefix() const { return prefix_; }

  bool operator==(const AddSceneGraphCommand& rhs) const;
  bool operator!=(const AddSceneGraphCommand& rhs) const;

private:
  tesseract_scene_graph::SceneGraph::ConstPtr scene_graph_;
  tesseract_scene_graph::Joint::ConstPtr joint_;
  std::string prefix_;

  friend class boost::serialization::access;
  template <class Archive>
  void serialize(Archive& ar, const unsigned int version);  // NOLINT
};
}  // namespace tesseract_environment
#include <boost/serialization/export.hpp>
#include <boost/serialization/tracking.hpp>
BOOST_CLASS_EXPORT_KEY2(tesseract_environment::AddSceneGraphCommand, "AddSceneGraphCommand")

#endif  // TESSERACT_ENVIRONMENT_ADD_SCENE_GRAPH_COMMAND_H
