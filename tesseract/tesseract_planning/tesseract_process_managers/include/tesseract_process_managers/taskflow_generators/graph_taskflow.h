/**
 * @file graph_taskflow.h
 * @brief Creates a directed graph taskflow
 *
 * @author Levi Armstrong
 * @date August 13. 2020
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

#ifndef TESSERACT_PROCESS_MANAGERS_GRAPH_TASKFLOW_H
#define TESSERACT_PROCESS_MANAGERS_GRAPH_TASKFLOW_H

#include <tesseract_common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <vector>
#include <memory>
#include <taskflow/taskflow.hpp>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract_process_managers/core/taskflow_generator.h>
#include <tesseract_process_managers/core/task_generator.h>

namespace tesseract_planning
{
/**
 * @brief This class generates taskflow graph. It allows you to connect different channels from the source to the
 *        destination process
 */
class GraphTaskflow : public TaskflowGenerator
{
public:
  using UPtr = std::unique_ptr<GraphTaskflow>;

  enum class NodeType : int
  {
    TASK = 0,
    CONDITIONAL = 1
  };

  enum class SourceChannel : int
  {
    NONE = 0,
    ON_SUCCESS = 1,
    ON_FAILURE = 2
  };

  enum class DestinationChannel : int
  {
    PROCESS_NODE = 0,
    DONE_CALLBACK = 1,
    ERROR_CALLBACK = 2
  };

  struct Edge
  {
    SourceChannel src_channel;
    int dest{ -1 };
    DestinationChannel dest_channel;
  };

  struct Node
  {
    TaskGenerator::UPtr process;
    NodeType process_type;
    std::vector<Edge> edges;
  };

  GraphTaskflow(std::string name = "GraphTaskflow");
  ~GraphTaskflow() override = default;
  GraphTaskflow(const GraphTaskflow&) = delete;
  GraphTaskflow& operator=(const GraphTaskflow&) = delete;
  GraphTaskflow(GraphTaskflow&&) = delete;
  GraphTaskflow& operator=(GraphTaskflow&&) = delete;

  const std::string& getName() const override;

  TaskflowContainer generateTaskflow(TaskInput input, TaskflowVoidFn done_cb, TaskflowVoidFn error_cb) override;

  /**
   * @brief Add a node to the taskflow graph along with setting the process type.
   * @param process The process generator assigned to the node
   * @param process_type The process type assigned to the node
   * @return The node ID which should be used with adding edges
   */
  int addNode(TaskGenerator::UPtr process, NodeType process_type);

  /**
   * @brief Add an edge to the taskflow graph
   * @param src The source node ID
   * @param src_channel The source channel (ON_SUCCESS, ON_FAILURE)
   * @param dest The destination node ID (This is only required for destination channels PROCESS_NODE) othewise pass a
   * -1
   * @param dest_channel The destination channel to connect with the source channel (PROCESS_NODE, DONE_CALLBACK,
   * ERROR_CALLBACK).
   */
  void addEdge(int src, SourceChannel src_channel, int dest, DestinationChannel dest_channel);

private:
  std::vector<Node> nodes_;
  std::string name_;
};

}  // namespace tesseract_planning

#endif  // TESSERACT_PROCESS_MANAGERS_GRAPH_TASKFLOW_H
