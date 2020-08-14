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
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract_process_managers/taskflow_generator.h>
#include <tesseract_process_managers/process_generator.h>

namespace tesseract_planning
{
/** @brief This class generates taskflows for a sequential failure tree. Each process is executed in order until one
 * succeeds. Between each process, the validator tasks are executed (if not empty). For a process to succeed, the
 * process itself must succeed and all of the validators must succeed*/
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
    ProcessGenerator::UPtr process;
    NodeType process_type;
    std::vector<Edge> edges;
  };

  GraphTaskflow(std::string name = "GraphTaskflow");
  ~GraphTaskflow() override = default;

  const std::string& getName() const override;

  tf::Taskflow& generateTaskflow(ProcessInput input,
                                 std::function<void()> done_cb,
                                 std::function<void()> error_cb) override;

  void abort() override;

  void reset() override;

  int addNode(ProcessGenerator::UPtr process, NodeType process_type);
  void addEdge(int src, SourceChannel src_channel, int dest, DestinationChannel dest_channel);

private:
  /** @brief If true, all tasks return immediately. Workaround for https://github.com/taskflow/taskflow/issues/201 */
  std::atomic<bool> abort_{ false };

  std::vector<Node> nodes_;
  std::vector<std::shared_ptr<tf::Taskflow>> taskflow_objects_;
  std::vector<tf::Task> process_tasks_;
  std::string name_;
};

}  // namespace tesseract_planning

#endif  // TESSERACT_PROCESS_MANAGERS_GRAPH_TASKFLOW_H
