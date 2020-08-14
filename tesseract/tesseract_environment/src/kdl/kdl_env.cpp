/**
 * @file kdl_env.cpp
 * @brief Tesseract environment kdl implementation.
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
#include "tesseract_environment/kdl/kdl_env.h"
#include "tesseract_environment/kdl/kdl_state_solver.h"

namespace tesseract_environment
{
bool KDLEnv::init(tesseract_scene_graph::SceneGraph::Ptr scene_graph)
{
  return Environment::create<KDLStateSolver>(scene_graph);
}

Environment::Ptr KDLEnv::clone() const
{
  auto cloned_env = std::make_shared<KDLEnv>();

  std::lock_guard<std::mutex> lock(mutex_);
  cloned_env->initialized_ = initialized_;
  cloned_env->revision_ = revision_;
  cloned_env->commands_ = commands_;
  cloned_env->scene_graph_ = scene_graph_->clone();
  cloned_env->scene_graph_const_ = cloned_env->scene_graph_;
  cloned_env->current_state_ = std::make_shared<EnvState>(*current_state_);
  cloned_env->state_solver_ = state_solver_->clone();
  cloned_env->link_names_ = link_names_;
  cloned_env->joint_names_ = joint_names_;
  cloned_env->active_link_names_ = active_link_names_;
  cloned_env->active_joint_names_ = active_joint_names_;
  cloned_env->is_contact_allowed_fn_ = is_contact_allowed_fn_;
  if (discrete_manager_)
    cloned_env->discrete_manager_ = discrete_manager_->clone();
  if (continuous_manager_)
    cloned_env->continuous_manager_ = continuous_manager_->clone();
  cloned_env->discrete_manager_name_ = discrete_manager_name_;
  cloned_env->continuous_manager_name_ = continuous_manager_name_;
  cloned_env->discrete_factory_ = discrete_factory_;
  cloned_env->continuous_factory_ = continuous_factory_;

  return cloned_env;
}

}  // namespace tesseract_environment
