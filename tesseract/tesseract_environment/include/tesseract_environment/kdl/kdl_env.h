/**
 * @file kdl_env.h
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
#ifndef TESSERACT_ENVIRONMENT_KDL_ENV_H
#define TESSERACT_ENVIRONMENT_KDL_ENV_H

#include <tesseract_environment/core/environment.h>

namespace tesseract_environment
{
class KDLEnv : public Environment
{
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  using Ptr = std::shared_ptr<KDLEnv>;
  using ConstPtr = std::shared_ptr<const KDLEnv>;

  KDLEnv() = default;
  ~KDLEnv() override = default;
  KDLEnv(const KDLEnv&) = delete;
  KDLEnv& operator=(const KDLEnv&) = delete;
  KDLEnv(KDLEnv&&) = delete;
  KDLEnv& operator=(KDLEnv&&) = delete;

  bool init(tesseract_scene_graph::SceneGraph::Ptr scene_graph) override;
};

}  // namespace tesseract_environment

#endif  // TESSERACT_ENVIRONMENT_KDL_ENV_H
