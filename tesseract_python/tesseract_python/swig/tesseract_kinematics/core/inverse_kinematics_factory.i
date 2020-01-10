/**
 * @file inverse_kinematics_factory.i
 * @brief SWIG interface file for tesseract_kinematics/core/inverse_kinematics_factory.h
 *
 * @author John Wason
 * @date December 10, 2019
 * @version TODO
 * @bug No known bugs
 *
 * @copyright Copyright (c) 2019, Wason Technology, LLC
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

%{
#include <tesseract_kinematics/core/inverse_kinematics_factory.h>
%}

%shared_ptr(tesseract_kinematics::InverseKinematicsFactory)

namespace tesseract_kinematics
{
enum class InverseKinematicsFactoryType
{
  CHAIN = 0,
  TREE = 1,
  GRAPH = 2
};

class InverseKinematicsFactory
{
public:
  using Ptr = std::shared_ptr<InverseKinematicsFactory>;
  using ConstPtr = std::shared_ptr<const InverseKinematicsFactory>;

  virtual ~InverseKinematicsFactory() = default;

  virtual const std::string& getName() const = 0;

  virtual InverseKinematicsFactoryType getType() const = 0;

  virtual InverseKinematics::Ptr create(tesseract_scene_graph::SceneGraph::ConstPtr scene_graph,
                                        const std::string& base_link,
                                        const std::string& tip_link,
                                        const std::string name) const;

  virtual InverseKinematics::Ptr
  create(tesseract_scene_graph::SceneGraph::ConstPtr scene_graph,
         const std::vector<std::string>& joint_names,
         const std::string name,
         std::unordered_map<std::string, double> start_state = std::unordered_map<std::string, double>()) const;
};

}  // namespace tesseract_kinematics
