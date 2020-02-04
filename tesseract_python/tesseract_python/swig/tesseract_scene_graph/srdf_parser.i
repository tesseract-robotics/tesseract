/**
 * @file srdf_parser.i
 * @brief SWIG interface file for tesseract_scene_graph/parser/srdf_parser.h
 *
 * @author Hervé Audren
 * @date January 20, 2020
 * @version TODO
 * @bug No known bugs
 *
 * @copyright Copyright (c) 2020, Ascent Robotics Inc.
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

%feature(flatnested)

%{
#include <tesseract_scene_graph/parser/srdf_parser.h>
%}

%shared_ptr(tesseract_scene_graph::SRDFModel)
%template(GroupVector) std::vector<tesseract_scene_graph::SRDFModel::Group>;
%template(GroupStateVector) std::vector<tesseract_scene_graph::SRDFModel::GroupState>;

namespace tesseract_scene_graph
{
class SRDFModel
{
public:
  using Ptr = std::shared_ptr<SRDFModel>;
  using ConstPtr = std::shared_ptr<const SRDFModel>;

  struct Group {
    std::string name_;
    std::vector<std::string> joints_;
    std::vector<std::string> links_;
    std::vector<std::pair<std::string, std::string> > chains_;
    std::vector<std::string> subgroups_;
  };

  struct GroupState {
    std::string name_;
    std::string group_;
    std::map<std::string, std::vector<double> > joint_values_;
  };

  const std::string& getName() const;

  const std::vector<Group>& getGroups() const;

  const std::vector<GroupState>& getGroupStates() const;
};

}  // namespace tesseract_scene_graph
