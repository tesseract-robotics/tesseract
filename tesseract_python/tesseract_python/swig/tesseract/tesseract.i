/**
 * @file tesseract.i
 * @brief SWIG interface file for tesseract/tesseract.h
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
#include <tesseract/tesseract.h>
%}

%shared_ptr(tesseract::Tesseract)

namespace tesseract
{
class Tesseract
{
public:
  using Ptr = std::shared_ptr<Tesseract>;
  using ConstPtr = std::shared_ptr<const Tesseract>;

  Tesseract();
  virtual ~Tesseract() = default;

  bool isInitialized() const;

  bool init(tesseract_scene_graph::SceneGraph::Ptr scene_graph);
  //bool init(tesseract_scene_graph::SceneGraph::Ptr scene_graph, tesseract_scene_graph::SRDFModel::ConstPtr srdf_model);
  bool init(const std::string& urdf_string, const tesseract_scene_graph::ResourceLocator::Ptr& locator);
  bool init(const std::string& urdf_string,
            const std::string& srdf_string,
            const tesseract_scene_graph::ResourceLocator::Ptr& locator);
  //bool init(const boost::filesystem::path& urdf_path, tesseract_scene_graph::ResourceLocator::Ptr locator);
  //bool init(const boost::filesystem::path& urdf_path,
  //          const boost::filesystem::path& srdf_path,
  //          tesseract_scene_graph::ResourceLocator::Ptr locator);

  const tesseract_scene_graph::SRDFModel::Ptr& getSRDFModel() const;
  const tesseract_scene_graph::SRDFModel::ConstPtr& getSRDFModelConst() const;

  const tesseract_environment::Environment::Ptr& getEnvironment();
  const tesseract_environment::Environment::ConstPtr& getEnvironmentConst() const;

  const ForwardKinematicsManager::Ptr& getFwdKinematicsManager();
  const ForwardKinematicsManager::ConstPtr& getFwdKinematicsManagerConst() const;

  const InverseKinematicsManager::Ptr& getInvKinematicsManager();
  const InverseKinematicsManager::ConstPtr& getInvKinematicsManagerConst() const;

  tesseract_scene_graph::SRDFModel::GroupStates& getGroupStates();
  const tesseract_scene_graph::SRDFModel::GroupStates& getGroupStatesConst() const;

  tesseract_scene_graph::SRDFModel::GroupTCPs& getGroupTCPs();
  const tesseract_scene_graph::SRDFModel::GroupTCPs& getGroupTCPs() const;

};

}  // namespace tesseract
