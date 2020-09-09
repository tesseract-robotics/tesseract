/**
 * @file tesseract.h
 * @brief This is a container class for all tesseract packages. Provides
 * methods to simplify construction of commonly used features.
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
#ifndef TESSERACT_TESSERACT_H
#define TESSERACT_TESSERACT_H

#include <tesseract_common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <boost/filesystem/path.hpp>
#include <memory>
#include <unordered_map>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract/tesseract_init_info.h>
#include <tesseract_environment/core/environment.h>
#include <tesseract_scene_graph/srdf_model.h>
#include <tesseract_scene_graph/utils.h>
#include <tesseract_scene_graph/resource_locator.h>
#include <tesseract/manipulator_manager.h>

namespace tesseract
{
/**
 * @brief The Tesseract class
 *
 * This is a container class which hold objects needed for motion planning.
 * It also provides several construction methods for loading from urdf, srdf
 *
 */
class Tesseract
{
public:
  using Ptr = std::shared_ptr<Tesseract>;
  using ConstPtr = std::shared_ptr<const Tesseract>;

  Tesseract();
  virtual ~Tesseract() = default;
  Tesseract(const Tesseract&) = default;
  Tesseract& operator=(const Tesseract&) = default;
  Tesseract(Tesseract&&) = default;
  Tesseract& operator=(Tesseract&&) = default;

  bool isInitialized() const;

  bool init(tesseract_scene_graph::SceneGraph::Ptr scene_graph);
  bool init(tesseract_scene_graph::SceneGraph::Ptr scene_graph, tesseract_scene_graph::SRDFModel::Ptr srdf_model);
  bool init(const std::string& urdf_string, const tesseract_scene_graph::ResourceLocator::Ptr& locator);
  bool init(const std::string& urdf_string,
            const std::string& srdf_string,
            const tesseract_scene_graph::ResourceLocator::Ptr& locator);
  bool init(const boost::filesystem::path& urdf_path, const tesseract_scene_graph::ResourceLocator::Ptr& locator);
  bool init(const boost::filesystem::path& urdf_path,
            const boost::filesystem::path& srdf_path,
            const tesseract_scene_graph::ResourceLocator::Ptr& locator);

  bool init(const tesseract_environment::Environment& env, const ManipulatorManager& manipulator_manager);

  bool init(const TesseractInitInfo::Ptr& init_info);

  /** @brief Clone the Tesseract and all of the internals
   * @return A Ptr to the cloned Tesseract   */
  Tesseract::Ptr clone() const;

  /** @brief reset to initialized state */
  bool reset();

  tesseract_environment::Environment::Ptr getEnvironment();
  tesseract_environment::Environment::ConstPtr getEnvironment() const;

  ManipulatorManager::Ptr getManipulatorManager();
  ManipulatorManager::ConstPtr getManipulatorManager() const;

  void setResourceLocator(tesseract_scene_graph::ResourceLocator::Ptr locator);
  const tesseract_scene_graph::ResourceLocator::Ptr& getResourceLocator() const;

private:
  bool initialized_;
  tesseract_environment::Environment::Ptr environment_;
  ManipulatorManager::Ptr manipulator_manager_;
  TesseractInitInfo::Ptr init_info_;

  bool registerDefaultContactManagers();

  void clear();
};
}  // namespace tesseract
#endif  // TESSERACT_TESSERACT_H
