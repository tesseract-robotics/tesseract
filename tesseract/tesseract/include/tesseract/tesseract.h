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
#include <vector>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract/tesseract_init_info.h>
#include <tesseract_environment/core/environment.h>
#include <tesseract_scene_graph/utils.h>
#include <tesseract_scene_graph/resource_locator.h>
#include <tesseract_command_language/manipulator_info.h>

namespace tesseract
{
/**
 * @brief Function signature for adding additional callbacks for looking up TCP information
 *
 * The function should throw and exception if not located
 */
using FindTCPCallbackFn = std::function<Eigen::Isometry3d(const tesseract_planning::ManipulatorInfo&)>;

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

  bool init(const tesseract_environment::Environment& env);
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

  bool init(const TesseractInitInfo::Ptr& init_info);

  /** @brief Clone the Tesseract and all of the internals
   * @return A Ptr to the cloned Tesseract   */
  Tesseract::Ptr clone() const;

  /** @brief reset to initialized state */
  bool reset();

  /** @brief clear content and uninitialize */
  void clear();

  tesseract_environment::Environment::Ptr getEnvironment();
  tesseract_environment::Environment::ConstPtr getEnvironment() const;

  DEPRECATED("Please use getEnvironment->getManipulatorManager()")
  tesseract_environment::ManipulatorManager::Ptr getManipulatorManager();
  DEPRECATED("Please use getEnvironment->getManipulatorManager()")
  tesseract_environment::ManipulatorManager::ConstPtr getManipulatorManager() const;

  /**
   * @brief Find tool center point provided in the manipulator info
   *
   * If manipulator information tcp is defined as a string it does the following
   *    - First check if manipulator info is empty, if so return identity
   *    - Next if not empty, it checks if the manipulator manager has tcp defined for the manipulator group
   *    - Next if not found, it looks up the tcp name in the EnvState along with manipulator tip link to calculate tcp
   *    - Next if not found, it leverages the user defind callbacks to try an locate the tcp information.
   *    - Next throw an exception, because no tcp information was located.
   *
   * @param manip_info The manipulator info
   * @return The tool center point
   */
  Eigen::Isometry3d findTCP(const tesseract_planning::ManipulatorInfo& manip_info) const;

  /**
   * @brief This allows for user defined callbacks for looking up TCP information
   * @param fn User defind callback function for locating TCP information
   */
  void addFindTCPCallback(FindTCPCallbackFn fn);

  void setResourceLocator(tesseract_scene_graph::ResourceLocator::Ptr locator);
  const tesseract_scene_graph::ResourceLocator::Ptr& getResourceLocator() const;

private:
  bool initialized_;
  tesseract_environment::Environment::Ptr environment_;
  TesseractInitInfo::Ptr init_info_;
  std::vector<FindTCPCallbackFn> find_tcp_cb_;

  bool registerDefaultContactManagers();
};
}  // namespace tesseract
#endif  // TESSERACT_TESSERACT_H
