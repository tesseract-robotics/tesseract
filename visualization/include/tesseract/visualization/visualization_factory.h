/**
 * @file visualization_factory.h
 * @brief Factory that visualization plugins export
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
#ifndef TESSERACT_VISUALIZATION_VISUALIZATION_FACTORY_H
#define TESSERACT_VISUALIZATION_VISUALIZATION_FACTORY_H

#include <tesseract/common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <memory>
#include <string>
#include <yaml-cpp/yaml.h>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract/common/fwd.h>
#include <boost_plugin_loader/fwd.h>
#include <boost_plugin_loader/macros.h>

// clang-format off
#define TESSERACT_ADD_VISUALIZATION_PLUGIN(DERIVED_CLASS, ALIAS)                                                       \
  EXPORT_CLASS_SECTIONED(DERIVED_CLASS, ALIAS, Plotter)
// clang-format on

namespace tesseract::visualization
{
class Visualization;
class VisualizationLoader;

/** @brief Creates Visualization objects. Each call to create() returns a new, independent object. */
class VisualizationFactory
{
public:
  using Ptr = std::shared_ptr<VisualizationFactory>;
  using ConstPtr = std::shared_ptr<const VisualizationFactory>;

  virtual ~VisualizationFactory() = default;

  /**
   * @brief Create a visualization object
   * @param name The name of the object
   * @param config The configuration, which is validated against schema()
   * @return The new object
   */
  std::unique_ptr<Visualization> create(const std::string& name, const YAML::Node& config) const;

  /** @brief The schema used to validate the configuration. By default it is empty. */
  virtual tesseract::common::PropertyTree schema() const;

protected:
  virtual std::unique_ptr<Visualization> createImpl(const std::string& name,
                                                    const tesseract::common::PropertyTree& config) const = 0;

  static std::string getSection();
  friend class boost_plugin_loader::PluginLoader;
  friend class VisualizationLoader;
};

}  // namespace tesseract::visualization

#endif  // TESSERACT_VISUALIZATION_VISUALIZATION_FACTORY_H
