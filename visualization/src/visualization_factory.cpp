/**
 * @file visualization_factory.cpp
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

#include <tesseract/common/property_tree.h>
#include <tesseract/visualization/visualization.h>
#include <tesseract/visualization/visualization_factory.h>

namespace tesseract::visualization
{
std::string VisualizationFactory::getSection() { return "Plotter"; }

tesseract::common::PropertyTree VisualizationFactory::schema() const
{
  return tesseract::common::PropertyTreeBuilder().build();
}

std::unique_ptr<Visualization> VisualizationFactory::create(const std::string& name, const YAML::Node& config) const
{
  auto validated_config = schema();
  auto errors = validated_config.applyConfig(config);
  if (!errors.empty())
    throw tesseract::common::PropertyTreeValidationError(std::move(errors));

  return createImpl(name, validated_config);
}

}  // namespace tesseract::visualization
