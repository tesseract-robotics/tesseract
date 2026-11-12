/**
 * @file test_visualization_plugin.cpp
 * @brief A minimal visualization plugin for testing the loader
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

#include <tesseract/common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <memory>
#include <string>
#include <vector>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract/common/property_tree.h>
#include <tesseract/visualization/visualization.h>
#include <tesseract/visualization/visualization_factory.h>

namespace tesseract::visualization
{
class TestVisualization : public Visualization
{
public:
  bool isConnected() const override { return true; }
  void waitForConnection(long /*seconds*/ = 0) const override {}
  void plotEnvironment(const tesseract::environment::Environment& /*env*/, std::string /*ns*/ = "") override {}
  void plotEnvironmentState(const tesseract::scene_graph::SceneState& /*state*/, std::string /*ns*/ = "") override {}
  void plotTrajectory(const tesseract::common::JointTrajectory& /*traj*/,
                      const tesseract::scene_graph::StateSolver& /*state_solver*/,
                      std::string /*ns*/ = "") override
  {
  }
  void plotMarker(const Marker& /*marker*/, std::string /*ns*/ = "") override {}
  void plotMarkers(const std::vector<std::shared_ptr<Marker>>& /*markers*/, std::string /*ns*/ = "") override {}
  void clear(std::string /*ns*/ = "") override {}
  void waitForInput(std::string /*message*/ = "") override {}
};

class TestVisualizationFactory : public VisualizationFactory
{
public:
  tesseract::common::PropertyTree schema() const override
  {
    // clang-format off
    return tesseract::common::PropertyTreeBuilder()
        .attribute(tesseract::common::property_attribute::TYPE, tesseract::common::property_type::CONTAINER)
        .int32("port").minimum(0).maximum(65535).done()
        .build();
    // clang-format on
  }

protected:
  std::unique_ptr<Visualization> createImpl(const std::string& /*name*/,
                                            const tesseract::common::PropertyTree& /*config*/) const override
  {
    return std::make_unique<TestVisualization>();
  }
};

/** @brief Keeps the default, empty schema */
class TestDefaultSchemaVisualizationFactory : public VisualizationFactory
{
protected:
  std::unique_ptr<Visualization> createImpl(const std::string& /*name*/,
                                            const tesseract::common::PropertyTree& /*config*/) const override
  {
    return std::make_unique<TestVisualization>();
  }
};
}  // namespace tesseract::visualization

// NOLINTNEXTLINE
TESSERACT_ADD_VISUALIZATION_PLUGIN(tesseract::visualization::TestVisualizationFactory, TestVisualizationPlugin)
// NOLINTNEXTLINE
TESSERACT_ADD_VISUALIZATION_PLUGIN(tesseract::visualization::TestDefaultSchemaVisualizationFactory,
                                   TestDefaultSchemaVisualizationPlugin)
