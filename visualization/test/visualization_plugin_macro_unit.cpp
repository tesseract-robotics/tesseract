/**
 * @file visualization_plugin_macro_unit.cpp
 * @brief Compiles TESSERACT_ADD_VISUALIZATION_PLUGIN, which no other code built by default uses
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
#include <gtest/gtest.h>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract/visualization/visualization.h>

namespace
{
class TestVisualization : public tesseract::visualization::Visualization
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
  void plotMarker(const tesseract::visualization::Marker& /*marker*/, std::string /*ns*/ = "") override {}
  void plotMarkers(const std::vector<std::shared_ptr<tesseract::visualization::Marker>>& /*markers*/,
                   std::string /*ns*/ = "") override
  {
  }
  void clear(std::string /*ns*/ = "") override {}
  void waitForInput(std::string /*message*/ = "") override {}
};
}  // namespace

// NOLINTNEXTLINE
TESSERACT_ADD_VISUALIZATION_PLUGIN(TestVisualization, TestVisualizationPlugin)

TEST(TesseractVisualizationPluginMacroUnit, ExportsAnInstance)  // NOLINT
{
  const tesseract::visualization::Visualization& plugin = TestVisualizationPlugin;
  EXPECT_TRUE(plugin.isConnected());
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);

  return RUN_ALL_TESTS();
}
