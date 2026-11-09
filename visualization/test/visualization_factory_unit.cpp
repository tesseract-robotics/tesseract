/**
 * @file visualization_factory_unit.cpp
 * @brief Tests loading visualization plugins through their factories
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
#include <memory>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract/visualization/visualization.h>
#include <tesseract/visualization/visualization_loader.h>

using tesseract::visualization::Visualization;
using tesseract::visualization::VisualizationLoader;

static void configure(VisualizationLoader& loader)
{
  loader.search_paths.emplace_back(TEST_PLUGIN_DIR);
  loader.search_libraries.emplace_back("tesseract_visualization_test_plugin");
}

TEST(TesseractVisualizationFactoryUnit, EachGetCreatesAnIndependentObject)  // NOLINT
{
  VisualizationLoader loader;
  configure(loader);

  auto first = loader.get("TestVisualizationPlugin");
  auto second = loader.get("TestVisualizationPlugin");
  ASSERT_NE(first, nullptr);
  ASSERT_NE(second, nullptr);
  EXPECT_NE(first.get(), second.get());
  EXPECT_TRUE(first->isConnected());
}

TEST(TesseractVisualizationFactoryUnit, ObjectMayOutliveTheLoader)  // NOLINT
{
  std::shared_ptr<Visualization> visualization;
  {
    VisualizationLoader loader;
    configure(loader);
    visualization = loader.get("TestVisualizationPlugin");
  }
  ASSERT_NE(visualization, nullptr);
  EXPECT_TRUE(visualization->isConnected());
}

TEST(TesseractVisualizationFactoryUnit, UnknownPluginReturnsNull)  // NOLINT
{
  VisualizationLoader loader;
  configure(loader);
  EXPECT_EQ(loader.get("DoesNotExist"), nullptr);
}

TEST(TesseractVisualizationFactoryUnit, InvalidConfigReturnsNull)  // NOLINT
{
  VisualizationLoader loader;
  configure(loader);
  EXPECT_EQ(loader.get("TestVisualizationPlugin", YAML::Load("port: 70000")), nullptr);
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);

  return RUN_ALL_TESTS();
}
