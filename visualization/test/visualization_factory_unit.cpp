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
#include <algorithm>
#include <memory>
#include <string>
#include <vector>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract/visualization/visualization.h>
#include <tesseract/visualization/visualization_factory.h>
#include <tesseract/visualization/visualization_loader.h>

using tesseract::visualization::Visualization;
using tesseract::visualization::VisualizationFactory;
using tesseract::visualization::VisualizationLoader;

/** @brief Exposes the factory's protected plugin section */
struct FactorySection : VisualizationFactory
{
  using VisualizationFactory::getSection;
};

static void configure(VisualizationLoader& loader)
{
  loader.search_paths.emplace_back(TEST_PLUGIN_DIR);
  loader.search_libraries.emplace_back("tesseract_visualization_test_plugin");
}

TEST(TesseractVisualizationFactoryUnit, EachLoaderOwnsOneInstance)  // NOLINT
{
  VisualizationLoader loader;
  configure(loader);

  auto first = loader.get("TestVisualizationPlugin");
  ASSERT_NE(first, nullptr);
  EXPECT_EQ(first.get(), loader.get("TestVisualizationPlugin").get());
  EXPECT_TRUE(first->isConnected());
}

TEST(TesseractVisualizationFactoryUnit, LoadersAreIndependent)  // NOLINT
{
  VisualizationLoader first_loader;
  configure(first_loader);
  VisualizationLoader second_loader;
  configure(second_loader);

  auto first = first_loader.get("TestVisualizationPlugin");
  auto second = second_loader.get("TestVisualizationPlugin");
  ASSERT_NE(first, nullptr);
  ASSERT_NE(second, nullptr);
  EXPECT_NE(first.get(), second.get());
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

TEST(TesseractVisualizationFactoryUnit, DefaultSchemaAcceptsNoConfig)  // NOLINT
{
  VisualizationLoader loader;
  configure(loader);
  auto visualization = loader.get("TestDefaultSchemaVisualizationPlugin");
  ASSERT_NE(visualization, nullptr);
  EXPECT_TRUE(visualization->isConnected());
}

TEST(TesseractVisualizationFactoryUnit, ListsPluginsInTheFactorySection)  // NOLINT
{
  VisualizationLoader loader;
  configure(loader);

  // TESSERACT_ADD_VISUALIZATION_PLUGIN exports into the section the factory declares
  const std::vector<std::string> plugins = loader.getAvailablePlugins(FactorySection::getSection());
  EXPECT_NE(std::find(plugins.begin(), plugins.end(), "TestVisualizationPlugin"), plugins.end());
  EXPECT_NE(std::find(plugins.begin(), plugins.end(), "TestDefaultSchemaVisualizationPlugin"), plugins.end());
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);

  return RUN_ALL_TESTS();
}
