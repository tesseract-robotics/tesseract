/**
 * @file plugin_loader_unit.h
 * @brief Plugin Loader Unit Tests
 *
 * @author Levi Armstrong
 * @date March 25, 2021
 * @version TODO
 * @bug No known bugs
 *
 * @copyright Copyright (c) 2021, Southwest Research Institute
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
#include <tesseract_common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <gtest/gtest.h>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract_common/plugin_loader.h>
#include "test_plugin_base.h"

TEST(TesseractPluginLoaderUnit, LoadTestPlugin)  // NOLINT
{
  {
    tesseract_common::PluginLoader loader(std::string(TEST_PLUGIN_DIR), "tesseract_common_test_plugin_multiply");
    EXPECT_TRUE(loader.isClassAvailable<tesseract_common::TestPluginBase>("plugin"));
    std::shared_ptr<tesseract_common::TestPluginBase> plugin =
        loader.createSharedInstance<tesseract_common::TestPluginBase>("plugin");
    EXPECT_TRUE(plugin != nullptr);
    EXPECT_NEAR(plugin->multiply(5, 5), 25, 1e-8);
  }

  {
    tesseract_common::PluginLoader loader("tesseract_common_test_plugin_multiply");
    EXPECT_TRUE(loader.isClassAvailable<tesseract_common::TestPluginBase>("plugin"));
    std::shared_ptr<tesseract_common::TestPluginBase> plugin =
        loader.createSharedInstance<tesseract_common::TestPluginBase>("plugin");
    EXPECT_TRUE(plugin != nullptr);
    EXPECT_NEAR(plugin->multiply(5, 5), 25, 1e-8);
  }
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);

  return RUN_ALL_TESTS();
}
