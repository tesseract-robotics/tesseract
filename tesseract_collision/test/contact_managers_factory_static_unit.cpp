/**
 * @file contact_managers_factory_unit.cpp
 * @brief Tesseract collision contact managers factory test
 *
 * @author Levi Armstrong
 * @date October 25, 2021
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
#include <fstream>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract_collision/core/contact_managers_plugin_factory.h>
#include <tesseract_collision/bullet/bullet_factories.h>

using namespace tesseract_collision;

TEST(TesseractContactManagersFactoryUnit, StaticLoadPlugin)  // NOLINT
{
  tesseract_common::PluginLoader::addSymbolLibraryToSearchLibrariesEnv(
      tesseract_collision::tesseract_collision_bullet::BulletFactoriesAnchor(), "TESSERACT_CONTACT_MANAGERS_PLUGINS");

  std::string config = R"(contact_manager_plugins:
                            search_paths:
                              - /usr/local/lib
                            search_libraries:
                              - tesseract_collision_bullet_factories_not_there
                              - tesseract_collision_fcl_factories_not_there
                            discrete_plugins:
                              default: BulletDiscreteBVHManager
                              plugins:
                                BulletDiscreteBVHManager:
                                  class: BulletDiscreteBVHManagerFactory
                                BulletDiscreteSimpleManager:
                                  class: BulletDiscreteSimpleManagerFactory
                                FCLDiscreteBVHManager:
                                  class: FCLDiscreteBVHManagerFactory
                            continuous_plugins:
                              default: BulletCastBVHManager
                              plugins:
                                BulletCastBVHManager:
                                  class: BulletCastBVHManagerFactory
                                BulletCastSimpleManager:
                                  class: BulletCastSimpleManagerFactory)";

  ContactManagersPluginFactory factory(config);
  factory.clearSearchLibraries();
  factory.clearSearchPaths();
  YAML::Node plugin_config = YAML::Load(config);

  DiscreteContactManager::UPtr cm = factory.createDiscreteContactManager("BulletDiscreteBVHManager");
  EXPECT_TRUE(cm != nullptr);
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);

  return RUN_ALL_TESTS();
}
