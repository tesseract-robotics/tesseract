/**
 * @file tesseract_common_serialization_unit.cpp
 * @brief Tests serialization of types in tesseract_common
 *
 * @author Levi Armstrong
 * @author Matthew Powelson
 * @date March 16, 2022
 * @version TODO
 * @bug No known bugs
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
#include <boost/serialization/shared_ptr.hpp>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract_common/serialization.h>
#include <tesseract_common/unit_test_utils.h>
#include <tesseract_common/utils.h>
#include <tesseract_common/allowed_collision_matrix.h>
#include <tesseract_common/collision_margin_data.h>

using namespace tesseract_common;

TEST(TesseractCommonSerializeUnit, AllowedCollisionMatrix)  // NOLINT
{
  auto object = std::make_shared<AllowedCollisionMatrix>();
  tesseract_common::testSerialization<AllowedCollisionMatrix>(*object, "EmptyAllowedCollisionMatrix");
  object->addAllowedCollision("link_1", "link2", "reason1");
  object->addAllowedCollision("link_2", "link1", "reason2");
  object->addAllowedCollision("link_4", "link3", "reason3");
  object->addAllowedCollision("link_5", "link2", "reason4");
  tesseract_common::testSerialization<AllowedCollisionMatrix>(*object, "AllowedCollisionMatrix");
}

TEST(TesseractCommonSerializeUnit, CalibrationInfo)  // NOLINT
{
  auto object = std::make_shared<CalibrationInfo>();
  tesseract_common::testSerialization<CalibrationInfo>(*object, "EmptyCalibrationInfo");
  object->joints["test"].setIdentity();
  object->joints["test"].translate(Eigen::Vector3d(2, 4, 8));
  tesseract_common::testSerialization<CalibrationInfo>(*object, "CalibrationInfo");
}

TEST(TesseractCommonSerializeUnit, CollisionMarginData)  // NOLINT
{
  auto object = std::make_shared<CollisionMarginData>();
  tesseract_common::testSerialization<CollisionMarginData>(*object, "EmptyCollisionMarginData");
  object->setPairCollisionMargin("link_1", "link2", 1.1);
  object->setPairCollisionMargin("link_2", "link1", 2.2);
  object->setPairCollisionMargin("link_4", "link3", 3.3);
  object->setPairCollisionMargin("link_5", "link2", -4.4);
  tesseract_common::testSerialization<CollisionMarginData>(*object, "CollisionMarginData");
}

TEST(TesseractCommonSerializeUnit, ContactManagersPluginInfo)  // NOLINT
{
  auto object = std::make_shared<ContactManagersPluginInfo>();
  object->search_paths.insert("path 1");
  object->search_paths.insert("path 2");
  object->search_libraries.insert("search_libraries 1");
  object->search_libraries.insert("search_libraries 2");
  object->search_libraries.insert("search_libraries 3");

  {
    PluginInfoContainer container;
    PluginInfo plugin;
    plugin.class_name = "test_class_name";
    plugin.config["test"] = "value";
    object->discrete_plugin_infos.default_plugin = "test_string";
    object->discrete_plugin_infos.plugins["plugin_key"] = plugin;
  }
  {
    PluginInfoContainer container;
    PluginInfo plugin;
    plugin.class_name = "test_class_name 2";
    plugin.config["test2"] = "value2";
    object->continuous_plugin_infos.default_plugin = "test_string2";
    object->continuous_plugin_infos.plugins["plugin_key2"] = plugin;
  }

  tesseract_common::testSerialization<ContactManagersPluginInfo>(*object, "ContactManagersPluginInfo");
}

TEST(TesseractCommonSerializeUnit, KinematicsPluginInfo)  // NOLINT
{
  auto object = std::make_shared<KinematicsPluginInfo>();
  object->search_paths.insert("path 1");
  object->search_paths.insert("path 2");
  object->search_libraries.insert("search_libraries 1");
  object->search_libraries.insert("search_libraries 2");
  object->search_libraries.insert("search_libraries 3");

  {
    PluginInfo plugin;
    plugin.class_name = "test_class_name";
    plugin.config["test"] = "value";
    object->fwd_plugin_infos["plugin 1"].default_plugin = "test_string";
    object->fwd_plugin_infos["plugin 1"].plugins["plugin_key"] = plugin;
    object->fwd_plugin_infos["plugin 2"].default_plugin = "test_string2";
    object->fwd_plugin_infos["plugin 2"].plugins["plugin_key2"] = plugin;
  }
  {
    PluginInfo plugin;
    plugin.class_name = "test_class_name 2";
    plugin.config["test2"] = "value2";
    object->inv_plugin_infos["inv plugin 1"].default_plugin = "test_string3";
    object->inv_plugin_infos["inv plugin 1"].plugins["plugin_key3"] = plugin;
    object->inv_plugin_infos["inv plugin 2"].default_plugin = "test_string4";
    object->inv_plugin_infos["inv plugin 2"].plugins["plugin_key4"] = plugin;
  }

  tesseract_common::testSerialization<KinematicsPluginInfo>(*object, "KinematicsPluginInfo");
}

TEST(TesseractCommonSerializeUnit, PluginInfo)  // NOLINT
{
  auto object = std::make_shared<PluginInfo>();
  object->class_name = "test_class_name";
  object->config["test"] = M_PI;
  tesseract_common::testSerialization<PluginInfo>(*object, "PluginInfo");
}

TEST(TesseractCommonSerializeUnit, PluginInfoContainer)  // NOLINT
{
  auto object = std::make_shared<PluginInfoContainer>();
  auto plugin = std::make_shared<PluginInfo>();
  plugin->class_name = "test_class_name";
  plugin->config["test"] = "value";
  object->default_plugin = "test_string";
  object->plugins["plugin_key"] = *plugin;
  tesseract_common::testSerialization<PluginInfoContainer>(*object, "PluginInfoContainer");
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);

  return RUN_ALL_TESTS();
}
