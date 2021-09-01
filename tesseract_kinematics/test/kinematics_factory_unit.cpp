/**
 * @file kinematics_factory_unit.cpp
 * @brief Tesseract kinematics factory test
 *
 * @author Levi Armstrong
 * @date Feb 4, 2021
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

#include "kinematics_test_utils.h"
#include <tesseract_kinematics/core/kinematics_plugin_factory.h>
#include <tesseract_state_solver/kdl/kdl_state_solver.h>

using namespace tesseract_kinematics::test_suite;
using namespace tesseract_kinematics;

void runKinematicsFactoryTest(tesseract_common::fs::path config_path)
{
  tesseract_scene_graph::SceneGraph::UPtr iiwa_scene_graph = getSceneGraphIIWA();
  tesseract_scene_graph::KDLStateSolver iiwa_state_solver(*iiwa_scene_graph);
  tesseract_scene_graph::SceneState iiwa_scene_state = iiwa_state_solver.getState();

  tesseract_scene_graph::SceneGraph::UPtr abb_scene_graph = getSceneGraphABB();
  tesseract_scene_graph::KDLStateSolver abb_state_solver(*abb_scene_graph);
  tesseract_scene_graph::SceneState abb_scene_state = abb_state_solver.getState();

  tesseract_scene_graph::SceneGraph::UPtr ur_scene_graph = getSceneGraphUR(UR10Parameters);
  tesseract_scene_graph::KDLStateSolver ur_state_solver(*ur_scene_graph);
  tesseract_scene_graph::SceneState ur_scene_state = ur_state_solver.getState();

  tesseract_scene_graph::SceneGraph::UPtr rop_scene_graph = getSceneGraphABBOnPositioner();
  tesseract_scene_graph::KDLStateSolver rop_state_solver(*rop_scene_graph);
  tesseract_scene_graph::SceneState rop_scene_state = rop_state_solver.getState();

  tesseract_scene_graph::SceneGraph::UPtr rep_scene_graph = getSceneGraphABBExternalPositioner();
  tesseract_scene_graph::KDLStateSolver rep_state_solver(*rep_scene_graph);
  tesseract_scene_graph::SceneState rep_scene_state = rep_state_solver.getState();

  KinematicsPluginFactory factory(config_path);
  YAML::Node plugin_config = YAML::LoadFile(config_path.string());

  const YAML::Node& plugin_info = plugin_config["kinematic_plugins"];
  const YAML::Node& search_paths = plugin_info["search_paths"];
  const YAML::Node& search_libraries = plugin_info["search_libraries"];
  const YAML::Node& fwd_kin_plugins = plugin_info["fwd_kin_plugins"];
  const YAML::Node& inv_kin_plugins = plugin_info["inv_kin_plugins"];

  {
    const std::set<std::string>& sp = factory.getSearchPaths();
    EXPECT_EQ(sp.size(), 1);

    for (auto it = search_paths.begin(); it != search_paths.end(); ++it)
    {
      EXPECT_TRUE(std::find(sp.begin(), sp.end(), it->as<std::string>()) != sp.end());
    }
  }

  {
    const std::set<std::string>& sl = factory.getSearchLibraries();
    EXPECT_EQ(sl.size(), 4);

    for (auto it = search_libraries.begin(); it != search_libraries.end(); ++it)
    {
      EXPECT_TRUE(std::find(sl.begin(), sl.end(), it->as<std::string>()) != sl.end());
    }
  }

  EXPECT_EQ(fwd_kin_plugins.size(), 1);
  for (auto it = fwd_kin_plugins.begin(); it != fwd_kin_plugins.end(); ++it)
  {
    const YAML::Node& plugin = *it;

    KinematicsPluginInfo info;
    info.name = plugin["name"].as<std::string>();
    info.class_name = plugin["class"].as<std::string>();
    info.group = plugin["group"].as<std::string>();
    info.config = plugin["config"];

    ForwardKinematics::UPtr kin;
    if (info.group == "iiwa_manipulator")
      kin = factory.createFwdKin(info.group, info.name, *iiwa_scene_graph, iiwa_scene_state);
    else if (info.group == "abb_manipulator")
      kin = factory.createFwdKin(info.group, info.name, *abb_scene_graph, abb_scene_state);
    else if (info.group == "ur_manipulator")
      kin = factory.createFwdKin(info.group, info.name, *ur_scene_graph, ur_scene_state);

    EXPECT_TRUE(kin != nullptr);
  }

  EXPECT_EQ(inv_kin_plugins.size(), 6);
  for (auto it = inv_kin_plugins.begin(); it != inv_kin_plugins.end(); ++it)
  {
    const YAML::Node& plugin = *it;

    KinematicsPluginInfo info;
    info.name = plugin["name"].as<std::string>();
    info.class_name = plugin["class"].as<std::string>();
    info.group = plugin["group"].as<std::string>();
    info.config = plugin["config"];

    InverseKinematics::UPtr kin;
    if (info.group == "iiwa_manipulator")
      kin = factory.createInvKin(info.group, info.name, *iiwa_scene_graph, iiwa_scene_state);
    else if (info.group == "abb_manipulator")
      kin = factory.createInvKin(info.group, info.name, *abb_scene_graph, abb_scene_state);
    else if (info.group == "ur_manipulator")
      kin = factory.createInvKin(info.group, info.name, *ur_scene_graph, ur_scene_state);
    else if (info.group == "rop_manipulator")
      kin = factory.createInvKin(info.group, info.name, *rop_scene_graph, rop_scene_state);
    else if (info.group == "rep_manipulator")
      kin = factory.createInvKin(info.group, info.name, *rep_scene_graph, rep_scene_state);

    EXPECT_TRUE(kin != nullptr);
  }

  factory.saveConfig(tesseract_common::fs::path(tesseract_common::getTempPath()) / "kinematic_plugins_export.yaml");
}

TEST(TesseractKinematicsFactoryUnit, KDL_OPW_UR_PluginTest)  // NOLINT
{
  tesseract_common::fs::path file_path(__FILE__);
  tesseract_common::fs::path config_path = file_path.parent_path() / "kinematic_plugins.yaml";
  runKinematicsFactoryTest(config_path);

  tesseract_common::fs::path export_config_path = tesseract_common::fs::path(tesseract_common::getTempPath()) / "kinema"
                                                                                                                "tic_"
                                                                                                                "plugin"
                                                                                                                "s_"
                                                                                                                "export"
                                                                                                                ".yaml";
  runKinematicsFactoryTest(export_config_path);
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);

  return RUN_ALL_TESTS();
}
