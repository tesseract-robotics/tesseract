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

TEST(TesseractKinematicsFactoryUnit, KDL_OPW_UR_PluginTest)  // NOLINT
{
  KinematicsPluginFactory factory;

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

  tesseract_common::fs::path file_path(__FILE__);
  tesseract_common::fs::path config_path = file_path.parent_path() / "kinematic_plugins.yaml";
  YAML::Node plugin_config = YAML::LoadFile(config_path.string());

  const YAML::Node& plugin_info = plugin_config["kinematic_plugins"];
  const YAML::Node& search_paths = plugin_info["search_paths"];
  const YAML::Node& search_libraries = plugin_info["search_libraries"];
  const YAML::Node& fwd_kin_plugins = plugin_info["fwd_kin_plugins"];
  const YAML::Node& inv_kin_plugins = plugin_info["inv_kin_plugins"];

  EXPECT_EQ(search_paths.size(), 1);
  for (auto it = search_paths.begin(); it != search_paths.end(); ++it)
    factory.addSearchPath(it->as<std::string>());

  EXPECT_EQ(search_libraries.size(), 1);
  for (auto it = search_libraries.begin(); it != search_libraries.end(); ++it)
    factory.addLibrary(it->as<std::string>());

  EXPECT_EQ(fwd_kin_plugins.size(), 1);
  for (auto it = fwd_kin_plugins.begin(); it != fwd_kin_plugins.end(); ++it)
  {
    const YAML::Node& plugin = *it;
    std::string name = plugin["name"].as<std::string>();
    std::string symbol_name = plugin["class"].as<std::string>();
    std::string group = plugin["group"].as<std::string>();
    const YAML::Node& config = plugin["config"];

    ForwardKinematics::UPtr kin;
    if (group == "iiwa_manipulator")
      kin = factory.createFwdKin(symbol_name, group, *iiwa_scene_graph, iiwa_scene_state, config);
    else if (group == "abb_manipulator")
      kin = factory.createFwdKin(symbol_name, group, *abb_scene_graph, abb_scene_state, config);
    else if (group == "ur_manipulator")
      kin = factory.createFwdKin(symbol_name, group, *ur_scene_graph, ur_scene_state, config);

    EXPECT_TRUE(kin != nullptr);
  }

  EXPECT_EQ(inv_kin_plugins.size(), 6);
  for (auto it = inv_kin_plugins.begin(); it != inv_kin_plugins.end(); ++it)
  {
    const YAML::Node& plugin = *it;
    std::string name = plugin["name"].as<std::string>();
    std::string symbol_name = plugin["class"].as<std::string>();
    std::string group = plugin["group"].as<std::string>();
    const YAML::Node& config = plugin["config"];

    InverseKinematics::UPtr kin;
    if (group == "iiwa_manipulator")
      kin = factory.createInvKin(symbol_name, group, *iiwa_scene_graph, iiwa_scene_state, config);
    else if (group == "abb_manipulator")
      kin = factory.createInvKin(symbol_name, group, *abb_scene_graph, abb_scene_state, config);
    else if (group == "ur_manipulator")
      kin = factory.createInvKin(symbol_name, group, *ur_scene_graph, ur_scene_state, config);
    else if (group == "rop_manipulator")
      kin = factory.createInvKin(symbol_name, group, *rop_scene_graph, rop_scene_state, config);
    else if (group == "rep_manipulator")
      kin = factory.createInvKin(symbol_name, group, *rep_scene_graph, rep_scene_state, config);

    EXPECT_TRUE(kin != nullptr);
  }
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);

  return RUN_ALL_TESTS();
}
