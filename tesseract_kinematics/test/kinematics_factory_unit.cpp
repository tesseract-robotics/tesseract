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
  for (auto group_it = fwd_kin_plugins.begin(); group_it != fwd_kin_plugins.end(); ++group_it)
  {
    std::string group_name = group_it->first.as<std::string>();
    for (auto solver_it = group_it->second.begin(); solver_it != group_it->second.end(); ++solver_it)
    {
      const YAML::Node& plugin = *solver_it;

      tesseract_common::PluginInfo info;
      info.name = plugin["name"].as<std::string>();
      info.class_name = plugin["class"].as<std::string>();
      info.config = plugin["config"];

      ForwardKinematics::UPtr kin;
      if (group_name == "iiwa_manipulator")
        kin = factory.createFwdKin(group_name, info.name, *iiwa_scene_graph, iiwa_scene_state);
      else if (group_name == "abb_manipulator")
        kin = factory.createFwdKin(group_name, info.name, *abb_scene_graph, abb_scene_state);
      else if (group_name == "ur_manipulator")
        kin = factory.createFwdKin(group_name, info.name, *ur_scene_graph, ur_scene_state);

      EXPECT_TRUE(kin != nullptr);
    }
  }

  EXPECT_EQ(inv_kin_plugins.size(), 5);
  for (auto group_it = inv_kin_plugins.begin(); group_it != inv_kin_plugins.end(); ++group_it)
  {
    std::string group_name = group_it->first.as<std::string>();
    for (auto solver_it = group_it->second.begin(); solver_it != group_it->second.end(); ++solver_it)
    {
      const YAML::Node& plugin = *solver_it;

      tesseract_common::PluginInfo info;
      info.name = plugin["name"].as<std::string>();
      info.class_name = plugin["class"].as<std::string>();
      info.config = plugin["config"];

      InverseKinematics::UPtr kin;
      if (group_name == "iiwa_manipulator")
        kin = factory.createInvKin(group_name, info.name, *iiwa_scene_graph, iiwa_scene_state);
      else if (group_name == "abb_manipulator")
        kin = factory.createInvKin(group_name, info.name, *abb_scene_graph, abb_scene_state);
      else if (group_name == "ur_manipulator")
        kin = factory.createInvKin(group_name, info.name, *ur_scene_graph, ur_scene_state);
      else if (group_name == "rop_manipulator")
        kin = factory.createInvKin(group_name, info.name, *rop_scene_graph, rop_scene_state);
      else if (group_name == "rep_manipulator")
        kin = factory.createInvKin(group_name, info.name, *rep_scene_graph, rep_scene_state);

      EXPECT_TRUE(kin != nullptr);
    }
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

TEST(TesseractKinematicsFactoryUnit, LoadKinematicsPluginInfoUnit)  // NOLINT
{
  using namespace tesseract_scene_graph;

  tesseract_scene_graph::SceneGraph::UPtr scene_graph = getSceneGraphABB();
  tesseract_scene_graph::KDLStateSolver state_solver(*scene_graph);
  tesseract_scene_graph::SceneState scene_state = state_solver.getState();

  std::string yaml_string =
      R"(kinematic_plugins:
           inv_kin_plugins:
             manipulator:
               - name: OPWInvKin
                 class: OPWInvKinFactory
                 default: true
                 config:
                   base_link: base_link
                   tip_link: tool0
                   params:
                     a1: 0.100
                     a2: -0.135
                     b: 0.00
                     c1: 0.615
                     c2: 0.705
                     c3: 0.755
                     c4: 0.086
                     offsets: [0, 0, -1.57079632679, 0, 0, 0]
                     sign_corrections: [1, 1, 1, -1, 1, 1])";

  {  // missing name
    YAML::Node config = YAML::Load(yaml_string);
    auto plugin = config["kinematic_plugins"]["inv_kin_plugins"]["manipulator"][0];
    plugin.remove("name");

    KinematicsPluginFactory factory(config);
    auto inv_kin = factory.createInvKin("manipulator", "OPWInvKin", *scene_graph, scene_state);
    EXPECT_TRUE(inv_kin == nullptr);
  }
  {  // missing class
    YAML::Node config = YAML::Load(yaml_string);
    auto plugin = config["kinematic_plugins"]["inv_kin_plugins"]["manipulator"][0];
    plugin.remove("class");

    KinematicsPluginFactory factory(config);
    auto inv_kin = factory.createInvKin("manipulator", "OPWInvKin", *scene_graph, scene_state);
    EXPECT_TRUE(inv_kin == nullptr);
  }
  {  // missing default (which is allowed)
    YAML::Node config = YAML::Load(yaml_string);
    auto plugin = config["kinematic_plugins"]["inv_kin_plugins"]["manipulator"][0];
    plugin.remove("default");

    KinematicsPluginFactory factory(config);
    auto inv_kin = factory.createInvKin("manipulator", "OPWInvKin", *scene_graph, scene_state);
    EXPECT_TRUE(inv_kin != nullptr);
  }
}

TEST(TesseractKinematicsFactoryUnit, LoadOPWKinematicsUnit)  // NOLINT
{
  using namespace tesseract_scene_graph;

  tesseract_scene_graph::SceneGraph::UPtr scene_graph = getSceneGraphABB();
  tesseract_scene_graph::KDLStateSolver state_solver(*scene_graph);
  tesseract_scene_graph::SceneState scene_state = state_solver.getState();

  std::string yaml_string =
      R"(kinematic_plugins:
           inv_kin_plugins:
             manipulator:
               - name: OPWInvKin
                 class: OPWInvKinFactory
                 default: true
                 config:
                   base_link: base_link
                   tip_link: tool0
                   params:
                     a1: 0.100
                     a2: -0.135
                     b: 0.00
                     c1: 0.615
                     c2: 0.705
                     c3: 0.755
                     c4: 0.086
                     offsets: [0, 0, -1.57079632679, 0, 0, 0]
                     sign_corrections: [1, 1, 1, -1, 1, 1])";

  KinematicsPluginFactory factory(YAML::Load(yaml_string));
  auto inv_kin = factory.createInvKin("manipulator", "OPWInvKin", *scene_graph, scene_state);
  EXPECT_TRUE(inv_kin != nullptr);

  {  // missing config
    YAML::Node config = YAML::Load(yaml_string);
    auto plugin = config["kinematic_plugins"]["inv_kin_plugins"]["manipulator"][0];
    plugin.remove("config");

    KinematicsPluginFactory factory(config);
    auto inv_kin = factory.createInvKin("manipulator", "OPWInvKin", *scene_graph, scene_state);
    EXPECT_TRUE(inv_kin == nullptr);
  }
  {  // missing base_link
    YAML::Node config = YAML::Load(yaml_string);
    auto plugin = config["kinematic_plugins"]["inv_kin_plugins"]["manipulator"][0];
    plugin["config"].remove("base_link");

    KinematicsPluginFactory factory(config);
    auto inv_kin = factory.createInvKin("manipulator", "OPWInvKin", *scene_graph, scene_state);
    EXPECT_TRUE(inv_kin == nullptr);
  }
  {  // missing tip_link
    YAML::Node config = YAML::Load(yaml_string);
    auto plugin = config["kinematic_plugins"]["inv_kin_plugins"]["manipulator"][0];
    plugin["config"].remove("tip_link");

    KinematicsPluginFactory factory(config);
    auto inv_kin = factory.createInvKin("manipulator", "OPWInvKin", *scene_graph, scene_state);
    EXPECT_TRUE(inv_kin == nullptr);
  }
  {  // missing params
    YAML::Node config = YAML::Load(yaml_string);
    auto plugin = config["kinematic_plugins"]["inv_kin_plugins"]["manipulator"][0];
    plugin["config"].remove("params");

    KinematicsPluginFactory factory(config);
    auto inv_kin = factory.createInvKin("manipulator", "OPWInvKin", *scene_graph, scene_state);
    EXPECT_TRUE(inv_kin == nullptr);
  }
  {  // missing a1
    YAML::Node config = YAML::Load(yaml_string);
    auto plugin = config["kinematic_plugins"]["inv_kin_plugins"]["manipulator"][0];
    plugin["config"]["params"].remove("a1");

    KinematicsPluginFactory factory(config);
    auto inv_kin = factory.createInvKin("manipulator", "OPWInvKin", *scene_graph, scene_state);
    EXPECT_TRUE(inv_kin == nullptr);
  }
  {  // missing a2
    YAML::Node config = YAML::Load(yaml_string);
    auto plugin = config["kinematic_plugins"]["inv_kin_plugins"]["manipulator"][0];
    plugin["config"]["params"].remove("a2");

    KinematicsPluginFactory factory(config);
    auto inv_kin = factory.createInvKin("manipulator", "OPWInvKin", *scene_graph, scene_state);
    EXPECT_TRUE(inv_kin == nullptr);
  }
  {  // missing b
    YAML::Node config = YAML::Load(yaml_string);
    auto plugin = config["kinematic_plugins"]["inv_kin_plugins"]["manipulator"][0];
    plugin["config"]["params"].remove("b");

    KinematicsPluginFactory factory(config);
    auto inv_kin = factory.createInvKin("manipulator", "OPWInvKin", *scene_graph, scene_state);
    EXPECT_TRUE(inv_kin == nullptr);
  }
  {  // missing c1
    YAML::Node config = YAML::Load(yaml_string);
    auto plugin = config["kinematic_plugins"]["inv_kin_plugins"]["manipulator"][0];
    plugin["config"]["params"].remove("c1");

    KinematicsPluginFactory factory(config);
    auto inv_kin = factory.createInvKin("manipulator", "OPWInvKin", *scene_graph, scene_state);
    EXPECT_TRUE(inv_kin == nullptr);
  }
  {  // missing c2
    YAML::Node config = YAML::Load(yaml_string);
    auto plugin = config["kinematic_plugins"]["inv_kin_plugins"]["manipulator"][0];
    plugin["config"]["params"].remove("c2");

    KinematicsPluginFactory factory(config);
    auto inv_kin = factory.createInvKin("manipulator", "OPWInvKin", *scene_graph, scene_state);
    EXPECT_TRUE(inv_kin == nullptr);
  }
  {  // missing c3
    YAML::Node config = YAML::Load(yaml_string);
    auto plugin = config["kinematic_plugins"]["inv_kin_plugins"]["manipulator"][0];
    plugin["config"]["params"].remove("c3");

    KinematicsPluginFactory factory(config);
    auto inv_kin = factory.createInvKin("manipulator", "OPWInvKin", *scene_graph, scene_state);
    EXPECT_TRUE(inv_kin == nullptr);
  }
  {  // missing c4
    YAML::Node config = YAML::Load(yaml_string);
    auto plugin = config["kinematic_plugins"]["inv_kin_plugins"]["manipulator"][0];
    plugin["config"]["params"].remove("c4");

    KinematicsPluginFactory factory(config);
    auto inv_kin = factory.createInvKin("manipulator", "OPWInvKin", *scene_graph, scene_state);
    EXPECT_TRUE(inv_kin == nullptr);
  }
  {  // missing offset is allowed
    YAML::Node config = YAML::Load(yaml_string);
    auto plugin = config["kinematic_plugins"]["inv_kin_plugins"]["manipulator"][0];
    plugin["config"]["params"].remove("offset");

    KinematicsPluginFactory factory(config);
    auto inv_kin = factory.createInvKin("manipulator", "OPWInvKin", *scene_graph, scene_state);
    EXPECT_TRUE(inv_kin != nullptr);
  }
  {  // missing sign_corrections is allowed
    YAML::Node config = YAML::Load(yaml_string);
    auto plugin = config["kinematic_plugins"]["inv_kin_plugins"]["manipulator"][0];
    plugin["config"]["params"].remove("sign_corrections");

    KinematicsPluginFactory factory(config);
    auto inv_kin = factory.createInvKin("manipulator", "OPWInvKin", *scene_graph, scene_state);
    EXPECT_TRUE(inv_kin != nullptr);
  }

  {  // invalid a1
    YAML::Node config = YAML::Load(yaml_string);
    auto plugin = config["kinematic_plugins"]["inv_kin_plugins"]["manipulator"][0];
    plugin["config"]["params"]["a1"] = "abcd";

    KinematicsPluginFactory factory(config);
    auto inv_kin = factory.createInvKin("manipulator", "OPWInvKin", *scene_graph, scene_state);
    EXPECT_TRUE(inv_kin == nullptr);
  }
  {  // invalid a2
    YAML::Node config = YAML::Load(yaml_string);
    auto plugin = config["kinematic_plugins"]["inv_kin_plugins"]["manipulator"][0];
    plugin["config"]["params"]["a2"] = "abcd";

    KinematicsPluginFactory factory(config);
    auto inv_kin = factory.createInvKin("manipulator", "OPWInvKin", *scene_graph, scene_state);
    EXPECT_TRUE(inv_kin == nullptr);
  }
  {  // invalid b
    YAML::Node config = YAML::Load(yaml_string);
    auto plugin = config["kinematic_plugins"]["inv_kin_plugins"]["manipulator"][0];
    plugin["config"]["params"]["b"] = "abcd";

    KinematicsPluginFactory factory(config);
    auto inv_kin = factory.createInvKin("manipulator", "OPWInvKin", *scene_graph, scene_state);
    EXPECT_TRUE(inv_kin == nullptr);
  }
  {  // invalid c1
    YAML::Node config = YAML::Load(yaml_string);
    auto plugin = config["kinematic_plugins"]["inv_kin_plugins"]["manipulator"][0];
    plugin["config"]["params"]["c1"] = "abcd";

    KinematicsPluginFactory factory(config);
    auto inv_kin = factory.createInvKin("manipulator", "OPWInvKin", *scene_graph, scene_state);
    EXPECT_TRUE(inv_kin == nullptr);
  }
  {  // invalid c2
    YAML::Node config = YAML::Load(yaml_string);
    auto plugin = config["kinematic_plugins"]["inv_kin_plugins"]["manipulator"][0];
    plugin["config"]["params"]["c2"] = "abcd";

    KinematicsPluginFactory factory(config);
    auto inv_kin = factory.createInvKin("manipulator", "OPWInvKin", *scene_graph, scene_state);
    EXPECT_TRUE(inv_kin == nullptr);
  }
  {  // invalide c3
    YAML::Node config = YAML::Load(yaml_string);
    auto plugin = config["kinematic_plugins"]["inv_kin_plugins"]["manipulator"][0];
    plugin["config"]["params"]["c3"] = "abcd";

    KinematicsPluginFactory factory(config);
    auto inv_kin = factory.createInvKin("manipulator", "OPWInvKin", *scene_graph, scene_state);
    EXPECT_TRUE(inv_kin == nullptr);
  }
  {  // invalid c4
    YAML::Node config = YAML::Load(yaml_string);
    auto plugin = config["kinematic_plugins"]["inv_kin_plugins"]["manipulator"][0];
    plugin["config"]["params"]["c4"] = "abcd";

    KinematicsPluginFactory factory(config);
    auto inv_kin = factory.createInvKin("manipulator", "OPWInvKin", *scene_graph, scene_state);
    EXPECT_TRUE(inv_kin == nullptr);
  }
  {  // invalid offset
    YAML::Node config = YAML::Load(yaml_string);
    auto plugin = config["kinematic_plugins"]["inv_kin_plugins"]["manipulator"][0];
    plugin["config"]["params"]["offsets"][0] = "abcd";

    KinematicsPluginFactory factory(config);
    auto inv_kin = factory.createInvKin("manipulator", "OPWInvKin", *scene_graph, scene_state);
    EXPECT_TRUE(inv_kin == nullptr);
  }
  {  // invalid sign_corrections
    YAML::Node config = YAML::Load(yaml_string);
    auto plugin = config["kinematic_plugins"]["inv_kin_plugins"]["manipulator"][0];
    plugin["config"]["params"]["sign_corrections"][0] = "a";

    KinematicsPluginFactory factory(config);
    auto inv_kin = factory.createInvKin("manipulator", "OPWInvKin", *scene_graph, scene_state);
    EXPECT_TRUE(inv_kin == nullptr);
  }
  {  // invalid sign_corrections
    YAML::Node config = YAML::Load(yaml_string);
    auto plugin = config["kinematic_plugins"]["inv_kin_plugins"]["manipulator"][0];
    plugin["config"]["params"]["sign_corrections"][0] = 5;

    KinematicsPluginFactory factory(config);
    auto inv_kin = factory.createInvKin("manipulator", "OPWInvKin", *scene_graph, scene_state);
    EXPECT_TRUE(inv_kin == nullptr);
  }
}

TEST(TesseractKinematicsFactoryUnit, LoadURKinematicsUnit)  // NOLINT
{
  using namespace tesseract_scene_graph;

  tesseract_scene_graph::SceneGraph::UPtr scene_graph = getSceneGraphUR(UR10Parameters);
  tesseract_scene_graph::KDLStateSolver state_solver(*scene_graph);
  tesseract_scene_graph::SceneState scene_state = state_solver.getState();

  std::string yaml_model_string =
      R"(kinematic_plugins:
           inv_kin_plugins:
             manipulator:
               - name: URInvKin
                 class: URInvKinFactory
                 default: true
                 config:
                   base_link: base_link
                   tip_link: tool0
                   model: UR10)";

  std::string yaml_params_string =
      R"(kinematic_plugins:
           inv_kin_plugins:
             manipulator:
               - name: URInvKin
                 class: URInvKinFactory
                 default: true
                 config:
                   base_link: base_link
                   tip_link: tool0
                   params:
                     d1: 0.1273
                     a2: -0.612
                     a3: -0.5723
                     d4: 0.163941
                     d5: 0.1157
                     d6: 0.0922)";

  {
    KinematicsPluginFactory factory(YAML::Load(yaml_model_string));
    auto inv_kin = factory.createInvKin("manipulator", "URInvKin", *scene_graph, scene_state);
    EXPECT_TRUE(inv_kin != nullptr);
  }

  {
    KinematicsPluginFactory factory(YAML::Load(yaml_params_string));
    auto inv_kin = factory.createInvKin("manipulator", "URInvKin", *scene_graph, scene_state);
    EXPECT_TRUE(inv_kin != nullptr);
  }

  {  // missing config
    YAML::Node config = YAML::Load(yaml_model_string);
    auto plugin = config["kinematic_plugins"]["inv_kin_plugins"]["manipulator"][0];
    plugin.remove("config");

    KinematicsPluginFactory factory(config);
    auto inv_kin = factory.createInvKin("manipulator", "URInvKin", *scene_graph, scene_state);
    EXPECT_TRUE(inv_kin == nullptr);
  }
  {  // missing base_link
    YAML::Node config = YAML::Load(yaml_model_string);
    auto plugin = config["kinematic_plugins"]["inv_kin_plugins"]["manipulator"][0];
    plugin["config"].remove("base_link");

    KinematicsPluginFactory factory(config);
    auto inv_kin = factory.createInvKin("manipulator", "URInvKin", *scene_graph, scene_state);
    EXPECT_TRUE(inv_kin == nullptr);
  }
  {  // missing tip_link
    YAML::Node config = YAML::Load(yaml_model_string);
    auto plugin = config["kinematic_plugins"]["inv_kin_plugins"]["manipulator"][0];
    plugin["config"].remove("tip_link");

    KinematicsPluginFactory factory(config);
    auto inv_kin = factory.createInvKin("manipulator", "URInvKin", *scene_graph, scene_state);
    EXPECT_TRUE(inv_kin == nullptr);
  }
  {  // missing model and params
    YAML::Node config = YAML::Load(yaml_model_string);
    auto plugin = config["kinematic_plugins"]["inv_kin_plugins"]["manipulator"][0];
    plugin["config"].remove("model");

    KinematicsPluginFactory factory(config);
    auto inv_kin = factory.createInvKin("manipulator", "URInvKin", *scene_graph, scene_state);
    EXPECT_TRUE(inv_kin == nullptr);
  }
  {  // missing model and params
    YAML::Node config = YAML::Load(yaml_params_string);
    auto plugin = config["kinematic_plugins"]["inv_kin_plugins"]["manipulator"][0];
    plugin["config"].remove("params");

    KinematicsPluginFactory factory(config);
    auto inv_kin = factory.createInvKin("manipulator", "URInvKin", *scene_graph, scene_state);
    EXPECT_TRUE(inv_kin == nullptr);
  }
  {  // missing d1
    YAML::Node config = YAML::Load(yaml_params_string);
    auto plugin = config["kinematic_plugins"]["inv_kin_plugins"]["manipulator"][0];
    plugin["config"]["params"].remove("d1");

    KinematicsPluginFactory factory(config);
    auto inv_kin = factory.createInvKin("manipulator", "URInvKin", *scene_graph, scene_state);
    EXPECT_TRUE(inv_kin == nullptr);
  }
  {  // missing a2
    YAML::Node config = YAML::Load(yaml_params_string);
    auto plugin = config["kinematic_plugins"]["inv_kin_plugins"]["manipulator"][0];
    plugin["config"]["params"].remove("a2");

    KinematicsPluginFactory factory(config);
    auto inv_kin = factory.createInvKin("manipulator", "URInvKin", *scene_graph, scene_state);
    EXPECT_TRUE(inv_kin == nullptr);
  }
  {  // missing a3
    YAML::Node config = YAML::Load(yaml_params_string);
    auto plugin = config["kinematic_plugins"]["inv_kin_plugins"]["manipulator"][0];
    plugin["config"]["params"].remove("a3");

    KinematicsPluginFactory factory(config);
    auto inv_kin = factory.createInvKin("manipulator", "URInvKin", *scene_graph, scene_state);
    EXPECT_TRUE(inv_kin == nullptr);
  }
  {  // missing d4
    YAML::Node config = YAML::Load(yaml_params_string);
    auto plugin = config["kinematic_plugins"]["inv_kin_plugins"]["manipulator"][0];
    plugin["config"]["params"].remove("d4");

    KinematicsPluginFactory factory(config);
    auto inv_kin = factory.createInvKin("manipulator", "URInvKin", *scene_graph, scene_state);
    EXPECT_TRUE(inv_kin == nullptr);
  }
  {  // missing d5
    YAML::Node config = YAML::Load(yaml_params_string);
    auto plugin = config["kinematic_plugins"]["inv_kin_plugins"]["manipulator"][0];
    plugin["config"]["params"].remove("d5");

    KinematicsPluginFactory factory(config);
    auto inv_kin = factory.createInvKin("manipulator", "URInvKin", *scene_graph, scene_state);
    EXPECT_TRUE(inv_kin == nullptr);
  }
  {  // missing d6
    YAML::Node config = YAML::Load(yaml_params_string);
    auto plugin = config["kinematic_plugins"]["inv_kin_plugins"]["manipulator"][0];
    plugin["config"]["params"].remove("d6");

    KinematicsPluginFactory factory(config);
    auto inv_kin = factory.createInvKin("manipulator", "URInvKin", *scene_graph, scene_state);
    EXPECT_TRUE(inv_kin == nullptr);
  }
  {  // invalid d1
    YAML::Node config = YAML::Load(yaml_params_string);
    auto plugin = config["kinematic_plugins"]["inv_kin_plugins"]["manipulator"][0];
    plugin["config"]["params"]["d1"] = "abcd";

    KinematicsPluginFactory factory(config);
    auto inv_kin = factory.createInvKin("manipulator", "URInvKin", *scene_graph, scene_state);
    EXPECT_TRUE(inv_kin == nullptr);
  }
  {  // invalid a2
    YAML::Node config = YAML::Load(yaml_params_string);
    auto plugin = config["kinematic_plugins"]["inv_kin_plugins"]["manipulator"][0];
    plugin["config"]["params"]["a2"] = "abcd";

    KinematicsPluginFactory factory(config);
    auto inv_kin = factory.createInvKin("manipulator", "URInvKin", *scene_graph, scene_state);
    EXPECT_TRUE(inv_kin == nullptr);
  }
  {  // invalid a3
    YAML::Node config = YAML::Load(yaml_params_string);
    auto plugin = config["kinematic_plugins"]["inv_kin_plugins"]["manipulator"][0];
    plugin["config"]["params"]["a3"] = "abcd";

    KinematicsPluginFactory factory(config);
    auto inv_kin = factory.createInvKin("manipulator", "URInvKin", *scene_graph, scene_state);
    EXPECT_TRUE(inv_kin == nullptr);
  }
  {  // invalid d4
    YAML::Node config = YAML::Load(yaml_params_string);
    auto plugin = config["kinematic_plugins"]["inv_kin_plugins"]["manipulator"][0];
    plugin["config"]["params"]["d4"] = "abcd";

    KinematicsPluginFactory factory(config);
    auto inv_kin = factory.createInvKin("manipulator", "URInvKin", *scene_graph, scene_state);
    EXPECT_TRUE(inv_kin == nullptr);
  }
  {  // invalid d5
    YAML::Node config = YAML::Load(yaml_params_string);
    auto plugin = config["kinematic_plugins"]["inv_kin_plugins"]["manipulator"][0];
    plugin["config"]["params"]["d5"] = "abcd";

    KinematicsPluginFactory factory(config);
    auto inv_kin = factory.createInvKin("manipulator", "URInvKin", *scene_graph, scene_state);
    EXPECT_TRUE(inv_kin == nullptr);
  }
  {  // invalide d6
    YAML::Node config = YAML::Load(yaml_params_string);
    auto plugin = config["kinematic_plugins"]["inv_kin_plugins"]["manipulator"][0];
    plugin["config"]["params"]["d6"] = "abcd";

    KinematicsPluginFactory factory(config);
    auto inv_kin = factory.createInvKin("manipulator", "URInvKin", *scene_graph, scene_state);
    EXPECT_TRUE(inv_kin == nullptr);
  }
}

TEST(TesseractKinematicsFactoryUnit, LoadREPKinematicsUnit)  // NOLINT
{
  using namespace tesseract_scene_graph;
  tesseract_scene_graph::SceneGraph::UPtr scene_graph = getSceneGraphABBExternalPositioner();
  tesseract_scene_graph::KDLStateSolver state_solver(*scene_graph);
  tesseract_scene_graph::SceneState scene_state = state_solver.getState();

  std::string yaml_string =
      R"(kinematic_plugins:
           inv_kin_plugins:
             manipulator:
               - name: REPInvKin
                 class: REPInvKinFactory
                 default: true
                 config:
                   manipulator_reach: 2.0
                   positioner_sample_resolution:
                     - name: positioner_joint_1
                       value: 0.1
                     - name: positioner_joint_2
                       value: 0.1
                   positioner:
                     class: KDLFwdKinChainFactory
                     config:
                       base_link: positioner_base_link
                       tip_link: positioner_tool0
                   manipulator:
                     class: OPWInvKinFactory
                     config:
                       base_link: base_link
                       tip_link: tool0
                       params:
                         a1: 0.100
                         a2: -0.135
                         b: 0.00
                         c1: 0.615
                         c2: 0.705
                         c3: 0.755
                         c4: 0.086
                         offsets: [0, 0, -1.57079632679, 0, 0, 0]
                         sign_corrections: [1, 1, 1, 1, 1, 1])";

  KinematicsPluginFactory factory(YAML::Load(yaml_string));
  auto inv_kin = factory.createInvKin("manipulator", "REPInvKin", *scene_graph, scene_state);
  EXPECT_TRUE(inv_kin != nullptr);

  {  // missing config
    YAML::Node config = YAML::Load(yaml_string);
    auto plugin = config["kinematic_plugins"]["inv_kin_plugins"]["manipulator"][0];
    plugin.remove("config");

    KinematicsPluginFactory factory(config);
    auto inv_kin = factory.createInvKin("manipulator", "REPInvKin", *scene_graph, scene_state);
    EXPECT_TRUE(inv_kin == nullptr);
  }
  {  // missing manipulator_reach
    YAML::Node config = YAML::Load(yaml_string);
    auto plugin = config["kinematic_plugins"]["inv_kin_plugins"]["manipulator"][0];
    plugin["config"].remove("manipulator_reach");

    KinematicsPluginFactory factory(config);
    auto inv_kin = factory.createInvKin("manipulator", "REPInvKin", *scene_graph, scene_state);
    EXPECT_TRUE(inv_kin == nullptr);
  }
  {  // missing positioner_sample_resolution
    YAML::Node config = YAML::Load(yaml_string);
    auto plugin = config["kinematic_plugins"]["inv_kin_plugins"]["manipulator"][0];
    plugin["config"].remove("positioner_sample_resolution");

    KinematicsPluginFactory factory(config);
    auto inv_kin = factory.createInvKin("manipulator", "REPInvKin", *scene_graph, scene_state);
    EXPECT_TRUE(inv_kin == nullptr);
  }
  {  // missing positioner
    YAML::Node config = YAML::Load(yaml_string);
    auto plugin = config["kinematic_plugins"]["inv_kin_plugins"]["manipulator"][0];
    plugin["config"].remove("positioner");

    KinematicsPluginFactory factory(config);
    auto inv_kin = factory.createInvKin("manipulator", "REPInvKin", *scene_graph, scene_state);
    EXPECT_TRUE(inv_kin == nullptr);
  }
  {  // missing manipulator
    YAML::Node config = YAML::Load(yaml_string);
    auto plugin = config["kinematic_plugins"]["inv_kin_plugins"]["manipulator"][0];
    plugin["config"].remove("manipulator");

    KinematicsPluginFactory factory(config);
    auto inv_kin = factory.createInvKin("manipulator", "REPInvKin", *scene_graph, scene_state);
    EXPECT_TRUE(inv_kin == nullptr);
  }
}

TEST(TesseractKinematicsFactoryUnit, LoadROPKinematicsUnit)  // NOLINT
{
  using namespace tesseract_scene_graph;
  tesseract_scene_graph::SceneGraph::UPtr scene_graph = getSceneGraphABBOnPositioner();
  tesseract_scene_graph::KDLStateSolver state_solver(*scene_graph);
  tesseract_scene_graph::SceneState scene_state = state_solver.getState();

  std::string yaml_string =
      R"(kinematic_plugins:
           inv_kin_plugins:
             manipulator:
               - name: ROPInvKin
                 class: ROPInvKinFactory
                 default: true
                 config:
                   manipulator_reach: 2.0
                   positioner_sample_resolution:
                     - name: positioner_joint_1
                       value: 0.1
                   positioner:
                     class: KDLFwdKinChainFactory
                     config:
                       base_link: positioner_base_link
                       tip_link: positioner_tool0
                   manipulator:
                     class: OPWInvKinFactory
                     config:
                       base_link: base_link
                       tip_link: tool0
                       params:
                         a1: 0.100
                         a2: -0.135
                         b: 0.00
                         c1: 0.615
                         c2: 0.705
                         c3: 0.755
                         c4: 0.086
                         offsets: [0, 0, -1.57079632679, 0, 0, 0]
                         sign_corrections: [1, 1, 1, 1, 1, 1])";

  KinematicsPluginFactory factory(YAML::Load(yaml_string));
  auto inv_kin = factory.createInvKin("manipulator", "ROPInvKin", *scene_graph, scene_state);
  EXPECT_TRUE(inv_kin != nullptr);

  {  // missing config
    YAML::Node config = YAML::Load(yaml_string);
    auto plugin = config["kinematic_plugins"]["inv_kin_plugins"]["manipulator"][0];
    plugin.remove("config");

    KinematicsPluginFactory factory(config);
    auto inv_kin = factory.createInvKin("manipulator", "ROPInvKin", *scene_graph, scene_state);
    EXPECT_TRUE(inv_kin == nullptr);
  }
  {  // missing manipulator_reach
    YAML::Node config = YAML::Load(yaml_string);
    auto plugin = config["kinematic_plugins"]["inv_kin_plugins"]["manipulator"][0];
    plugin["config"].remove("manipulator_reach");

    KinematicsPluginFactory factory(config);
    auto inv_kin = factory.createInvKin("manipulator", "ROPInvKin", *scene_graph, scene_state);
    EXPECT_TRUE(inv_kin == nullptr);
  }
  {  // missing positioner_sample_resolution
    YAML::Node config = YAML::Load(yaml_string);
    auto plugin = config["kinematic_plugins"]["inv_kin_plugins"]["manipulator"][0];
    plugin["config"].remove("positioner_sample_resolution");

    KinematicsPluginFactory factory(config);
    auto inv_kin = factory.createInvKin("manipulator", "ROPInvKin", *scene_graph, scene_state);
    EXPECT_TRUE(inv_kin == nullptr);
  }
  {  // missing positioner
    YAML::Node config = YAML::Load(yaml_string);
    auto plugin = config["kinematic_plugins"]["inv_kin_plugins"]["manipulator"][0];
    plugin["config"].remove("positioner");

    KinematicsPluginFactory factory(config);
    auto inv_kin = factory.createInvKin("manipulator", "ROPInvKin", *scene_graph, scene_state);
    EXPECT_TRUE(inv_kin == nullptr);
  }
  {  // missing manipulator
    YAML::Node config = YAML::Load(yaml_string);
    auto plugin = config["kinematic_plugins"]["inv_kin_plugins"]["manipulator"][0];
    plugin["config"].remove("manipulator");

    KinematicsPluginFactory factory(config);
    auto inv_kin = factory.createInvKin("manipulator", "ROPInvKin", *scene_graph, scene_state);
    EXPECT_TRUE(inv_kin == nullptr);
  }
}

TEST(TesseractKinematicsFactoryUnit, LoadKDLKinematicsUnit)  // NOLINT
{
  using namespace tesseract_scene_graph;

  tesseract_scene_graph::SceneGraph::UPtr scene_graph = getSceneGraphABB();
  tesseract_scene_graph::KDLStateSolver state_solver(*scene_graph);
  tesseract_scene_graph::SceneState scene_state = state_solver.getState();

  std::string yaml_string =
      R"(kinematic_plugins:
           fwd_kin_plugins:
             manipulator:
               - name: KDLFwdKinChain
                 class: KDLFwdKinChainFactory
                 default: true
                 config:
                   base_link: base_link
                   tip_link: tool0
           inv_kin_plugins:
             manipulator:
               - name: KDLInvKinChainLMA
                 class: KDLInvKinChainLMAFactory
                 default: true
                 config:
                   base_link: base_link
                   tip_link: tool0
               - name: KDLInvKinChainNR
                 class: KDLInvKinChainNRFactory
                 config:
                   base_link: base_link
                   tip_link: tool0)";

  {
    KinematicsPluginFactory factory(YAML::Load(yaml_string));
    auto fwd_kin = factory.createFwdKin("manipulator", "KDLFwdKinChain", *scene_graph, scene_state);
    EXPECT_TRUE(fwd_kin != nullptr);
  }

  {
    KinematicsPluginFactory factory(YAML::Load(yaml_string));
    auto inv_kin = factory.createInvKin("manipulator", "KDLInvKinChainLMA", *scene_graph, scene_state);
    EXPECT_TRUE(inv_kin != nullptr);
  }

  {
    KinematicsPluginFactory factory(YAML::Load(yaml_string));
    auto inv_kin = factory.createInvKin("manipulator", "KDLInvKinChainNR", *scene_graph, scene_state);
    EXPECT_TRUE(inv_kin != nullptr);
  }

  {  // KDLFwdKinChain missing config
    YAML::Node config = YAML::Load(yaml_string);
    auto plugin = config["kinematic_plugins"]["fwd_kin_plugins"][0];
    plugin.remove("config");

    KinematicsPluginFactory factory(config);
    auto kin = factory.createInvKin("manipulator", "KDLFwdKinChain", *scene_graph, scene_state);
    EXPECT_TRUE(kin == nullptr);
  }
  {  // KDLInvKinChainLMA missing config
    YAML::Node config = YAML::Load(yaml_string);
    auto plugin = config["kinematic_plugins"]["inv_kin_plugins"]["manipulator"][0];
    plugin.remove("config");

    KinematicsPluginFactory factory(config);
    auto kin = factory.createInvKin("manipulator", "KDLInvKinChainLMA", *scene_graph, scene_state);
    EXPECT_TRUE(kin == nullptr);
  }
  {  // KDLInvKinChainNR missing config
    YAML::Node config = YAML::Load(yaml_string);
    auto plugin = config["kinematic_plugins"]["inv_kin_plugins"]["manipulator"][1];
    plugin.remove("config");

    KinematicsPluginFactory factory(config);
    auto kin = factory.createInvKin("manipulator", "KDLInvKinChainNR", *scene_graph, scene_state);
    EXPECT_TRUE(kin == nullptr);
  }
  {  // KDLFwdKinChain missing base_link
    YAML::Node config = YAML::Load(yaml_string);
    auto plugin = config["kinematic_plugins"]["fwd_kin_plugins"]["manipulator"][0];
    plugin["config"].remove("base_link");

    KinematicsPluginFactory factory(config);
    auto kin = factory.createInvKin("manipulator", "KDLFwdKinChain", *scene_graph, scene_state);
    EXPECT_TRUE(kin == nullptr);
  }
  {  // KDLInvKinChainLMA missing base_link
    YAML::Node config = YAML::Load(yaml_string);
    auto plugin = config["kinematic_plugins"]["inv_kin_plugins"]["manipulator"][0];
    plugin["config"].remove("base_link");

    KinematicsPluginFactory factory(config);
    auto kin = factory.createInvKin("manipulator", "KDLInvKinChainLMA", *scene_graph, scene_state);
    EXPECT_TRUE(kin == nullptr);
  }
  {  // KDLInvKinChainNR missing base_link
    YAML::Node config = YAML::Load(yaml_string);
    auto plugin = config["kinematic_plugins"]["inv_kin_plugins"]["manipulator"][1];
    plugin["config"].remove("base_link");

    KinematicsPluginFactory factory(config);
    auto kin = factory.createInvKin("manipulator", "KDLInvKinChainNR", *scene_graph, scene_state);
    EXPECT_TRUE(kin == nullptr);
  }
  {  // KDLFwdKinChain missing tip_link
    YAML::Node config = YAML::Load(yaml_string);
    auto plugin = config["kinematic_plugins"]["fwd_kin_plugins"]["manipulator"][0];
    plugin["config"].remove("tip_link");

    KinematicsPluginFactory factory(config);
    auto kin = factory.createInvKin("manipulator", "KDLFwdKinChain", *scene_graph, scene_state);
    EXPECT_TRUE(kin == nullptr);
  }
  {  // KDLInvKinChainLMA missing tip_link
    YAML::Node config = YAML::Load(yaml_string);
    auto plugin = config["kinematic_plugins"]["inv_kin_plugins"]["manipulator"][0];
    plugin["config"].remove("tip_link");

    KinematicsPluginFactory factory(config);
    auto kin = factory.createInvKin("manipulator", "KDLInvKinChainLMA", *scene_graph, scene_state);
    EXPECT_TRUE(kin == nullptr);
  }
  {  // KDLInvKinChainNR missing tip_link
    YAML::Node config = YAML::Load(yaml_string);
    auto plugin = config["kinematic_plugins"]["inv_kin_plugins"]["manipulator"][1];
    plugin["config"].remove("tip_link");

    KinematicsPluginFactory factory(config);
    auto kin = factory.createInvKin("manipulator", "KDLInvKinChainNR", *scene_graph, scene_state);
    EXPECT_TRUE(kin == nullptr);
  }
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);

  return RUN_ALL_TESTS();
}
