/**
 * @file environment_commands_serialization_unit.cpp
 * @brief Tests serialization of environment commands
 *
 * @author Levi Armstrong
 * @author Matthew Powelson
 * @date June 22, 2021
 * @version TODO
 * @bug No known bugs
 *
 * @copyright Copyright (c) 2020, Southwest Research Institute
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

#include <tesseract_common/serialization.h>
#include <tesseract_common/resource_locator.h>
#include <tesseract_common/unit_test_utils.h>
#include <tesseract_common/utils.h>
#include <tesseract_environment/commands.h>
#include <tesseract_environment/environment.h>
#include <tesseract_urdf/urdf_parser.h>
#include <tesseract_srdf/srdf_model.h>
#include <tesseract_support/tesseract_support_resource_locator.h>

using namespace tesseract_common;
using namespace tesseract_environment;
using namespace tesseract_scene_graph;
using namespace tesseract_srdf;

SceneGraph::UPtr getSceneGraph()
{
  std::string path = std::string(TESSERACT_SUPPORT_DIR) + "/urdf/lbr_iiwa_14_r820.urdf";

  tesseract_common::TesseractSupportResourceLocator locator;
  return tesseract_urdf::parseURDFFile(path, locator);
}

SRDFModel::Ptr getSRDFModel(const SceneGraph& scene_graph)
{
  std::string path = std::string(TESSERACT_SUPPORT_DIR) + "/urdf/lbr_iiwa_14_r820.srdf";
  tesseract_common::TesseractSupportResourceLocator locator;

  auto srdf = std::make_shared<SRDFModel>();
  srdf->initFile(scene_graph, path, locator);

  return srdf;
}
Environment::Ptr getEnvironment()
{
  auto env = std::make_shared<Environment>();
  tesseract_scene_graph::SceneGraph::Ptr scene_graph = getSceneGraph();
  auto srdf = getSRDFModel(*scene_graph);
  env->init(*scene_graph, srdf);
  env->setResourceLocator(std::make_shared<tesseract_common::TesseractSupportResourceLocator>());
  return env;
}

TEST(EnvironmentSerializeUnit, Environment)  // NOLINT
{
  Environment::Ptr env = getEnvironment();
  testSerializationPtr<Environment>(env, "Environment");
}

TEST(EnvironmentCommandsSerializeUnit, ModifyAllowedCollisionsCommand)  // NOLINT
{
  {  // ADD
    tesseract_common::AllowedCollisionMatrix add_ac;
    add_ac.addAllowedCollision("link1", "link2", "description");
    auto object = std::make_shared<ModifyAllowedCollisionsCommand>(add_ac, ModifyAllowedCollisionsType::ADD);
    testSerialization<ModifyAllowedCollisionsCommand>(*object, "ModifyAllowedCollisionsCommand");
    testSerializationDerivedClass<Command, ModifyAllowedCollisionsCommand>(object, "ModifyAllowedCollisionsCommand");
  }

  {  // REMOVE
    tesseract_common::AllowedCollisionMatrix remove_ac;
    remove_ac.addAllowedCollision("link1", "link2", "description");
    auto object = std::make_shared<ModifyAllowedCollisionsCommand>(remove_ac, ModifyAllowedCollisionsType::REMOVE);
    testSerialization<ModifyAllowedCollisionsCommand>(*object, "ModifyAllowedCollisionsCommand");
    testSerializationDerivedClass<Command, ModifyAllowedCollisionsCommand>(object, "ModifyAllowedCollisionsCommand");
  }

  {  // REMOVE
    tesseract_common::AllowedCollisionMatrix replace_ac;
    replace_ac.addAllowedCollision("link1", "link2", "description");
    auto object = std::make_shared<ModifyAllowedCollisionsCommand>(replace_ac, ModifyAllowedCollisionsType::REPLACE);
    testSerialization<ModifyAllowedCollisionsCommand>(*object, "ModifyAllowedCollisionsCommand");
    testSerializationDerivedClass<Command, ModifyAllowedCollisionsCommand>(object, "ModifyAllowedCollisionsCommand");
  }
}

TEST(EnvironmentCommandsSerializeUnit, AddContactManagersPluginInfoCommand)  // NOLINT
{
  tesseract_common::ContactManagersPluginInfo info = getEnvironment()->getContactManagersPluginInfo();
  auto object = std::make_shared<AddContactManagersPluginInfoCommand>(info);
  testSerialization<AddContactManagersPluginInfoCommand>(*object, "AddContactManagersPluginInfoCommand");
  testSerializationDerivedClass<Command, AddContactManagersPluginInfoCommand>(object,
                                                                              "AddContactManagersPluginInfoCommand");
}

TEST(EnvironmentCommandsSerializeUnit, AddKinematicsInformationCommand)  // NOLINT
{
  tesseract_srdf::KinematicsInformation info = getEnvironment()->getKinematicsInformation();
  auto object = std::make_shared<AddKinematicsInformationCommand>(info);
  testSerialization<AddKinematicsInformationCommand>(*object, "AddKinematicsInformationCommand");
  testSerializationDerivedClass<Command, AddKinematicsInformationCommand>(object, "AddKinematicsInformationCommand");
}

TEST(EnvironmentCommandsSerializeUnit, AddLinkCommand)  // NOLINT
{
  Link link_1("link_1");
  Link link_2("link_2");
  Joint joint_1("joint_1");
  joint_1.parent_to_joint_origin_transform.translation()(0) = 1.25;
  joint_1.parent_link_name = "link_1";
  joint_1.child_link_name = "link_2";
  joint_1.type = JointType::FIXED;

  auto object = std::make_shared<AddLinkCommand>(link_2, joint_1, false);
  testSerialization<AddLinkCommand>(*object, "AddLinkCommand");
  testSerializationDerivedClass<Command, AddLinkCommand>(object, "AddLinkCommand");
}

TEST(EnvironmentCommandsSerializeUnit, AddSceneGraphCommand)  // NOLINT
{
  tesseract_scene_graph::Joint joint;
  Joint joint_1("joint_1");
  joint_1.parent_to_joint_origin_transform.translation()(0) = 1.25;
  joint_1.parent_link_name = "world";
  joint_1.child_link_name = "joint_a1";
  joint_1.type = JointType::FIXED;
  auto object = std::make_shared<AddSceneGraphCommand>(*getSceneGraph(), joint, "prefix");
  testSerialization<AddSceneGraphCommand>(*object, "AddSceneGraphCommand");
  testSerializationDerivedClass<Command, AddSceneGraphCommand>(object, "AddSceneGraphCommand");
}

TEST(EnvironmentCommandsSerializeUnit, ChangeCollisionMarginsCommand)  // NOLINT
{
  CollisionMarginData collision_margin_data = getEnvironment()->getCollisionMarginData();
  auto object = std::make_shared<ChangeCollisionMarginsCommand>(collision_margin_data,
                                                                CollisionMarginOverrideType::OVERRIDE_DEFAULT_MARGIN);
  testSerialization<ChangeCollisionMarginsCommand>(*object, "ChangeCollisionMarginsCommand");
  testSerializationDerivedClass<Command, ChangeCollisionMarginsCommand>(object, "ChangeCollisionMarginsCommand");
}

TEST(EnvironmentCommandsSerializeUnit, ChangeJointAccelerationLimitsCommand)  // NOLINT
{
  auto object = std::make_shared<ChangeJointAccelerationLimitsCommand>("joint6", 3001);
  testSerialization<ChangeJointAccelerationLimitsCommand>(*object, "ChangeJointAccelerationLimitsCommand");
  testSerializationDerivedClass<Command, ChangeJointAccelerationLimitsCommand>(object,
                                                                               "ChangeJointAccelerationLimitsCommand");
}

TEST(EnvironmentCommandsSerializeUnit, ChangeJointOriginCommand)  // NOLINT
{
  Eigen::Isometry3d origin = Eigen::Isometry3d::Identity();
  origin.translate(Eigen::Vector3d(1, 5, 9));
  auto object = std::make_shared<ChangeJointOriginCommand>("joint6", origin);
  testSerialization<ChangeJointOriginCommand>(*object, "ChangeJointOriginCommand");
  testSerializationDerivedClass<Command, ChangeJointOriginCommand>(object, "ChangeJointOriginCommand");
}

TEST(EnvironmentCommandsSerializeUnit, ChangeJointPositionLimitsCommand)  // NOLINT
{
  auto object = std::make_shared<ChangeJointPositionLimitsCommand>("joint6", -M_PI, M_PI);
  testSerialization<ChangeJointPositionLimitsCommand>(*object, "ChangeJointPositionLimitsCommand");
  testSerializationDerivedClass<Command, ChangeJointPositionLimitsCommand>(object, "ChangeJointPositionLimitsCommand");
}

TEST(EnvironmentCommandsSerializeUnit, ChangeJointVelocityLimitsCommand)  // NOLINT
{
  auto object = std::make_shared<ChangeJointVelocityLimitsCommand>("joint6", 5001);
  testSerialization<ChangeJointVelocityLimitsCommand>(*object, "ChangeJointVelocityLimitsCommand");
  testSerializationDerivedClass<Command, ChangeJointVelocityLimitsCommand>(object, "ChangeJointVelocityLimitsCommand");
}

TEST(EnvironmentCommandsSerializeUnit, ChangeLinkCollisionEnabledCommand)  // NOLINT
{
  auto object = std::make_shared<ChangeLinkCollisionEnabledCommand>("joint3", true);
  testSerialization<ChangeLinkCollisionEnabledCommand>(*object, "ChangeLinkCollisionEnabledCommand");
  testSerializationDerivedClass<Command, ChangeLinkCollisionEnabledCommand>(object,
                                                                            "ChangeLinkCollisionEnabledCommand");
}

TEST(EnvironmentCommandsSerializeUnit, ChangeLinkOriginCommand)  // NOLINT
{
  Eigen::Isometry3d origin = Eigen::Isometry3d::Identity();
  origin.translate(Eigen::Vector3d(1, 5, 9));
  auto object = std::make_shared<ChangeLinkOriginCommand>("really long test joint name with whitespace", origin);
  testSerialization<ChangeLinkOriginCommand>(*object, "ChangeLinkOriginCommand");
  testSerializationDerivedClass<Command, ChangeLinkOriginCommand>(object, "ChangeLinkOriginCommand");
}

TEST(EnvironmentCommandsSerializeUnit, ChangeLinkVisibilityCommand)  // NOLINT
{
  auto object = std::make_shared<ChangeLinkVisibilityCommand>("j1", true);
  testSerialization<ChangeLinkVisibilityCommand>(*object, "ChangeLinkVisibilityCommand");
  testSerializationDerivedClass<Command, ChangeLinkVisibilityCommand>(object, "ChangeLinkVisibilityCommand");
}

TEST(EnvironmentCommandsSerializeUnit, MoveJointCommand)  // NOLINT
{
  auto object = std::make_shared<MoveJointCommand>("j1", "j2");
  testSerialization<MoveJointCommand>(*object, "MoveJointCommand");
  testSerializationDerivedClass<Command, MoveJointCommand>(object, "MoveJointCommand");
}

TEST(EnvironmentCommandsSerializeUnit, MoveLinkCommand)  // NOLINT
{
  auto joint_1 = std::make_shared<Joint>("name");
  joint_1->parent_to_joint_origin_transform.translation()(0) = 1.25;
  joint_1->parent_link_name = "l1";
  joint_1->child_link_name = "l2";
  joint_1->type = JointType::FIXED;

  auto object = std::make_shared<MoveLinkCommand>(*joint_1);
  testSerialization<MoveLinkCommand>(*object, "MoveLinkCommand");
  testSerializationDerivedClass<Command, MoveLinkCommand>(object, "MoveLinkCommand");
}

TEST(EnvironmentCommandsSerializeUnit, RemoveAllowedCollisionLinkCommand)  // NOLINT
{
  auto object = std::make_shared<RemoveAllowedCollisionLinkCommand>("link name");
  testSerialization<RemoveAllowedCollisionLinkCommand>(*object, "RemoveAllowedCollisionLinkCommand");
  testSerializationDerivedClass<Command, RemoveAllowedCollisionLinkCommand>(object,
                                                                            "RemoveAllowedCollisionLinkCommand");
}

TEST(EnvironmentCommandsSerializeUnit, RemoveJointCommand)  // NOLINT
{
  auto object = std::make_shared<RemoveJointCommand>("joint name");
  testSerialization<RemoveJointCommand>(*object, "RemoveJointCommand");
  testSerializationDerivedClass<Command, RemoveJointCommand>(object, "RemoveJointCommand");
}

TEST(EnvironmentCommandsSerializeUnit, RemoveLinkCommand)  // NOLINT
{
  auto object = std::make_shared<RemoveLinkCommand>("link name");
  testSerialization<RemoveLinkCommand>(*object, "RemoveLinkCommand");
  testSerializationDerivedClass<Command, RemoveLinkCommand>(object, "RemoveLinkCommand");
}

TEST(EnvironmentCommandsSerializeUnit, ReplaceJointCommand)  // NOLINT
{
  auto joint_1 = std::make_shared<Joint>("name");
  joint_1->parent_to_joint_origin_transform.translation()(0) = 1.25;
  joint_1->parent_link_name = "l1";
  joint_1->child_link_name = "l2";
  joint_1->type = JointType::FIXED;
  auto object = std::make_shared<ReplaceJointCommand>(*joint_1);
  testSerialization<ReplaceJointCommand>(*object, "ReplaceJointCommand");
  testSerializationDerivedClass<Command, ReplaceJointCommand>(object, "ReplaceJointCommand");
}

TEST(EnvironmentCommandsSerializeUnit, SetActiveContinuousContactManagerCommand)  // NOLINT
{
  auto object = std::make_shared<SetActiveContinuousContactManagerCommand>("my contact manager");
  testSerialization<SetActiveContinuousContactManagerCommand>(*object, "SetActiveContinuousContactManagerCommand");
  testSerializationDerivedClass<Command, SetActiveContinuousContactManagerCommand>(object,
                                                                                   "SetActiveContinuousContactManagerCo"
                                                                                   "mmand");
}

TEST(EnvironmentCommandsSerializeUnit, SetActiveDiscreteContactManagerCommand)  // NOLINT
{
  auto object = std::make_shared<SetActiveDiscreteContactManagerCommand>("my contact manager 2");
  testSerialization<SetActiveDiscreteContactManagerCommand>(*object, "SetActiveDiscreteContactManagerCommand");
  testSerializationDerivedClass<Command, SetActiveDiscreteContactManagerCommand>(object,
                                                                                 "SetActiveDiscreteContactManagerComman"
                                                                                 "d");
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);

  return RUN_ALL_TESTS();
}
