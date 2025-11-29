/**
 * @file tesseract_srdf_serialization_unit.cpp
 * @brief Tests serialization
 *
 * @author Levi Armstrong
 * @author Matthew Powelson
 * @date March 16, 2022
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

#include <tesseract_common/resource_locator.h>
#include <tesseract_common/serialization.h>
#include <tesseract_common/unit_test_utils.h>
#include <tesseract_common/utils.h>
#include <tesseract_srdf/kinematics_information.h>
#include <tesseract_srdf/srdf_model.h>
#include <tesseract_srdf/cereal_serialization.h>
#include <tesseract_scene_graph/graph.h>
#include <tesseract_scene_graph/link.h>
#include <tesseract_scene_graph/joint.h>

using namespace tesseract_common;
using namespace tesseract_scene_graph;
using namespace tesseract_srdf;

SceneGraph getSceneGraph()
{
  SceneGraph g;

  g.setName("kuka_lbr_iiwa_14_r820");

  Link base_link("base_link");
  Link link_1("link_1");
  Link link_2("link_2");
  Link link_3("link_3");
  Link link_4("link_4");
  Link link_5("link_5");
  Link link_6("link_6");
  Link link_7("link_7");
  Link tool0("tool0");

  EXPECT_TRUE(g.addLink(base_link));
  EXPECT_TRUE(g.addLink(link_1));
  EXPECT_TRUE(g.addLink(link_2));
  EXPECT_TRUE(g.addLink(link_3));
  EXPECT_TRUE(g.addLink(link_4));
  EXPECT_TRUE(g.addLink(link_5));
  EXPECT_TRUE(g.addLink(link_6));
  EXPECT_TRUE(g.addLink(link_7));
  EXPECT_TRUE(g.addLink(tool0));

  Joint joint_1("joint_a1");
  joint_1.parent_link_name = "base_link";
  joint_1.child_link_name = "link_1";
  joint_1.type = JointType::FIXED;
  EXPECT_TRUE(g.addJoint(joint_1));

  Joint joint_2("joint_a2");
  joint_2.parent_link_name = "link_1";
  joint_2.child_link_name = "link_2";
  joint_2.type = JointType::REVOLUTE;
  joint_2.limits = std::make_shared<JointLimits>(-7, 7, 0, 5, 10, 20);
  EXPECT_TRUE(g.addJoint(joint_2));

  Joint joint_3("joint_a3");
  joint_3.parent_to_joint_origin_transform.translation()(0) = 1.25;
  joint_3.parent_link_name = "link_2";
  joint_3.child_link_name = "link_3";
  joint_3.type = JointType::REVOLUTE;
  joint_3.limits = std::make_shared<JointLimits>(-7, 7, 0, 5, 10, 20);
  EXPECT_TRUE(g.addJoint(joint_3));

  Joint joint_4("joint_a4");
  joint_4.parent_to_joint_origin_transform.translation()(0) = 1.25;
  joint_4.parent_link_name = "link_3";
  joint_4.child_link_name = "link_4";
  joint_4.type = JointType::REVOLUTE;
  joint_4.limits = std::make_shared<JointLimits>(-7, 7, 0, 5, 10, 20);
  EXPECT_TRUE(g.addJoint(joint_4));

  Joint joint_5("joint_a5");
  joint_5.parent_to_joint_origin_transform.translation()(1) = 1.25;
  joint_5.parent_link_name = "link_4";
  joint_5.child_link_name = "link_5";
  joint_5.type = JointType::REVOLUTE;
  joint_5.limits = std::make_shared<JointLimits>(-7, 7, 0, 5, 10, 20);
  EXPECT_TRUE(g.addJoint(joint_5));

  Joint joint_6("joint_a6");
  joint_6.parent_to_joint_origin_transform.translation()(1) = 1.25;
  joint_6.parent_link_name = "link_5";
  joint_6.child_link_name = "link_6";
  joint_6.type = JointType::REVOLUTE;
  joint_6.limits = std::make_shared<JointLimits>(-7, 7, 0, 5, 10, 20);
  EXPECT_TRUE(g.addJoint(joint_6));

  Joint joint_7("joint_a7");
  joint_7.parent_to_joint_origin_transform.translation()(1) = 1.25;
  joint_7.parent_link_name = "link_6";
  joint_7.child_link_name = "link_7";
  joint_7.type = JointType::REVOLUTE;
  joint_7.limits = std::make_shared<JointLimits>(-7, 7, 0, 5, 10, 20);
  EXPECT_TRUE(g.addJoint(joint_7));

  Joint joint_tool0("joint_tool0");
  joint_tool0.parent_link_name = "link_7";
  joint_tool0.child_link_name = "tool0";
  joint_tool0.type = JointType::FIXED;
  EXPECT_TRUE(g.addJoint(joint_tool0));

  return g;
}

SRDFModel::Ptr getSRDFModel(const SceneGraph& scene_graph, const tesseract_common::ResourceLocator& locator)
{
  std::string path = locator.locateResource("package://tesseract_support/urdf/lbr_iiwa_14_r820.srdf")->getFilePath();

  auto srdf = std::make_shared<SRDFModel>();
  srdf->initFile(scene_graph, path, locator);

  return srdf;
}

TEST(TesseractSRDFSerializeUnit, KinematicsInformation)  // NOLINT
{
  GeneralResourceLocator locator;
  auto graph = getSceneGraph();
  auto srdf = getSRDFModel(graph, locator);

  tesseract_common::testSerialization<KinematicsInformation>(srdf->kinematics_information, "KinematicsInformation");
}

TEST(TesseractSRDFSerializeUnit, SRDFModel)  // NOLINT
{
  GeneralResourceLocator locator;
  auto graph = getSceneGraph();
  auto srdf = getSRDFModel(graph, locator);

  tesseract_common::testSerialization<SRDFModel>(*srdf, "SRDFModel");
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);

  return RUN_ALL_TESTS();
}
