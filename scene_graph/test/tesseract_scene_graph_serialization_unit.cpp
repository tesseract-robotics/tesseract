/**
 * @file tesseract_scene_graph_serialization_unit.cpp
 * @brief Tests serialization of tesseract::scene_graph
 *
 * @author Levi Armstrong
 * @author Matthew Powelson
 * @date March 18, 2022
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
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract/geometry/geometries.h>
#include <tesseract/scene_graph/graph.h>
#include <tesseract/scene_graph/joint.h>
#include <tesseract/scene_graph/link.h>
#include <tesseract/scene_graph/scene_state.h>
#include <tesseract/scene_graph/cereal_serialization.h>
#include <tesseract/common/serialization.h>
#include <tesseract/common/unit_test_utils.h>
#include <tesseract/common/utils.h>

using namespace tesseract::scene_graph;

/*********************************************************/
/******                     Joint                    *****/
/*********************************************************/

TEST(TesseractSceneGraphSerializationUnit, JointDynamics)  // NOLINT
{
  auto object = std::make_shared<JointDynamics>(1.1, 2.2);
  tesseract::common::testSerialization<JointDynamics>(*object, "JointDynamics");
}

TEST(TesseractSceneGraphSerializationUnit, JointLimits)  // NOLINT
{
  auto object = std::make_shared<JointLimits>(1.1, 2.2, 3.3, 4.4, 5.5, 6.5);
  tesseract::common::testSerialization<JointLimits>(*object, "JointLimits");
}

TEST(TesseractSceneGraphSerializationUnit, JointSafety)  // NOLINT
{
  auto object = std::make_shared<JointSafety>(1.1, 2.2, 3.3, 4.4);
  tesseract::common::testSerialization<JointSafety>(*object, "JointSafety");
}

TEST(TesseractSceneGraphSerializationUnit, JointCalibration)  // NOLINT
{
  auto object = std::make_shared<JointCalibration>(1.1, 2.2, 3.3);
  tesseract::common::testSerialization<JointCalibration>(*object, "JointCalibration");
}

TEST(TesseractSceneGraphSerializationUnit, JointMimic)  // NOLINT
{
  auto object = std::make_shared<JointMimic>(1.1, 2.2, "mimic_name");
  tesseract::common::testSerialization<JointMimic>(*object, "JointMimic");
}

TEST(TesseractSceneGraphSerializationUnit, Joint)  // NOLINT
{
  auto object = std::make_shared<Joint>("serialized_joint");
  object->type = JointType::PLANAR;
  object->axis = Eigen::Vector3d(1.1, 2.2, 3.3);
  object->child_link_name = "child_name";
  object->parent_link_name = "parent_name";
  object->parent_to_joint_origin_transform.translate(Eigen::Vector3d(5.5, 6.6, 7.7));
  object->dynamics = std::make_shared<JointDynamics>(1.1, 2.2);
  object->limits = std::make_shared<JointLimits>(1.1, 2.2, 3.3, 4.4, 5.5, 6.5);
  object->safety = std::make_shared<JointSafety>(1.1, 2.2, 3.3, 4.4);
  object->calibration = std::make_shared<JointCalibration>(1.1, 2.2, 3.3);
  object->mimic = std::make_shared<JointMimic>(1.1, 2.2, "mimic_name");
  tesseract::common::testSerialization<Joint>(*object, "Joint");
}

/*********************************************************/
/******                     Link                     *****/
/*********************************************************/

TEST(TesseractSceneGraphSerializationUnit, Material)  // NOLINT
{
  auto object = std::make_shared<Material>("test_name");
  object->color = Eigen::Vector4d(0.1, 0.2, 0.3, 1.0);
  object->texture_filename = "test_filename";
  tesseract::common::testSerialization<Material>(*object, "Material");
}

TEST(TesseractSceneGraphSerializationUnit, Inertial)  // NOLINT
{
  auto object = std::make_shared<Inertial>();
  object->origin.translate(Eigen::Vector3d(5.5, 6.6, 7.7));
  object->mass = 1.2;
  object->ixx = 2.3;
  object->ixy = 3.4;
  object->ixz = 4.5;
  object->iyy = 5.6;
  object->iyz = 6.7;
  object->izz = 7.8;
  tesseract::common::testSerialization<Inertial>(*object, "Inertial");
}

TEST(TesseractSceneGraphSerializationUnit, Visual)  // NOLINT
{
  auto object = std::make_shared<Visual>();
  object->origin.translate(Eigen::Vector3d(5.5, 6.6, 7.7));
  object->geometry = std::make_shared<tesseract::geometry::Cone>(1.1, 2.2);
  object->material = std::make_shared<Material>("test_name");
  object->name = "visual_name";
  tesseract::common::testSerialization<Visual>(*object, "Visual");
}

TEST(TesseractSceneGraphSerializationUnit, Collision)  // NOLINT
{
  auto object = std::make_shared<Collision>();
  object->origin.translate(Eigen::Vector3d(5.6, 7.8, -1.7));
  object->geometry = std::make_shared<tesseract::geometry::Sphere>(1.1);
  object->name = "collision_name";
  tesseract::common::testSerialization<Collision>(*object, "Collision");
}

TEST(TesseractSceneGraphSerializationUnit, Link)  // NOLINT
{
  auto object = std::make_shared<Link>("test_link");
  object->inertial = std::make_shared<Inertial>();

  auto vis = std::make_shared<Visual>();
  vis->origin.translate(Eigen::Vector3d(5.5, 6.6, 7.7));
  vis->geometry = std::make_shared<tesseract::geometry::Cone>(1.1, 2.2);
  vis->material = std::make_shared<Material>("test_name");
  vis->name = "visual_name";
  object->visual.push_back(std::make_shared<Visual>());
  object->visual.push_back(vis);

  auto col = std::make_shared<Collision>();
  col->origin.translate(Eigen::Vector3d(5.6, 7.8, -1.7));
  col->geometry = std::make_shared<tesseract::geometry::Sphere>(1.1);
  col->name = "collision_name";
  object->collision.push_back(std::make_shared<Collision>());
  object->collision.push_back(col);
  tesseract::common::testSerialization<Link>(*object, "Link");
}

tesseract::scene_graph::SceneGraph createTestSceneGraph()
{
  using namespace tesseract::scene_graph;
  SceneGraph g;

  Link link_1("link_1");
  Link link_2("link_2");
  Link link_3("link_3");
  Link link_4("link_4");
  Link link_5("link_5");

  EXPECT_TRUE(g.addLink(link_1));
  EXPECT_TRUE(g.addLink(link_2));
  EXPECT_TRUE(g.addLink(link_3));
  EXPECT_TRUE(g.addLink(link_4));
  EXPECT_TRUE(g.addLink(link_5));

  Joint joint_1("joint_1");
  joint_1.parent_to_joint_origin_transform.translation()(0) = 1.25;
  joint_1.parent_link_name = "link_1";
  joint_1.child_link_name = "link_2";
  joint_1.type = JointType::FIXED;
  EXPECT_TRUE(g.addJoint(joint_1));

  Joint joint_2("joint_2");
  joint_2.parent_to_joint_origin_transform.translation()(0) = 1.25;
  joint_2.parent_link_name = "link_2";
  joint_2.child_link_name = "link_3";
  joint_2.type = JointType::PLANAR;
  joint_2.limits = std::make_shared<JointLimits>(-1, 1, 0, 2, 3, 4);
  EXPECT_TRUE(g.addJoint(joint_2));

  Joint joint_3("joint_3");
  joint_3.parent_to_joint_origin_transform.translation()(0) = 1.25;
  joint_3.parent_link_name = "link_3";
  joint_3.child_link_name = "link_4";
  joint_3.type = JointType::FLOATING;
  EXPECT_TRUE(g.addJoint(joint_3));

  Joint joint_4("joint_4");
  joint_4.parent_to_joint_origin_transform.translation()(1) = 1.25;
  joint_4.parent_link_name = "link_2";
  joint_4.child_link_name = "link_5";
  joint_4.type = JointType::REVOLUTE;
  joint_4.limits = std::make_shared<JointLimits>(-1, 1, 0, 2, 3, 4);
  EXPECT_TRUE(g.addJoint(joint_4));

  g.addAllowedCollision("link_1", "link_2", "Adjacent");
  g.addAllowedCollision("link_2", "link_3", "Adjacent");
  g.addAllowedCollision("link_3", "link_5", "Adjacent");
  g.addAllowedCollision("link_2", "link_5", "Adjacent");

  return g;
}

TEST(TesseractSceneGraphSerializationUnit, SceneGraph)  // NOLINT
{
  auto object = createTestSceneGraph();
  tesseract::common::testSerialization<SceneGraph>(object, "SceneGraph");
}

TEST(TesseractSceneGraphSerializationUnit, SceneState)  // NOLINT
{
  auto object = std::make_shared<SceneState>();
  object->joints["j_1"] = 12.3;
  object->joints["j_2"] = -12.3;
  object->joints["j_3"] = 1.23;
  object->link_transforms["link_transforms_key"].setIdentity();
  object->link_transforms["link_transforms_key"].translate(Eigen::Vector3d(1, 2, 3));
  object->joint_transforms["joint_transforms_key"].setIdentity();
  object->joint_transforms["joint_transforms_key"].translate(Eigen::Vector3d(5, 6, 7));
  tesseract::common::testSerialization<SceneState>(*object, "SceneState");
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);

  return RUN_ALL_TESTS();
}
