#include <tesseract_common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <gtest/gtest.h>
#include <Eigen/Geometry>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract_urdf/joint.h>
#include "tesseract_urdf_common_unit.h"

TEST(TesseractURDFUnit, parse_joint)  // NOLINT
{
  {
    std::string str =
        R"(<joint name="my_joint" type="floating" extra="0 0 0">
             <origin xyz="0 0 1" rpy="0 0 3.1416"/>
             <parent link="link1" extra="0 0 0"/>
             <child link="link2" extra="0 0 0"/>
             <calibration rising="0.0"/>
             <dynamics damping="0.0" friction="0.0"/>
             <limit effort="30" velocity="1.0" lower="-2.2" upper="0.7" />
             <safety_controller k_velocity="10" k_position="15" soft_lower_limit="-2.0" soft_upper_limit="0.5" />
           </joint>)";
    tesseract_scene_graph::Joint::Ptr elem;
    EXPECT_TRUE(runTest<tesseract_scene_graph::Joint::Ptr>(elem, &tesseract_urdf::parseJoint, str, "joint", 2));
    EXPECT_TRUE(elem->getName() == "my_joint");
    EXPECT_TRUE(elem->type == tesseract_scene_graph::JointType::FLOATING);
    EXPECT_FALSE(elem->parent_to_joint_origin_transform.isApprox(Eigen::Isometry3d::Identity(), 1e-8));
    EXPECT_TRUE(elem->axis.isApprox(Eigen::Vector3d(1, 0, 0), 1e-8));
    EXPECT_TRUE(elem->parent_link_name == "link1");
    EXPECT_TRUE(elem->child_link_name == "link2");
    EXPECT_TRUE(elem->calibration != nullptr);
    EXPECT_TRUE(elem->dynamics != nullptr);
    EXPECT_TRUE(elem->limits == nullptr);
    EXPECT_TRUE(elem->safety != nullptr);
    EXPECT_TRUE(elem->mimic == nullptr);
  }

  {
    std::string str =
        R"(<joint name="my_joint" type="revolute">
             <origin xyz="0 0 1" rpy="0 0 3.1416"/>
             <parent link="link1"/>
             <child link="link2"/>
             <calibration rising="0.0"/>
             <dynamics damping="0.0" friction="0.0"/>
             <limit effort="30" velocity="1.0" lower="-2.2" upper="0.7" />
             <safety_controller k_velocity="10" k_position="15" soft_lower_limit="-2.0" soft_upper_limit="0.5" />
           </joint>)";
    tesseract_scene_graph::Joint::Ptr elem;
    EXPECT_TRUE(runTest<tesseract_scene_graph::Joint::Ptr>(elem, &tesseract_urdf::parseJoint, str, "joint", 2));
    EXPECT_TRUE(elem->getName() == "my_joint");
    EXPECT_TRUE(elem->type == tesseract_scene_graph::JointType::REVOLUTE);
    EXPECT_FALSE(elem->parent_to_joint_origin_transform.isApprox(Eigen::Isometry3d::Identity(), 1e-8));
    EXPECT_TRUE(elem->axis.isApprox(Eigen::Vector3d(1, 0, 0), 1e-8));
    EXPECT_TRUE(elem->parent_link_name == "link1");
    EXPECT_TRUE(elem->child_link_name == "link2");
    EXPECT_TRUE(elem->calibration != nullptr);
    EXPECT_TRUE(elem->dynamics != nullptr);
    EXPECT_TRUE(elem->limits != nullptr);
    EXPECT_TRUE(elem->safety != nullptr);
    EXPECT_TRUE(elem->mimic == nullptr);
  }

  {
    std::string str = R"(<joint name="my_joint" type="revolute">
                           <axis xyz="0 0 1"/>
                           <parent link="link1"/>
                           <child link="link2"/>
                           <limit effort="30" velocity="1.0" lower="-2.2" upper="0.7" />
                         </joint>)";
    tesseract_scene_graph::Joint::Ptr elem;
    EXPECT_TRUE(runTest<tesseract_scene_graph::Joint::Ptr>(elem, &tesseract_urdf::parseJoint, str, "joint", 2));
    EXPECT_TRUE(elem->getName() == "my_joint");
    EXPECT_TRUE(elem->type == tesseract_scene_graph::JointType::REVOLUTE);
    EXPECT_TRUE(elem->parent_to_joint_origin_transform.isApprox(Eigen::Isometry3d::Identity(), 1e-8));
    EXPECT_TRUE(elem->axis.isApprox(Eigen::Vector3d(0, 0, 1), 1e-8));
    EXPECT_TRUE(elem->parent_link_name == "link1");
    EXPECT_TRUE(elem->child_link_name == "link2");
    EXPECT_TRUE(elem->calibration == nullptr);
    EXPECT_TRUE(elem->dynamics == nullptr);
    EXPECT_TRUE(elem->limits != nullptr);
    EXPECT_TRUE(elem->safety == nullptr);
    EXPECT_TRUE(elem->mimic == nullptr);
  }

  {
    std::string str = R"(<joint name="my_joint" type="continuous">
                           <axis xyz="0 0 1"/>
                           <parent link="link1"/>
                           <child link="link2"/>
                           <limit effort="30" velocity="1.0"/>
                         </joint>)";
    tesseract_scene_graph::Joint::Ptr elem;
    EXPECT_TRUE(runTest<tesseract_scene_graph::Joint::Ptr>(elem, &tesseract_urdf::parseJoint, str, "joint", 2));
    EXPECT_TRUE(elem->getName() == "my_joint");
    EXPECT_TRUE(elem->type == tesseract_scene_graph::JointType::CONTINUOUS);
    EXPECT_TRUE(elem->parent_to_joint_origin_transform.isApprox(Eigen::Isometry3d::Identity(), 1e-8));
    EXPECT_TRUE(elem->axis.isApprox(Eigen::Vector3d(0, 0, 1), 1e-8));
    EXPECT_TRUE(elem->parent_link_name == "link1");
    EXPECT_TRUE(elem->child_link_name == "link2");
    EXPECT_TRUE(elem->calibration == nullptr);
    EXPECT_TRUE(elem->dynamics == nullptr);
    EXPECT_TRUE(elem->limits != nullptr);
    EXPECT_TRUE(elem->safety == nullptr);
    EXPECT_TRUE(elem->mimic == nullptr);
  }

  {
    std::string str = R"(<joint name="my_joint" type="continuous">
                           <axis xyz="0 0 1"/>
                           <parent link="link1"/>
                           <child link="link2"/>
                         </joint>)";
    tesseract_scene_graph::Joint::Ptr elem;
    EXPECT_TRUE(runTest<tesseract_scene_graph::Joint::Ptr>(elem, &tesseract_urdf::parseJoint, str, "joint", 2));
    EXPECT_TRUE(elem->getName() == "my_joint");
    EXPECT_TRUE(elem->type == tesseract_scene_graph::JointType::CONTINUOUS);
    EXPECT_TRUE(elem->parent_to_joint_origin_transform.isApprox(Eigen::Isometry3d::Identity(), 1e-8));
    EXPECT_TRUE(elem->axis.isApprox(Eigen::Vector3d(0, 0, 1), 1e-8));
    EXPECT_TRUE(elem->parent_link_name == "link1");
    EXPECT_TRUE(elem->child_link_name == "link2");
    EXPECT_TRUE(elem->calibration == nullptr);
    EXPECT_TRUE(elem->dynamics == nullptr);
    EXPECT_TRUE(elem->limits != nullptr);
    EXPECT_TRUE(elem->safety == nullptr);
    EXPECT_TRUE(elem->mimic == nullptr);
  }

  {
    std::string str = R"(<joint name="my_joint" type="fixed">
                           <axis xyz="0 0 1"/>
                           <parent link="link1"/>
                           <child link="link2"/>
                           <limit effort="30" velocity="1.0" lower="-2.2" upper="0.7" />
                         </joint>)";
    tesseract_scene_graph::Joint::Ptr elem;
    EXPECT_TRUE(runTest<tesseract_scene_graph::Joint::Ptr>(elem, &tesseract_urdf::parseJoint, str, "joint", 2));
    EXPECT_TRUE(elem->getName() == "my_joint");
    EXPECT_TRUE(elem->type == tesseract_scene_graph::JointType::FIXED);
    EXPECT_TRUE(elem->parent_to_joint_origin_transform.isApprox(Eigen::Isometry3d::Identity(), 1e-8));
    EXPECT_TRUE(elem->axis.isApprox(Eigen::Vector3d(1, 0, 0), 1e-8));
    EXPECT_TRUE(elem->parent_link_name == "link1");
    EXPECT_TRUE(elem->child_link_name == "link2");
    EXPECT_TRUE(elem->calibration == nullptr);
    EXPECT_TRUE(elem->dynamics == nullptr);
    EXPECT_TRUE(elem->limits == nullptr);
    EXPECT_TRUE(elem->safety == nullptr);
    EXPECT_TRUE(elem->mimic == nullptr);
  }

  {
    std::string str = R"(<joint name="my_joint" type="prismatic">
                           <axis xyz="0 0 -1"/>
                           <parent link="link1"/>
                           <child link="link2"/>
                           <limit effort="30" velocity="1.0" lower="-2.2" upper="0.7" />
                         </joint>)";
    tesseract_scene_graph::Joint::Ptr elem;
    EXPECT_TRUE(runTest<tesseract_scene_graph::Joint::Ptr>(elem, &tesseract_urdf::parseJoint, str, "joint", 2));
    EXPECT_TRUE(elem->getName() == "my_joint");
    EXPECT_TRUE(elem->type == tesseract_scene_graph::JointType::PRISMATIC);
    EXPECT_TRUE(elem->parent_to_joint_origin_transform.isApprox(Eigen::Isometry3d::Identity(), 1e-8));
    EXPECT_TRUE(elem->axis.isApprox(Eigen::Vector3d(0, 0, -1), 1e-8));
    EXPECT_TRUE(elem->parent_link_name == "link1");
    EXPECT_TRUE(elem->child_link_name == "link2");
    EXPECT_TRUE(elem->calibration == nullptr);
    EXPECT_TRUE(elem->dynamics == nullptr);
    EXPECT_TRUE(elem->limits != nullptr);
    EXPECT_TRUE(elem->safety == nullptr);
    EXPECT_TRUE(elem->mimic == nullptr);
  }

  {
    std::string str = R"(<joint name="my_joint" type="revolute">
                           <axis xyz="0 0 -1"/>
                           <parent link="link1"/>
                           <child link="link2"/>
                           <limit effort="30" velocity="1.0" lower="-2.2" upper="0.7" />
                         </joint>)";
    tesseract_scene_graph::Joint::Ptr elem;
    EXPECT_TRUE(runTest<tesseract_scene_graph::Joint::Ptr>(elem, &tesseract_urdf::parseJoint, str, "joint", 2));
    EXPECT_TRUE(elem->getName() == "my_joint");
    EXPECT_TRUE(elem->type == tesseract_scene_graph::JointType::REVOLUTE);
    EXPECT_TRUE(elem->parent_to_joint_origin_transform.isApprox(Eigen::Isometry3d::Identity(), 1e-8));
    EXPECT_TRUE(elem->axis.isApprox(Eigen::Vector3d(0, 0, -1), 1e-8));
    EXPECT_TRUE(elem->parent_link_name == "link1");
    EXPECT_TRUE(elem->child_link_name == "link2");
    EXPECT_TRUE(elem->calibration == nullptr);
    EXPECT_TRUE(elem->dynamics == nullptr);
    EXPECT_TRUE(elem->limits != nullptr);
    EXPECT_TRUE(elem->safety == nullptr);
    EXPECT_TRUE(elem->mimic == nullptr);
  }

  {
    std::string str = R"(<joint name="my_joint" type="planar">
                           <axis xyz="0 0 -1.0"/>
                           <parent link="link1"/>
                           <child link="link2"/>
                           <limit effort="30" velocity="1.0" lower="-2.2" upper="0.7" />
                         </joint>)";
    tesseract_scene_graph::Joint::Ptr elem;
    EXPECT_TRUE(runTest<tesseract_scene_graph::Joint::Ptr>(elem, &tesseract_urdf::parseJoint, str, "joint", 2));
    EXPECT_TRUE(elem->getName() == "my_joint");
    EXPECT_TRUE(elem->type == tesseract_scene_graph::JointType::PLANAR);
    EXPECT_TRUE(elem->parent_to_joint_origin_transform.isApprox(Eigen::Isometry3d::Identity(), 1e-8));
    EXPECT_TRUE(elem->axis.isApprox(Eigen::Vector3d(0, 0, -1), 1e-8));
    EXPECT_TRUE(elem->parent_link_name == "link1");
    EXPECT_TRUE(elem->child_link_name == "link2");
    EXPECT_TRUE(elem->calibration == nullptr);
    EXPECT_TRUE(elem->dynamics == nullptr);
    EXPECT_TRUE(elem->limits == nullptr);
    EXPECT_TRUE(elem->safety == nullptr);
    EXPECT_TRUE(elem->mimic == nullptr);
  }

  {
    std::string str = R"(<joint name="my_joint">
                           <parent link="link1"/>
                           <child link="link2"/>
                         </joint>)";
    tesseract_scene_graph::Joint::Ptr elem;
    EXPECT_FALSE(runTest<tesseract_scene_graph::Joint::Ptr>(elem, &tesseract_urdf::parseJoint, str, "joint", 2));
  }

  {
    std::string str = R"(<joint type="planar">
                           <parent link="link1"/>
                           <child link="link2"/>
                         </joint>)";
    tesseract_scene_graph::Joint::Ptr elem;
    EXPECT_FALSE(runTest<tesseract_scene_graph::Joint::Ptr>(elem, &tesseract_urdf::parseJoint, str, "joint", 2));
  }

  {
    std::string str = R"(<joint name="my_joint" type="planar">
                           <child link="link2"/>
                         </joint>)";
    tesseract_scene_graph::Joint::Ptr elem;
    EXPECT_FALSE(runTest<tesseract_scene_graph::Joint::Ptr>(elem, &tesseract_urdf::parseJoint, str, "joint", 2));
  }

  {
    std::string str = R"(<joint name="my_joint" type="planar">
                           <parent link="link1"/>
                         </joint>)";
    tesseract_scene_graph::Joint::Ptr elem;
    EXPECT_FALSE(runTest<tesseract_scene_graph::Joint::Ptr>(elem, &tesseract_urdf::parseJoint, str, "joint", 2));
  }

  {
    std::string str = R"(<joint name="my_joint" type="planar">
                           <axis />
                           <parent link="link1"/>
                           <child link="link2"/>
                           <limit effort="30" velocity="1.0" lower="-2.2" upper="0.7" />
                         </joint>)";
    tesseract_scene_graph::Joint::Ptr elem;
    EXPECT_FALSE(runTest<tesseract_scene_graph::Joint::Ptr>(elem, &tesseract_urdf::parseJoint, str, "joint", 2));
  }

  {
    std::string str = R"(<joint name="my_joint" type="planar">
                           <axis xyz="0 0 -1.0 4"/>
                           <parent link="link1"/>
                           <child link="link2"/>
                           <limit effort="30" velocity="1.0" lower="-2.2" upper="0.7" />
                         </joint>)";
    tesseract_scene_graph::Joint::Ptr elem;
    EXPECT_FALSE(runTest<tesseract_scene_graph::Joint::Ptr>(elem, &tesseract_urdf::parseJoint, str, "joint", 2));
  }

  {
    std::string str = R"(<joint name="my_joint" type="planar">
                           <axis xyz="a 0 -1.0"/>
                           <parent link="link1"/>
                           <child link="link2"/>
                           <limit effort="30" velocity="1.0" lower="-2.2" upper="0.7" />
                         </joint>)";
    tesseract_scene_graph::Joint::Ptr elem;
    EXPECT_FALSE(runTest<tesseract_scene_graph::Joint::Ptr>(elem, &tesseract_urdf::parseJoint, str, "joint", 2));
  }

  {
    std::string str = R"(<joint name="my_joint" type="planar">
                           <axis xyz="0 a -1.0"/>
                           <parent link="link1"/>
                           <child link="link2"/>
                           <limit effort="30" velocity="1.0" lower="-2.2" upper="0.7" />
                         </joint>)";
    tesseract_scene_graph::Joint::Ptr elem;
    EXPECT_FALSE(runTest<tesseract_scene_graph::Joint::Ptr>(elem, &tesseract_urdf::parseJoint, str, "joint", 2));
  }

  {
    std::string str = R"(<joint name="my_joint" type="planar">
                           <axis xyz="0 0 a"/>
                           <parent link="link1"/>
                           <child link="link2"/>
                           <limit effort="30" velocity="1.0" lower="-2.2" upper="0.7" />
                         </joint>)";
    tesseract_scene_graph::Joint::Ptr elem;
    EXPECT_FALSE(runTest<tesseract_scene_graph::Joint::Ptr>(elem, &tesseract_urdf::parseJoint, str, "joint", 2));
  }

  {
    std::string str =
        R"(<joint name="my_joint" type="floating" extra="0 0 0">
             <origin xyz="0 0 1 4" rpy="0 0 3.1416"/>
             <parent link="link1" extra="0 0 0"/>
             <child link="link2" extra="0 0 0"/>
             <calibration rising="0.0"/>
             <dynamics damping="0.0" friction="0.0"/>
             <limit effort="30" velocity="1.0" lower="-2.2" upper="0.7" />
             <safety_controller k_velocity="10" k_position="15" soft_lower_limit="-2.0" soft_upper_limit="0.5" />
           </joint>)";
    tesseract_scene_graph::Joint::Ptr elem;
    EXPECT_FALSE(runTest<tesseract_scene_graph::Joint::Ptr>(elem, &tesseract_urdf::parseJoint, str, "joint", 2));
  }

  {
    std::string str =
        R"(<joint name="my_joint" type="floating" extra="0 0 0">
             <origin xyz="0 0 1" rpy="0 0 3.1416"/>
             <parent extra="0 0 0"/>
             <child link="link2" extra="0 0 0"/>
             <calibration rising="0.0"/>
             <dynamics damping="0.0" friction="0.0"/>
             <limit effort="30" velocity="1.0" lower="-2.2" upper="0.7" />
             <safety_controller k_velocity="10" k_position="15" soft_lower_limit="-2.0" soft_upper_limit="0.5" />
           </joint>)";
    tesseract_scene_graph::Joint::Ptr elem;
    EXPECT_FALSE(runTest<tesseract_scene_graph::Joint::Ptr>(elem, &tesseract_urdf::parseJoint, str, "joint", 2));
  }

  {
    std::string str =
        R"(<joint name="my_joint" type="floating" extra="0 0 0">
             <origin xyz="0 0 1" rpy="0 0 3.1416"/>
             <parent link="link1" extra="0 0 0"/>
             <child extra="0 0 0"/>
             <calibration rising="0.0"/>
             <dynamics damping="0.0" friction="0.0"/>
             <limit effort="30" velocity="1.0" lower="-2.2" upper="0.7" />
             <safety_controller k_velocity="10" k_position="15" soft_lower_limit="-2.0" soft_upper_limit="0.5" />
           </joint>)";
    tesseract_scene_graph::Joint::Ptr elem;
    EXPECT_FALSE(runTest<tesseract_scene_graph::Joint::Ptr>(elem, &tesseract_urdf::parseJoint, str, "joint", 2));
  }

  {
    std::string str =
        R"(<joint name="my_joint" type="error" extra="0 0 0">
             <origin xyz="0 0 1" rpy="0 0 3.1416"/>
             <parent link="link1" extra="0 0 0"/>
             <child link="link2" extra="0 0 0"/>
             <calibration rising="0.0"/>
             <dynamics damping="0.0" friction="0.0"/>
             <limit effort="30" velocity="1.0" lower="-2.2" upper="0.7" />
             <safety_controller k_velocity="10" k_position="15" soft_lower_limit="-2.0" soft_upper_limit="0.5" />
           </joint>)";
    tesseract_scene_graph::Joint::Ptr elem;
    EXPECT_FALSE(runTest<tesseract_scene_graph::Joint::Ptr>(elem, &tesseract_urdf::parseJoint, str, "joint", 2));
  }

  {
    std::string str =
        R"(<joint name="my_joint" type="floating" extra="0 0 0">
             <origin xyz="0 0 1" rpy="0 0 3.1416"/>
             <parent link="link1" extra="0 0 0"/>
             <child link="link2" extra="0 0 0"/>
             <calibration/>
             <dynamics damping="0.0" friction="0.0"/>
             <limit effort="30" velocity="1.0" lower="-2.2" upper="0.7" />
             <safety_controller k_velocity="10" k_position="15" soft_lower_limit="-2.0" soft_upper_limit="0.5" />
           </joint>)";
    tesseract_scene_graph::Joint::Ptr elem;
    EXPECT_FALSE(runTest<tesseract_scene_graph::Joint::Ptr>(elem, &tesseract_urdf::parseJoint, str, "joint", 2));
  }

  {
    std::string str =
        R"(<joint name="my_joint" type="floating" extra="0 0 0">
             <origin xyz="0 0 1" rpy="0 0 3.1416"/>
             <parent link="link1" extra="0 0 0"/>
             <child link="link2" extra="0 0 0"/>
             <calibration rising="0.0"/>
             <dynamics />
             <limit effort="30" velocity="1.0" lower="-2.2" upper="0.7" />
             <safety_controller k_velocity="10" k_position="15" soft_lower_limit="-2.0" soft_upper_limit="0.5" />
           </joint>)";
    tesseract_scene_graph::Joint::Ptr elem;
    EXPECT_FALSE(runTest<tesseract_scene_graph::Joint::Ptr>(elem, &tesseract_urdf::parseJoint, str, "joint", 2));
  }

  {
    std::string str =
        R"(<joint name="my_joint" type="revolute" extra="0 0 0">
             <axis xyz="0 0 1"/>
             <origin xyz="0 0 1" rpy="0 0 3.1416"/>
             <parent link="link1" extra="0 0 0"/>
             <child link="link2" extra="0 0 0"/>
             <calibration rising="0.0"/>
             <dynamics damping="0.0" friction="0.0"/>
             <limit  />
             <safety_controller k_velocity="10" k_position="15" soft_lower_limit="-2.0" soft_upper_limit="0.5" />
           </joint>)";
    tesseract_scene_graph::Joint::Ptr elem;
    EXPECT_FALSE(runTest<tesseract_scene_graph::Joint::Ptr>(elem, &tesseract_urdf::parseJoint, str, "joint", 2));
  }

  {
    std::string str =
        R"(<joint name="my_joint" type="floating" extra="0 0 0">
             <origin xyz="0 0 1" rpy="0 0 3.1416"/>
             <parent link="link1" extra="0 0 0"/>
             <child link="link2" extra="0 0 0"/>
             <calibration rising="0.0"/>
             <dynamics damping="0.0" friction="0.0"/>
             <limit effort="30" velocity="1.0" lower="-2.2" upper="0.7" />
             <safety_controller/>
           </joint>)";
    tesseract_scene_graph::Joint::Ptr elem;
    EXPECT_FALSE(runTest<tesseract_scene_graph::Joint::Ptr>(elem, &tesseract_urdf::parseJoint, str, "joint", 2));
  }

  {
    std::string str = R"(<joint name="my_joint" type="mimic">
                           <axis xyz="0 0 -1"/>
                           <parent link="link1"/>
                           <child link="link2"/>
                           <limit effort="30" velocity="1.0" lower="-2.2" upper="0.7" />
                           <mimic multiplier="a" offset="1" />
                         </joint>)";
    tesseract_scene_graph::Joint::Ptr elem;
    EXPECT_FALSE(runTest<tesseract_scene_graph::Joint::Ptr>(elem, &tesseract_urdf::parseJoint, str, "joint", 2));
  }

  {
    std::string str = R"(<joint name="my_joint" type="revolute">
                           <axis xyz="0 0 -1"/>
                           <parent link="link1"/>
                           <child link="link2"/>
                         </joint>)";
    tesseract_scene_graph::Joint::Ptr elem;
    EXPECT_FALSE(runTest<tesseract_scene_graph::Joint::Ptr>(elem, &tesseract_urdf::parseJoint, str, "joint", 2));
  }
}

TEST(TesseractURDFUnit, write_joint)  // NOLINT
{
  {  // trigger nullptr input
    tesseract_scene_graph::Joint::Ptr joint = nullptr;
    std::string text;
    EXPECT_EQ(1, writeTest<tesseract_scene_graph::Joint::Ptr>(joint, &tesseract_urdf::writeJoint, text));
    EXPECT_EQ(text, "");
  }

  {  // trigger type planar, set joint axis
    tesseract_scene_graph::Joint::Ptr joint = std::make_shared<tesseract_scene_graph::Joint>("joint_0");
    joint->type = tesseract_scene_graph::JointType::PLANAR;
    joint->axis = Eigen::Vector3d::UnitX();
    std::string text;
    EXPECT_EQ(0, writeTest<tesseract_scene_graph::Joint::Ptr>(joint, &tesseract_urdf::writeJoint, text));
    EXPECT_NE(text, "");
  }

  {  // trigger type floating
    tesseract_scene_graph::Joint::Ptr joint = std::make_shared<tesseract_scene_graph::Joint>("joint_0");
    joint->type = tesseract_scene_graph::JointType::FLOATING;
    std::string text;
    EXPECT_EQ(0, writeTest<tesseract_scene_graph::Joint::Ptr>(joint, &tesseract_urdf::writeJoint, text));
    EXPECT_NE(text, "");
  }

  {  // trigger type revolute; set joint axis, joint limits, joint safety, joint cal, mimic joint, dynamics
    tesseract_scene_graph::Joint::Ptr joint = std::make_shared<tesseract_scene_graph::Joint>("joint_0");
    joint->type = tesseract_scene_graph::JointType::REVOLUTE;
    joint->axis = Eigen::Vector3d::UnitY();
    joint->limits = std::make_shared<tesseract_scene_graph::JointLimits>();
    joint->safety = std::make_shared<tesseract_scene_graph::JointSafety>();
    joint->calibration = std::make_shared<tesseract_scene_graph::JointCalibration>();
    joint->mimic = std::make_shared<tesseract_scene_graph::JointMimic>();
    joint->dynamics = std::make_shared<tesseract_scene_graph::JointDynamics>();
    std::string text;
    EXPECT_EQ(0, writeTest<tesseract_scene_graph::Joint::Ptr>(joint, &tesseract_urdf::writeJoint, text));
    EXPECT_NE(text, "");
  }

  {  // trigger type continuous
    tesseract_scene_graph::Joint::Ptr joint = std::make_shared<tesseract_scene_graph::Joint>("joint_0");
    joint->type = tesseract_scene_graph::JointType::CONTINUOUS;
    joint->axis = Eigen::Vector3d::UnitZ();
    joint->limits = std::make_shared<tesseract_scene_graph::JointLimits>();
    std::string text;
    EXPECT_EQ(0, writeTest<tesseract_scene_graph::Joint::Ptr>(joint, &tesseract_urdf::writeJoint, text));
    EXPECT_NE(text, "");
  }

  {  // trigger type prismatic
    tesseract_scene_graph::Joint::Ptr joint = std::make_shared<tesseract_scene_graph::Joint>("joint_0");
    joint->type = tesseract_scene_graph::JointType::PRISMATIC;
    joint->limits = std::make_shared<tesseract_scene_graph::JointLimits>();
    joint->axis = Eigen::Vector3d::Ones();
    std::string text;
    EXPECT_EQ(0, writeTest<tesseract_scene_graph::Joint::Ptr>(joint, &tesseract_urdf::writeJoint, text));
    EXPECT_NE(text, "");
  }

  {  // trigger type fixed
    tesseract_scene_graph::Joint::Ptr joint = std::make_shared<tesseract_scene_graph::Joint>("joint_0");
    joint->type = tesseract_scene_graph::JointType::FIXED;
    std::string text;
    EXPECT_EQ(0, writeTest<tesseract_scene_graph::Joint::Ptr>(joint, &tesseract_urdf::writeJoint, text));
    EXPECT_NE(text, "");
  }

  {  // trigger no joint limits
    tesseract_scene_graph::Joint::Ptr joint = std::make_shared<tesseract_scene_graph::Joint>("joint_0");
    joint->type = tesseract_scene_graph::JointType::PRISMATIC;
    joint->limits = nullptr;
    joint->axis = Eigen::Vector3d::Ones();
    std::string text;
    EXPECT_EQ(1, writeTest<tesseract_scene_graph::Joint::Ptr>(joint, &tesseract_urdf::writeJoint, text));
    EXPECT_EQ(text, "");
  }
}
