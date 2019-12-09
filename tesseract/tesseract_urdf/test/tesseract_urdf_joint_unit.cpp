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
    auto status = runTest<tesseract_scene_graph::Joint::Ptr>(elem, str, "joint", 2);
    EXPECT_TRUE(*status);
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
    auto status = runTest<tesseract_scene_graph::Joint::Ptr>(elem, str, "joint", 2);
    EXPECT_TRUE(*status);
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
    auto status = runTest<tesseract_scene_graph::Joint::Ptr>(elem, str, "joint", 2);
    EXPECT_TRUE(*status);
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
    auto status = runTest<tesseract_scene_graph::Joint::Ptr>(elem, str, "joint", 2);
    EXPECT_TRUE(*status);
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
    auto status = runTest<tesseract_scene_graph::Joint::Ptr>(elem, str, "joint", 2);
    EXPECT_TRUE(*status);
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
    auto status = runTest<tesseract_scene_graph::Joint::Ptr>(elem, str, "joint", 2);
    EXPECT_TRUE(*status);
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
    std::string str = R"(<joint name="my_joint" type="planar">
                           <axis xyz="0 0 -1.0"/>
                           <parent link="link1"/>
                           <child link="link2"/>
                           <limit effort="30" velocity="1.0" lower="-2.2" upper="0.7" />
                         </joint>)";
    tesseract_scene_graph::Joint::Ptr elem;
    auto status = runTest<tesseract_scene_graph::Joint::Ptr>(elem, str, "joint", 2);
    EXPECT_TRUE(*status);
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
    auto status = runTest<tesseract_scene_graph::Joint::Ptr>(elem, str, "joint", 2);
    EXPECT_FALSE(*status);
  }

  {
    std::string str = R"(<joint type="planar">
                           <parent link="link1"/>
                           <child link="link2"/>
                         </joint>)";
    tesseract_scene_graph::Joint::Ptr elem;
    auto status = runTest<tesseract_scene_graph::Joint::Ptr>(elem, str, "joint", 2);
    EXPECT_FALSE(*status);
  }

  {
    std::string str = R"(<joint name="my_joint" type="planar">
                           <child link="link2"/>
                         </joint>)";
    tesseract_scene_graph::Joint::Ptr elem;
    auto status = runTest<tesseract_scene_graph::Joint::Ptr>(elem, str, "joint", 2);
    EXPECT_FALSE(*status);
  }

  {
    std::string str = R"(<joint name="my_joint" type="planar">
                           <parent link="link1"/>
                         </joint>)";
    tesseract_scene_graph::Joint::Ptr elem;
    auto status = runTest<tesseract_scene_graph::Joint::Ptr>(elem, str, "joint", 2);
    EXPECT_FALSE(*status);
  }
}
