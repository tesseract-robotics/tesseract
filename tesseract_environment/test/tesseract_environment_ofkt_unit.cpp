#include <tesseract_common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <gtest/gtest.h>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract_environment/ofkt/ofkt_nodes.h>
#include <tesseract_environment/ofkt/ofkt_state_solver.h>

using namespace tesseract_environment;

// Most of OFKT is tested in the tesseract_environment_unit.cpp
TEST(TesseractEnvironmentUnit, OFKTNodeBaseAndFailuresUnit)  // NOLINT
{
  {  // OFKTRootNode
    OFKTRootNode node("base_link");
    EXPECT_ANY_THROW(node.setParent(nullptr));
    EXPECT_ANY_THROW(node.storeJointValue(0));
    EXPECT_ANY_THROW(node.setStaticTransformation(Eigen::Isometry3d::Identity()));
    EXPECT_FALSE(node.updateWorldTransformationRequired());
    EXPECT_TRUE(Eigen::Isometry3d::Identity().isApprox(node.computeLocalTransformation(0), 1e-6));
  }

  {  // OFKTRootNode
    OFKTRootNode node("base_link");
    EXPECT_ANY_THROW(node.setParent(nullptr));
    EXPECT_ANY_THROW(node.storeJointValue(0));
    EXPECT_ANY_THROW(node.setStaticTransformation(Eigen::Isometry3d::Identity()));
    EXPECT_FALSE(node.updateWorldTransformationRequired());
    EXPECT_TRUE(Eigen::Isometry3d::Identity().isApprox(node.computeLocalTransformation(0), 1e-6));
  }

  {  // OFKTFixedNode
    OFKTRootNode root_node("base_link");
    OFKTFixedNode node(&root_node, "base_link", "joint_a1", Eigen::Isometry3d::Identity());
    EXPECT_ANY_THROW(node.storeJointValue(0));
    EXPECT_ANY_THROW(node.getJointValue());
    EXPECT_FALSE(node.updateWorldTransformationRequired());
    EXPECT_TRUE(Eigen::Isometry3d::Identity().isApprox(node.computeLocalTransformation(0), 1e-6));
  }
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);

  return RUN_ALL_TESTS();
}
