#include <tesseract_common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <gtest/gtest.h>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract_state_solver/ofkt/ofkt_nodes.h>
#include <tesseract_state_solver/ofkt/ofkt_state_solver.h>
#include "state_solver_test_suite.h"

using namespace tesseract_scene_graph;

// Most of OFKT is tested in the tesseract_environment_unit.cpp
TEST(TesseractStateSolverUnit, OFKTNodeBaseAndFailuresUnit)  // NOLINT
{
  {  // OFKTRootNode
    OFKTRootNode node("base_link");
    EXPECT_ANY_THROW(node.setParent(nullptr));                                      // NOLINT
    EXPECT_ANY_THROW(node.storeJointValue(0));                                      // NOLINT
    EXPECT_ANY_THROW(node.setStaticTransformation(Eigen::Isometry3d::Identity()));  // NOLINT
    EXPECT_FALSE(node.updateWorldTransformationRequired());
    EXPECT_TRUE(Eigen::Isometry3d::Identity().isApprox(node.computeLocalTransformation(0), 1e-6));
  }

  {  // OFKTRootNode
    OFKTRootNode node("base_link");
    EXPECT_ANY_THROW(node.setParent(nullptr));                                      // NOLINT
    EXPECT_ANY_THROW(node.storeJointValue(0));                                      // NOLINT
    EXPECT_ANY_THROW(node.setStaticTransformation(Eigen::Isometry3d::Identity()));  // NOLINT
    EXPECT_FALSE(node.updateWorldTransformationRequired());
    EXPECT_TRUE(Eigen::Isometry3d::Identity().isApprox(node.computeLocalTransformation(0), 1e-6));
  }

  {  // OFKTFixedNode
    OFKTRootNode root_node("base_link");
    OFKTFixedNode node(&root_node, "base_link", "joint_a1", Eigen::Isometry3d::Identity());
    EXPECT_ANY_THROW(node.storeJointValue(0));  // NOLINT
    EXPECT_ANY_THROW(node.getJointValue());     // NOLINT
    EXPECT_FALSE(node.updateWorldTransformationRequired());
    EXPECT_TRUE(Eigen::Isometry3d::Identity().isApprox(node.computeLocalTransformation(0), 1e-6));
  }
}

TEST(TesseractStateSolverUnit, OFKTAddRemoveLinkUnit)  // NOLINT
{
  test_suite::runAddandRemoveLinkTest<OFKTStateSolver>();
}

TEST(TesseractStateSolverUnit, OFKTAddSceneGraphUnit)  // NOLINT
{
  test_suite::runAddSceneGraphTest<OFKTStateSolver>();
}

TEST(TesseractStateSolverUnit, OFKTChangeJointOriginUnit)  // NOLINT
{
  test_suite::runChangeJointOriginTest<OFKTStateSolver>();
}

TEST(TesseractStateSolverUnit, OFKTMoveJointUnit)  // NOLINT
{
  test_suite::runMoveJointTest<OFKTStateSolver>();
}

TEST(TesseractStateSolverUnit, OFKTMoveLinkUnit)  // NOLINT
{
  test_suite::runMoveLinkTest<OFKTStateSolver>();
}

TEST(TesseractStateSolverUnit, OFKTReplaceJointUnit)  // NOLINT
{
  test_suite::runReplaceJointTest<OFKTStateSolver>();
}

TEST(TesseractStateSolverUnit, OFKTChangeJointLimitsUnit)  // NOLINT
{
  test_suite::runChangeJointLimitsTest<OFKTStateSolver>();
}

TEST(TesseractStateSolverUnit, KDLGetJacobianUnit)  // NOLINT
{
  test_suite::runJacobianTest<KDLStateSolver>();
}

TEST(TesseractStateSolverUnit, OFKTGetJacobianUnit)  // NOLINT
{
  test_suite::runJacobianTest<OFKTStateSolver>();
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);

  return RUN_ALL_TESTS();
}
