#include <tesseract_common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <gtest/gtest.h>
#include <fstream>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include "kinematics_test_utils.h"

#include <tesseract_kinematics/kdl/kdl_fwd_kin_chain.h>
#include <tesseract_kinematics/kdl/kdl_inv_kin_chain_lma.h>
#include <tesseract_kinematics/kdl/kdl_inv_kin_chain_nr.h>
#include <tesseract_kinematics/kdl/kdl_inv_kin_chain_nr_jl.h>

using namespace tesseract::kinematics::test_suite;

TEST(TesseractKinematicsUnit, KDLKinChainLMAInverseKinematicUnit)  // NOLINT
{
  tesseract::common::GeneralResourceLocator locator;
  auto scene_graph = getSceneGraphIIWA(locator);

  tesseract::kinematics::KDLInvKinChainLMA::Config config;
  tesseract::kinematics::KDLInvKinChainLMA derived_kin(*scene_graph, "base_link", "tool0", config);

  tesseract::kinematics::KinematicsPluginFactory factory;
  runInvKinIIWATest(factory, "KDLInvKinChainLMAFactory", "KDLFwdKinChainFactory");
}

TEST(TesseractKinematicsUnit, KDLKinChainNRInverseKinematicUnit)  // NOLINT
{
  tesseract::common::GeneralResourceLocator locator;
  auto scene_graph = getSceneGraphIIWA(locator);

  tesseract::kinematics::KDLInvKinChainNR::Config config;
  tesseract::kinematics::KDLInvKinChainNR derived_kin(*scene_graph, "base_link", "tool0", config);

  tesseract::kinematics::KinematicsPluginFactory factory;
  runInvKinIIWATest(factory, "KDLInvKinChainNRFactory", "KDLFwdKinChainFactory");
}

TEST(TesseractKinematicsUnit, KDLKinChainNR_JLInverseKinematicUnit)  // NOLINT
{
  tesseract::common::GeneralResourceLocator locator;
  auto scene_graph = getSceneGraphIIWA(locator);

  tesseract::kinematics::KDLInvKinChainNR_JL::Config config;
  tesseract::kinematics::KDLInvKinChainNR_JL derived_kin(*scene_graph, "base_link", "tool0", config);

  tesseract::kinematics::KinematicsPluginFactory factory;
  runInvKinIIWATest(factory, "KDLInvKinChainNR_JLFactory", "KDLFwdKinChainFactory");
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);

  return RUN_ALL_TESTS();
}
