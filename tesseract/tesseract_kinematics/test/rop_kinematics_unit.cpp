#include <tesseract_common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <gtest/gtest.h>
#include <fstream>
#include <tesseract_urdf/urdf_parser.h>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract_kinematics/kdl/kdl_fwd_kin_chain.h>
#include <tesseract_kinematics/core/rop_inverse_kinematics.h>
#include <tesseract_kinematics/opw/opw_inv_kin.h>
#include <tesseract_kinematics/core/utils.h>

std::string locateResource(const std::string& url)
{
  std::string mod_url = url;
  if (url.find("package://tesseract_support") == 0)
  {
    mod_url.erase(0, strlen("package://tesseract_support"));
    size_t pos = mod_url.find('/');
    if (pos == std::string::npos)
    {
      return std::string();
    }

    std::string package = mod_url.substr(0, pos);
    mod_url.erase(0, pos);
    std::string package_path = std::string(TESSERACT_SUPPORT_DIR);

    if (package_path.empty())
    {
      return std::string();
    }

    mod_url = package_path + mod_url;  // "file://" + package_path + mod_url;
  }

  return mod_url;
}

tesseract_scene_graph::SceneGraph::Ptr getSceneGraph()
{
  std::string path = std::string(TESSERACT_SUPPORT_DIR) + "/urdf/abb_irb2400_on_positioner.urdf";

  tesseract_scene_graph::ResourceLocator::Ptr locator =
      std::make_shared<tesseract_scene_graph::SimpleResourceLocator>(locateResource);

  return tesseract_urdf::parseURDFFile(path, locator);
}

tesseract_kinematics::ForwardKinematics::Ptr getRobotFwdKinematics(const tesseract_scene_graph::SceneGraph::Ptr& scene_graph)
{
  auto fwd_kin = std::make_shared<tesseract_kinematics::KDLFwdKinChain>();
  EXPECT_TRUE(fwd_kin->init(scene_graph, "base_link", "tool0", "manip"));
  return fwd_kin;
}

tesseract_kinematics::ForwardKinematics::Ptr getFullFwdKinematics(const tesseract_scene_graph::SceneGraph::Ptr& scene_graph)
{
  auto fwd_kin = std::make_shared<tesseract_kinematics::KDLFwdKinChain>();
  EXPECT_TRUE(fwd_kin->init(scene_graph, "positioner_base_link", "tool0", "robot_on_positioner"));
  return fwd_kin;
}

tesseract_kinematics::ForwardKinematics::Ptr getPositionerFwdKinematics(const tesseract_scene_graph::SceneGraph::Ptr& scene_graph)
{
  auto fwd_kin = std::make_shared<tesseract_kinematics::KDLFwdKinChain>();
  EXPECT_TRUE(fwd_kin->init(scene_graph, "positioner_base_link", "positioner_tool0", "positioner"));
  return fwd_kin;
}

tesseract_kinematics::InverseKinematics::Ptr getFullInvKinematics(const tesseract_scene_graph::SceneGraph::Ptr& scene_graph)
{
  auto robot_fwd_kin = getRobotFwdKinematics(scene_graph);

  opw_kinematics::Parameters<double> opw_params;
  opw_params.a1 = (0.100);
  opw_params.a2 = (-0.135);
  opw_params.b = (0.000);
  opw_params.c1 = (0.615);
  opw_params.c2 = (0.705);
  opw_params.c3 = (0.755);
  opw_params.c4 = (0.085);

  opw_params.offsets[2] = -M_PI / 2.0;

  auto opw_kin = std::make_shared<tesseract_kinematics::OPWInvKin>();
  opw_kin->init("robot",
                opw_params,
                robot_fwd_kin->getBaseLinkName(),
                robot_fwd_kin->getTipLinkName(),
                robot_fwd_kin->getJointNames(),
                robot_fwd_kin->getLinkNames(),
                robot_fwd_kin->getActiveLinkNames(),
                robot_fwd_kin->getLimits());

  auto positioner_kin = getPositionerFwdKinematics(scene_graph);
  Eigen::VectorXd positioner_resolution = Eigen::VectorXd::Constant(1, 1, 0.1);
  auto rop_inv_kin = std::make_shared<tesseract_kinematics::RobotOnPositionerInvKin>();
  rop_inv_kin->init(scene_graph, opw_kin, 2.5, positioner_kin, positioner_resolution, "robot_on_positioner");

  return rop_inv_kin;
}

void runInvKinTest(const tesseract_kinematics::InverseKinematics& inv_kin,
                   const tesseract_kinematics::ForwardKinematics& fwd_kin)
{
  ///////////////////////////
  // Test Inverse kinematics
  ///////////////////////////
  const std::vector<std::string>& fwd_joint_names = fwd_kin.getJointNames();
  const std::vector<std::string>& inv_joint_names = inv_kin.getJointNames();

  // TODO: Need to add a way to synchronize two kinematics so joints are in the same order.
  EXPECT_TRUE(std::equal(fwd_joint_names.begin(), fwd_joint_names.end(), inv_joint_names.begin()));

  Eigen::Isometry3d pose;
  pose.setIdentity();
  pose.translation()[0] = 0;
  pose.translation()[1] = 0;
  pose.translation()[2] = 1.306;

  Eigen::VectorXd seed = Eigen::VectorXd::Zero(fwd_kin.numJoints());

  Eigen::VectorXd solutions;
  EXPECT_TRUE(inv_kin.calcInvKin(solutions, pose, seed));

  long dof = static_cast<long>(inv_kin.numJoints());
  long num_sols = solutions.size() / dof;
  for (long i = 0; i < num_sols; i++)
  {
    Eigen::Isometry3d result;
    EXPECT_TRUE(fwd_kin.calcFwdKin(result, solutions.segment(dof * i, dof)));
    EXPECT_TRUE(pose.translation().isApprox(result.translation(), 1e-4));

    Eigen::Quaterniond rot_pose(pose.rotation());
    Eigen::Quaterniond rot_result(result.rotation());
    EXPECT_TRUE(rot_pose.isApprox(rot_result, 1e-3));
  }
}

void runActiveLinkNamesTest(tesseract_kinematics::InverseKinematics& kin)
{
  std::vector<std::string> link_names = kin.getActiveLinkNames();
  EXPECT_TRUE(link_names.size() == 9);
  EXPECT_TRUE(std::find(link_names.begin(), link_names.end(), "positioner_base_link") == link_names.end());
  EXPECT_TRUE(std::find(link_names.begin(), link_names.end(), "positioner_tool0") != link_names.end());
  EXPECT_TRUE(std::find(link_names.begin(), link_names.end(), "base_link") != link_names.end());
  EXPECT_TRUE(std::find(link_names.begin(), link_names.end(), "link_1") != link_names.end());
  EXPECT_TRUE(std::find(link_names.begin(), link_names.end(), "link_2") != link_names.end());
  EXPECT_TRUE(std::find(link_names.begin(), link_names.end(), "link_3") != link_names.end());
  EXPECT_TRUE(std::find(link_names.begin(), link_names.end(), "link_4") != link_names.end());
  EXPECT_TRUE(std::find(link_names.begin(), link_names.end(), "link_5") != link_names.end());
  EXPECT_TRUE(std::find(link_names.begin(), link_names.end(), "link_6") != link_names.end());
  EXPECT_TRUE(std::find(link_names.begin(), link_names.end(), "tool0") != link_names.end());


  link_names = kin.getLinkNames();
  EXPECT_TRUE(link_names.size() == 10);
  EXPECT_TRUE(std::find(link_names.begin(), link_names.end(), "positioner_base_link") != link_names.end());
  EXPECT_TRUE(std::find(link_names.begin(), link_names.end(), "positioner_tool0") != link_names.end());
  EXPECT_TRUE(std::find(link_names.begin(), link_names.end(), "base_link") != link_names.end());
  EXPECT_TRUE(std::find(link_names.begin(), link_names.end(), "link_1") != link_names.end());
  EXPECT_TRUE(std::find(link_names.begin(), link_names.end(), "link_2") != link_names.end());
  EXPECT_TRUE(std::find(link_names.begin(), link_names.end(), "link_3") != link_names.end());
  EXPECT_TRUE(std::find(link_names.begin(), link_names.end(), "link_4") != link_names.end());
  EXPECT_TRUE(std::find(link_names.begin(), link_names.end(), "link_5") != link_names.end());
  EXPECT_TRUE(std::find(link_names.begin(), link_names.end(), "link_6") != link_names.end());
  EXPECT_TRUE(std::find(link_names.begin(), link_names.end(), "tool0") != link_names.end());
}

TEST(TesseractKinematicsUnit, RobotOnPositionerInverseKinematicUnit)  // NOLINT
{
  tesseract_scene_graph::SceneGraph::Ptr scene_graph = getSceneGraph();

  auto fwd_kin = getFullFwdKinematics(scene_graph);
  auto inv_kin = getFullInvKinematics(scene_graph);

  runInvKinTest(*inv_kin, *fwd_kin);
  runActiveLinkNamesTest(*inv_kin);
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);

  return RUN_ALL_TESTS();
}
