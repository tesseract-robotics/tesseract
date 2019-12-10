#include <tesseract_common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <gtest/gtest.h>
#include <fstream>
#include <console_bridge/console.h>
TESSERACT_COMMON_IGNORE_WARNINGS_POP
#include <tesseract/tesseract.h>

#include <tesseract_motion_planners/descartes/impl/descartes_tesseract_kinematics.hpp>

using namespace tesseract;
using namespace tesseract_scene_graph;
using namespace tesseract_motion_planners;

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

/** @brief Always returns false */
template <typename FloatType>
inline bool isNotValid(const FloatType* /*vertex*/)
{
  return false;
}

/** @brief Always returns true */
template <typename FloatType>
inline bool isCompletelyValid(const FloatType* /*vertex*/)
{
  return true;
}

/** @brief Returns an empty vector corresponding to no redundant solutions*/
template <typename FloatType>
inline std::vector<FloatType> noRedundantSolutions(const FloatType* /*sol*/, unsigned int& /*dof*/)
{
  std::vector<FloatType> redundant_sols;
  redundant_sols.clear();
  return redundant_sols;
}

/** @brief Returns the first solution as a duplicate redundant solution */
template <typename FloatType>
inline std::vector<FloatType> oneRedundantSolution(const FloatType* sol, unsigned int& dof)
{
  std::vector<FloatType> redundant_sols(sol, sol + dof);
  return redundant_sols;
}

/** @brief This is used to test that private members are set correctly */
template <typename FloatType>
class DescartesTesseractKinematicsTest : public tesseract_motion_planners::DescartesTesseractKinematics<FloatType>
{
public:
  using tesseract_motion_planners::DescartesTesseractKinematics<FloatType>::DescartesTesseractKinematics;

  Eigen::VectorXd getIKSeed() { return this->ik_seed_; }
};

class DescartesTesseractKinematicsUnit : public ::testing::Test
{
protected:
  Tesseract::Ptr tesseract_ptr_;
  tesseract_motion_planners::DescartesTesseractKinematics<double>::Ptr descartes_tesseract_kinematics_d_;
  tesseract_motion_planners::DescartesTesseractKinematics<float>::Ptr descartes_tesseract_kinematics_f_;

  tesseract_kinematics::ForwardKinematics::Ptr kdl_fk_;
  tesseract_kinematics::InverseKinematics::Ptr kdl_ik_;

  void SetUp() override
  {
    // Set up the Tesseract
    tesseract_scene_graph::ResourceLocator::Ptr locator =
        std::make_shared<tesseract_scene_graph::SimpleResourceLocator>(locateResource);
    Tesseract::Ptr tesseract = std::make_shared<Tesseract>();
    boost::filesystem::path urdf_path(std::string(TESSERACT_SUPPORT_DIR) + "/urdf/lbr_iiwa_14_r820.urdf");
    boost::filesystem::path srdf_path(std::string(TESSERACT_SUPPORT_DIR) + "/urdf/lbr_iiwa_14_r820.srdf");
    ASSERT_TRUE(tesseract->init(urdf_path, srdf_path, locator));
    tesseract_ptr_ = tesseract;

    // Set up the kinematics objects
    kdl_fk_ = tesseract_ptr_->getFwdKinematicsManagerConst()->getFwdKinematicSolver("manipulator");
    kdl_ik_ = tesseract_ptr_->getInvKinematicsManagerConst()->getInvKinematicSolver("manipulator");

    descartes_tesseract_kinematics_d_ =
        std::make_shared<tesseract_motion_planners::DescartesTesseractKinematics<double>>(kdl_fk_, kdl_ik_);
    descartes_tesseract_kinematics_f_ =
        std::make_shared<tesseract_motion_planners::DescartesTesseractKinematics<float>>(kdl_fk_, kdl_ik_);
  }
};

/** @brief Used to test IK by running the IK results through FK to check that they match up. This also checks that the
 * number of solutions matches the user defined expected number and that ik returns false when no solutions are found*/
static void testIKDouble(tesseract_motion_planners::DescartesTesseractKinematics<double> kin,
                         Eigen::Isometry3d pose_d,
                         const Eigen::VectorXd& seed_d,
                         int expected_sols)
{
  int dof = kin.dof();
  kin.setIKSeed(seed_d);
  std::vector<double> ik_solutions;
  // IK should fail if there are no solutions
  EXPECT_EQ(kin.ik(pose_d, ik_solutions), static_cast<bool>(expected_sols));

  int num_sols = static_cast<int>(ik_solutions.size()) / dof;
  EXPECT_EQ(num_sols, expected_sols);

  // Passing each solution through FK should yield the input to IK
  for (int i = 0; i < num_sols; i++)
  {
    Eigen::Isometry3d fk_result;
    EXPECT_TRUE(kin.fk(ik_solutions.data() + i * dof, fk_result));
    EXPECT_TRUE(pose_d.translation().isApprox(fk_result.translation(), 1e-4));

    Eigen::Quaterniond rot_pose(pose_d.rotation());
    Eigen::Quaterniond rot_result(fk_result.rotation());
    EXPECT_TRUE(rot_pose.isApprox(rot_result, 1e-3));
  }
}

/** @brief Used to test IK by running the IK results through FK to check that they match up. This also checks that the
 * number of solutions matches the user defined expected number and that ik returns false when no solutions are found*/
static void testIKFloat(tesseract_motion_planners::DescartesTesseractKinematics<float> kin,
                        Eigen::Isometry3f pose_f,
                        const Eigen::VectorXf& seed_f,
                        int expected_sols)
{
  int dof = kin.dof();
  kin.setIKSeed(seed_f);
  std::vector<float> ik_solutions;
  // IK should fail if there are no solutions
  EXPECT_EQ(kin.ik(pose_f, ik_solutions), static_cast<bool>(expected_sols));

  int num_sols = static_cast<int>(ik_solutions.size()) / dof;
  EXPECT_EQ(num_sols, expected_sols);

  // Passing each solution through FK should yield the input to IK
  for (int i = 0; i < num_sols; i++)
  {
    Eigen::Isometry3f fk_result;
    EXPECT_TRUE(kin.fk(ik_solutions.data() + i * dof, fk_result));
    EXPECT_TRUE(pose_f.translation().isApprox(fk_result.translation(), 1e-4f));

    Eigen::Quaternionf rot_pose(pose_f.rotation());
    Eigen::Quaternionf rot_result(fk_result.rotation());
    EXPECT_TRUE(rot_pose.isApprox(rot_result, 1e-3f));
  }
}

/** @brief Tests IK by calculating it for a pose and then testing each solution that is returned against the fk solution
 *
 * Note that these tests assume that the default IK solver only returns one solution
 */
TEST_F(DescartesTesseractKinematicsUnit, IKTest)  // NOLINT
{
  unsigned int dof = kdl_ik_->numJoints();
  Eigen::Isometry3d pose_d;
  pose_d.setIdentity();
  pose_d.translation()[0] = 0;
  pose_d.translation()[1] = 0;
  pose_d.translation()[2] = 1.306;
  Eigen::Isometry3f pose_f = pose_d.cast<float>();

  Eigen::VectorXd seed_d;
  seed_d.resize(7);
  seed_d(0) = -0.785398;
  seed_d(1) = 0.785398;
  seed_d(2) = -0.785398;
  seed_d(3) = 0.785398;
  seed_d(4) = -0.785398;
  seed_d(5) = 0.785398;
  seed_d(6) = -0.785398;
  Eigen::VectorXf seed_f = seed_d.cast<float>();

  {
    tesseract_motion_planners::DescartesTesseractKinematics<double> kin(kdl_fk_, kdl_ik_);
    CONSOLE_BRIDGE_logDebug("Double: default, default");
    testIKDouble(kin, pose_d, seed_d, 1);
  }
  {
    tesseract_motion_planners::DescartesTesseractKinematics<float> kin(kdl_fk_, kdl_ik_);
    CONSOLE_BRIDGE_logDebug("Float: default, default");
    testIKFloat(kin, pose_f, seed_f, 1);
  }
  {
    tesseract_motion_planners::DescartesTesseractKinematics<double> kin(kdl_fk_, kdl_ik_, nullptr, nullptr);
    CONSOLE_BRIDGE_logDebug("Double: nullptr, nullptr");
    testIKDouble(kin, pose_d, seed_d, 1);
  }
  {
    tesseract_motion_planners::DescartesTesseractKinematics<float> kin(kdl_fk_, kdl_ik_, nullptr, nullptr);
    CONSOLE_BRIDGE_logDebug("Float: nullptr, nullptr");
    testIKFloat(kin, pose_f, seed_f, 1);
  }
  {
    tesseract_motion_planners::DescartesTesseractKinematics<double> kin(kdl_fk_, kdl_ik_, &isNotValid<double>, nullptr);
    CONSOLE_BRIDGE_logDebug("Double: isNotValid, nullptr");
    testIKDouble(kin, pose_d, seed_d, 0);
  }
  {
    tesseract_motion_planners::DescartesTesseractKinematics<float> kin(kdl_fk_, kdl_ik_, &isNotValid<float>, nullptr);
    CONSOLE_BRIDGE_logDebug("Float: isNotValid, nullptr");
    testIKFloat(kin, pose_f, seed_f, 0);
  }
  {
    tesseract_motion_planners::DescartesTesseractKinematics<double> kin(
        kdl_fk_, kdl_ik_, &isCompletelyValid<double>, nullptr);
    CONSOLE_BRIDGE_logDebug("Double: isCompletelyValid, nullptr");
    testIKDouble(kin, pose_d, seed_d, 1);
  }
  {
    tesseract_motion_planners::DescartesTesseractKinematics<float> kin(
        kdl_fk_, kdl_ik_, &isCompletelyValid<float>, nullptr);
    CONSOLE_BRIDGE_logDebug("Float: isCompletelyValid, nullptr");
    testIKFloat(kin, pose_f, seed_f, 1);
  }
  {
    tesseract_motion_planners::DescartesTesseractKinematics<double> kin(
        kdl_fk_, kdl_ik_, nullptr, std::bind(&noRedundantSolutions<double>, std::placeholders::_1, dof));
    CONSOLE_BRIDGE_logDebug("Double: nullptr, noRedundantSolutions");
    testIKDouble(kin, pose_d, seed_d, 1);
  }
  {
    tesseract_motion_planners::DescartesTesseractKinematics<float> kin(
        kdl_fk_, kdl_ik_, nullptr, std::bind(&noRedundantSolutions<float>, std::placeholders::_1, dof));
    CONSOLE_BRIDGE_logDebug("Float: nullptr, noRedundantSolutions");
    testIKFloat(kin, pose_f, seed_f, 1);
  }
  {
    tesseract_motion_planners::DescartesTesseractKinematics<double> kin(
        kdl_fk_, kdl_ik_, nullptr, std::bind(&oneRedundantSolution<double>, std::placeholders::_1, dof));
    CONSOLE_BRIDGE_logDebug("Double: nullptr, oneRedundantSolution");
    testIKDouble(kin, pose_d, seed_d, 2);
  }
  {
    tesseract_motion_planners::DescartesTesseractKinematics<float> kin(
        kdl_fk_, kdl_ik_, nullptr, std::bind(&oneRedundantSolution<float>, std::placeholders::_1, dof));
    CONSOLE_BRIDGE_logDebug("Float: nullptr, oneRedundantSolution");
    testIKFloat(kin, pose_f, seed_f, 2);
  }
  {
    tesseract_motion_planners::DescartesTesseractKinematics<double> kin(
        kdl_fk_, kdl_ik_, &isNotValid<double>, std::bind(&noRedundantSolutions<double>, std::placeholders::_1, dof));
    CONSOLE_BRIDGE_logDebug("Double: isNotValid, noRedundantSolutions");
    testIKDouble(kin, pose_d, seed_d, 0);
  }
  {
    tesseract_motion_planners::DescartesTesseractKinematics<float> kin(
        kdl_fk_, kdl_ik_, &isNotValid<float>, std::bind(&noRedundantSolutions<float>, std::placeholders::_1, dof));
    CONSOLE_BRIDGE_logDebug("Float: isNotValid, noRedundantSolutions");
    testIKFloat(kin, pose_f, seed_f, 0);
  }
}

/** @brief This checks fk() against calling the tesseract kinematics object directly */
TEST_F(DescartesTesseractKinematicsUnit, FKTest)  // NOLINT
{
  Eigen::VectorXd joints_d(7);
  Eigen::VectorXf joints_f(7);
  joints_d << 0., 0.25, 0.5, 0.75, 1.0, 1.25, 1.5;
  joints_f << 0., 0.25, 0.5, 0.75, 1.0, 1.25, 1.5;
  Eigen::Isometry3d kdl_result_d = Eigen::Isometry3d::Identity();
  Eigen::Isometry3d result_d = Eigen::Isometry3d::Identity();
  Eigen::Isometry3f result_f = Eigen::Isometry3f::Identity();

  // Test the fk against the tesseract object directly
  kdl_fk_->calcFwdKin(kdl_result_d, joints_d);
  EXPECT_TRUE(descartes_tesseract_kinematics_d_->fk(joints_d.data(), result_d));
  EXPECT_TRUE(descartes_tesseract_kinematics_f_->fk(joints_f.data(), result_f));

  EXPECT_TRUE(result_d.isApprox(kdl_result_d, 0.00001));
  EXPECT_TRUE(result_f.isApprox(kdl_result_d.cast<float>(), 0.00001f));
}

/** @brief This checks that that dof() returns the correct value*/
TEST_F(DescartesTesseractKinematicsUnit, DOFTest)  // NOLINT
{
  // Sanity Check that urdf has 7 joints
  EXPECT_EQ(kdl_fk_->numJoints(), 7);
  // Actual Check
  EXPECT_EQ(descartes_tesseract_kinematics_d_->dof(), 7);
  EXPECT_EQ(descartes_tesseract_kinematics_f_->dof(), 7);
}

/** @brief Since analyzeIK() does not have any results, this just checkst that it doesn't crash */
TEST_F(DescartesTesseractKinematicsUnit, AnalyzeIKTest)  // NOLINT
{
  // Check that this doesn't crash
  descartes_tesseract_kinematics_d_->analyzeIK(Eigen::Isometry3d::Identity());
  descartes_tesseract_kinematics_f_->analyzeIK(Eigen::Isometry3f::Identity());
  EXPECT_TRUE(true);
}

/** @brief This tests that the ik seed is set correctly */
TEST_F(DescartesTesseractKinematicsUnit, SetIKSeedTest)  // NOLINT
{
  auto kin_d = DescartesTesseractKinematicsTest<double>(kdl_fk_, kdl_ik_);
  auto kin_f = DescartesTesseractKinematicsTest<float>(kdl_fk_, kdl_ik_);

  std::vector<double> double_vec(7, 1.234567);
  std::vector<float> float_vec(7, 1.234567f);
  Eigen::Map<Eigen::VectorXd> double_eigen(double_vec.data(), 7);
  Eigen::Map<Eigen::VectorXf> float_eigen(float_vec.data(), 7);

  // Check when using std::vector
  kin_d.setIKSeed(double_vec);
  EXPECT_TRUE(kin_d.getIKSeed().isApprox(double_eigen, 0.00001));
  kin_f.setIKSeed(float_vec);
  EXPECT_TRUE(kin_f.getIKSeed().isApprox(double_eigen, 0.00001));

  // Check when using Eigen::VectorX
  kin_d.setIKSeed(double_eigen);
  EXPECT_TRUE(kin_d.getIKSeed().isApprox(double_eigen, 0.00001));
  kin_f.setIKSeed(float_eigen);
  EXPECT_TRUE(kin_f.getIKSeed().isApprox(double_eigen, 0.00001));  // Note that the seed is stored as a VectorXd
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);

  return RUN_ALL_TESTS();
}
