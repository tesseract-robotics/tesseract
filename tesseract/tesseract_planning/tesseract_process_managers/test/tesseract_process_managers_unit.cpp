#include <tesseract_common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <gtest/gtest.h>
#include <opw_kinematics/opw_parameters.h>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract/tesseract.h>
#include <tesseract_kinematics/opw/opw_inv_kin.h>
#include <tesseract_motion_planners/core/types.h>
#include <tesseract_motion_planners/simple/simple_motion_planner.h>
#include <tesseract_command_language/command_language_utils.h>
#include <tesseract_process_managers/examples/raster_example_program.h>
#include <tesseract_process_managers/examples/freespace_example_program.h>

using namespace tesseract;
using namespace tesseract_kinematics;
using namespace tesseract_environment;
using namespace tesseract_scene_graph;
using namespace tesseract_planning;
using namespace opw_kinematics;

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

    mod_url = package_path + mod_url;
  }

  return mod_url;
}

class TesseractProcessManagerUnit : public ::testing::Test
{
protected:
  Tesseract::Ptr tesseract_ptr_;
  opw_kinematics::Parameters<double> opw_params_;
  ManipulatorInfo manip;

  void SetUp() override
  {
    tesseract_scene_graph::ResourceLocator::Ptr locator =
        std::make_shared<tesseract_scene_graph::SimpleResourceLocator>(locateResource);
    Tesseract::Ptr tesseract = std::make_shared<Tesseract>();
    boost::filesystem::path urdf_path(std::string(TESSERACT_SUPPORT_DIR) + "/urdf/abb_irb2400.urdf");
    boost::filesystem::path srdf_path(std::string(TESSERACT_SUPPORT_DIR) + "/urdf/abb_irb2400.srdf");
    EXPECT_TRUE(tesseract->init(urdf_path, srdf_path, locator));
    tesseract_ptr_ = tesseract;

    manip.manipulator = "manipulator";
    manip.manipulator_ik_solver = "OPWInvKin";
    manip.working_frame = "base_link";
  }
};

TEST_F(TesseractProcessManagerUnit, RasterSimpleMotionPlannerTest)
{
  tesseract_planning::CompositeInstruction program = rasterExampleProgram();
  EXPECT_FALSE(program.getManipulatorInfo().isEmpty());

  program.setManipulatorInfo(manip);
  EXPECT_TRUE(program.hasStartInstruction());
  EXPECT_FALSE(program.getManipulatorInfo().isEmpty());

  auto interpolator = std::make_shared<SimpleMotionPlanner>("INTERPOLATOR");

  // Create Planning Request
  PlannerRequest request;
  request.instructions = program;
  request.tesseract = tesseract_ptr_;
  request.env_state = tesseract_ptr_->getEnvironment()->getCurrentState();

  PlannerResponse response;
  auto status = interpolator->solve(request, response);
  EXPECT_TRUE(status);

  auto pcnt = getPlanInstructionsCount(request.instructions);
  auto mcnt = getMoveInstructionsCount(response.results);

  // The first plan instruction is the start instruction and every other plan instruction should be converted into
  // ten move instruction.
  EXPECT_EQ(((pcnt - 1) * 10) + 1, mcnt);
  EXPECT_TRUE(response.results.hasStartInstruction());
  EXPECT_FALSE(response.results.getManipulatorInfo().isEmpty());
}

TEST_F(TesseractProcessManagerUnit, FreespaceSimpleMotionPlannerTest)
{
  tesseract_planning::CompositeInstruction program = freespaceExampleProgram();
  EXPECT_FALSE(program.getManipulatorInfo().isEmpty());

  program.setManipulatorInfo(manip);
  EXPECT_TRUE(program.hasStartInstruction());
  EXPECT_FALSE(program.getManipulatorInfo().isEmpty());

  auto interpolator = std::make_shared<SimpleMotionPlanner>("INTERPOLATOR");

  // Create Planning Request
  PlannerRequest request;
  request.instructions = program;
  request.tesseract = tesseract_ptr_;
  request.env_state = tesseract_ptr_->getEnvironment()->getCurrentState();

  PlannerResponse response;
  auto status = interpolator->solve(request, response);
  EXPECT_TRUE(status);

  auto pcnt = getPlanInstructionsCount(request.instructions);
  auto mcnt = getMoveInstructionsCount(response.results);

  // The first plan instruction is the start instruction and every other plan instruction should be converted into
  // ten move instruction.
  EXPECT_EQ(((pcnt - 1) * 10) + 1, mcnt);
  EXPECT_TRUE(response.results.hasStartInstruction());
  EXPECT_FALSE(response.results.getManipulatorInfo().isEmpty());
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);

  return RUN_ALL_TESTS();
}
