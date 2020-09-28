#include <tesseract_common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <gtest/gtest.h>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract/tesseract.h>

#include <tesseract_motion_planners/core/types.h>
#include <tesseract_motion_planners/simple/simple_motion_planner.h>
#include <tesseract_motion_planners/core/utils.h>

#include <tesseract_process_managers/process_input.h>
#include <tesseract_process_managers/process_managers/raster_process_manager.h>
#include <tesseract_process_managers/process_managers/raster_dt_process_manager.h>
#include <tesseract_process_managers/process_managers/raster_waad_process_manager.h>
#include <tesseract_process_managers/process_managers/raster_waad_dt_process_manager.h>
#include <tesseract_process_managers/taskflows/cartesian_taskflow.h>
#include <tesseract_process_managers/taskflows/freespace_taskflow.h>

#include "raster_example_program.h"
#include "raster_dt_example_program.h"
#include "raster_waad_example_program.h"
#include "raster_waad_dt_example_program.h"
#include "freespace_example_program.h"

using namespace tesseract;
using namespace tesseract_kinematics;
using namespace tesseract_environment;
using namespace tesseract_scene_graph;
using namespace tesseract_planning;

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
  EXPECT_FALSE(program.getManipulatorInfo().empty());

  program.setManipulatorInfo(manip);
  EXPECT_TRUE(program.hasStartInstruction());
  EXPECT_FALSE(program.getManipulatorInfo().empty());

  auto interpolator = std::make_shared<SimpleMotionPlanner>("INTERPOLATOR");

  // Create Planning Request
  PlannerRequest request;
  request.instructions = program;
  request.tesseract = tesseract_ptr_;
  request.env_state = tesseract_ptr_->getEnvironment()->getCurrentState();

  PlannerResponse response;
  auto status = interpolator->solve(request, response);
  EXPECT_TRUE(status);

  auto pcnt = getPlanInstructionCount(request.instructions);
  auto mcnt = getMoveInstructionCount(response.results);

  // The first plan instruction is the start instruction and every other plan instruction should be converted into
  // ten move instruction.
  EXPECT_EQ(((pcnt - 1) * 10) + 1, mcnt);
  EXPECT_TRUE(response.results.hasStartInstruction());
  EXPECT_FALSE(response.results.getManipulatorInfo().empty());
}

TEST_F(TesseractProcessManagerUnit, FreespaceSimpleMotionPlannerTest)
{
  CompositeInstruction program = freespaceExampleProgram();
  EXPECT_FALSE(program.getManipulatorInfo().empty());

  program.setManipulatorInfo(manip);
  EXPECT_TRUE(program.hasStartInstruction());
  EXPECT_FALSE(program.getManipulatorInfo().empty());

  auto interpolator = std::make_shared<SimpleMotionPlanner>("INTERPOLATOR");

  // Create Planning Request
  PlannerRequest request;
  request.instructions = program;
  request.tesseract = tesseract_ptr_;
  request.env_state = tesseract_ptr_->getEnvironment()->getCurrentState();

  PlannerResponse response;
  auto status = interpolator->solve(request, response);
  EXPECT_TRUE(status);

  auto pcnt = getPlanInstructionCount(request.instructions);
  auto mcnt = getMoveInstructionCount(response.results);

  // The first plan instruction is the start instruction and every other plan instruction should be converted into
  // ten move instruction.
  EXPECT_EQ(((pcnt - 1) * 10) + 1, mcnt);
  EXPECT_TRUE(response.results.hasStartInstruction());
  EXPECT_FALSE(response.results.getManipulatorInfo().empty());
}

TEST_F(TesseractProcessManagerUnit, RasterProcessManagerTest)
{
  // Define the program
  CompositeInstruction program = rasterExampleProgram();
  const Instruction program_instruction{ program };
  Instruction seed = generateSkeletonSeed(program);

  // Define the Process Input
  ProcessInput input(tesseract_ptr_, &program_instruction, program.getManipulatorInfo(), &seed);

  // Initialize Freespace Manager
  auto freespace_taskflow_generator = createFreespaceTaskflow(FreespaceTaskflowParams());
  auto transition_taskflow_generator = createFreespaceTaskflow(FreespaceTaskflowParams());
  auto raster_taskflow_generator = createCartesianTaskflow(true);
  RasterProcessManager raster_manager(std::move(freespace_taskflow_generator),
                                      std::move(transition_taskflow_generator),
                                      std::move(raster_taskflow_generator),
                                      1);
  EXPECT_TRUE(raster_manager.init(input));

  // Solve
  EXPECT_TRUE(raster_manager.execute());
}

TEST_F(TesseractProcessManagerUnit, RasterDTProcessManagerTest)
{
  // Define the program
  CompositeInstruction program = rasterDTExampleProgram();
  const Instruction program_instruction{ program };
  Instruction seed = generateSkeletonSeed(program);

  // Define the Process Input
  ProcessInput input(tesseract_ptr_, &program_instruction, program.getManipulatorInfo(), &seed);

  // Initialize Freespace Manager
  auto freespace_taskflow_generator = createFreespaceTaskflow(FreespaceTaskflowParams());
  auto transition_taskflow_generator = createFreespaceTaskflow(FreespaceTaskflowParams());
  auto raster_taskflow_generator = createCartesianTaskflow(true);
  RasterDTProcessManager raster_manager(std::move(freespace_taskflow_generator),
                                        std::move(transition_taskflow_generator),
                                        std::move(raster_taskflow_generator),
                                        1);
  EXPECT_TRUE(raster_manager.init(input));

  // Solve
  EXPECT_TRUE(raster_manager.execute());
}

TEST_F(TesseractProcessManagerUnit, RasterWAADProcessManagerTest)
{
  // Define the program
  CompositeInstruction program = rasterWAADExampleProgram();
  const Instruction program_instruction{ program };
  Instruction seed = generateSkeletonSeed(program);

  // Define the Process Input
  ProcessInput input(tesseract_ptr_, &program_instruction, program.getManipulatorInfo(), &seed);

  // Initialize Freespace Manager
  auto freespace_taskflow_generator = createFreespaceTaskflow(FreespaceTaskflowParams());
  auto transition_taskflow_generator = createFreespaceTaskflow(FreespaceTaskflowParams());
  auto raster_taskflow_generator = createCartesianTaskflow(true);
  RasterWAADProcessManager raster_manager(std::move(freespace_taskflow_generator),
                                          std::move(transition_taskflow_generator),
                                          std::move(raster_taskflow_generator),
                                          1);
  EXPECT_TRUE(raster_manager.init(input));

  // Solve
  EXPECT_TRUE(raster_manager.execute());
}

TEST_F(TesseractProcessManagerUnit, RasterWAADDTProcessManagerTest)
{
  // Define the program
  CompositeInstruction program = rasterWAADDTExampleProgram();
  const Instruction program_instruction{ program };
  Instruction seed = generateSkeletonSeed(program);

  // Define the Process Input
  ProcessInput input(tesseract_ptr_, &program_instruction, program.getManipulatorInfo(), &seed);

  // Initialize Freespace Manager
  auto freespace_taskflow_generator = createFreespaceTaskflow(FreespaceTaskflowParams());
  auto transition_taskflow_generator = createFreespaceTaskflow(FreespaceTaskflowParams());
  auto raster_taskflow_generator = createCartesianTaskflow(true);
  RasterWAADDTProcessManager raster_manager(std::move(freespace_taskflow_generator),
                                            std::move(transition_taskflow_generator),
                                            std::move(raster_taskflow_generator),
                                            1);
  EXPECT_TRUE(raster_manager.init(input));

  // Solve
  EXPECT_TRUE(raster_manager.execute());
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);

  return RUN_ALL_TESTS();
}
