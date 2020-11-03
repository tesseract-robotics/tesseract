
#include <tesseract_common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <taskflow/taskflow.hpp>
#include <fstream>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <raster_example_program.h>
#include <tesseract/tesseract.h>
#include <tesseract_command_language/command_language.h>
#include <tesseract_command_language/utils/utils.h>
#include <tesseract_process_managers/process_managers/raster_process_manager.h>
#include <tesseract_process_managers/taskflows/freespace_taskflow.h>
#include <tesseract_process_managers/taskflows/cartesian_taskflow.h>
#include <tesseract_visualization/visualization_loader.h>

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

int main()
{
  // --------------------
  // Perform setup
  // --------------------
  tesseract_scene_graph::ResourceLocator::Ptr locator =
      std::make_shared<tesseract_scene_graph::SimpleResourceLocator>(locateResource);
  auto tesseract = std::make_shared<tesseract::Tesseract>();
  boost::filesystem::path urdf_path(std::string(TESSERACT_SUPPORT_DIR) + "/urdf/abb_irb2400.urdf");
  boost::filesystem::path srdf_path(std::string(TESSERACT_SUPPORT_DIR) + "/urdf/abb_irb2400.srdf");
  tesseract->init(urdf_path, srdf_path, locator);

  // Dynamically load ignition visualizer if exist
  tesseract_visualization::VisualizationLoader loader;
  auto plotter = loader.get();

  if (plotter != nullptr)
  {
    plotter->init(tesseract);
    plotter->waitForConnection();
    plotter->plotEnvironment();
  }

  // --------------------
  // Define the program
  // --------------------
  CompositeInstruction program = rasterExampleProgram();
  const Instruction program_instruction{ program };
  Instruction seed = generateSkeletonSeed(program);

  // --------------------
  // Print Diagnostics
  // --------------------
  program_instruction.print("Program: ");
  seed.print("Seed:    ");

  // --------------------
  // Define the Process Input
  // --------------------
  ProcessInput input(tesseract, &program_instruction, program.getManipulatorInfo(), &seed);
  std::cout << "Input size: " << input.size() << std::endl;

  // --------------------
  // Initialize Freespace Manager
  // --------------------
  auto freespace_taskflow_generator = createFreespaceTaskflow(FreespaceTaskflowParams());
  auto transition_taskflow_generator = createFreespaceTaskflow(FreespaceTaskflowParams());
  auto raster_taskflow_generator = createCartesianTaskflow(CartesianTaskflowParams());
  RasterProcessManager raster_manager(std::move(freespace_taskflow_generator),
                                      std::move(transition_taskflow_generator),
                                      std::move(raster_taskflow_generator));
  if (!raster_manager.init(input))
    CONSOLE_BRIDGE_logError("Initialization Failed");

  // --------------------
  // Solve
  // --------------------
  if (!raster_manager.execute())
  {
    CONSOLE_BRIDGE_logError("Execution Failed");
  }

  // Plot OMPL Trajectory
  if (plotter)
  {
    plotter->waitForInput();
    plotter->plotTrajectory(*(input.getResults()));
  }

  std::cout << "Execution Complete" << std::endl;

  return 0;
}
