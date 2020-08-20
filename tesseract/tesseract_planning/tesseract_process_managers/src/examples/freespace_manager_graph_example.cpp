
#include <tesseract_common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <taskflow/taskflow.hpp>
#include <fstream>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract/tesseract.h>
#include <tesseract_command_language/command_language.h>
#include <tesseract_command_language/command_language_utils.h>
#include <tesseract_process_managers/taskflow_generators/graph_taskflow.h>
#include <tesseract_process_managers/examples/freespace_example_program.h>
#include <tesseract_process_managers/process_managers/freespace_process_manager.h>
#include <tesseract_visualization/visualization_loader.h>

#include <tesseract_process_managers/process_generators/motion_planner_process_generator.h>
#include <tesseract_process_managers/process_generators/continuous_contact_check_process_generator.h>
#include <tesseract_process_managers/process_generators/iterative_spline_parameterization_process_generator.h>

#include <tesseract_motion_planners/simple/simple_motion_planner.h>
#include <tesseract_motion_planners/simple/profile/simple_planner_default_plan_profile.h>

#include <tesseract_motion_planners/trajopt/trajopt_motion_planner.h>
#include <tesseract_motion_planners/trajopt/problem_generators/default_problem_generator.h>
#include <tesseract_motion_planners/trajopt/profile/trajopt_default_composite_profile.h>
#include <tesseract_motion_planners/trajopt/profile/trajopt_default_plan_profile.h>

#include <tesseract_motion_planners/ompl/ompl_motion_planner.h>
#include <tesseract_motion_planners/ompl/problem_generators/default_problem_generator.h>
#include <tesseract_motion_planners/ompl/profile/ompl_default_plan_profile.h>

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

GraphTaskflow::UPtr createGraphTaskflow()
{
  auto graph = std::make_unique<GraphTaskflow>();

  ///////////////////
  /// Add Process ///
  ///////////////////

  // Setup Interpolator
  auto interpolator = std::make_shared<SimpleMotionPlanner>("INTERPOLATOR");
  interpolator->plan_profiles["FREESPACE"] = std::make_shared<SimplePlannerDefaultPlanProfile>();
  auto interpolator_generator = std::make_unique<MotionPlannerProcessGenerator>(interpolator);
  int interpolator_idx = graph->addNode(std::move(interpolator_generator), GraphTaskflow::NodeType::CONDITIONAL);

  // Setup OMPL
  auto ompl_planner = std::make_shared<OMPLMotionPlanner>();
  ompl_planner->problem_generator = &DefaultOMPLProblemGenerator;
  ompl_planner->plan_profiles["FREESPACE"] = std::make_shared<OMPLDefaultPlanProfile>();
  auto ompl_generator = std::make_unique<MotionPlannerProcessGenerator>(ompl_planner);
  int ompl_idx = graph->addNode(std::move(ompl_generator), GraphTaskflow::NodeType::CONDITIONAL);

  // Setup TrajOpt
  auto trajopt_planner = std::make_shared<TrajOptMotionPlanner>();
  trajopt_planner->problem_generator = &DefaultTrajoptProblemGenerator;
  trajopt_planner->plan_profiles["FREESPACE"] = std::make_shared<TrajOptDefaultPlanProfile>();
  trajopt_planner->composite_profiles["FREESPACE"] = std::make_shared<TrajOptDefaultCompositeProfile>();
  auto trajopt_generator = std::make_unique<MotionPlannerProcessGenerator>(trajopt_planner);
  int trajopt_idx = graph->addNode(std::move(trajopt_generator), GraphTaskflow::NodeType::CONDITIONAL);

  // Add Final Continuous Contact Check of trajectory
  auto contact_check_generator = std::make_unique<ContinuousContactCheckProcessGenerator>();
  int contact_check_idx = graph->addNode(std::move(contact_check_generator), GraphTaskflow::NodeType::CONDITIONAL);

  // Time parameterization trajectory
  auto time_parameterization_generator = std::make_unique<IterativeSplineParameterizationProcessGenerator>();
  int time_parameterization_idx =
      graph->addNode(std::move(time_parameterization_generator), GraphTaskflow::NodeType::CONDITIONAL);

  /////////////////
  /// Add Edges ///
  /////////////////
  auto ON_SUCCESS = GraphTaskflow::SourceChannel::ON_SUCCESS;
  auto ON_FAILURE = GraphTaskflow::SourceChannel::ON_FAILURE;
  auto PROCESS_NODE = GraphTaskflow::DestinationChannel::PROCESS_NODE;
  auto ERROR_CALLBACK = GraphTaskflow::DestinationChannel::ERROR_CALLBACK;
  auto DONE_CALLBACK = GraphTaskflow::DestinationChannel::DONE_CALLBACK;

  graph->addEdge(interpolator_idx, ON_SUCCESS, ompl_idx, PROCESS_NODE);
  graph->addEdge(interpolator_idx, ON_FAILURE, -1, ERROR_CALLBACK);

  graph->addEdge(ompl_idx, ON_SUCCESS, trajopt_idx, PROCESS_NODE);
  graph->addEdge(ompl_idx, ON_FAILURE, -1, ERROR_CALLBACK);

  graph->addEdge(trajopt_idx, ON_SUCCESS, contact_check_idx, PROCESS_NODE);
  graph->addEdge(trajopt_idx, ON_FAILURE, -1, ERROR_CALLBACK);

  graph->addEdge(contact_check_idx, ON_SUCCESS, time_parameterization_idx, PROCESS_NODE);
  graph->addEdge(contact_check_idx, ON_FAILURE, -1, ERROR_CALLBACK);

  graph->addEdge(time_parameterization_idx, ON_SUCCESS, -1, DONE_CALLBACK);
  graph->addEdge(time_parameterization_idx, ON_FAILURE, -1, ERROR_CALLBACK);

  return graph;
}

int main()
{
  // --------------------
  // Perform setup
  // --------------------
  tesseract_scene_graph::ResourceLocator::Ptr locator =
      std::make_shared<tesseract_scene_graph::SimpleResourceLocator>(locateResource);
  tesseract::Tesseract::Ptr tesseract = std::make_shared<tesseract::Tesseract>();
  boost::filesystem::path urdf_path(std::string(TESSERACT_SUPPORT_DIR) + "/urdf/lbr_iiwa_14_r820.urdf");
  boost::filesystem::path srdf_path(std::string(TESSERACT_SUPPORT_DIR) + "/urdf/lbr_iiwa_14_r820.srdf");
  tesseract->init(urdf_path, srdf_path, locator);

  // Dynamically load ignition visualizer if exist
  tesseract_visualization::VisualizationLoader loader;
  auto plotter = loader.get();

  if (plotter != nullptr)
    plotter->init(tesseract);

  // --------------------
  // Define the program
  // --------------------
  CompositeInstruction program = freespaceExampleProgram();
  const Instruction program_instruction{ program };
  Instruction seed = generateSkeletonSeed(program);

  // --------------------
  // Print Diagnostics
  // --------------------
  program_instruction.print("Program: ");

  // --------------------
  // Define the Process Input
  // --------------------
  ProcessInput input(tesseract, &program_instruction, program.getManipulatorInfo(), &seed);
  std::cout << "Input size: " << input.size() << std::endl;

  // --------------------
  // Initialize Freespace Manager
  // --------------------
  FreespaceProcessManager freespace_manager(createGraphTaskflow());
  freespace_manager.init(input);

  // --------------------
  // Solve
  // --------------------
  if (!freespace_manager.execute())
  {
    CONSOLE_BRIDGE_logError("Execution Failed");
  }

  // Plot Trajectory
  if (plotter)
  {
    plotter->waitForInput();
    plotter->plotTrajectory(*(input.results));
  }

  std::cout << "Execution Complete" << std::endl;

  return 0;
}
