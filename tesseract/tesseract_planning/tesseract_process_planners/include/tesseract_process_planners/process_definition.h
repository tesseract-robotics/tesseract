#ifndef TESSERACT_PLANNING_PROCESS_DEFINITION_H
#define TESSERACT_PLANNING_PROCESS_DEFINITION_H

#include <Eigen/Core>
#include <tesseract_motion_planners/core/waypoint.h>
#include <tesseract_process_planners/process_segment_definition.h>

namespace tesseract_process_planners
{
/**
 * @brief The ProcessTransitionDefinition struct which contains the waypoint data to allow moving between adjacent
 * process segments
 */
struct ProcessTransitionDefinition
{
  std::vector<tesseract_motion_planners::Waypoint::Ptr> transition_from_start; /**< A transition plans from the start of
                                                                                  segment[i] to the end of segment[i+1],
                                                                                       this data can be used for finding
                                                                                  collision free exit moves after
                                                                                  cancelling an ongoing process */

  std::vector<tesseract_motion_planners::Waypoint::Ptr> transition_from_end; /**< A transition plans from the end of
                                                                                segment[i] to the start of segment[i+1]
                                                                              */
};

/**
 * @class tesseract_process_planners::ProcessDefinition
 * @details A process definition.
 *
 * This is not intended to support or handle all processes but should covers most of the common ones
 * like sanding, painting, grinding, etc.
 *
 * In a process, we assume they follow this pattern.
 *    * The robot starts from a start position,
 *    * Moves to an approach position just above the object
 *    * Executes a process(sanding, painting, etc.)
 *    * Return to the start position
 *
 * Given the process described the user is only required to define two objects. The start position of
 * the robot and a vector of Process Segment Definitions.
 */
struct ProcessDefinition
{
  tesseract_motion_planners::Waypoint::Ptr start; /**< The start position of the robot */
  std::vector<ProcessSegmentDefinition> segments; /**< All of the raster segments with approaches and departures */
  std::vector<ProcessTransitionDefinition> transitions; /**< All of the transition to/from a given segment. Must be same
                                                           length as segments */
};

/**definition
 * @class tesseract_process_planners::ProcessTransitionGenerator
 */
class ProcessTransitionGenerator
{
public:
  using Ptr = std::shared_ptr<ProcessTransitionGenerator>;
  using ConstPtr = std::shared_ptr<const ProcessTransitionGenerator>;

  ProcessTransitionGenerator() = default;
  virtual ~ProcessTransitionGenerator() = default;
  ProcessTransitionGenerator(const ProcessTransitionGenerator&) = default;
  ProcessTransitionGenerator& operator=(const ProcessTransitionGenerator&) = default;
  ProcessTransitionGenerator(ProcessTransitionGenerator&&) = default;
  ProcessTransitionGenerator& operator=(ProcessTransitionGenerator&&) = default;

  virtual std::vector<tesseract_motion_planners::Waypoint::Ptr>
  generate(const tesseract_motion_planners::Waypoint::Ptr& start_waypoint,
           const tesseract_motion_planners::Waypoint::Ptr& end_waypoint) const = 0;
};

/**
 * @class tesseract_process_planners::ProcessDefinitionConfig
 * @details The Process Definition Config
 *
 * This provides the high level process configuration information. It requires the user to provide
 * the start position waypoint (JointWaypoint) and a set of tool paths (strokes) that should be
 * converted into a process results definition leveraging both this configuration information and
 * the ProcessSegmentDefinitions.
 *
 * Also, other operations that are nice to have is the ability to offset the process. Particularly useful
 * when wanting to verify the process without making contact with a surface in the case of sanding.
 */
struct ProcessDefinitionConfig
{
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  tesseract_motion_planners::Waypoint::Ptr start;
  std::vector<std::vector<tesseract_motion_planners::Waypoint::Ptr>> tool_paths;

  std::vector<ProcessTransitionGenerator::ConstPtr> transition_generator;

  Eigen::Isometry3d local_offset_direction;
  Eigen::Isometry3d world_offset_direction;

  ProcessDefinitionConfig()
  {
    local_offset_direction.setIdentity();
    world_offset_direction.setIdentity();
  }
};

/**
 * @class tesseract_process_planners::ProcessStepGenerator
 * @details
 * This is a base class for process step generators. A process step generator takes tool path segments and
 * converts them into process tool path. Currently we assume that each tool path segment has three
 * steps Approach, Process, and Departure. Example case is using raster tool paths on surfaces but these need
 * to be modified based on the process that will be using those tool paths. Custom implementations of this class
 *  could for instance not want to stay normal to the surface but apply an angle of attack instead.
 * Therefore the specialized implementation would take the tool path segments and generate a new segments with
 * modified poses that accommodate the angle of attack. This class aims to bridge the gap between surface rastering
 * libraries and the planners so neither of these library need to know anything about the particular process.
 */
class ProcessStepGenerator
{
public:
  using Ptr = std::shared_ptr<ProcessStepGenerator>;
  using ConstPtr = std::shared_ptr<const ProcessStepGenerator>;

  ProcessStepGenerator() = default;
  virtual ~ProcessStepGenerator() = default;
  ProcessStepGenerator(const ProcessStepGenerator&) = default;
  ProcessStepGenerator& operator=(const ProcessStepGenerator&) = default;
  ProcessStepGenerator(ProcessStepGenerator&&) = default;
  ProcessStepGenerator& operator=(ProcessStepGenerator&&) = default;

  virtual std::vector<tesseract_motion_planners::Waypoint::Ptr>
  generate(const std::vector<tesseract_motion_planners::Waypoint::Ptr>& waypoints,
           const ProcessDefinitionConfig& config) const = 0;
};

/**
 * @brief The Process Segment Definition Configuration
 *
 * In most manufacturing process like (sanding, painting, etc.) it includes a series of process paths (strokes).
 * Each of the process paths may have an approach, departure and transition step.
 *
 * Example: In the case of painting, when approaching the part you may want to take a specific trajectory
 * and the same for when leaving the part.
 */
struct ProcessSegmentDefinitionConfig
{
  ProcessStepGenerator::ConstPtr approach;
  ProcessStepGenerator::ConstPtr process;
  ProcessStepGenerator::ConstPtr departure;
};

ProcessDefinition generateProcessDefinition(const ProcessDefinitionConfig& process_config,
                                            const ProcessSegmentDefinitionConfig& segment_config);

ProcessDefinition generateProcessDefinition(const ProcessDefinitionConfig& process_config,
                                            const std::vector<ProcessSegmentDefinitionConfig>& segment_config);

}  // namespace tesseract_process_planners

#endif  // TESSERACT_PLANNING_PROCESS_DEFINITION_H
