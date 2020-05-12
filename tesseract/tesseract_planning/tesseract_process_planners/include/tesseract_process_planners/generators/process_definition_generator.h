#ifndef TESSERACT_PROCESS_PLANNERS_PROCESS_DEFINITION_GENERATOR_H
#define TESSERACT_PROCESS_PLANNERS_PROCESS_DEFINITION_GENERATOR_H

#include <tesseract_process_planners/process_definition.h>
#include <tesseract_motion_planners/core/waypoint.h>

#include <tesseract_common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <Eigen/Geometry>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

namespace tesseract_process_planners
{
class ProcessDefinitionGenerator
{
public:
  using ToolPath = std::vector<tesseract_motion_planners::Waypoint::Ptr>;
  using ToolPaths = std::vector<ToolPath>;

  virtual ~ProcessDefinitionGenerator() = default;

  /**
   * @brief generate Generates a process definition from a toolpath
   * @param start The intial robot position
   * @param end  The final robot position
   * @param tool_paths The vector of toolpath rasters to be processed
   * @return A process definition encompassing the segments and transitions of the process
   */
  virtual ProcessDefinition generate(tesseract_motion_planners::Waypoint::ConstPtr start,
                                     tesseract_motion_planners::Waypoint::ConstPtr end,
                                     const ToolPaths& tool_paths) const = 0;
  /**
   * @brief generate Generates a process definition from a toolpath
   * @param tool_paths The vector of toolpath rasters to be processed
   * @return A process definition encompassing the segments and transitions of the process
   */
  virtual ProcessDefinition generate(const ToolPaths& tool_paths) const = 0;
};

}  // namespace tesseract_process_planners

#endif  // TESSERACT_PROCESS_PLANNERS_PROCESS_DEFINITION_GENERATOR_H
