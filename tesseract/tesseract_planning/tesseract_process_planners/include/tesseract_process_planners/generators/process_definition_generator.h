#ifndef PROCESS_DEFINITION_GENERATOR_H
#define PROCESS_DEFINITION_GENERATOR_H

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

  ProcessDefinitionGenerator() = default;
  virtual ~ProcessDefinitionGenerator() = default;

  virtual ProcessDefinition generate(tesseract_motion_planners::Waypoint::ConstPtr start,
                                     tesseract_motion_planners::Waypoint::ConstPtr end,
                                     const ToolPaths& tool_paths) const = 0;

  virtual ProcessDefinition generate(const ToolPaths& tool_paths) const = 0;
};

}  // namespace tesseract_process_planners

#endif  // PROCESS_DEFINITION_GENERATOR_H
