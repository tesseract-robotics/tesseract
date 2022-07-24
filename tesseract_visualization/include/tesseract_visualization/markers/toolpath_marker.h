#ifndef TESSERACT_VISUALIZATION_MARKERS_TOOLPATH_MARKER_H
#define TESSERACT_VISUALIZATION_MARKERS_TOOLPATH_MARKER_H

#include <tesseract_common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <vector>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract_visualization/markers/marker.h>
#include <tesseract_common/types.h>

namespace tesseract_visualization
{
/** @brief An arrow defined by two points */
class ToolpathMarker : public Marker
{
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  ToolpathMarker() = default;
  ToolpathMarker(tesseract_common::Toolpath toolpath) : toolpath(std::move(toolpath)) {}

  int getType() const override { return static_cast<int>(MarkerType::TOOLPATH); }

  bool show_path{ true };
  bool show_axis{ true };
  tesseract_common::Toolpath toolpath;
  Eigen::Vector3d scale{ Eigen::Vector3d::Constant(0.03) };
};

}  // namespace tesseract_visualization
#endif  // TESSERACT_VISUALIZATION_MARKERS_TOOLPATH_MARKER_H
