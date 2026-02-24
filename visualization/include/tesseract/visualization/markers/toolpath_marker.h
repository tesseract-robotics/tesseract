#ifndef TESSERACT_VISUALIZATION_MARKERS_TOOLPATH_MARKER_H
#define TESSERACT_VISUALIZATION_MARKERS_TOOLPATH_MARKER_H

#include <tesseract/visualization/markers/marker.h>
#include <tesseract/common/eigen_types.h>

namespace tesseract::visualization
{
/** @brief An arrow defined by two points */
class ToolpathMarker : public Marker
{
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  ToolpathMarker() = default;
  ToolpathMarker(tesseract::common::Toolpath toolpath) : toolpath(std::move(toolpath)) {}

  int getType() const override { return static_cast<int>(MarkerType::TOOLPATH); }

  bool show_path{ true };
  bool show_axis{ true };
  tesseract::common::Toolpath toolpath;
  Eigen::Vector3d scale{ Eigen::Vector3d::Constant(0.03) };
};

}  // namespace tesseract::visualization
#endif  // TESSERACT_VISUALIZATION_MARKERS_TOOLPATH_MARKER_H
