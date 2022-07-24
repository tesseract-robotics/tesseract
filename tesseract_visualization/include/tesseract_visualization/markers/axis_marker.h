#ifndef TESSERACT_VISUALIZATION_MARKERS_AXIS_MARKER_H
#define TESSERACT_VISUALIZATION_MARKERS_AXIS_MARKER_H

#include <tesseract_visualization/markers/marker.h>

namespace tesseract_visualization
{
/** @brief An axis */
class AxisMarker : public Marker
{
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  AxisMarker() = default;
  AxisMarker(const Eigen::Isometry3d& axis) : axis(axis) {}

  int getType() const override { return static_cast<int>(MarkerType::AXIS); }

  /** @brief The axis definition */
  Eigen::Isometry3d axis{ Eigen::Isometry3d::Identity() };
};

}  // namespace tesseract_visualization

#endif  // TESSERACT_VISUALIZATION_MARKERS_AXIS_MARKER_H
