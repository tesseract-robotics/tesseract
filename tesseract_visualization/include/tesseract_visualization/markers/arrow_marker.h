#ifndef TESSERACT_VISUALIZATION_MARKERS_ARROW_MARKER_H
#define TESSERACT_VISUALIZATION_MARKERS_ARROW_MARKER_H

#include <tesseract_visualization/markers/marker.h>
#include <tesseract_scene_graph/link.h>

namespace tesseract_visualization
{
/**
 * @brief An arrow marker
 * @details The arrow will be created along the z-axis of the provided pose with the base of the shaft at the origin
 * point in the positive direction
 */
class ArrowMarker : public Marker
{
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  ArrowMarker() = default;
  /**
   * @brief Define an arrow marker using two points
   * @details The class parameters are calculated based on distance
   * @param pt1 The starting point
   * @param pt2 The final point
   */
  ArrowMarker(const Eigen::Vector3d& pt1, const Eigen::Vector3d& pt2) : shaft_radius(0.005), head_radius(0.01)
  {
    Eigen::Vector3d x, y, z;
    z = (pt2 - pt1).normalized();
    y = z.unitOrthogonal();
    x = (y.cross(z)).normalized();
    Eigen::Matrix3d rot;
    rot.col(0) = x;
    rot.col(1) = y;
    rot.col(2) = z;
    pose.linear() = rot;
    pose.translation() = pt1 + (((pt2 - pt1).norm() / 2) * z);

    double length = (pt2 - pt1).norm();
    shaft_length = length - 0.01;
    head_length = length - shaft_length;
  }

  int getType() const override { return static_cast<int>(MarkerType::ARROW); }

  /** @brief The arrow shaft length */
  double shaft_length{ 0.8 };

  /** @brief The arrow shaft radius */
  double shaft_radius{ 0.1 };

  /** @brief The arrow head length */
  double head_length{ 0.2 };

  /** @brief The arrow head radius */
  double head_radius{ 0.15 };

  /** @brief The arrow pose */
  Eigen::Isometry3d pose{ Eigen::Isometry3d::Identity() };

  /** @brief The material information for the marker */
  tesseract_scene_graph::Material::Ptr material;
};

}  // namespace tesseract_visualization
#endif  // TESSERACT_VISUALIZATION_MARKERS_ARROW_MARKER_H
