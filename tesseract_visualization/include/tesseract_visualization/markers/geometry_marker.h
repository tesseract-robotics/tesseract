#ifndef TESSERACT_VISUALIZATION_MARKERS_GEOMETRY_MARKER_H
#define TESSERACT_VISUALIZATION_MARKERS_GEOMETRY_MARKER_H

#include <tesseract_visualization/markers/marker.h>
#include <tesseract_geometry/geometry.h>

namespace tesseract_visualization
{
/** @brief An geometry marker */
class GeometryMarker : public Marker
{
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  GeometryMarker() = default;
  GeometryMarker(tesseract_geometry::Geometry::ConstPtr geom, Eigen::Isometry3d origin = Eigen::Isometry3d::Identity())
    : geom(std::move(geom)), origin(std::move(origin))
  {
  }

  int getType() const override { return static_cast<int>(MarkerType::GEOMETRY); }

  /** @brief The geometry object */
  tesseract_geometry::Geometry::ConstPtr geom;

  /** @brief The origin definition */
  Eigen::Isometry3d origin{ Eigen::Isometry3d::Identity() };
};

}  // namespace tesseract_visualization
#endif  // TESSERACT_VISUALIZATION_MARKERS_GEOMETRY_MARKER_H
