#ifndef TESSERACT_COLLISION_BENCHMARK_UTILS_HPP
#define TESSERACT_COLLISION_BENCHMARK_UTILS_HPP

#include <benchmark/benchmark.h>
#include <Eigen/Eigen>
#include <console_bridge/console.h>

using namespace tesseract_collision;
using namespace test_suite;
using namespace tesseract_geometry;

inline tesseract_geometry::Geometry::Ptr CreateUnitPrimative(const tesseract_geometry::GeometryType type,
                                                             const double scale = 1.0)
{
  tesseract_geometry::Geometry::Ptr geom;
  switch (type)
  {
    case tesseract_geometry::GeometryType::BOX:
      geom = std::make_shared<tesseract_geometry::Box>(scale, scale, scale);
      break;
    case tesseract_geometry::GeometryType::CONE:
      geom = std::make_shared<tesseract_geometry::Cone>(scale, scale);
      break;
    case tesseract_geometry::GeometryType::PLANE:
      geom = std::make_shared<tesseract_geometry::Plane>(scale, scale, scale, scale);
      break;
    case tesseract_geometry::GeometryType::SPHERE:
      geom = std::make_shared<tesseract_geometry::Sphere>(scale);
      break;
    case tesseract_geometry::GeometryType::CAPSULE:
      geom = std::make_shared<tesseract_geometry::Capsule>(scale, scale);
      break;
    case tesseract_geometry::GeometryType::CYLINDER:
      geom = std::make_shared<tesseract_geometry::Cylinder>(scale, scale);
      break;
    default:
      CONSOLE_BRIDGE_logError("Invalid Geometry Type. Can only create primatives");
      break;
  }
  return geom;
};
#endif
