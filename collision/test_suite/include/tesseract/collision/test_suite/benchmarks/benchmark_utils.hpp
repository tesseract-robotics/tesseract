#ifndef TESSERACT_COLLISION_BENCHMARK_UTILS_HPP
#define TESSERACT_COLLISION_BENCHMARK_UTILS_HPP

#include <benchmark/benchmark.h>
#include <Eigen/Eigen>
#include <console_bridge/console.h>

using namespace tesseract::collision;
using namespace test_suite;
using namespace tesseract::geometry;

inline tesseract::geometry::Geometry::Ptr CreateUnitPrimative(const tesseract::geometry::GeometryType type,
                                                              const double scale = 1.0)
{
  tesseract::geometry::Geometry::Ptr geom;
  switch (type)
  {
    case tesseract::geometry::GeometryType::BOX:
      geom = std::make_shared<tesseract::geometry::Box>(scale, scale, scale);
      break;
    case tesseract::geometry::GeometryType::CONE:
      geom = std::make_shared<tesseract::geometry::Cone>(scale, scale);
      break;
    case tesseract::geometry::GeometryType::PLANE:
      geom = std::make_shared<tesseract::geometry::Plane>(scale, scale, scale, scale);
      break;
    case tesseract::geometry::GeometryType::SPHERE:
      geom = std::make_shared<tesseract::geometry::Sphere>(scale);
      break;
    case tesseract::geometry::GeometryType::CAPSULE:
      geom = std::make_shared<tesseract::geometry::Capsule>(scale, scale);
      break;
    case tesseract::geometry::GeometryType::CYLINDER:
      geom = std::make_shared<tesseract::geometry::Cylinder>(scale, scale);
      break;
    default:
      CONSOLE_BRIDGE_logError("Invalid Geometry Type. Can only create primatives");
      break;
  }
  return geom;
}
#endif
