#include <tesseract/geometry/cereal_serialization.h>

// These must be include before calling macro CEREAL_REGISTER_TYPE
#include <cereal/archives/binary.hpp>
#include <cereal/archives/xml.hpp>
#include <cereal/archives/json.hpp>

CEREAL_REGISTER_TYPE(tesseract::geometry::Box)
CEREAL_REGISTER_TYPE(tesseract::geometry::Capsule)
CEREAL_REGISTER_TYPE(tesseract::geometry::Cone)
CEREAL_REGISTER_TYPE(tesseract::geometry::Cylinder)
CEREAL_REGISTER_TYPE(tesseract::geometry::Octree)
CEREAL_REGISTER_TYPE(tesseract::geometry::Plane)
CEREAL_REGISTER_TYPE(tesseract::geometry::Sphere)
CEREAL_REGISTER_TYPE(tesseract::geometry::CompoundMesh)
CEREAL_REGISTER_TYPE(tesseract::geometry::PolygonMesh)
CEREAL_REGISTER_TYPE(tesseract::geometry::Mesh)
CEREAL_REGISTER_TYPE(tesseract::geometry::ConvexMesh)
CEREAL_REGISTER_TYPE(tesseract::geometry::SDFMesh)

CEREAL_REGISTER_POLYMORPHIC_RELATION(tesseract::geometry::Geometry, tesseract::geometry::Box)
CEREAL_REGISTER_POLYMORPHIC_RELATION(tesseract::geometry::Geometry, tesseract::geometry::Capsule)
CEREAL_REGISTER_POLYMORPHIC_RELATION(tesseract::geometry::Geometry, tesseract::geometry::Cone)
CEREAL_REGISTER_POLYMORPHIC_RELATION(tesseract::geometry::Geometry, tesseract::geometry::Cylinder)
CEREAL_REGISTER_POLYMORPHIC_RELATION(tesseract::geometry::Geometry, tesseract::geometry::Octree)
CEREAL_REGISTER_POLYMORPHIC_RELATION(tesseract::geometry::Geometry, tesseract::geometry::Plane)
CEREAL_REGISTER_POLYMORPHIC_RELATION(tesseract::geometry::Geometry, tesseract::geometry::Sphere)
CEREAL_REGISTER_POLYMORPHIC_RELATION(tesseract::geometry::Geometry, tesseract::geometry::CompoundMesh)
CEREAL_REGISTER_POLYMORPHIC_RELATION(tesseract::geometry::Geometry, tesseract::geometry::PolygonMesh)
CEREAL_REGISTER_POLYMORPHIC_RELATION(tesseract::geometry::Geometry, tesseract::geometry::Mesh)
CEREAL_REGISTER_POLYMORPHIC_RELATION(tesseract::geometry::Geometry, tesseract::geometry::ConvexMesh)
CEREAL_REGISTER_POLYMORPHIC_RELATION(tesseract::geometry::Geometry, tesseract::geometry::SDFMesh)
CEREAL_REGISTER_POLYMORPHIC_RELATION(tesseract::geometry::PolygonMesh, tesseract::geometry::Mesh)
CEREAL_REGISTER_POLYMORPHIC_RELATION(tesseract::geometry::PolygonMesh, tesseract::geometry::ConvexMesh)
CEREAL_REGISTER_POLYMORPHIC_RELATION(tesseract::geometry::PolygonMesh, tesseract::geometry::SDFMesh)

// LCOV_EXCL_START
CEREAL_REGISTER_DYNAMIC_INIT(tesseract_geometry_cereal)
// LCOV_EXCL_STOP
