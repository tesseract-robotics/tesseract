#include <tesseract_geometry/cereal_serialization.h>

// These must be include before calling macro CEREAL_REGISTER_TYPE
#include <cereal/archives/binary.hpp>
#include <cereal/archives/xml.hpp>
#include <cereal/archives/json.hpp>

CEREAL_REGISTER_TYPE(tesseract_geometry::Box)
CEREAL_REGISTER_TYPE(tesseract_geometry::Capsule)
CEREAL_REGISTER_TYPE(tesseract_geometry::Cone)
CEREAL_REGISTER_TYPE(tesseract_geometry::Cylinder)
CEREAL_REGISTER_TYPE(tesseract_geometry::Octree)
CEREAL_REGISTER_TYPE(tesseract_geometry::Plane)
CEREAL_REGISTER_TYPE(tesseract_geometry::Sphere)
CEREAL_REGISTER_TYPE(tesseract_geometry::CompoundMesh)
CEREAL_REGISTER_TYPE(tesseract_geometry::PolygonMesh)
CEREAL_REGISTER_TYPE(tesseract_geometry::Mesh)
CEREAL_REGISTER_TYPE(tesseract_geometry::ConvexMesh)
CEREAL_REGISTER_TYPE(tesseract_geometry::SDFMesh)

CEREAL_REGISTER_POLYMORPHIC_RELATION(tesseract_geometry::Geometry, tesseract_geometry::Box)
CEREAL_REGISTER_POLYMORPHIC_RELATION(tesseract_geometry::Geometry, tesseract_geometry::Capsule)
CEREAL_REGISTER_POLYMORPHIC_RELATION(tesseract_geometry::Geometry, tesseract_geometry::Cone)
CEREAL_REGISTER_POLYMORPHIC_RELATION(tesseract_geometry::Geometry, tesseract_geometry::Cylinder)
CEREAL_REGISTER_POLYMORPHIC_RELATION(tesseract_geometry::Geometry, tesseract_geometry::Octree)
CEREAL_REGISTER_POLYMORPHIC_RELATION(tesseract_geometry::Geometry, tesseract_geometry::Plane)
CEREAL_REGISTER_POLYMORPHIC_RELATION(tesseract_geometry::Geometry, tesseract_geometry::Sphere)
CEREAL_REGISTER_POLYMORPHIC_RELATION(tesseract_geometry::Geometry, tesseract_geometry::CompoundMesh)
CEREAL_REGISTER_POLYMORPHIC_RELATION(tesseract_geometry::Geometry, tesseract_geometry::PolygonMesh)
CEREAL_REGISTER_POLYMORPHIC_RELATION(tesseract_geometry::Geometry, tesseract_geometry::Mesh)
CEREAL_REGISTER_POLYMORPHIC_RELATION(tesseract_geometry::Geometry, tesseract_geometry::ConvexMesh)
CEREAL_REGISTER_POLYMORPHIC_RELATION(tesseract_geometry::Geometry, tesseract_geometry::SDFMesh)
CEREAL_REGISTER_POLYMORPHIC_RELATION(tesseract_geometry::PolygonMesh, tesseract_geometry::Mesh)
CEREAL_REGISTER_POLYMORPHIC_RELATION(tesseract_geometry::PolygonMesh, tesseract_geometry::ConvexMesh)
CEREAL_REGISTER_POLYMORPHIC_RELATION(tesseract_geometry::PolygonMesh, tesseract_geometry::SDFMesh)

// LCOV_EXCL_START
CEREAL_REGISTER_DYNAMIC_INIT(tesseract_geometry_cereal)
// LCOV_EXCL_STOP
