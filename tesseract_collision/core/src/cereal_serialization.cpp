#include <tesseract_collision/core/cereal_serialization.h>

// These must be include before calling macro CEREAL_REGISTER_TYPE
#include <cereal/archives/binary.hpp>
#include <cereal/archives/xml.hpp>
#include <cereal/archives/json.hpp>

CEREAL_REGISTER_TYPE(tesseract::collision::ContactResultAnyPoly)
CEREAL_REGISTER_TYPE(tesseract::collision::ContactResultMapAnyPoly)
CEREAL_REGISTER_TYPE(tesseract::collision::ContactResultMapVectorAnyPoly)

CEREAL_REGISTER_POLYMORPHIC_RELATION(tesseract::common::AnyInterface, tesseract::collision::ContactResultAnyPoly)
CEREAL_REGISTER_POLYMORPHIC_RELATION(tesseract::common::AnyInterface, tesseract::collision::ContactResultMapAnyPoly)
CEREAL_REGISTER_POLYMORPHIC_RELATION(tesseract::common::AnyInterface,
                                     tesseract::collision::ContactResultMapVectorAnyPoly)

// LCOV_EXCL_START
CEREAL_REGISTER_DYNAMIC_INIT(tesseract_collision_cereal)
// LCOV_EXCL_STOP
