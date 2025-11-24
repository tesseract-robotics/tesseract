#include <tesseract_collision/core/cereal_serialization.h>

// These must be include before calling macro CEREAL_REGISTER_TYPE
#include <cereal/archives/binary.hpp>
#include <cereal/archives/xml.hpp>
#include <cereal/archives/json.hpp>

CEREAL_REGISTER_TYPE(tesseract_collision::ContactResultAnyPoly)
CEREAL_REGISTER_TYPE(tesseract_collision::ContactResultMapAnyPoly)
CEREAL_REGISTER_TYPE(tesseract_collision::ContactResultMapVectorAnyPoly)

CEREAL_REGISTER_POLYMORPHIC_RELATION(tesseract_common::AnyInterface, tesseract_collision::ContactResultAnyPoly)
CEREAL_REGISTER_POLYMORPHIC_RELATION(tesseract_common::AnyInterface, tesseract_collision::ContactResultMapAnyPoly)
CEREAL_REGISTER_POLYMORPHIC_RELATION(tesseract_common::AnyInterface, tesseract_collision::ContactResultMapVectorAnyPoly)

CEREAL_REGISTER_DYNAMIC_INIT(tesseract_collision_cereal)
