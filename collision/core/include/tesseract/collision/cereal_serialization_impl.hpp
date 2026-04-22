#ifndef TESSERACT_COLLISION_CEREAL_SERIALIZATION_IMPL_HPP
#define TESSERACT_COLLISION_CEREAL_SERIALIZATION_IMPL_HPP

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

#endif  // TESSERACT_COLLISION_CEREAL_SERIALIZATION_IMPL_HPP
