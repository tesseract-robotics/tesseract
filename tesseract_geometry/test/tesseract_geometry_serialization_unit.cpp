/**
 * @file tesseract_geometry_serialization_unit.cpp
 * @brief Tests serialization of geometry
 *
 * @author Levi Armstrong
 * @author Matthew Powelson
 * @date March 16, 2022
 * @version TODO
 * @bug No known bugs
 *
 * @par License
 * Software License Agreement (Apache License)
 * @par
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 * http://www.apache.org/licenses/LICENSE-2.0
 * @par
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */
#include <tesseract_common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <gtest/gtest.h>
#include <boost/serialization/shared_ptr.hpp>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract_common/serialization.h>
#include <tesseract_common/unit_test_utils.h>
#include <tesseract_common/utils.h>
#include <tesseract_geometry/geometries.h>

using namespace tesseract_geometry;

TEST(TesseractGeometrySerializeUnit, Box)  // NOLINT
{
  auto object = std::make_shared<Box>(1, 2, 3);
  tesseract_common::testSerialization<Box>(*object, "Box");
  tesseract_common::testSerializationDerivedClass<Geometry, Box>(object, "Box");
}

TEST(TesseractGeometrySerializeUnit, Capsule)  // NOLINT
{
  auto object = std::make_shared<Capsule>(1, 2);
  tesseract_common::testSerialization<Capsule>(*object, "Capsule");
  tesseract_common::testSerializationDerivedClass<Geometry, Capsule>(object, "Capsule");
}

TEST(TesseractGeometrySerializeUnit, Cone)  // NOLINT
{
  auto object = std::make_shared<Cone>(1.1, 2.2);
  tesseract_common::testSerialization<Cone>(*object, "Cone");
  tesseract_common::testSerializationDerivedClass<Geometry, Cone>(object, "Cone");
}

TEST(TesseractGeometrySerializeUnit, Cylinder)  // NOLINT
{
  auto object = std::make_shared<Cylinder>(3.3, 4.4);
  tesseract_common::testSerialization<Cylinder>(*object, "Cylinder");
  tesseract_common::testSerializationDerivedClass<Geometry, Cylinder>(object, "Cylinder");
}

TEST(TesseractGeometrySerializeUnit, Plane)  // NOLINT
{
  auto object = std::make_shared<Plane>(1.1, 2, 3.3, 4);
  tesseract_common::testSerialization<Plane>(*object, "Plane");
  tesseract_common::testSerializationDerivedClass<Geometry, Plane>(object, "Plane");
}

TEST(TesseractGeometrySerializeUnit, Sphere)  // NOLINT
{
  auto object = std::make_shared<Sphere>(3.3);
  tesseract_common::testSerialization<Sphere>(*object, "Sphere");
  tesseract_common::testSerializationDerivedClass<Geometry, Sphere>(object, "Sphere");
}


int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);

  return RUN_ALL_TESTS();
}
