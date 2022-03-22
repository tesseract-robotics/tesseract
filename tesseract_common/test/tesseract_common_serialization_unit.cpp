/**
 * @file tesseract_common_serialization_unit.cpp
 * @brief Tests serialization of types in tesseract_common
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
#include <tesseract_common/allowed_collision_matrix.h>
#include <tesseract_common/collision_margin_data.h>

using namespace tesseract_common;

TEST(TesseractCommonSerializeUnit, AllowedCollisionMatrix)  // NOLINT
{
  auto object = std::make_shared<AllowedCollisionMatrix>();
  tesseract_common::testSerialization<AllowedCollisionMatrix>(*object, "EmptyAllowedCollisionMatrix");
  object->addAllowedCollision("link_1", "link2", "reason1");
  object->addAllowedCollision("link_2", "link1", "reason2");
  object->addAllowedCollision("link_4", "link3", "reason3");
  object->addAllowedCollision("link_5", "link2", "reason4");
  tesseract_common::testSerialization<AllowedCollisionMatrix>(*object, "AllowedCollisionMatrix");
}

TEST(TesseractCommonSerializeUnit, CollisionMarginData)  // NOLINT
{
  auto object = std::make_shared<CollisionMarginData>();
  tesseract_common::testSerialization<CollisionMarginData>(*object, "EmptyCollisionMarginData");
  object->setPairCollisionMargin("link_1", "link2", 1.1);
  object->setPairCollisionMargin("link_2", "link1", 2.2);
  object->setPairCollisionMargin("link_4", "link3", 3.3);
  object->setPairCollisionMargin("link_5", "link2", -4.4);
  tesseract_common::testSerialization<CollisionMarginData>(*object, "CollisionMarginData");
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);

  return RUN_ALL_TESTS();
}
