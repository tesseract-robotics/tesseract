/**
 * @file get_collision_objects_transform_unit.cpp
 * @brief Verifies getCollisionObjectsTransform(LinkId) round-trips the setter for every backend.
 *
 * @copyright Copyright (c) 2026, Southwest Research Institute
 *
 * @par License
 * Software License Agreement (Apache License)
 * @par
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 * http://www.apache.org/licenses/LICENSE-2.0
 */
#include <tesseract/common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <gtest/gtest.h>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract/collision/test_suite/get_collision_objects_transform_unit.hpp>
#include <tesseract/collision/bullet/bullet_discrete_simple_manager.h>
#include <tesseract/collision/bullet/bullet_discrete_bvh_manager.h>
#include <tesseract/collision/bullet/bullet_cast_simple_manager.h>
#include <tesseract/collision/bullet/bullet_cast_bvh_manager.h>
#include <tesseract/collision/fcl/fcl_discrete_managers.h>

using namespace tesseract::collision;

TEST(TesseractCollisionUnit, BulletDiscreteSimpleGetCollisionObjectsTransform)  // NOLINT
{
  BulletDiscreteSimpleManager checker;
  test_suite::runDiscreteGetCollisionObjectsTransformUnit(checker);
}

TEST(TesseractCollisionUnit, BulletDiscreteBVHGetCollisionObjectsTransform)  // NOLINT
{
  BulletDiscreteBVHManager checker;
  test_suite::runDiscreteGetCollisionObjectsTransformUnit(checker);
}

TEST(TesseractCollisionUnit, FCLDiscreteBVHGetCollisionObjectsTransform)  // NOLINT
{
  FCLDiscreteBVHManager checker;
  test_suite::runDiscreteGetCollisionObjectsTransformUnit(checker);
}

TEST(TesseractCollisionUnit, BulletCastSimpleGetCollisionObjectsTransform)  // NOLINT
{
  BulletCastSimpleManager checker;
  test_suite::runContinuousGetCollisionObjectsTransformUnit(checker);
}

TEST(TesseractCollisionUnit, BulletCastBVHGetCollisionObjectsTransform)  // NOLINT
{
  BulletCastBVHManager checker;
  test_suite::runContinuousGetCollisionObjectsTransformUnit(checker);
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
