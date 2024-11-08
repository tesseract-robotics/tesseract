/**
 * @file contact_managers_config_unit.cpp
 * @brief Tesseract collision contact managers config test
 *
 * @author Levi Armstrong
 * @date December 26, 2022
 * @version TODO
 * @bug No known bugs
 *
 * @copyright Copyright (c) 2022, Southwest Research Institute
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
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract_collision/test_suite/contact_manager_config_unit.hpp>
#include <tesseract_collision/bullet/bullet_discrete_simple_manager.h>
#include <tesseract_collision/bullet/bullet_discrete_bvh_manager.h>
#include <tesseract_collision/bullet/bullet_cast_simple_manager.h>
#include <tesseract_collision/bullet/bullet_cast_bvh_manager.h>
#include <tesseract_collision/fcl/fcl_discrete_managers.h>
#include <tesseract_collision/hpp_fcl/hpp_fcl_discrete_managers.h>
#include <tesseract_collision/core/utils.h>

using namespace tesseract_collision;

TEST(TesseractCollisionUnit, BulletDiscreteSimpleContactManagerConfigUnit)  // NOLINT
{
  tesseract_collision_bullet::BulletDiscreteSimpleManager checker;
  test_suite::runTest(checker);
}

TEST(TesseractCollisionUnit, BulletDiscreteBVHContactManagerConfigUnit)  // NOLINT
{
  tesseract_collision_bullet::BulletDiscreteBVHManager checker;
  test_suite::runTest(checker);
}

TEST(TesseractCollisionUnit, BulletCastSimpleContactManagerConfigUnit)  // NOLINT
{
  tesseract_collision_bullet::BulletCastSimpleManager checker;
  test_suite::runTest(checker);
}

TEST(TesseractCollisionUnit, BulletCastBVHContactManagerConfigUnit)  // NOLINT
{
  tesseract_collision_bullet::BulletCastBVHManager checker;
  test_suite::runTest(checker);
}

TEST(TesseractCollisionUnit, FCLDiscreteBVHContactManagerConfigUnit)  // NOLINT
{
  tesseract_collision_fcl::FCLDiscreteBVHManager checker;
  test_suite::runTest(checker);
}

TEST(TesseractCollisionUnit, HPP_FCLDiscreteBVHContactManagerConfigUnit)  // NOLINT
{
  tesseract_collision_hpp_fcl::HPP_FCLDiscreteBVHManager checker;
  test_suite::runTest(checker);
}

TEST(TesseractCollisionUnit, CombineContactAllowedFnUnit)  // NOLINT
{
  {  // tesseract_collision::ACMOverrideType::NONE
    tesseract_collision::IsContactAllowedFn orig = [](const std::string& /*link_name1*/,
                                                      const std::string& /*link_name2*/) { return true; };

    tesseract_collision::IsContactAllowedFn ovrd = [](const std::string& /*link_name1*/,
                                                      const std::string& /*link_name2*/) { return false; };

    tesseract_collision::IsContactAllowedFn comb =
        tesseract_collision::combineContactAllowedFn(orig, ovrd, tesseract_collision::ACMOverrideType::NONE);
    EXPECT_TRUE(comb("", ""));
  }

  {  // tesseract_collision::ACMOverrideType::ASSIGN
    tesseract_collision::IsContactAllowedFn orig = [](const std::string& /*link_name1*/,
                                                      const std::string& /*link_name2*/) { return true; };

    tesseract_collision::IsContactAllowedFn ovrd = [](const std::string& /*link_name1*/,
                                                      const std::string& /*link_name2*/) { return false; };

    tesseract_collision::IsContactAllowedFn comb =
        tesseract_collision::combineContactAllowedFn(orig, ovrd, tesseract_collision::ACMOverrideType::ASSIGN);
    EXPECT_FALSE(comb("", ""));
  }

  {  // tesseract_collision::ACMOverrideType::AND
    auto orig = [](const std::string& link_name1, const std::string& link_name2) {
      if (link_name1 == "link_1" && link_name2 == "link_2")
        return true;

      if (link_name1 == "link_1" && link_name2 == "link_3")
        return true;

      return false;
    };

    auto ovrd = [](const std::string& link_name1, const std::string& link_name2) {
      return (link_name1 == "link_1" && link_name2 == "link_2");
    };

    auto comb = tesseract_collision::combineContactAllowedFn(orig, ovrd, tesseract_collision::ACMOverrideType::AND);
    EXPECT_TRUE(comb("link_1", "link_2"));
    EXPECT_FALSE(comb("link_1", "link_3"));
    EXPECT_FALSE(comb("abc", "def"));

    auto comb1 = tesseract_collision::combineContactAllowedFn(nullptr, ovrd, tesseract_collision::ACMOverrideType::AND);
    EXPECT_FALSE(comb1("link_1", "link_2"));
    EXPECT_FALSE(comb1("link_1", "link_3"));
    EXPECT_FALSE(comb1("abc", "def"));
  }

  {  // tesseract_collision::ACMOverrideType::AND
    auto orig = [](const std::string& link_name1, const std::string& link_name2) {
      if (link_name1 == "link_1" && link_name2 == "link_2")
        return true;

      if (link_name1 == "link_1" && link_name2 == "link_3")
        return true;

      return false;
    };

    auto ovrd = [](const std::string& link_name1, const std::string& link_name2) {
      return (link_name1 == "link_1" && link_name2 == "link_2");
    };

    auto comb = tesseract_collision::combineContactAllowedFn(orig, ovrd, tesseract_collision::ACMOverrideType::OR);
    EXPECT_TRUE(comb("link_1", "link_2"));
    EXPECT_TRUE(comb("link_1", "link_3"));
    EXPECT_FALSE(comb("abc", "def"));

    auto comb1 = tesseract_collision::combineContactAllowedFn(nullptr, ovrd, tesseract_collision::ACMOverrideType::OR);
    EXPECT_TRUE(comb1("link_1", "link_2"));
    EXPECT_FALSE(comb1("link_1", "link_3"));
    EXPECT_FALSE(comb1("abc", "def"));
  }
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);

  return RUN_ALL_TESTS();
}
