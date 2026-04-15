/**
 * @file contact_managers_config_unit.cpp
 * @brief Tesseract collision contact managers config test
 *
 * @author Levi Armstrong
 * @date December 26, 2022
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
#include <tesseract/common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <gtest/gtest.h>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract/collision/test_suite/contact_manager_config_unit.hpp>
#include <tesseract/collision/bullet/bullet_discrete_simple_manager.h>
#include <tesseract/collision/bullet/bullet_discrete_bvh_manager.h>
#include <tesseract/collision/bullet/bullet_cast_simple_manager.h>
#include <tesseract/collision/bullet/bullet_cast_bvh_manager.h>
#include <tesseract/collision/fcl/fcl_discrete_managers.h>
#include <tesseract/collision/utils.h>

using namespace tesseract::collision;

class AlwaysTrueContactAllowedValidator : public tesseract::common::ContactAllowedValidator
{
public:
  bool operator()(const tesseract::common::LinkId&, const tesseract::common::LinkId&) const override { return true; }
};

class AlwaysFalseContactAllowedValidator : public tesseract::common::ContactAllowedValidator
{
public:
  bool operator()(const tesseract::common::LinkId&, const tesseract::common::LinkId&) const override { return false; }
};

class TestOrigContactAllowedValidator : public tesseract::common::ContactAllowedValidator
{
public:
  bool operator()(const tesseract::common::LinkId& id1, const tesseract::common::LinkId& id2) const override
  {
    auto pair = tesseract::common::LinkIdPair::make(id1, id2);

    if (pair == tesseract::common::LinkIdPair::make(tesseract::common::LinkId("link_1"),
                                                    tesseract::common::LinkId("link_2")))
      return true;

    if (pair == tesseract::common::LinkIdPair::make(tesseract::common::LinkId("link_1"),
                                                    tesseract::common::LinkId("link_3")))
      return true;

    return false;
  }
};

class TestOvrdContactAllowedValidator : public tesseract::common::ContactAllowedValidator
{
public:
  bool operator()(const tesseract::common::LinkId& id1, const tesseract::common::LinkId& id2) const override
  {
    return tesseract::common::LinkIdPair::make(id1, id2) ==
           tesseract::common::LinkIdPair::make(tesseract::common::LinkId("link_1"),
                                               tesseract::common::LinkId("link_2"));
  }
};

TEST(TesseractCollisionUnit, BulletDiscreteSimpleContactManagerConfigUnit)  // NOLINT
{
  BulletDiscreteSimpleManager checker;
  test_suite::runTest(checker);
}

TEST(TesseractCollisionUnit, BulletDiscreteBVHContactManagerConfigUnit)  // NOLINT
{
  BulletDiscreteBVHManager checker;
  test_suite::runTest(checker);
}

TEST(TesseractCollisionUnit, BulletCastSimpleContactManagerConfigUnit)  // NOLINT
{
  BulletCastSimpleManager checker;
  test_suite::runTest(checker);
}

TEST(TesseractCollisionUnit, BulletCastBVHContactManagerConfigUnit)  // NOLINT
{
  BulletCastBVHManager checker;
  test_suite::runTest(checker);
}

TEST(TesseractCollisionUnit, FCLDiscreteBVHContactManagerConfigUnit)  // NOLINT
{
  FCLDiscreteBVHManager checker;
  test_suite::runTest(checker);
}

TEST(TesseractCollisionUnit, CombineContactAllowedFnUnit)  // NOLINT
{
  using tesseract::common::LinkId;

  {  // tesseract::collision::ACMOverrideType::NONE
    auto orig = std::make_shared<AlwaysTrueContactAllowedValidator>();
    auto ovrd = std::make_shared<AlwaysFalseContactAllowedValidator>();

    auto comb = combineContactAllowedValidators(orig, ovrd, tesseract::collision::ACMOverrideType::NONE);
    EXPECT_TRUE((*comb)(LinkId(""), LinkId("")));
  }

  {  // tesseract::collision::ACMOverrideType::ASSIGN
    auto orig = std::make_shared<AlwaysTrueContactAllowedValidator>();
    auto ovrd = std::make_shared<AlwaysFalseContactAllowedValidator>();

    auto comb = combineContactAllowedValidators(orig, ovrd, tesseract::collision::ACMOverrideType::ASSIGN);
    EXPECT_FALSE((*comb)(LinkId(""), LinkId("")));
  }

  {  // tesseract::collision::ACMOverrideType::AND
    auto orig = std::make_shared<TestOrigContactAllowedValidator>();
    auto ovrd = std::make_shared<TestOvrdContactAllowedValidator>();

    auto comb = combineContactAllowedValidators(orig, ovrd, tesseract::collision::ACMOverrideType::AND);
    EXPECT_TRUE((*comb)(LinkId("link_1"), LinkId("link_2")));
    EXPECT_FALSE((*comb)(LinkId("link_1"), LinkId("link_3")));
    EXPECT_FALSE((*comb)(LinkId("abc"), LinkId("def")));

    auto comb1 = combineContactAllowedValidators(nullptr, ovrd, tesseract::collision::ACMOverrideType::AND);
    EXPECT_TRUE(comb1 == nullptr);
  }

  {  // tesseract::collision::ACMOverrideType::OR
    auto orig = std::make_shared<TestOrigContactAllowedValidator>();
    auto ovrd = std::make_shared<TestOvrdContactAllowedValidator>();

    auto comb = combineContactAllowedValidators(orig, ovrd, tesseract::collision::ACMOverrideType::OR);
    EXPECT_TRUE((*comb)(LinkId("link_1"), LinkId("link_2")));
    EXPECT_TRUE((*comb)(LinkId("link_1"), LinkId("link_3")));
    EXPECT_FALSE((*comb)(LinkId("abc"), LinkId("def")));

    auto comb1 = combineContactAllowedValidators(nullptr, ovrd, tesseract::collision::ACMOverrideType::OR);
    EXPECT_TRUE((*comb1)(LinkId("link_1"), LinkId("link_2")));
    EXPECT_FALSE((*comb1)(LinkId("link_1"), LinkId("link_3")));
    EXPECT_FALSE((*comb1)(LinkId("abc"), LinkId("def")));
  }
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);

  return RUN_ALL_TESTS();
}
