#include <tesseract_common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <gtest/gtest.h>
#include <algorithm>
#include <memory>
#include <Eigen/Geometry>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract_process_planners/process_segment_definition.h>

TEST(CustomIteratorUnit, ProcessSegementDefinitionIterator)  // NOLINT
{
  using namespace tesseract_process_planners;
  using namespace tesseract_motion_planners;

  ProcessSegmentDefinition psd;
  std::vector<CartesianWaypoint::Ptr> check;

  for (int i = 0; i < 5; ++i)
  {
    auto wp = std::make_shared<CartesianWaypoint>(Eigen::Isometry3d::Identity());
    psd.approach.push_back(wp);
    check.push_back(wp);
  }

  for (int i = 0; i < 5; ++i)
  {
    auto wp = std::make_shared<CartesianWaypoint>(Eigen::Isometry3d::Identity());
    psd.process.push_back(wp);
    check.push_back(wp);
  }

  for (int i = 0; i < 5; ++i)
  {
    auto wp = std::make_shared<CartesianWaypoint>(Eigen::Isometry3d::Identity());
    psd.departure.push_back(wp);
    check.push_back(wp);
  }

  auto f1 = std::find(psd.begin(), psd.end(), check[4]);
  auto f2 = std::find(check.begin(), check.end(), check[4]);
  EXPECT_TRUE(*f1 == *f2);

  long d = std::distance(psd.begin(), f1);
  EXPECT_TRUE(d == 4);

  EXPECT_TRUE(psd.size() == 15);
  EXPECT_TRUE(psd.isApproach(4));
  EXPECT_FALSE(psd.isApproach(5));
  EXPECT_FALSE(psd.isApproach(10));
  EXPECT_FALSE(psd.isProcess(0));
  EXPECT_TRUE(psd.isProcess(5));
  EXPECT_FALSE(psd.isProcess(10));
  EXPECT_FALSE(psd.isDeparture(0));
  EXPECT_FALSE(psd.isDeparture(5));
  EXPECT_TRUE(psd.isDeparture(10));

  for (size_t i = 0; i < check.size(); ++i)
  {
    EXPECT_TRUE(check[i] == psd[i]);
    EXPECT_TRUE(check.at(i) == psd.at(i));
  }

  int cnt = 0;
  for (auto& a : psd)
  {
    ++cnt;
    UNUSED(a);
  }

  EXPECT_TRUE(cnt == 15);

  cnt = 0;
  for (const auto& a : psd)
  {
    ++cnt;
    UNUSED(a);
  }

  EXPECT_TRUE(cnt == 15);

  psd.erase(psd.begin());
  check.erase(check.begin());
  EXPECT_TRUE(psd.size() == 14);
  EXPECT_TRUE(psd.approach.size() == 4);
  EXPECT_TRUE(psd.process.size() == 5);
  EXPECT_TRUE(psd.departure.size() == 5);

  for (size_t i = 0; i < check.size(); ++i)
  {
    EXPECT_TRUE(check[i] == psd[i]);
    EXPECT_TRUE(check.at(i) == psd.at(i));
  }

  psd.erase(psd.end() - 1);
  check.erase(check.end() - 1);
  EXPECT_TRUE(psd.size() == 13);
  EXPECT_TRUE(psd.approach.size() == 4);
  EXPECT_TRUE(psd.process.size() == 5);
  EXPECT_TRUE(psd.departure.size() == 4);

  for (size_t i = 0; i < check.size(); ++i)
  {
    EXPECT_TRUE(check[i] == psd[i]);
    EXPECT_TRUE(check.at(i) == psd.at(i));
  }

  psd.erase(psd.begin(), psd.begin() + 3);
  check.erase(check.begin(), check.begin() + 3);
  EXPECT_TRUE(psd.size() == 10);
  EXPECT_TRUE(psd.approach.size() == 1);
  EXPECT_TRUE(psd.process.size() == 5);
  EXPECT_TRUE(psd.departure.size() == 4);

  for (size_t i = 0; i < check.size(); ++i)
  {
    EXPECT_TRUE(check[i] == psd[i]);
    EXPECT_TRUE(check.at(i) == psd.at(i));
  }

  psd.erase(psd.end() - 3, psd.end());
  check.erase(check.end() - 3, check.end());
  EXPECT_TRUE(psd.size() == 7);
  EXPECT_TRUE(psd.approach.size() == 1);
  EXPECT_TRUE(psd.process.size() == 5);
  EXPECT_TRUE(psd.departure.size() == 1);

  for (size_t i = 0; i < check.size(); ++i)
  {
    EXPECT_TRUE(check[i] == psd[i]);
    EXPECT_TRUE(check.at(i) == psd.at(i));
  }

  EXPECT_ANY_THROW(check.at(check.size()));
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);

  return RUN_ALL_TESTS();
}
