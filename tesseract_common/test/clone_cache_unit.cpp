#include <tesseract_common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <gtest/gtest.h>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract_common/clone_cache.h>

using namespace tesseract_common;
/**
 * @brief Object used to test the CloneCache
 */
class TestObject
{
public:
  using Ptr = std::shared_ptr<TestObject>;
  using ConstPtr = std::shared_ptr<const TestObject>;

  TestObject() = default;
  virtual ~TestObject() = default;
  TestObject(const TestObject&) = delete;
  TestObject& operator=(const TestObject&) = delete;
  TestObject(TestObject&&) = delete;
  TestObject& operator=(TestObject&&) = delete;

  TestObject::Ptr clone() const
  {
    auto clone = std::make_shared<TestObject>();
    clone->val_1 = val_1;
    clone->val_2 = val_2;
    clone->revision_ = revision_;
    return clone;
  }

  int getRevision() const { return revision_; }

  int val_1{ 0 };
  int val_2{ 0 };

  int revision_{ 0 };
};

class TestObjectSupportsUpdate : public TestObject
{
public:
  using Ptr = std::shared_ptr<TestObjectSupportsUpdate>;
  using ConstPtr = std::shared_ptr<const TestObjectSupportsUpdate>;

  bool update(const TestObjectSupportsUpdate::ConstPtr& pattern)
  {
    val_1 = pattern->val_1;
    val_2 = pattern->val_2;
    revision_ = pattern->revision_;
    return true;
  }

  TestObjectSupportsUpdate::Ptr clone() const
  {
    TestObjectSupportsUpdate::Ptr clone = std::make_shared<TestObjectSupportsUpdate>();
    clone->val_1 = val_1;
    clone->val_2 = val_2;
    clone->revision_ = revision_;
    return clone;
  }
};

class TestObjectSupportsUpdateFailure : public TestObject
{
public:
  using Ptr = std::shared_ptr<TestObjectSupportsUpdateFailure>;
  using ConstPtr = std::shared_ptr<const TestObjectSupportsUpdateFailure>;

  bool update(const TestObjectSupportsUpdateFailure::ConstPtr& pattern)
  {
    val_1 = pattern->val_1;
    val_2 = pattern->val_2;
    revision_ = pattern->revision_;
    return true;
  }

  TestObjectSupportsUpdateFailure::Ptr clone() const
  {
    throw std::runtime_error("TestObjectSupportsUpdateFailure: clone failed!");
  }
};

TEST(TesseractCloneCacheUnit, WithoutUpdate)  // NOLINT
{
  auto original = std::make_shared<TestObject>();
  original->val_1 = 1;
  original->val_2 = 2;
  auto clone_cache = std::make_shared<CloneCache<TestObject>>(original, 3);
  EXPECT_EQ(clone_cache->getCacheSize(), 3);
  EXPECT_EQ(clone_cache->getCurrentCacheSize(), 3);

  // Baseline clone of original
  {
    auto clone = clone_cache->clone();
    EXPECT_EQ(original->val_1, clone->val_1);
    EXPECT_EQ(original->val_2, clone->val_2);
  }
  // Change value without changing revision would be expected to fail
  {
    auto clone = clone_cache->clone();

    original->val_1 = 3;
    EXPECT_NE(original->val_1, clone->val_1);
  }
  // Now it should update the cache and be the same
  {
    auto clone = clone_cache->clone();

    original->revision_++;
    EXPECT_NE(original->val_1, clone->val_1);
  }
  // Cache should be empty so call update
  {
    EXPECT_EQ(clone_cache->getCurrentCacheSize(), 0);
    clone_cache->updateCache();
    EXPECT_EQ(clone_cache->getCurrentCacheSize(), 3);
  }
  // Now change revision and call update
  {
    original->revision_++;
    original->val_1 = 5;
    clone_cache->updateCache();
    auto clone = clone_cache->clone();
    EXPECT_EQ(original->val_1, clone->val_1);
  }
  // Now change revision and call clone
  {
    original->revision_++;
    original->val_1 = 6;
    auto clone = clone_cache->clone();
    EXPECT_EQ(original->val_1, clone->val_1);
  }
  // Try cloning more times than the cache is big
  for (int i = 0; i < 10; i++)
  {
    auto clone = clone_cache->clone();
  }
  // Test getters/setters
  EXPECT_EQ(clone_cache->getCacheSize(), 3);
  clone_cache->setCacheSize(8);
  EXPECT_EQ(clone_cache->getCacheSize(), 8);
  EXPECT_EQ(clone_cache->getCurrentCacheSize(), 8);
}

TEST(TesseractCloneCacheUnit, SupportsUpdate)  // NOLINT
{
  auto original = std::make_shared<TestObjectSupportsUpdate>();
  original->val_1 = 1;
  original->val_2 = 2;
  auto clone_cache = std::make_shared<CloneCache<TestObjectSupportsUpdate>>(original, 3);
  EXPECT_EQ(clone_cache->getCacheSize(), 3);
  EXPECT_EQ(clone_cache->getCurrentCacheSize(), 3);

  // Baseline clone of original
  {
    auto clone = clone_cache->clone();
    EXPECT_EQ(original->val_1, clone->val_1);
    EXPECT_EQ(original->val_2, clone->val_2);
  }
  // Change value without changing revision would be expected to fail
  {
    auto clone = clone_cache->clone();

    original->val_1 = 3;
    EXPECT_NE(original->val_1, clone->val_1);
  }
  // Now it should update the cache and be the same
  {
    auto clone = clone_cache->clone();

    original->revision_++;
    EXPECT_NE(original->val_1, clone->val_1);

    auto updated_clone = clone_cache->clone();
    EXPECT_EQ(original->val_1, updated_clone->val_1);
  }
  // Cache should be empty so call update
  {
    EXPECT_EQ(clone_cache->getCurrentCacheSize(), 0);
    clone_cache->updateCache();
    EXPECT_EQ(clone_cache->getCurrentCacheSize(), 3);
  }
  // Now change revision and call update
  {
    original->revision_++;
    original->val_1 = 5;
    clone_cache->updateCache();
    auto clone = clone_cache->clone();
    EXPECT_EQ(original->val_1, clone->val_1);
  }
  // Now change revision and call clone
  {
    original->revision_++;
    original->val_1 = 6;
    auto clone = clone_cache->clone();
    EXPECT_EQ(original->val_1, clone->val_1);
  }
  // Try cloning more times than the cache is big
  for (int i = 0; i < 10; i++)
  {
    auto clone = clone_cache->clone();
  }
  // Test getters/setters
  EXPECT_EQ(clone_cache->getCacheSize(), 3);
  clone_cache->setCacheSize(8);
  EXPECT_EQ(clone_cache->getCacheSize(), 8);
  EXPECT_EQ(clone_cache->getCurrentCacheSize(), 8);
}

TEST(TesseractCloneCacheUnit, SupportsUpdateFailure)  // NOLINT
{
  {  // Test original is a nullptr
    std::shared_ptr<TestObjectSupportsUpdate> original;
    auto clone_cache = std::make_shared<CloneCache<TestObjectSupportsUpdate>>(original, 3);
    EXPECT_TRUE(clone_cache->clone() == nullptr);
    EXPECT_EQ(clone_cache->getCacheSize(), 3);
    EXPECT_EQ(clone_cache->getCurrentCacheSize(), 0);
    clone_cache->updateCache();
    EXPECT_EQ(clone_cache->getCurrentCacheSize(), 0);
  }

  // Test when clone throws an exception
  auto original = std::make_shared<TestObjectSupportsUpdateFailure>();
  original->val_1 = 1;
  original->val_2 = 2;
  auto clone_cache = std::make_shared<CloneCache<TestObjectSupportsUpdateFailure>>(original, 3);
  EXPECT_EQ(clone_cache->getCacheSize(), 3);
  EXPECT_EQ(clone_cache->getCurrentCacheSize(), 0);

  // Baseline clone of original which should be nullptr because of exception during clone
  {
    EXPECT_TRUE(clone_cache->clone() == nullptr);
    EXPECT_EQ(clone_cache->getCacheSize(), 3);
    EXPECT_EQ(clone_cache->getCurrentCacheSize(), 0);
  }
  // Now it should update the cache and clone should be nullptr because of exception during clone
  {
    original->revision_++;
    EXPECT_EQ(clone_cache->getCacheSize(), 3);
    EXPECT_EQ(clone_cache->getCurrentCacheSize(), 0);
    EXPECT_TRUE(clone_cache->clone() == nullptr);
  }
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);

  return RUN_ALL_TESTS();
}
