#include <tesseract_common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <gtest/gtest.h>
#include <iostream>
#include <fstream>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract_common/resource_locator.h>
#include <tesseract_common/types.h>
#include <tesseract_common/unit_test_utils.h>

/** @brief Resource locator implementation using a provided function to locate file resources */
class TestResourceLocator : public tesseract_common::ResourceLocator
{
public:
  using Ptr = std::shared_ptr<TestResourceLocator>;
  using ConstPtr = std::shared_ptr<const TestResourceLocator>;

  ~TestResourceLocator() override = default;

  tesseract_common::Resource::Ptr locateResource(const std::string& url) const override final
  {
    std::string mod_url = url;
    if (url.find("package://tesseract_common") == 0)
    {
      mod_url.erase(0, strlen("package://tesseract_common"));
      size_t pos = mod_url.find('/');
      if (pos == std::string::npos)
        return nullptr;

      std::string package = mod_url.substr(0, pos);
      mod_url.erase(0, pos);

      tesseract_common::fs::path file_path(__FILE__);
      std::string package_path = file_path.parent_path().parent_path().string();

      if (package_path.empty())
        return nullptr;

      mod_url = package_path + mod_url;
    }

    if (!tesseract_common::fs::path(mod_url).is_complete())
      return nullptr;

    return std::make_shared<tesseract_common::SimpleLocatedResource>(
        url, mod_url, std::make_shared<TestResourceLocator>(*this));
  }
};

TEST(ResourceLocatorUnit, SimpleResourceLocatorUnit)  // NOLINT
{
  using namespace tesseract_common;
  tesseract_common::fs::path file_path(__FILE__);
  tesseract_common::fs::path package_path = file_path.parent_path().parent_path();

  ResourceLocator::Ptr locator = std::make_shared<TestResourceLocator>();

  Resource::Ptr resource = locator->locateResource("package://tesseract_common/package.xml");
  EXPECT_TRUE(resource != nullptr);
  EXPECT_TRUE(resource->isFile());
  EXPECT_EQ(resource->getUrl(), "package://tesseract_common/package.xml");
  EXPECT_EQ(tesseract_common::fs::path(resource->getFilePath()), (package_path / "package.xml"));
  EXPECT_FALSE(resource->getResourceContents().empty());
  EXPECT_TRUE(resource->getResourceContentStream() != nullptr);

  Resource::Ptr sub_resource = resource->locateResource("colcon.pkg");
  EXPECT_TRUE(sub_resource != nullptr);
  EXPECT_TRUE(sub_resource->isFile());
  EXPECT_EQ(sub_resource->getUrl(), "package://tesseract_common/colcon.pkg");
  EXPECT_EQ(tesseract_common::fs::path(sub_resource->getFilePath()), (package_path / "colcon.pkg"));
  EXPECT_FALSE(sub_resource->getResourceContents().empty());
  EXPECT_TRUE(sub_resource->getResourceContentStream() != nullptr);

  tesseract_common::Resource::Ptr sub_resource_empty = sub_resource->locateResource("");
  EXPECT_TRUE(sub_resource_empty == nullptr);

  tesseract_common::Resource::Ptr resource_empty = locator->locateResource("");
  EXPECT_TRUE(resource_empty == nullptr);

  tesseract_common::Resource::Ptr resource_does_not_exist = locator->locateResource("package://tesseract_common/"
                                                                                    "does_not_exist.txt");
  EXPECT_TRUE(resource_does_not_exist != nullptr);
  EXPECT_TRUE(resource_does_not_exist->getResourceContents().empty());
  EXPECT_TRUE(resource_does_not_exist->getResourceContentStream() == nullptr);
}

TEST(ResourceLocatorUnit, GeneralResourceLocatorUnit1)  // NOLINT
{
  using namespace tesseract_common;
  tesseract_common::fs::path file_path(__FILE__);
  tesseract_common::fs::path package_path = file_path.parent_path().parent_path();

#ifndef _WIN32
  std::string env_var = "TESSERACT_RESOURCE_PATH=" + package_path.string();
#else
  std::string env_var = "TESSERACT_RESOURCE_PATH=" + package_path.string();
#endif
  putenv(env_var.data());

  ResourceLocator::Ptr locator = std::make_shared<GeneralResourceLocator>();

  Resource::Ptr resource = locator->locateResource("package://tesseract_common/package.xml");
  EXPECT_TRUE(resource != nullptr);
  EXPECT_TRUE(resource->isFile());
  EXPECT_EQ(resource->getUrl(), "package://tesseract_common/package.xml");
  EXPECT_EQ(tesseract_common::fs::path(resource->getFilePath()), (package_path / "package.xml"));
  EXPECT_FALSE(resource->getResourceContents().empty());
  EXPECT_TRUE(resource->getResourceContentStream() != nullptr);

  Resource::Ptr sub_resource = resource->locateResource("colcon.pkg");
  EXPECT_TRUE(sub_resource != nullptr);
  EXPECT_TRUE(sub_resource->isFile());
  EXPECT_EQ(sub_resource->getUrl(), "package://tesseract_common/colcon.pkg");
  EXPECT_EQ(tesseract_common::fs::path(sub_resource->getFilePath()), (package_path / "colcon.pkg"));
  EXPECT_FALSE(sub_resource->getResourceContents().empty());
  EXPECT_TRUE(sub_resource->getResourceContentStream() != nullptr);

  tesseract_common::Resource::Ptr sub_resource_empty = sub_resource->locateResource("");
  EXPECT_TRUE(sub_resource_empty == nullptr);

  tesseract_common::Resource::Ptr resource_empty = locator->locateResource("");
  EXPECT_TRUE(resource_empty == nullptr);

  tesseract_common::Resource::Ptr resource_does_not_exist = locator->locateResource("package://tesseract_common/"
                                                                                    "does_not_exist.txt");
  EXPECT_TRUE(resource_does_not_exist != nullptr);
  EXPECT_TRUE(resource_does_not_exist->getResourceContents().empty());
  EXPECT_TRUE(resource_does_not_exist->getResourceContentStream() == nullptr);
}

TEST(ResourceLocatorUnit, GeneralResourceLocatorUnit2)  // NOLINT
{
  using namespace tesseract_common;
  tesseract_common::fs::path file_path(__FILE__);
  tesseract_common::fs::path package_path = file_path.parent_path().parent_path();

#ifndef _WIN32
  std::string env_var = "ROS_PACKAGE_PATH=" + package_path.string();
#else
  std::string env_var = "ROS_PACKAGE_PATH=" + package_path.string();
#endif
  putenv(env_var.data());

  ResourceLocator::Ptr locator = std::make_shared<GeneralResourceLocator>();

  Resource::Ptr resource = locator->locateResource("package://tesseract_common/package.xml");
  EXPECT_TRUE(resource != nullptr);
  EXPECT_TRUE(resource->isFile());
  EXPECT_EQ(resource->getUrl(), "package://tesseract_common/package.xml");
  EXPECT_EQ(tesseract_common::fs::path(resource->getFilePath()), (package_path / "package.xml"));
  EXPECT_FALSE(resource->getResourceContents().empty());
  EXPECT_TRUE(resource->getResourceContentStream() != nullptr);

  Resource::Ptr sub_resource = resource->locateResource("colcon.pkg");
  EXPECT_TRUE(sub_resource != nullptr);
  EXPECT_TRUE(sub_resource->isFile());
  EXPECT_EQ(sub_resource->getUrl(), "package://tesseract_common/colcon.pkg");
  EXPECT_EQ(tesseract_common::fs::path(sub_resource->getFilePath()), (package_path / "colcon.pkg"));
  EXPECT_FALSE(sub_resource->getResourceContents().empty());
  EXPECT_TRUE(sub_resource->getResourceContentStream() != nullptr);

  tesseract_common::Resource::Ptr sub_resource_empty = sub_resource->locateResource("");
  EXPECT_TRUE(sub_resource_empty == nullptr);

  tesseract_common::Resource::Ptr resource_empty = locator->locateResource("");
  EXPECT_TRUE(resource_empty == nullptr);

  tesseract_common::Resource::Ptr resource_does_not_exist = locator->locateResource("package://tesseract_common/"
                                                                                    "does_not_exist.txt");
  EXPECT_TRUE(resource_does_not_exist != nullptr);
  EXPECT_TRUE(resource_does_not_exist->getResourceContents().empty());
  EXPECT_TRUE(resource_does_not_exist->getResourceContentStream() == nullptr);
}

TEST(ResourceLocatorUnit, ByteResourceUnit)  // NOLINT
{
  using namespace tesseract_common;
  tesseract_common::fs::path file_path(__FILE__);
  tesseract_common::fs::path package_path = file_path.parent_path().parent_path();

  ResourceLocator::Ptr locator = std::make_shared<TestResourceLocator>();
  Resource::Ptr resource = locator->locateResource("package://tesseract_common/package.xml");

  auto byte_resource = std::make_shared<BytesResource>(
      "package://tesseract_common/package.xml", std::vector<uint8_t>({ 1, 2, 3, 4 }), resource);
  EXPECT_TRUE(byte_resource != nullptr);
  EXPECT_FALSE(byte_resource->isFile());
  EXPECT_EQ(byte_resource->getUrl(), "package://tesseract_common/package.xml");
  EXPECT_TRUE(byte_resource->getFilePath().empty());
  EXPECT_FALSE(byte_resource->getResourceContents().empty());
  EXPECT_TRUE(byte_resource->getResourceContentStream() != nullptr);

  Resource::Ptr sub_resource = byte_resource->locateResource("colcon.pkg");
  EXPECT_TRUE(sub_resource != nullptr);
  EXPECT_TRUE(sub_resource->isFile());
  EXPECT_EQ(sub_resource->getUrl(), "package://tesseract_common/colcon.pkg");
  EXPECT_EQ(tesseract_common::fs::path(sub_resource->getFilePath()), (package_path / "colcon.pkg"));
  EXPECT_FALSE(sub_resource->getResourceContents().empty());
  EXPECT_TRUE(sub_resource->getResourceContentStream() != nullptr);

  tesseract_common::Resource::Ptr resource_empty = byte_resource->locateResource("");
  EXPECT_TRUE(resource_empty == nullptr);

  tesseract_common::Resource::Ptr resource_does_not_exist = byte_resource->locateResource("package://tesseract_common/"
                                                                                          "does_not_exist.txt");
  EXPECT_TRUE(resource_does_not_exist != nullptr);
  EXPECT_TRUE(resource_does_not_exist->getResourceContents().empty());
  EXPECT_TRUE(resource_does_not_exist->getResourceContentStream() == nullptr);
}

TEST(ResourceLocatorUnit, SimpleLocatedResourceSerializUnit)  // NOLINT
{
  using namespace tesseract_common;
  SimpleLocatedResource resource("url", "file_path");
  tesseract_common::testSerialization<SimpleLocatedResource>(resource, "SimpleLocatedResource");
}

TEST(ResourceLocatorUnit, BytesResourceSerializUnit)  // NOLINT
{
  using namespace tesseract_common;
  BytesResource resource("url", { 1, 2, 3, 4, 5 });
  tesseract_common::testSerialization<BytesResource>(resource, "BytesResource");
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);

  return RUN_ALL_TESTS();
}
