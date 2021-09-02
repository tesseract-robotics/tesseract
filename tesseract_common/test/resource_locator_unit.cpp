#include <tesseract_common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <gtest/gtest.h>
#include <iostream>
#include <fstream>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract_common/resource_locator.h>
#include <tesseract_common/types.h>

std::string locateResource(const std::string& url)
{
  std::string mod_url = url;
  if (url.find("package://tesseract_common") == 0)
  {
    mod_url.erase(0, strlen("package://tesseract_common"));
    size_t pos = mod_url.find('/');
    if (pos == std::string::npos)
    {
      return std::string();
    }

    std::string package = mod_url.substr(0, pos);
    mod_url.erase(0, pos);

    tesseract_common::fs::path file_path(__FILE__);
    std::string package_path = file_path.parent_path().parent_path().string();

    if (package_path.empty())
    {
      return std::string();
    }

    mod_url = package_path + mod_url;
  }

  return mod_url;
}

TEST(TesseractSceneGraphResourceUnit, SimpleResourceLocatorUnit)  // NOLINT
{
  using namespace tesseract_common;
  tesseract_common::fs::path file_path(__FILE__);
  tesseract_common::fs::path package_path = file_path.parent_path().parent_path();

  ResourceLocator::Ptr locator = std::make_shared<SimpleResourceLocator>(locateResource);

  Resource::Ptr resource = locator->locateResource("package://tesseract_common/package.xml");
  EXPECT_TRUE(resource != nullptr);
  EXPECT_TRUE(resource->isFile());
  EXPECT_EQ(resource->getUrl(), "package://tesseract_common/package.xml");
  EXPECT_EQ(resource->getFilePath(), (package_path / "package.xml").string());
  EXPECT_FALSE(resource->getResourceContents().empty());
  EXPECT_TRUE(resource->getResourceContentStream() != nullptr);

  Resource::Ptr sub_resource = resource->locateResource("colcon.pkg");
  EXPECT_TRUE(sub_resource != nullptr);
  EXPECT_TRUE(sub_resource->isFile());
  EXPECT_EQ(sub_resource->getUrl(), "package://tesseract_common/colcon.pkg");
  EXPECT_EQ(sub_resource->getFilePath(), (package_path / "colcon.pkg").string());
  EXPECT_FALSE(sub_resource->getResourceContents().empty());
  EXPECT_TRUE(sub_resource->getResourceContentStream() != nullptr);

  tesseract_common::Resource::Ptr resource_empty = locator->locateResource("");
  EXPECT_TRUE(resource_empty == nullptr);

  tesseract_common::Resource::Ptr resource_does_not_exist = locator->locateResource("package://tesseract_commong/"
                                                                                    "does_not_exist.txt");
  EXPECT_TRUE(resource_does_not_exist != nullptr);
  EXPECT_TRUE(resource_does_not_exist->getResourceContents().empty());
  EXPECT_TRUE(resource_does_not_exist->getResourceContentStream() == nullptr);
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);

  return RUN_ALL_TESTS();
}
