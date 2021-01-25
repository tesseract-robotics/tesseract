#include <tesseract_common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <gtest/gtest.h>
#include <iostream>
#include <fstream>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract_scene_graph/graph.h>
#include <tesseract_scene_graph/utils.h>
#include <tesseract_scene_graph/resource_locator.h>

std::string locateResource(const std::string& url)
{
  std::string mod_url = url;
  if (url.find("package://tesseract_support") == 0)
  {
    mod_url.erase(0, strlen("package://tesseract_support"));
    size_t pos = mod_url.find('/');
    if (pos == std::string::npos)
    {
      return std::string();
    }

    std::string package = mod_url.substr(0, pos);
    mod_url.erase(0, pos);
    std::string package_path = std::string(TESSERACT_SUPPORT_DIR);

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
  using namespace tesseract_scene_graph;

  ResourceLocator::Ptr locator = std::make_shared<SimpleResourceLocator>(locateResource);

  tesseract_common::Resource::Ptr resource = locator->locateResource("package://tesseract_support/urdf/abb_irb2400/"
                                                                     "irb2400/collision/base_link.stl");
  EXPECT_TRUE(resource != nullptr);
  EXPECT_TRUE(resource->isFile());
  EXPECT_EQ(resource->getUrl(), "package://tesseract_support/urdf/abb_irb2400/irb2400/collision/base_link.stl");
  EXPECT_EQ(resource->getFilePath(),
            std::string(TESSERACT_SUPPORT_DIR) + "/urdf/abb_irb2400/irb2400/collision/base_link.stl");
  EXPECT_FALSE(resource->getResourceContents().empty());
  EXPECT_TRUE(resource->getResourceContentStream() != nullptr);

  tesseract_common::Resource::Ptr sub_resource = resource->locateSubResource("link_1.stl");
  EXPECT_TRUE(sub_resource != nullptr);
  EXPECT_TRUE(sub_resource->isFile());
  EXPECT_EQ(sub_resource->getUrl(), "package://tesseract_support/urdf/abb_irb2400/irb2400/collision/link_1.stl");
  EXPECT_EQ(sub_resource->getFilePath(),
            std::string(TESSERACT_SUPPORT_DIR) + "/urdf/abb_irb2400/irb2400/collision/link_1.stl");
  EXPECT_FALSE(sub_resource->getResourceContents().empty());
  EXPECT_TRUE(sub_resource->getResourceContentStream() != nullptr);

  tesseract_common::Resource::Ptr resource_empty = locator->locateResource("");
  EXPECT_TRUE(resource_empty == nullptr);

  tesseract_common::Resource::Ptr resource_does_not_exist = locator->locateResource("package://tesseract_support/urdf/"
                                                                                    "abb_irb2400/"
                                                                                    "irb2400/collision/"
                                                                                    "does_not_exist.stl");
  EXPECT_TRUE(resource_does_not_exist != nullptr);
  EXPECT_TRUE(resource_does_not_exist->getResourceContents().empty());
  EXPECT_TRUE(resource_does_not_exist->getResourceContentStream() == nullptr);
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);

  return RUN_ALL_TESTS();
}
