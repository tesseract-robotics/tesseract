#include <tesseract_common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <gtest/gtest.h>
#include <iostream>
#include <fstream>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract_support/tesseract_support_resource_locator.h>
#include <tesseract_common/types.h>
#include <tesseract_common/unit_test_utils.h>

TEST(TesseractSupportUnit, TesseractSupportResourceLocatorUnit)  // NOLINT
{
  using namespace tesseract_common;
  tesseract_common::fs::path file_path(INSTALL_LOCATION);
  tesseract_common::fs::path package_path = file_path / "share/tesseract_support";

  ResourceLocator::Ptr locator = std::make_shared<TesseractSupportResourceLocator>();

  Resource::Ptr resource = locator->locateResource("package://tesseract_support/urdf/abb_irb2400.urdf");
  EXPECT_TRUE(resource != nullptr);
  EXPECT_TRUE(resource->isFile());
  EXPECT_EQ(resource->getUrl(), "package://tesseract_support/urdf/abb_irb2400.urdf");
  EXPECT_EQ(tesseract_common::fs::path(resource->getFilePath()), (package_path / "urdf/abb_irb2400.urdf"));
  EXPECT_FALSE(resource->getResourceContents().empty());
  EXPECT_TRUE(resource->getResourceContentStream() != nullptr);

  Resource::Ptr sub_resource = resource->locateResource("boxbot.urdf");
  EXPECT_TRUE(sub_resource != nullptr);
  EXPECT_TRUE(sub_resource->isFile());
  EXPECT_EQ(sub_resource->getUrl(), "package://tesseract_support/urdf/boxbot.urdf");
  EXPECT_EQ(tesseract_common::fs::path(sub_resource->getFilePath()), (package_path / "urdf/boxbot.urdf"));
  EXPECT_FALSE(sub_resource->getResourceContents().empty());
  EXPECT_TRUE(sub_resource->getResourceContentStream() != nullptr);

  tesseract_common::Resource::Ptr resource_empty = locator->locateResource("");
  EXPECT_TRUE(resource_empty == nullptr);

  tesseract_common::Resource::Ptr resource_does_not_exist = locator->locateResource("package://tesseract_support/urdf/"
                                                                                    "does_not_exist.urdf");
  EXPECT_TRUE(resource_does_not_exist != nullptr);
  EXPECT_TRUE(resource_does_not_exist->getResourceContents().empty());
  EXPECT_TRUE(resource_does_not_exist->getResourceContentStream() == nullptr);
}

TEST(TesseractSupportUnit, TesseractSupportResourceLocatorSerializUnit)  // NOLINT
{
  using namespace tesseract_common;
  ResourceLocator::Ptr locator = std::make_shared<TesseractSupportResourceLocator>();
  tesseract_common::testSerializationDerivedClass<ResourceLocator, TesseractSupportResourceLocator>(locator,
                                                                                                    "TesseractSupportRe"
                                                                                                    "sourceLocator");
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);

  return RUN_ALL_TESTS();
}
