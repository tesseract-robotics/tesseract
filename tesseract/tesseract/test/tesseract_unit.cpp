#include <tesseract_common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <gtest/gtest.h>
#include <algorithm>
#include <vector>
#include <tesseract_urdf/urdf_parser.h>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract/tesseract.h>

using namespace tesseract_scene_graph;
using namespace tesseract_collision;
using namespace tesseract_environment;

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

SceneGraph::Ptr getSceneGraph()
{
  std::string path = std::string(TESSERACT_SUPPORT_DIR) + "/urdf/lbr_iiwa_14_r820.urdf";

  tesseract_scene_graph::ResourceLocator::Ptr locator =
      std::make_shared<tesseract_scene_graph::SimpleResourceLocator>(locateResource);
  return tesseract_urdf::parseURDFFile(path, locator);
}

void testIdentical(const tesseract::Tesseract::Ptr& one, const tesseract::Tesseract::Ptr& two)
{
  EXPECT_TRUE(one != nullptr);
  EXPECT_TRUE(two != nullptr);

  // TODO: Actually test the components
}

TEST(TesseractUnit, TesseractSceneGraphInitUnit)  // NOLINT
{
  tesseract::Tesseract::Ptr tesseract = std::make_shared<tesseract::Tesseract>();
  SceneGraph::Ptr scene_graph = nullptr;
  EXPECT_FALSE(tesseract->init(scene_graph));
  EXPECT_TRUE(tesseract->init(getSceneGraph()));
}

TEST(TesseractUnit, TesseractSceneGraphSRDFModelInitUnit)  // NOLINT
{
}

TEST(TesseractUnit, TesseractURDFStringLocatorInitUnit)  // NOLINT
{
}

TEST(TesseractUnit, TesseractURDFStringSRDFStringLocatorStringInitUnit)  // NOLINT
{
}

TEST(TesseractUnit, TesseractURDFPathLocatorInitUnit)  // NOLINT
{
}

TEST(TesseractUnit, TesseractURDFPathSRDFPathLocatorInitUnit)  // NOLINT
{
}

TEST(TesseractUnit, TesseractInitInfoInitUnit)  // NOLINT
{
}

TEST(TesseractUnit, TesseractCloneUnit)  // NOLINT
{
  tesseract::Tesseract::Ptr tesseract = std::make_shared<tesseract::Tesseract>();

  tesseract->init(getSceneGraph());
  auto cloned_tesseract = tesseract->clone();
  testIdentical(tesseract, cloned_tesseract);
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);

  return RUN_ALL_TESTS();
}
