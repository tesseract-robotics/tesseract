#include <tesseract_common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <gtest/gtest.h>
#include <Eigen/Geometry>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract_urdf/geometry.h>
#include "tesseract_urdf_common_unit.h"

TEST(TesseractURDFUnit, parse_geometry)  // NOLINT
{
  std::shared_ptr<tesseract_scene_graph::SimpleResourceLocator> resource_locator =
      std::make_shared<tesseract_scene_graph::SimpleResourceLocator>(locateResource);
  {
    std::string str = R"(<geometry extra="0 0 0">
                           <box size="1 1 1" />
                         </geometry>)";
    std::vector<tesseract_geometry::Geometry::Ptr> elem;
    auto status =
        runTest<std::vector<tesseract_geometry::Geometry::Ptr>>(elem, str, "geometry", resource_locator, 2, true);
    EXPECT_TRUE(*status);
    EXPECT_TRUE(elem.size() == 1);
    EXPECT_TRUE(elem[0]->getType() == tesseract_geometry::GeometryType::BOX);
  }

  {
    std::string str = R"(<geometry>
                         </geometry>)";
    std::vector<tesseract_geometry::Geometry::Ptr> elem;
    auto status =
        runTest<std::vector<tesseract_geometry::Geometry::Ptr>>(elem, str, "geometry", resource_locator, 2, true);
    EXPECT_FALSE(*status);
  }

  {
    std::string str = R"(<geometry>
                           <box size="1 1 a" />
                         </geometry>)";
    std::vector<tesseract_geometry::Geometry::Ptr> elem;
    auto status =
        runTest<std::vector<tesseract_geometry::Geometry::Ptr>>(elem, str, "geometry", resource_locator, 2, true);
    EXPECT_FALSE(*status);
  }
}
