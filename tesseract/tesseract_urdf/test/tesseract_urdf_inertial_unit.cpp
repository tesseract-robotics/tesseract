#include <tesseract_common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <gtest/gtest.h>
#include <Eigen/Geometry>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract_urdf/inertial.h>
#include "tesseract_urdf_common_unit.h"

TEST(TesseractURDFUnit, parse_inertial)  // NOLINT
{
  {
    std::string str = R"(<inertial extra="0 0 0">
                           <origin xyz="0 0 0" rpy="0 0 0"/>
                           <mass value="1.0" extra="0 0 0"/>
                           <inertia ixx="1.0" ixy="2.0" ixz="3.0" iyy="4.0" iyz="5.0" izz="6.0" extra="0 0 0"/>
                         </inertial>)";
    tesseract_scene_graph::Inertial::Ptr elem;
    auto status = runTest<tesseract_scene_graph::Inertial::Ptr>(elem, str, "inertial", 2);
    EXPECT_TRUE(*status);
    EXPECT_NEAR(elem->mass, 1, 1e-8);
    EXPECT_NEAR(elem->ixx, 1, 1e-8);
    EXPECT_NEAR(elem->ixy, 2, 1e-8);
    EXPECT_NEAR(elem->ixz, 3, 1e-8);
    EXPECT_NEAR(elem->iyy, 4, 1e-8);
    EXPECT_NEAR(elem->iyz, 5, 1e-8);
    EXPECT_NEAR(elem->izz, 6, 1e-8);
  }

  {
    std::string str = R"(<inertial>
                           <mass value="1.0"/>
                           <inertia ixx="1.0" ixy="2.0" ixz="3.0" iyy="4.0" iyz="5.0" izz="6.0"/>
                         </inertial>)";
    tesseract_scene_graph::Inertial::Ptr elem;
    auto status = runTest<tesseract_scene_graph::Inertial::Ptr>(elem, str, "inertial", 2);
    EXPECT_TRUE(*status);
    EXPECT_NEAR(elem->mass, 1, 1e-8);
    EXPECT_NEAR(elem->ixx, 1, 1e-8);
    EXPECT_NEAR(elem->ixy, 2, 1e-8);
    EXPECT_NEAR(elem->ixz, 3, 1e-8);
    EXPECT_NEAR(elem->iyy, 4, 1e-8);
    EXPECT_NEAR(elem->iyz, 5, 1e-8);
    EXPECT_NEAR(elem->izz, 6, 1e-8);
  }

  {
    std::string str = R"(<inertial>
                           <inertia ixx="1.0" ixy="2.0" ixz="3.0" iyy="4.0" iyz="5.0" izz="6.0"/>
                         </inertial>)";
    tesseract_scene_graph::Inertial::Ptr elem;
    auto status = runTest<tesseract_scene_graph::Inertial::Ptr>(elem, str, "inertial", 2);
    EXPECT_FALSE(*status);
  }

  {
    std::string str = R"(<inertial>
                           <mass value="1.0"/>
                         </inertial>)";
    tesseract_scene_graph::Inertial::Ptr elem;
    auto status = runTest<tesseract_scene_graph::Inertial::Ptr>(elem, str, "inertial", 2);
    EXPECT_FALSE(*status);
  }

  {
    std::string str = R"(<inertial>
                           <mass value="a"/>
                           <inertia ixx="1.0" ixy="2.0" ixz="3.0" iyy="4.0" iyz="5.0" izz="6.0"/>
                         </inertial>)";
    tesseract_scene_graph::Inertial::Ptr elem;
    auto status = runTest<tesseract_scene_graph::Inertial::Ptr>(elem, str, "inertial", 2);
    EXPECT_FALSE(*status);
  }

  {
    std::string str = R"(<inertial>
                           <mass />
                           <inertia ixx="1.0" ixy="2.0" ixz="3.0" iyy="4.0" iyz="5.0" izz="6.0"/>
                         </inertial>)";
    tesseract_scene_graph::Inertial::Ptr elem;
    auto status = runTest<tesseract_scene_graph::Inertial::Ptr>(elem, str, "inertial", 2);
    EXPECT_FALSE(*status);
  }

  {
    std::string str = R"(<inertial>
                           <mass value="1.0"/>
                           <inertia ixx="a" ixy="2.0" ixz="3.0" iyy="4.0" iyz="5.0" izz="6.0"/>
                         </inertial>)";
    tesseract_scene_graph::Inertial::Ptr elem;
    auto status = runTest<tesseract_scene_graph::Inertial::Ptr>(elem, str, "inertial", 2);
    EXPECT_FALSE(*status);
  }

  {
    std::string str = R"(<inertial>
                           <mass value="1.0"/>
                           <inertia ixx="1.0" ixy="a" ixz="3.0" iyy="4.0" iyz="5.0" izz="6.0"/>
                         </inertial>)";
    tesseract_scene_graph::Inertial::Ptr elem;
    auto status = runTest<tesseract_scene_graph::Inertial::Ptr>(elem, str, "inertial", 2);
    EXPECT_FALSE(*status);
  }

  {
    std::string str = R"(<inertial>
                           <mass value="1.0"/>
                           <inertia ixx="1.0" ixy="2.0" ixz="a" iyy="4.0" iyz="5.0" izz="6.0"/>
                         </inertial>)";
    tesseract_scene_graph::Inertial::Ptr elem;
    auto status = runTest<tesseract_scene_graph::Inertial::Ptr>(elem, str, "inertial", 2);
    EXPECT_FALSE(*status);
  }

  {
    std::string str = R"(<inertial>
                           <mass value="1.0"/>
                           <inertia ixx="1.0" ixy="2.0" ixz="3.0" iyy="a" iyz="5.0" izz="6.0"/>
                         </inertial>)";
    tesseract_scene_graph::Inertial::Ptr elem;
    auto status = runTest<tesseract_scene_graph::Inertial::Ptr>(elem, str, "inertial", 2);
    EXPECT_FALSE(*status);
  }

  {
    std::string str = R"(<inertial>
                           <mass value="1.0"/>
                           <inertia ixx="1.0" ixy="2.0" ixz="3.0" iyy="4.0" iyz="a" izz="6.0"/>
                         </inertial>)";
    tesseract_scene_graph::Inertial::Ptr elem;
    auto status = runTest<tesseract_scene_graph::Inertial::Ptr>(elem, str, "inertial", 2);
    EXPECT_FALSE(*status);
  }

  {
    std::string str = R"(<inertial>
                           <mass value="1.0"/>
                           <inertia ixx="1.0" ixy="2.0" ixz="3.0" iyy="4.0" iyz="5.0" izz="a"/>
                         </inertial>)";
    tesseract_scene_graph::Inertial::Ptr elem;
    auto status = runTest<tesseract_scene_graph::Inertial::Ptr>(elem, str, "inertial", 2);
    EXPECT_FALSE(*status);
  }

  {
    std::string str = R"(<inertial>
                           <mass value="1.0"/>
                           <inertia ixy="2.0" ixz="3.0" iyy="4.0" iyz="5.0" izz="6.0"/>
                         </inertial>)";
    tesseract_scene_graph::Inertial::Ptr elem;
    auto status = runTest<tesseract_scene_graph::Inertial::Ptr>(elem, str, "inertial", 2);
    EXPECT_FALSE(*status);
  }

  {
    std::string str = R"(<inertial>
                           <mass value="1.0"/>
                           <inertia ixx="1.0" ixz="3.0" iyy="4.0" iyz="5.0" izz="6.0"/>
                         </inertial>)";
    tesseract_scene_graph::Inertial::Ptr elem;
    auto status = runTest<tesseract_scene_graph::Inertial::Ptr>(elem, str, "inertial", 2);
    EXPECT_FALSE(*status);
  }

  {
    std::string str = R"(<inertial>
                           <mass value="1.0"/>
                           <inertia ixx="1.0" ixy="2.0" iyy="4.0" iyz="5.0" izz="6.0"/>
                         </inertial>)";
    tesseract_scene_graph::Inertial::Ptr elem;
    auto status = runTest<tesseract_scene_graph::Inertial::Ptr>(elem, str, "inertial", 2);
    EXPECT_FALSE(*status);
  }

  {
    std::string str = R"(<inertial>
                           <mass value="1.0"/>
                           <inertia ixx="1.0" ixy="2.0" ixz="3.0" iyz="5.0" izz="6.0"/>
                         </inertial>)";
    tesseract_scene_graph::Inertial::Ptr elem;
    auto status = runTest<tesseract_scene_graph::Inertial::Ptr>(elem, str, "inertial", 2);
    EXPECT_FALSE(*status);
  }

  {
    std::string str = R"(<inertial>
                           <mass value="1.0"/>
                           <inertia ixx="1.0" ixy="2.0" ixz="3.0" iyy="4.0" izz="6.0"/>
                         </inertial>)";
    tesseract_scene_graph::Inertial::Ptr elem;
    auto status = runTest<tesseract_scene_graph::Inertial::Ptr>(elem, str, "inertial", 2);
    EXPECT_FALSE(*status);
  }

  {
    std::string str = R"(<inertial>
                           <mass value="1.0"/>
                           <inertia ixx="1.0" ixy="2.0" ixz="3.0" iyy="4.0" iyz="5.0"/>
                         </inertial>)";
    tesseract_scene_graph::Inertial::Ptr elem;
    auto status = runTest<tesseract_scene_graph::Inertial::Ptr>(elem, str, "inertial", 2);
    EXPECT_FALSE(*status);
  }

  {
    std::string str = R"(<inertial>
                           <mass value="1.0"/>
                           <inertia />
                         </inertial>)";
    tesseract_scene_graph::Inertial::Ptr elem;
    auto status = runTest<tesseract_scene_graph::Inertial::Ptr>(elem, str, "inertial", 2);
    EXPECT_FALSE(*status);
  }
}
