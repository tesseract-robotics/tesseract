#include <tesseract_common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <gtest/gtest.h>
//#include <vector>
//#include <tinyxml2.h>
//#include <console_bridge/console.h>
//#include <unordered_map>
//#include <pcl/io/pcd_io.h>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

//#include <tesseract_geometry/geometries.h>
//#include <tesseract_scene_graph/utils.h>

//#include <tesseract_urdf/box.h>
//#include <tesseract_urdf/sphere.h>
//#include <tesseract_urdf/cone.h>
//#include <tesseract_urdf/cylinder.h>
//#include <tesseract_urdf/capsule.h>
//#include <tesseract_urdf/mesh.h>
//#include <tesseract_urdf/convex_mesh.h>
//#include <tesseract_urdf/sdf_mesh.h>
//#include <tesseract_urdf/octree.h>
//#include <tesseract_urdf/calibration.h>
//#include <tesseract_urdf/dynamics.h>
//#include <tesseract_urdf/inertial.h>
//#include <tesseract_urdf/limits.h>
//#include <tesseract_urdf/material.h>
//#include <tesseract_urdf/mimic.h>
//#include <tesseract_urdf/safety_controller.h>
//#include <tesseract_urdf/geometry.h>
//#include <tesseract_urdf/visual.h>
//#include <tesseract_urdf/collision.h>
//#include <tesseract_urdf/link.h>
//#include <tesseract_urdf/joint.h>
//#include <tesseract_urdf/urdf_parser.h>

// using namespace tesseract_urdf;

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);

  // use the environment locale so that the unit test can be repeated with various locales easily
  setlocale(LC_ALL, "");

  return RUN_ALL_TESTS();
}
