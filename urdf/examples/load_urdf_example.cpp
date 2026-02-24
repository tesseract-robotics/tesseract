#include <tesseract/common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <console_bridge/console.h>
#include <tesseract/scene_graph/graph.h>
#include <tesseract/common/utils.h>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract/urdf/urdf_parser.h>
#include <tesseract/common/resource_locator.h>

using namespace tesseract::scene_graph;
using namespace tesseract::urdf;

std::string toString(const ShortestPath& path)
{
  std::stringstream ss;
  ss << path;
  return ss.str();
}

std::string toString(bool b) { return b ? "true" : "false"; }

int main(int /*argc*/, char** /*argv*/)
{
  // documentation:start:2: Get the urdf file path
  tesseract::common::GeneralResourceLocator locator;
  std::string urdf_file =
      locator.locateResource("package://tesseract/support/urdf/lbr_iiwa_14_r820.urdf")->getFilePath();
  // documentation:end:2: Get the urdf file path

  // documentation:start:3: Create scene graph
  SceneGraph::Ptr g = parseURDFFile(urdf_file, locator);
  // documentation:end:3: Create scene graph

  // documentation:start:4: Print information
  CONSOLE_BRIDGE_logInform(std::to_string(g->getJoints().size()).c_str());
  CONSOLE_BRIDGE_logInform(std::to_string(g->getLinks().size()).c_str());
  CONSOLE_BRIDGE_logInform(toString(g->isTree()).c_str());
  CONSOLE_BRIDGE_logInform(toString(g->isAcyclic()).c_str());
  // documentation:end:4: Print information

  // documentation:start:5: Save graph
  g->saveDOT(tesseract::common::getTempPath() + "tesseract_urdf_import.dot");
  // documentation:end:5: Save graph
}
