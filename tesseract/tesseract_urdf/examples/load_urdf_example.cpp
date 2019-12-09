#include <console_bridge/console.h>
#include <tesseract_scene_graph/graph.h>
#include <tesseract_urdf/urdf_parser.h>
#include <iostream>

using namespace tesseract_scene_graph;
using namespace tesseract_urdf;

std::string toString(const SceneGraph::Path& path)
{
  std::stringstream ss;
  ss << path;
  return ss.str();
}

std::string toString(bool b) { return b ? "true" : "false"; }

// Define a resource locator function
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

int main(int /*argc*/, char** /*argv*/)
{
  // Get the urdf file path
  std::string urdf_file = std::string(TESSERACT_SUPPORT_DIR) + "/urdf/lbr_iiwa_14_r820.urdf";

  // Create scene graph
  tesseract_scene_graph::ResourceLocator::Ptr locator =
      std::make_shared<tesseract_scene_graph::SimpleResourceLocator>(locateResource);
  SceneGraph::Ptr g = parseURDFFile(urdf_file, locator);

  // Print information
  CONSOLE_BRIDGE_logInform(std::to_string(g->getJoints().size()).c_str());
  CONSOLE_BRIDGE_logInform(std::to_string(g->getLinks().size()).c_str());
  CONSOLE_BRIDGE_logInform(toString(g->isTree()).c_str());
  CONSOLE_BRIDGE_logInform(toString(g->isAcyclic()).c_str());

  // Save graph
  g->saveDOT("/tmp/tesseract_urdf_import.dot");
}
