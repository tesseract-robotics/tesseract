#include <console_bridge/console.h>
#include <tesseract_scene_graph/graph.h>
#include <tesseract_scene_graph/parser/urdf_parser.h>
#include <tesseract_scene_graph/parser/srdf_parser.h>
#include <tesseract_scene_graph/utils.h>
#include <iostream>

using namespace tesseract_scene_graph;

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
    size_t pos = mod_url.find("/");
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
  // Get the urdf and srdf file paths
  std::string urdf_file = std::string(TESSERACT_SUPPORT_DIR) + "/urdf/lbr_iiwa_14_r820.urdf";
  std::string srdf_file = std::string(TESSERACT_SUPPORT_DIR) + "/urdf/lbr_iiwa_14_r820.srdf";

  // Create scene graph
  ResourceLocatorFn locator = locateResource;
  SceneGraph::Ptr g = parseURDFFile(urdf_file, locator);

  // Parse the srdf
  SRDFModel srdf;
  bool success = srdf.initFile(*g, srdf_file);
  CONSOLE_BRIDGE_logInform("SRDF loaded: %s", toString(success).c_str());

  // Add allowed collision matrix to scene graph
  processSRDFAllowedCollisions(*g, srdf);

  // Get info about allowed collision matrix
  AllowedCollisionMatrix::ConstPtr acm = g->getAllowedCollisionMatrix();
  const AllowedCollisionMatrix::AllowedCollisionEntries& acm_entries = acm->getAllAllowedCollisions();
  CONSOLE_BRIDGE_logInform("ACM Number of entries: %d", acm_entries.size());
}
