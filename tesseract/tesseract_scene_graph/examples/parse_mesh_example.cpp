#include <console_bridge/console.h>
#include <tesseract_geometry/impl/mesh.h>
#include <tesseract_scene_graph/parser/mesh_parser.h>
#include <iostream>

using namespace tesseract_scene_graph;

int main(int /*argc*/, char** /*argv*/)
{
  std::string mesh_file = std::string(TESSERACT_SUPPORT_DIR) + "/meshes/sphere_p25m.dae";
  std::vector<tesseract_geometry::Mesh::Ptr> meshes = createMeshFromPath<tesseract_geometry::Mesh>(mesh_file);

  CONSOLE_BRIDGE_logInform("Number of meshes: %f", meshes.size());
  CONSOLE_BRIDGE_logInform("Mesh #1 Triangle Count: %f", meshes[0]->getTriangleCount());
  CONSOLE_BRIDGE_logInform("Mesh #1 Triangle Count: %f", meshes[0]->getVerticeCount());
  CONSOLE_BRIDGE_logInform("Mesh #2 Triangle Count: %f", meshes[1]->getTriangleCount());
  CONSOLE_BRIDGE_logInform("Mesh #2 Triangle Count: %f", meshes[1]->getVerticeCount());
}
