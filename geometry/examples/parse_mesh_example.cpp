#include <console_bridge/console.h>
#include <tesseract/geometry/impl/mesh.h>
#include <tesseract/geometry/mesh_parser.h>
#include <tesseract/common/resource_locator.h>
#include <iostream>

using namespace tesseract::geometry;

int main(int /*argc*/, char** /*argv*/)
{
  // documentation:start:1: Create meshes
  tesseract::common::GeneralResourceLocator locator;
  std::string mesh_file = "package://tesseract/support/meshes/sphere_p25m.dae";
  std::vector<Mesh::Ptr> meshes = createMeshFromPath<Mesh>(locator.locateResource(mesh_file)->getFilePath());
  // documentation:end:1: Create meshes

  // documentation:start:2: Print mesh information
  CONSOLE_BRIDGE_logInform("Number of meshes: %f", meshes.size());
  CONSOLE_BRIDGE_logInform("Mesh #1 Triangle Count: %f", meshes[0]->getFaceCount());
  CONSOLE_BRIDGE_logInform("Mesh #1 Triangle Count: %f", meshes[0]->getVertexCount());
  CONSOLE_BRIDGE_logInform("Mesh #2 Triangle Count: %f", meshes[1]->getFaceCount());
  CONSOLE_BRIDGE_logInform("Mesh #2 Triangle Count: %f", meshes[1]->getVertexCount());
  // documentation:end:2: Print mesh information
}
