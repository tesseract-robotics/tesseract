#ifndef TESSERACT_URDF_MESH_WRITER_H
#define TESSERACT_URDF_MESH_WRITER_H

#include <tesseract_common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH

// #include <assimp/scene.h>

TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract_geometry/impl/polygon_mesh.h>

namespace tesseract_urdf
{
/* Commented out due to nebulous errors during initial testing.
 * If errors are resolved, this is probably superior to the writeSimplePLY method used.
aiScene createAssetFromMesh(const std::shared_ptr<const tesseract_geometry::PolygonMesh>& mesh);
*/

void writeMeshToFile(const std::shared_ptr<const tesseract_geometry::PolygonMesh>& mesh, const std::string& filepath);

}  // namespace tesseract_urdf

#endif  // TESSERACT_URDF_MESH_WRITER_H
