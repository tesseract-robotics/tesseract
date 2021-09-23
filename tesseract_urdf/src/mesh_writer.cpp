#include <tesseract_urdf/mesh_writer.h>

#include <tesseract_common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH

#include <pcl/conversions.h>
#include <pcl/io/vtk_lib_io.h>
#include <pcl/point_cloud.h>

// #include <assimp/Exporter.hpp>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

namespace tesseract_urdf
{
/*
aiScene createAssetFromMesh(const std::shared_ptr<const tesseract_geometry::PolygonMesh>& mesh)
{
  // Create an assimp scene
  aiScene scene;
  scene.mRootNode = new aiNode();

  // Make space for and create a default material
  scene.mMaterials = new aiMaterial*[1];
  scene.mMaterials[0] = new aiMaterial();
  scene.mNumMaterials = 1;

  // Make space for and create a mesh
  scene.mMeshes = new aiMesh*[1];
  scene.mMeshes[0] = new aiMesh();
  scene.mMeshes[0]->mMaterialIndex = 0;
  scene.mNumMeshes = 1;

  // Register the mesh as part of the scene root node
  scene.mRootNode->mMeshes = new unsigned int[1];
  scene.mRootNode->mMeshes[0] = 0;
  scene.mRootNode->mNumMeshes = 1;

  // Transcribe in the mesh vertices
  scene.mMeshes[0]->mVertices = new aiVector3D[static_cast<unsigned long>(mesh->getVertexCount())];
  scene.mMeshes[0]->mNumVertices = static_cast<unsigned int>(mesh->getVertexCount());

  for (std::size_t i = 0; i < static_cast<std::size_t>(mesh->getVertexCount()); ++i)
  {
    scene.mMeshes[0]->mVertices[i] = aiVector3D(static_cast<float>(mesh->getVertices()->at(i).x()),
                                                static_cast<float>(mesh->getVertices()->at(i).y()),
                                                static_cast<float>(mesh->getVertices()->at(i).z()));
  }

  // Transcribe in the mesh triangles
  scene.mMeshes[0]->mFaces = new aiFace[static_cast<unsigned long>(mesh->getFaceCount())];
  scene.mMeshes[0]->mNumFaces = static_cast<unsigned int>(mesh->getFaceCount());

  int indices = 0;
  for (std::size_t i = 0; i < static_cast<std::size_t>(mesh->getFaceCount()) && indices < mesh->getFaces()->size(); ++i)
  {
    // Find and set the number of vertices for this face
    int num_vertices = (*mesh->getFaces())(indices);
    scene.mMeshes[0]->mFaces[i].mIndices = new unsigned int[static_cast<unsigned long>(num_vertices)];
    scene.mMeshes[0]->mFaces[i].mNumIndices = static_cast<unsigned int>(num_vertices);

    // Copy over the index pointing to each vertex
    for (int j = 1; j <= num_vertices; ++j)
      scene.mMeshes[0]->mFaces[i].mIndices[j - 1] = static_cast<unsigned int>((*mesh->getFaces())(indices + j));

    // Move along all the vertex indices just applied
    indices += num_vertices;

    // And move one more to get to the next face-size-indicator
    ++indices;
  }

  // mTextureCoords? mNumUVComponents?

  return scene;
}
*/

pcl::PolygonMesh createPCLMesh(const std::shared_ptr<const tesseract_geometry::PolygonMesh>& mesh)
{
  // Create a PCL polygon mesh
  pcl::PolygonMesh pcl_mesh;

  // Transcribe in the mesh vertices
  pcl::PointCloud<pcl::PointXYZ> cloud;
  cloud.resize(static_cast<unsigned long>(mesh->getVertexCount()));
  for (std::size_t i = 0; i < static_cast<std::size_t>(mesh->getVertexCount()); ++i)
  {
    cloud[i] = pcl::PointXYZ(static_cast<float>(mesh->getVertices()->at(i).x()),
                             static_cast<float>(mesh->getVertices()->at(i).y()),
                             static_cast<float>(mesh->getVertices()->at(i).z()));
  }
  pcl::toPCLPointCloud2(cloud, pcl_mesh.cloud);

  // Transcribe in the mesh polygons
  pcl_mesh.polygons.resize(static_cast<std::size_t>(mesh->getFaceCount()));

  std::size_t indices = 0;
  for (std::size_t i = 0; i < static_cast<std::size_t>(mesh->getFaceCount()) &&
                          indices < static_cast<std::size_t>(mesh->getFaces()->size());
       ++i)
  {
    // Find and set the number of vertices for this face
    int num_vertices = (*mesh->getFaces())(static_cast<Eigen::Index>(indices));
    pcl_mesh.polygons[i].vertices.resize(static_cast<std::size_t>(num_vertices));

    // Copy over the index pointing to each vertex
    for (std::size_t j = 0; j < static_cast<std::size_t>(num_vertices); ++j)
      pcl_mesh.polygons[i].vertices[j] =
          static_cast<unsigned int>((*mesh->getFaces())(static_cast<Eigen::Index>(indices + 1 + j)));

    // Move along all the vertex indices just applied
    indices += static_cast<std::size_t>(num_vertices);

    // And move one more to get to the next face-size-indicator
    ++indices;
  }

  return pcl_mesh;
}

void writeMeshToFile(const std::shared_ptr<const tesseract_geometry::PolygonMesh>& mesh, const std::string& filepath)
{
  pcl::PolygonMesh pcl_mesh = createPCLMesh(mesh);
  if (pcl::io::savePolygonFile(filepath, pcl_mesh) == 0)
    std::throw_with_nested(std::runtime_error("Could not export file"));

  /* Option to use the Assimp code if errors are resolved.
  aiScene scene = createAssetFromMesh(mesh);
  Assimp::Exporter exporter;

  aiReturn ret = exporter.Export(&scene, "pFormatId", filepath.c_str());

  if (ret != AI_SUCCESS)
    std::throw_with_nested(std::runtime_error("Could not export file"));
  */
}
}  // namespace tesseract_urdf
