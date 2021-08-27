#ifndef TESSERACT_SCENE_GRAPH_MESH_WRITER_H
#define TESSERACT_SCENE_GRAPH_MESH_WRITER_H

#include <tesseract_common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH

#include <assimp/scene.h>
#include <assimp/Exporter.hpp>

TESSERACT_COMMON_IGNORE_WARNINGS_POP

namespace tesseract_urdf
{

template <class T>
aiScene createAssetFromMesh(const std::shared_ptr<const T>& mesh)
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
  scene.mMeshes[0]->mVertices = new aiVector3D[mesh->getVertexCount()];
  scene.mMeshes[0]->mNumVertices = static_cast<unsigned int>(mesh->getVertexCount());

  for (std::size_t i = 0; i < mesh->getVertexCount(); ++i)
  {
    scene.mMeshes[0]->mVertices[i] = aiVector3D(
          static_cast<float>(mesh->getVertices()->at(i).x()),
          static_cast<float>(mesh->getVertices()->at(i).y()),
          static_cast<float>(mesh->getVertices()->at(i).z()));
  }

  // Transcribe in the mesh triangles
  scene.mMeshes[0]->mFaces = new aiFace[mesh->getFaceCount()];
  scene.mMeshes[0]->mNumFaces = static_cast<unsigned int>(mesh->getFaceCount());

  int indices = 0;
  for (std::size_t i = 0; i < mesh->getFaceCount() && indices < mesh->getFaces()->size(); ++i)
  {
    // Find and set the number of vertices for this face
    int num_vertices = (*mesh->getFaces())(indices);
    scene.mMeshes[0]->mFaces[i].mIndices = new unsigned int[num_vertices];
    scene.mMeshes[0]->mFaces[i].mNumIndices = num_vertices;

    // Copy over the index pointing to each vertex
    for (int j = 1; j <= num_vertices; ++j)
      scene.mMeshes[0]->mFaces[i].mIndices[j] = static_cast<unsigned int>((*mesh->getFaces())(indices + j));

    // Move along all the vertex indices just applied
    indices += num_vertices;

    // And move one more to get to the next face-size-indicator
    ++indices;
  }

  // mTextureCoords? mNumUVComponents?

  return scene;
}

template <class T>
void writeMeshToFile(const std::shared_ptr<const T>& mesh,
                     const std::string& filepath)
{
  aiScene scene = createAssetFromMesh(mesh);
  Assimp::Exporter exporter;

  aiReturn ret = exporter.Export(&scene, "pFormatId", filepath.c_str());

  if (ret != AI_SUCCESS)
    std::throw_with_nested(std::runtime_error("Could not export file"));
  return;
}

} // namespace tesseract_urdf

#ifdef SWIG
%pybuffer_binary(const uint8_t* bytes, size_t bytes_len);
%template(createAssetFromMesh) tesseract_urdf::createAssetFromMesh<tesseract_geometry::Mesh>;
%template(createAssetFromSDFMesh) tesseract_urdf::createAssetFromMesh<tesseract_geometry::SDFMesh>;
%template(createAssetFromConvexMesh) tesseract_urdf::createAssetFromMesh<tesseract_geometry::ConvexMesh>;
%template(writeMeshToFile) tesseract_urdf::writeMeshToFile<tesseract_geometry::Mesh>;
%template(writeSDFMeshToFile) tesseract_urdf::writeMeshToFile<tesseract_geometry::SDFMesh>;
%template(writeConvexMeshToFile) tesseract_urdf::writeMeshToFile<tesseract_geometry::ConvexMesh>;
#endif

#endif // TESSERACT_SCENE_GRAPH_MESH_WRITER_H
