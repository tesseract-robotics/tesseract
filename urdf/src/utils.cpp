#include <tesseract/common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
// #include <assimp/Exporter.hpp>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract/common/ply_io.h>
#include <tesseract/geometry/impl/polygon_mesh.h>
#include <tesseract/urdf/utils.h>

namespace tesseract::urdf
{
std::string toString(const double& float_value, const int precision)
{
  std::stringstream sstring;
  sstring.precision(precision);
  sstring << float_value;
  return sstring.str();
}

std::string trailingSlash(const std::string& path)
{
  std::string ret;
  if (path.empty())
    ret = "/";
  else
  {
    if (path.back() == '/')
      ret = path;
    else
      ret = path + "/";
  }
  return ret;
}

std::string noTrailingSlash(const std::string& path)
{
  std::string ret = path;
  while (!ret.empty() && (ret.back() == '/' || ret.back() == '\\'))
  {
    ret = ret.substr(0, ret.size() - 1);
  }
  return ret;
}

std::string noLeadingSlash(const std::string& filename)
{
  std::string ret = filename;
  while (!ret.empty() && (ret.front() == '/' || ret.front() == '\\'))
  {
    ret = ret.substr(1);  // from second char to end
  }
  return ret;
}

std::string makeURDFFilePath(const std::string& package_path, const std::string& filename)
{
  std::string ret;
  if (!package_path.empty())
  {
    // Use a package-relative path if a package was specified

    // Extract package name
    std::string package_name = noTrailingSlash(package_path);
    package_name = package_name.substr(package_name.find_last_of("/\\") + 1);

    // Set the path to the file
    ret = "package://" + trailingSlash(package_name) + noLeadingSlash(filename);
  }
  else
  {
    // Use an absolute path if no package was specified
    ret = filename;
  }
  return ret;
}

/*
aiScene createAssetFromMesh(const std::shared_ptr<const tesseract::geometry::PolygonMesh>& mesh)
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

void writeMeshToFile(const std::shared_ptr<const tesseract::geometry::PolygonMesh>& mesh, const std::string& filepath)
{
  if (!tesseract::common::writeSimplePlyFile(
          filepath, *(mesh->getVertices()), *(mesh->getFaces()), mesh->getFaceCount()))
    std::throw_with_nested(std::runtime_error("Could not export file"));

  /* Option to use the Assimp code if errors are resolved.
  aiScene scene = createAssetFromMesh(mesh);
  Assimp::Exporter exporter;

  aiReturn ret = exporter.Export(&scene, "pFormatId", filepath.c_str());

  if (ret != AI_SUCCESS)
    std::throw_with_nested(std::runtime_error("Could not export file"));
  */
}

}  // namespace tesseract::urdf
