/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2008, Willow Garage, Inc.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the Willow Garage nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

/* Author: Ioan Sucan */

#ifndef TESSERACT_SCENE_GRAPH_MESH_PARSER_H
#define TESSERACT_SCENE_GRAPH_MESH_PARSER_H

#include <tesseract_common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <fstream>

#include <assimp/scene.h>
#include <assimp/Importer.hpp>
#include <assimp/postprocess.h>

#include <console_bridge/console.h>

#include <tesseract_common/types.h>
#include <tesseract_common/resource.h>

#include <regex>

TESSERACT_COMMON_IGNORE_WARNINGS_POP

namespace tesseract_geometry
{
/**
 * The template type must have a constructor as follows
 * Constructor(std::shared_ptr<tesseract_geometry::VectorVector3d> vertices, std::shared_ptr<std::vector<int>> faces,
 * int face_count)
 */

template <class T>
inline std::vector<std::shared_ptr<T>> extractMeshData(const aiScene* scene,
                                                       const aiNode* node,
                                                       const aiMatrix4x4& parent_transform,
                                                       const Eigen::Vector3d& scale,
                                                       tesseract_common::Resource::Ptr resource)
{
  std::vector<std::shared_ptr<T>> meshes;

  aiMatrix4x4 transform = parent_transform;
  transform *= node->mTransformation;
  for (unsigned int j = 0; j < node->mNumMeshes; ++j)
  {
    auto vertices = std::make_shared<tesseract_common::VectorVector3d>();
    auto triangles = std::make_shared<Eigen::VectorXi>();

    const aiMesh* a = scene->mMeshes[node->mMeshes[j]];
    for (unsigned int i = 0; i < a->mNumVertices; ++i)
    {
      aiVector3D v = transform * a->mVertices[i];
      vertices->push_back(Eigen::Vector3d(static_cast<double>(v.x) * scale(0),
                                          static_cast<double>(v.y) * scale(1),
                                          static_cast<double>(v.z) * scale(2)));
    }

    long triangle_count = 0;
    std::vector<int> local_triangles;
    local_triangles.reserve(a->mNumFaces);
    for (unsigned int i = 0; i < a->mNumFaces; ++i)
    {
      if (a->mFaces[i].mNumIndices >= 3)
      {
        triangle_count += 1;
        local_triangles.push_back(static_cast<int>(a->mFaces[i].mNumIndices));
        for (size_t k = 0; k < a->mFaces[i].mNumIndices; ++k)
          local_triangles.push_back(static_cast<int>(a->mFaces[i].mIndices[k]));
      }
      else
      {
        CONSOLE_BRIDGE_logDebug("Mesh had a face with less than three verticies");
      }
    }

    triangles->resize(static_cast<long>(local_triangles.size()));
    for (long i = 0; i < triangles->size(); ++i)
      (*triangles)[i] = local_triangles[static_cast<size_t>(i)];

    meshes.push_back(std::make_shared<T>(vertices, triangles, static_cast<int>(triangle_count), resource, scale));
  }

  for (unsigned int n = 0; n < node->mNumChildren; ++n)
  {
    std::vector<std::shared_ptr<T>> child_meshes =
        extractMeshData<T>(scene, node->mChildren[n], transform, scale, resource);
    meshes.insert(meshes.end(), child_meshes.begin(), child_meshes.end());
  }
  return meshes;
}

/**
 * @brief Create list of meshes from the assimp scene
 * @param scene The assimp scene
 * @param scale Perform an axis scaling
 * @param resource The mesh resource that generated the scene
 * @return A list of tesseract meshes
 */
template <class T>
inline std::vector<std::shared_ptr<T>> createMeshFromAsset(const aiScene* scene,
                                                           const Eigen::Vector3d& scale,
                                                           tesseract_common::Resource::Ptr resource)
{
  if (!scene->HasMeshes())
  {
    CONSOLE_BRIDGE_logWarn("Assimp reports scene in %s has no meshes", resource->getUrl().c_str());
    return std::vector<std::shared_ptr<T>>();
  }
  std::vector<std::shared_ptr<T>> meshes = extractMeshData<T>(scene, scene->mRootNode, aiMatrix4x4(), scale, resource);
  if (meshes.empty())
  {
    CONSOLE_BRIDGE_logWarn("There are no meshes in the scene %s", resource->getUrl().c_str());
    return std::vector<std::shared_ptr<T>>();
  }

  return meshes;
}

/**
 * @brief Create a mesh using assimp from file path
 * @param path The file path to the mesh
 * @param scale Perform an axis scaling
 * @param trianglulate If true the mesh will be trianglulated. This should be done for visual meshes.
 *        In the case of collision meshes do not triangulate convex hull meshes.
 * @param flatten If true all meshes will be condensed into a single mesh. This should only be used for visual meshes,
 * do not flatten collision meshes.
 * @return
 */
template <class T>
inline std::vector<std::shared_ptr<T>> createMeshFromPath(const std::string& path,
                                                          Eigen::Vector3d scale = Eigen::Vector3d(1, 1, 1),
                                                          bool triangulate = false,
                                                          bool flatten = false)
{
  // Create an instance of the Importer class
  Assimp::Importer importer;

  // Issue #38 fix: as part of the post-processing, we remove all other components in file but
  // the meshes, as anyway the resulting shapes:Mesh object just receives vertices and triangles.
  importer.SetPropertyInteger(AI_CONFIG_PP_RVC_FLAGS,
                              aiComponent_NORMALS | aiComponent_TANGENTS_AND_BITANGENTS | aiComponent_COLORS |
                                  aiComponent_TEXCOORDS | aiComponent_BONEWEIGHTS | aiComponent_ANIMATIONS |
                                  aiComponent_TEXTURES | aiComponent_LIGHTS | aiComponent_CAMERAS |
                                  aiComponent_MATERIALS);

  // And have it read the given file with some post-processing
  const aiScene* scene = nullptr;
  if (triangulate)
    scene = importer.ReadFile(path.c_str(),
                              aiProcess_Triangulate | aiProcess_JoinIdenticalVertices | aiProcess_SortByPType |
                                  aiProcess_RemoveComponent);
  else
    scene = importer.ReadFile(path.c_str(),
                              aiProcess_JoinIdenticalVertices | aiProcess_SortByPType | aiProcess_RemoveComponent);

  if (!scene)
    return std::vector<std::shared_ptr<T>>();

  // Assimp enforces Y_UP convention by rotating models with different conventions.
  // However, that behaviour is confusing and doesn't match the ROS convention
  // where the Z axis is pointing up.
  // Hopefully this doesn't undo legit use of the root node transformation...
  // Note that this is also what RViz does internally.
  scene->mRootNode->mTransformation = aiMatrix4x4();

  if (flatten)
  {
    // These post processing steps flatten the root node transformation into child nodes,
    // so they must be delayed until after clearing the root node transform above.
    importer.ApplyPostProcessing(aiProcess_OptimizeMeshes | aiProcess_OptimizeGraph);
  }
  else
  {
    importer.ApplyPostProcessing(aiProcess_OptimizeGraph);
  }

  return createMeshFromAsset<T>(scene, scale, nullptr);
}

/**
 * @brief Create a mesh using assimp from resource
 * @param resource The located resource
 * @param scale Perform an axis scaling
 * @param trianglulate If true the mesh will be trianglulated. This should be done for visual meshes.
 *        In the case of collision meshes do not triangulate convex hull meshes.
 * @param flatten If true all meshes will be condensed into a single mesh. This should only be used for visual meshes,
 * do not flatten collision meshes.
 * @return
 */
template <class T>
inline std::vector<std::shared_ptr<T>> createMeshFromResource(tesseract_common::Resource::Ptr resource,
                                                              Eigen::Vector3d scale = Eigen::Vector3d(1, 1, 1),
                                                              bool triangulate = false,
                                                              bool flatten = false)
{
  if (!resource)
    return std::vector<std::shared_ptr<T>>();

  const char* hint = nullptr;

  std::string resource_url = resource->getUrl();
  std::regex hint_re("^.*\\.([A-Za-z0-9]{1,8})$");
  std::smatch hint_match;
  if (std::regex_match(resource_url, hint_match, hint_re))
  {
    if (hint_match.size() == 2)
    {
      hint = hint_match[1].str().c_str();
    }
  }

  std::vector<uint8_t> data = resource->getResourceContents();

  // Create an instance of the Importer class
  Assimp::Importer importer;

  // Issue #38 fix: as part of the post-processing, we remove all other components in file but
  // the meshes, as anyway the resulting shapes:Mesh object just receives vertices and triangles.
  importer.SetPropertyInteger(AI_CONFIG_PP_RVC_FLAGS,
                              aiComponent_NORMALS | aiComponent_TANGENTS_AND_BITANGENTS | aiComponent_COLORS |
                                  aiComponent_TEXCOORDS | aiComponent_BONEWEIGHTS | aiComponent_ANIMATIONS |
                                  aiComponent_TEXTURES | aiComponent_LIGHTS | aiComponent_CAMERAS |
                                  aiComponent_MATERIALS);

  // And have it read the given file with some post-processing
  const aiScene* scene = nullptr;
  if (triangulate)
    scene = importer.ReadFileFromMemory(&data[0],
                                        data.size(),
                                        aiProcess_Triangulate | aiProcess_JoinIdenticalVertices |
                                            aiProcess_SortByPType | aiProcess_RemoveComponent,
                                        hint);
  else
    scene =
        importer.ReadFileFromMemory(&data[0],
                                    data.size(),
                                    aiProcess_JoinIdenticalVertices | aiProcess_SortByPType | aiProcess_RemoveComponent,
                                    hint);

  if (!scene)
  {
    CONSOLE_BRIDGE_logError(
        "Could not load mesh from \"%s\": %s", resource->getUrl().c_str(), importer.GetErrorString());
    return std::vector<std::shared_ptr<T>>();
  }

  // Assimp enforces Y_UP convention by rotating models with different conventions.
  // However, that behaviour is confusing and doesn't match the ROS convention
  // where the Z axis is pointing up.
  // Hopefully this doesn't undo legit use of the root node transformation...
  // Note that this is also what RViz does internally.
  scene->mRootNode->mTransformation = aiMatrix4x4();

  if (flatten)
  {
    // These post processing steps flatten the root node transformation into child nodes,
    // so they must be delayed until after clearing the root node transform above.
    importer.ApplyPostProcessing(aiProcess_OptimizeMeshes | aiProcess_OptimizeGraph);
  }
  else
  {
    importer.ApplyPostProcessing(aiProcess_OptimizeGraph);
  }

  return createMeshFromAsset<T>(scene, scale, resource);
}

}  // namespace tesseract_geometry
#endif
