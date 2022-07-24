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

/* Updated by John Wason, Wason Technology, LLC, Dec 2020. Add loading mesh normals,
   materials, and textures */

#ifndef TESSERACT_SCENE_GRAPH_MESH_PARSER_H
#define TESSERACT_SCENE_GRAPH_MESH_PARSER_H

#include <tesseract_common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <fstream>

#include <assimp/scene.h>
#include <assimp/Importer.hpp>
#include <assimp/postprocess.h>

#ifdef TESSERACT_ASSIMP_USE_PBRMATERIAL
#include <assimp/pbrmaterial.h>
#endif

#include <console_bridge/console.h>

#include <tesseract_common/types.h>
#include <tesseract_common/resource_locator.h>

#include <regex>
#include <boost/filesystem/path.hpp>

TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract_geometry/impl/mesh_material.h>

namespace tesseract_geometry
{
/**
 * The template type must have a constructor as follows
 * Constructor(std::shared_ptr<tesseract_geometry::VectorVector3d> vertices, std::shared_ptr<std::vector<int>> faces,
 * int face_count)
 */

template <class T>
std::vector<std::shared_ptr<T>> extractMeshData(const aiScene* scene,
                                                const aiNode* node,
                                                const aiMatrix4x4& parent_transform,
                                                const Eigen::Vector3d& scale,
                                                tesseract_common::Resource::Ptr resource,
                                                bool normals,
                                                bool vertex_colors,
                                                bool material_and_texture)
{
  std::vector<std::shared_ptr<T>> meshes;

  aiMatrix4x4 transform = parent_transform;
  transform *= node->mTransformation;
  for (unsigned int j = 0; j < node->mNumMeshes; ++j)
  {
    auto vertices = std::make_shared<tesseract_common::VectorVector3d>();
    auto triangles = std::make_shared<Eigen::VectorXi>();
    std::shared_ptr<tesseract_common::VectorVector3d> vertex_normals = nullptr;
    std::shared_ptr<tesseract_common::VectorVector4d> vertex_colors = nullptr;
    MeshMaterial::Ptr material = nullptr;
    std::shared_ptr<std::vector<MeshTexture::Ptr>> textures = nullptr;

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
        CONSOLE_BRIDGE_logDebug("Mesh had a face with less than three vertices: %s", resource->getUrl().c_str());
      }
    }

    triangles->resize(static_cast<long>(local_triangles.size()));
    for (long i = 0; i < triangles->size(); ++i)
      (*triangles)[i] = local_triangles[static_cast<size_t>(i)];

    if (normals && a->HasNormals())
    {
      vertex_normals = std::make_shared<tesseract_common::VectorVector3d>();
      for (unsigned int i = 0; i < a->mNumVertices; ++i)
      {
        aiVector3D v = transform * a->mNormals[i];
        vertex_normals->push_back(Eigen::Vector3d(static_cast<double>(v.x) * scale(0),
                                                  static_cast<double>(v.y) * scale(1),
                                                  static_cast<double>(v.z) * scale(2)));
      }
    }

    if (vertex_colors && a->HasVertexColors(0))
    {
      vertex_colors = std::make_shared<tesseract_common::VectorVector4d>();
      for (unsigned int i = 0; i < a->mNumVertices; ++i)
      {
        aiColor4D v = a->mColors[0][i];
        vertex_colors->push_back(Eigen::Vector4d(
            static_cast<double>(v.r), static_cast<double>(v.g), static_cast<double>(v.b), static_cast<double>(v.a)));
      }
    }

    if (material_and_texture)
    {
      aiMaterial* mat = scene->mMaterials[a->mMaterialIndex];
      {
        Eigen::Vector4d base_color;
        double metallic = 0.0;
        double roughness = 0.5;
        Eigen::Vector4d emissive;

        aiColor4D pbr_base_color;
#ifdef TESSERACT_ASSIMP_USE_PBRMATERIAL
        if (mat->Get(AI_MATKEY_GLTF_PBRMETALLICROUGHNESS_BASE_COLOR_FACTOR, pbr_base_color) == AI_SUCCESS)
        {
          // Use PBR Metallic material properties if available
          base_color = Eigen::Vector4d(pbr_base_color.r, pbr_base_color.g, pbr_base_color.b, pbr_base_color.a);
          float metallicFactor{ 0 };
          if (mat->Get(AI_MATKEY_GLTF_PBRMETALLICROUGHNESS_METALLIC_FACTOR, metallicFactor) == AI_SUCCESS)
          {
            metallic = metallicFactor;
          }
          float roughnessFactor{ 0.5 };
          if (mat->Get(AI_MATKEY_GLTF_PBRMETALLICROUGHNESS_ROUGHNESS_FACTOR, roughnessFactor) == AI_SUCCESS)
          {
            roughness = roughnessFactor;
          }
          aiColor4D pbr_emissive_color;
          if (mat->Get(AI_MATKEY_COLOR_EMISSIVE, pbr_emissive_color) == AI_SUCCESS)
          {
            emissive =
                Eigen::Vector4d(pbr_emissive_color.r, pbr_emissive_color.g, pbr_emissive_color.b, pbr_emissive_color.a);
          }
        }
        else
#endif
        {
          // Use old style material. Ambient and specular not supported
          aiColor4D diffuse_color;
          if (mat->Get(AI_MATKEY_COLOR_DIFFUSE, diffuse_color) == AI_SUCCESS)
          {
            base_color = Eigen::Vector4d(diffuse_color.r, diffuse_color.g, diffuse_color.b, diffuse_color.a);
          }

          aiColor4D emissive_color;
          if (mat->Get(AI_MATKEY_COLOR_EMISSIVE, emissive_color) == AI_SUCCESS)
          {
            emissive = Eigen::Vector4d(emissive_color.r, emissive_color.g, emissive_color.b, emissive_color.a);
          }
        }

        material = std::make_shared<MeshMaterial>(base_color, metallic, roughness, emissive);

        for (unsigned int i = 0; i < a->GetNumUVChannels(); i++)
        {
          if (a->HasTextureCoords(i))
          {
            aiString texName;
            aiTextureMapping mapping{ aiTextureMapping_OTHER };
            unsigned int uvIndex{ 0 };
            if (mat->GetTexture(aiTextureType_DIFFUSE, i, &texName, &mapping, &uvIndex) == AI_SUCCESS)
            {
              tesseract_common::Resource::Ptr texture_image;
              tesseract_common::VectorVector2d uvs;
              // https://stackoverflow.com/questions/56820244/assimp-doenst-return-texture-data
              const char* texNamec = texName.C_Str();
              if ('*' == *texNamec)
              {
                int tex_index = std::atoi(texNamec + 1);
                if (0 > tex_index || scene->mNumTextures <= static_cast<unsigned>(tex_index))
                  continue;
                auto* texture_data = scene->mTextures[tex_index];
                // returned pointer is not null, read texture from memory
                std::string file_type = texture_data->achFormatHint;
                if (file_type == "jpg" || file_type == "png")
                {
                  texture_image = std::make_shared<tesseract_common::BytesResource>(
                      "data://", (const uint8_t*)texture_data->pcData, texture_data->mWidth);  // NOLINT
                }
                else
                {
                  // TODO: handle other file types
                  continue;
                }
              }
              else
              {
                if (!resource)
                {
                  continue;
                }
                std::string texName_str(texName.C_Str());
                auto tex_resource = resource->locateResource(texName_str);
                if (!tex_resource)
                {
                  continue;
                }
                texture_image = tex_resource;
              }
              aiVector3D* tex_coords = a->mTextureCoords[i];
              for (unsigned int j = 0; j < a->mNumVertices; ++j)
              {
                aiVector3D v = tex_coords[j];
                uvs.push_back(Eigen::Vector2d(static_cast<double>(v.x), static_cast<double>(v.y)));
              }
              auto tex = std::make_shared<MeshTexture>(
                  texture_image, std::make_shared<tesseract_common::VectorVector2d>(std::move(uvs)));
              if (!textures)
              {
                textures = std::make_shared<std::vector<MeshTexture::Ptr>>();
              }
              textures->push_back(tex);
            }
          }
        }
      }
    }

    meshes.push_back(std::make_shared<T>(vertices,
                                         triangles,
                                         static_cast<int>(triangle_count),
                                         resource,
                                         scale,
                                         vertex_normals,
                                         vertex_colors,
                                         material,
                                         textures));
  }

  for (unsigned int n = 0; n < node->mNumChildren; ++n)
  {
    std::vector<std::shared_ptr<T>> child_meshes = extractMeshData<T>(
        scene, node->mChildren[n], transform, scale, resource, normals, vertex_colors, material_and_texture);
    meshes.insert(meshes.end(), child_meshes.begin(), child_meshes.end());
  }
  return meshes;
}

/**
 * @brief Create list of meshes from the assimp scene
 * @param scene The assimp scene
 * @param scale Perform an axis scaling
 * @param resource The mesh resource that generated the scene
 * @param normals If true, loads mesh normals
 * @param vertex_colors If true, loads mesh vertex colors
 * @param material_and_texture If true, loads mesh materials and textures
 * @return A list of tesseract meshes
 */
template <class T>
std::vector<std::shared_ptr<T>> createMeshFromAsset(const aiScene* scene,
                                                    const Eigen::Vector3d& scale,
                                                    tesseract_common::Resource::Ptr resource,
                                                    bool normals,
                                                    bool vertex_colors,
                                                    bool material_and_texture)
{
  if (!scene->HasMeshes())
  {
    CONSOLE_BRIDGE_logWarn("Assimp reports scene in %s has no meshes", resource->getUrl().c_str());
    return std::vector<std::shared_ptr<T>>();
  }
  std::vector<std::shared_ptr<T>> meshes = extractMeshData<T>(
      scene, scene->mRootNode, aiMatrix4x4(), scale, resource, normals, vertex_colors, material_and_texture);
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
 * @param triangulate If true the mesh will be triangulated. This should be done for visual meshes.
 *        In the case of collision meshes do not triangulate convex hull meshes.
 * @param flatten If true all meshes will be condensed into a single mesh. This should only be used for visual meshes,
 * do not flatten collision meshes.
 * @param normals If true, loads mesh normals
 * @param vertex_colors If true, loads mesh vertex colors
 * @param material_and_texture If true, loads mesh materials and textures
 * @return
 */
template <class T>
std::vector<std::shared_ptr<T>> createMeshFromPath(const std::string& path,
                                                   Eigen::Vector3d scale = Eigen::Vector3d(1, 1, 1),
                                                   bool triangulate = false,
                                                   bool flatten = false,
                                                   bool normals = false,
                                                   bool vertex_colors = false,
                                                   bool material_and_texture = false)
{
  // Create an instance of the Importer class
  Assimp::Importer importer;

  // Issue #38 fix: as part of the post-processing, we remove all other components in file but
  // the meshes, as anyway the resulting shapes:Mesh object just receives vertices and triangles.
  // John Wason Jan 2021 - Adjust flags based on normals, vertex_colors, and material_and_texture parameters
  unsigned int ai_config_pp_rcv_flags = aiComponent_TANGENTS_AND_BITANGENTS | aiComponent_BONEWEIGHTS |
                                        aiComponent_ANIMATIONS | aiComponent_LIGHTS | aiComponent_CAMERAS;
  if (!normals)
  {
    ai_config_pp_rcv_flags |= aiComponent_NORMALS;
  }
  if (!vertex_colors)
  {
    ai_config_pp_rcv_flags |= aiComponent_COLORS;
  }
  if (!material_and_texture)
  {
    ai_config_pp_rcv_flags |= aiComponent_TEXCOORDS | aiComponent_TEXTURES | aiComponent_MATERIALS;
  }
  importer.SetPropertyInteger(AI_CONFIG_PP_RVC_FLAGS, (int)ai_config_pp_rcv_flags);

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
  {
    CONSOLE_BRIDGE_logError("Could not load mesh from \"%s\": %s", path.c_str(), importer.GetErrorString());
    return std::vector<std::shared_ptr<T>>();
  }

  // Assimp enforces Y_UP convention by rotating models with different conventions.
  // However, that behavior is confusing and doesn't match the ROS convention
  // where the Z axis is pointing up.
  // Hopefully this doesn't undo legit use of the root node transformation...
  // Note that this is also what RViz does internally.
  // @todo See this issue for possible fix parsing scene metadata https://github.com/assimp/assimp/issues/849
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

  return createMeshFromAsset<T>(scene, scale, nullptr, normals, vertex_colors, material_and_texture);
}

/**
 * @brief Create a mesh using assimp from resource
 * @param resource The located resource
 * @param scale Perform an axis scaling
 * @param triangulate If true the mesh will be triangulated. This should be done for visual meshes.
 *        In the case of collision meshes do not triangulate convex hull meshes.
 * @param flatten If true all meshes will be condensed into a single mesh. This should only be used for visual meshes,
 * do not flatten collision meshes.
 * @param normals If true, loads mesh normals
 * @param vertex_colors If true, loads mesh vertex colors
 * @param material_and_texture If true, loads mesh materials and textures
 * @return
 */
template <class T>
std::vector<std::shared_ptr<T>> createMeshFromResource(tesseract_common::Resource::Ptr resource,
                                                       Eigen::Vector3d scale = Eigen::Vector3d(1, 1, 1),
                                                       bool triangulate = false,
                                                       bool flatten = false,
                                                       bool normals = false,
                                                       bool vertex_colors = false,
                                                       bool material_and_texture = false)
{
  if (!resource)
    return std::vector<std::shared_ptr<T>>();

  const char* hint = nullptr;
  std::string hint_storage;

  std::string resource_url = resource->getUrl();
  std::regex hint_re("^.*\\.([A-Za-z0-9]{1,8})$");
  std::smatch hint_match;
  if (std::regex_match(resource_url, hint_match, hint_re))
  {
    if (hint_match.size() == 2)
    {
      hint_storage = hint_match[1].str();
      hint = hint_storage.c_str();
    }
  }

  std::vector<uint8_t> data = resource->getResourceContents();
  if (data.empty())
  {
    if (resource->isFile())
      return createMeshFromPath<T>(
          resource->getFilePath(), scale, triangulate, flatten, normals, vertex_colors, material_and_texture);

    return std::vector<std::shared_ptr<T>>();
  }

  // Create an instance of the Importer class
  Assimp::Importer importer;

  // Issue #38 fix: as part of the post-processing, we remove all other components in file but
  // the meshes, as anyway the resulting shapes:Mesh object just receives vertices and triangles.
  // John Wason Jan 2021 - Adjust flags based on normals, vertex_colors, and material_and_texture parameters
  unsigned int ai_config_pp_rcv_flags = aiComponent_TANGENTS_AND_BITANGENTS | aiComponent_BONEWEIGHTS |
                                        aiComponent_ANIMATIONS | aiComponent_LIGHTS | aiComponent_CAMERAS;
  if (!normals)
  {
    ai_config_pp_rcv_flags |= aiComponent_NORMALS;
  }
  if (!vertex_colors)
  {
    ai_config_pp_rcv_flags |= aiComponent_COLORS;
  }
  if (!material_and_texture)
  {
    ai_config_pp_rcv_flags |= aiComponent_TEXCOORDS | aiComponent_TEXTURES | aiComponent_MATERIALS;
  }
  importer.SetPropertyInteger(AI_CONFIG_PP_RVC_FLAGS, (int)ai_config_pp_rcv_flags);

  // And have it read the given file with some post-processing
  const aiScene* scene = nullptr;
  if (triangulate)
    scene = importer.ReadFileFromMemory(data.data(),
                                        data.size(),
                                        aiProcess_Triangulate | aiProcess_JoinIdenticalVertices |
                                            aiProcess_SortByPType | aiProcess_RemoveComponent,
                                        hint);
  else
    scene =
        importer.ReadFileFromMemory(data.data(),
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
  // However, that behavior is confusing and doesn't match the ROS convention
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

  return createMeshFromAsset<T>(scene, scale, resource, normals, vertex_colors, material_and_texture);
}

/**
 * @brief Create a mesh from byte array
 * @param url The URL of source resource
 * @param bytes Byte array
 * @param bytes_len The length of bytes
 * @param scale Perform an axis scaling
 * @param triangulate If true the mesh will be triangulated. This should be done for visual meshes.
 *        In the case of collision meshes do not triangulate convex hull meshes.
 * @param flatten If true all meshes will be condensed into a single mesh. This should only be used for visual meshes,
 * do not flatten collision meshes.
 * @param normals If true, loads mesh normals
 * @param vertex_colors If true, loads mesh vertex colors
 * @param material_and_texture If true, loads mesh materials and textures
 * @return
 */
template <typename T>
static std::vector<std::shared_ptr<T>> createMeshFromBytes(const std::string& url,
                                                           const uint8_t* bytes,
                                                           size_t bytes_len,
                                                           Eigen::Vector3d scale = Eigen::Vector3d(1, 1, 1),
                                                           bool triangulate = false,
                                                           bool flatten = false,
                                                           bool normals = false,
                                                           bool vertex_colors = false,
                                                           bool material_and_texture = false)
{
  std::shared_ptr<tesseract_common::Resource> resource =
      std::make_shared<tesseract_common::BytesResource>(url, bytes, bytes_len);
  return tesseract_geometry::createMeshFromResource<T>(
      resource, scale, triangulate, flatten, normals, vertex_colors, material_and_texture);
}

}  // namespace tesseract_geometry

#endif
