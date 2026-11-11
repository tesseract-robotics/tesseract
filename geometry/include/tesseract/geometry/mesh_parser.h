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

#include <tesseract/common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <assimp/scene.h>
#include <assimp/Importer.hpp>
#include <assimp/postprocess.h>

#ifdef TESSERACT_ASSIMP_USE_PBRMATERIAL
#include <assimp/pbrmaterial.h>
#endif

#include <tesseract/common/logging.h>
#include <tesseract/common/types.h>
#include <tesseract/common/resource_locator.h>

#include <map>
#include <regex>
#include <tuple>
#include <utility>
#include <boost/filesystem/path.hpp>

TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract/geometry/impl/mesh_material.h>

namespace tesseract::geometry
{
namespace detail
{
/** @brief Convert an assimp texture map mode to a MeshTextureWrapMode */
inline MeshTextureWrapMode toMeshTextureWrapMode(int mode)
{
  switch (mode)
  {
    case aiTextureMapMode_Clamp:
    case aiTextureMapMode_Decal:
      return MeshTextureWrapMode::CLAMP_TO_EDGE;
    case aiTextureMapMode_Mirror:
      return MeshTextureWrapMode::MIRRORED_REPEAT;
    default:
      return MeshTextureWrapMode::REPEAT;
  }
}

/** @brief Convert an OpenGL / glTF 2.0 sampler filter enum to a MeshTextureFilter */
inline MeshTextureFilter toMeshTextureFilter(int gl_filter)
{
  switch (gl_filter)
  {
    case 9728:  // NEAREST
      return MeshTextureFilter::NEAREST;
    case 9729:  // LINEAR
      return MeshTextureFilter::LINEAR;
    case 9984:  // NEAREST_MIPMAP_NEAREST
      return MeshTextureFilter::NEAREST_MIPMAP_NEAREST;
    case 9985:  // LINEAR_MIPMAP_NEAREST
      return MeshTextureFilter::LINEAR_MIPMAP_NEAREST;
    case 9986:  // NEAREST_MIPMAP_LINEAR
      return MeshTextureFilter::NEAREST_MIPMAP_LINEAR;
    case 9987:  // LINEAR_MIPMAP_LINEAR
      return MeshTextureFilter::LINEAR_MIPMAP_LINEAR;
    default:
      return MeshTextureFilter::DEFAULT;
  }
}

/**
 * @brief Load the image of a texture referenced by a material
 * @param scene The assimp scene, which holds embedded textures
 * @param tex_path The texture path from the material. Embedded textures are "*<index>" or, for glTF, the image name.
 * @param resource The mesh resource, used to locate external texture files. May be null.
 * @return The image, or null if it is missing or not a png or jpg
 */
inline tesseract::common::Resource::Ptr loadTextureImage(const aiScene* scene,
                                                         const aiString& tex_path,
                                                         const tesseract::common::Resource::Ptr& resource)
{
  // https://stackoverflow.com/questions/56820244/assimp-doenst-return-texture-data
  const aiTexture* texture_data = scene->GetEmbeddedTexture(tex_path.C_Str());
  if (texture_data != nullptr)
  {
    // mHeight == 0 means pcData holds a compressed image (png, jpg...) of mWidth bytes. Otherwise it is raw texels.
    const std::string file_type = texture_data->achFormatHint;
    if (texture_data->mHeight != 0 || (file_type != "jpg" && file_type != "jpeg" && file_type != "png"))
    {
      TESSERACT_LOG_DEBUG("Embedded mesh texture '{}' with format '{}' is not supported",
                          tex_path.C_Str(),
                          texture_data->mHeight != 0 ? "raw" : file_type);
      return nullptr;
    }
    const std::string ext = (file_type == "png") ? ".png" : ".jpg";
    std::string name = tex_path.C_Str();
    if (!name.empty() && name.front() == '*')
      name = "texture_" + name.substr(1);
    return std::make_shared<tesseract::common::BytesResource>("data://" + name + ext,
                                                              reinterpret_cast<const uint8_t*>(texture_data->pcData),
                                                              texture_data->mWidth);  // NOLINT
  }

  if (!resource || tex_path.length == 0 || tex_path.C_Str()[0] == '*')
    return nullptr;

  return resource->locateResource(tex_path.C_Str());
}

/**
 * @brief Extract the PBR material and the textures of an assimp mesh
 * @details Reads the glTF 2.0 metallic-roughness material when the importer provides one, and falls back to the
 * diffuse and emissive colors otherwise. One MeshTexture is created for each supported texture slot that has an
 * image: base color, metallic-roughness, normal, occlusion and emissive. Textures that use the same UV channel
 * share a single UV vector.
 */
inline std::pair<MeshMaterial::Ptr, std::shared_ptr<std::vector<MeshTexture::Ptr>>>
extractMeshMaterialAndTextures(const aiScene* scene, const aiMesh* a, const tesseract::common::Resource::Ptr& resource)
{
  std::shared_ptr<std::vector<MeshTexture::Ptr>> textures = nullptr;
  if (scene->mMaterials == nullptr || a->mMaterialIndex >= scene->mNumMaterials)
    return { nullptr, nullptr };

  const aiMaterial* mat = scene->mMaterials[a->mMaterialIndex];
  auto material = std::make_shared<MeshMaterial>();

  aiColor4D color;
  bool is_pbr = false;
#ifdef TESSERACT_ASSIMP_USE_PBRMATERIAL
  if (mat->Get(AI_MATKEY_GLTF_PBRMETALLICROUGHNESS_BASE_COLOR_FACTOR, color) == AI_SUCCESS)
  {
    // Use PBR Metallic material properties if available
    is_pbr = true;
    material->setBaseColorFactor(Eigen::Vector4d(color.r, color.g, color.b, color.a));
    float value{ 0 };
    if (mat->Get(AI_MATKEY_GLTF_PBRMETALLICROUGHNESS_METALLIC_FACTOR, value) == AI_SUCCESS)
      material->setMetallicFactor(value);
    if (mat->Get(AI_MATKEY_GLTF_PBRMETALLICROUGHNESS_ROUGHNESS_FACTOR, value) == AI_SUCCESS)
      material->setRoughnessFactor(value);

    aiString alpha_mode;
    if (mat->Get(AI_MATKEY_GLTF_ALPHAMODE, alpha_mode) == AI_SUCCESS)
    {
      const std::string mode = alpha_mode.C_Str();
      if (mode == "MASK")
        material->setAlphaMode(MeshMaterialAlphaMode::MASK);
      else if (mode == "BLEND")
        material->setAlphaMode(MeshMaterialAlphaMode::BLEND);
    }
    if (mat->Get(AI_MATKEY_GLTF_ALPHACUTOFF, value) == AI_SUCCESS)
      material->setAlphaCutoff(value);
#ifdef AI_MATKEY_EMISSIVE_INTENSITY
    if (mat->Get(AI_MATKEY_EMISSIVE_INTENSITY, value) == AI_SUCCESS)
      material->setEmissiveStrength(value);
#endif
#ifdef AI_MATKEY_GLTF_TEXTURE_SCALE
    if (mat->Get(AI_MATKEY_GLTF_TEXTURE_SCALE(aiTextureType_NORMALS, 0), value) == AI_SUCCESS)
      material->setNormalScale(value);
#endif
#ifdef AI_MATKEY_GLTF_TEXTURE_STRENGTH
    if (mat->Get(AI_MATKEY_GLTF_TEXTURE_STRENGTH(aiTextureType_LIGHTMAP, 0), value) == AI_SUCCESS)
      material->setOcclusionStrength(value);
    // Some assimp versions (5.3) write the occlusion strength under a different key than the one they define
    else if (mat->Get("$tex.file.strength", aiTextureType_LIGHTMAP, 0, value) == AI_SUCCESS)
      material->setOcclusionStrength(value);
#endif
  }
  else
#endif
  {
    // Use old style material. Ambient and specular not supported
    Eigen::Vector4d base_color = Eigen::Vector4d::Zero();
    if (mat->Get(AI_MATKEY_COLOR_DIFFUSE, color) == AI_SUCCESS)
      base_color = Eigen::Vector4d(color.r, color.g, color.b, color.a);
    float opacity{ 1 };
    if (mat->Get(AI_MATKEY_OPACITY, opacity) == AI_SUCCESS && opacity < 1.0F)
      base_color[3] *= opacity;
    material->setBaseColorFactor(base_color);
    // These formats have no alpha mode, so a translucent material is blended
    if (base_color[3] < 1.0)
      material->setAlphaMode(MeshMaterialAlphaMode::BLEND);
  }

  if (mat->Get(AI_MATKEY_COLOR_EMISSIVE, color) == AI_SUCCESS)
    material->setEmissiveFactor(Eigen::Vector4d(color.r, color.g, color.b, color.a));

  int two_sided{ 0 };
  if (mat->Get(AI_MATKEY_TWOSIDED, two_sided) == AI_SUCCESS)
    material->setDoubleSided(two_sided != 0);

  // The assimp texture slot for each glTF 2.0 texture. Formats without PBR only have a diffuse texture.
  struct Slot
  {
    MeshTextureType type;
    aiTextureType ai_type;
    unsigned ai_index;
  };
  std::vector<Slot> slots;
  if (is_pbr)
  {
    slots.push_back({ MeshTextureType::BASE_COLOR, aiTextureType_DIFFUSE, 0 });
#ifdef AI_MATKEY_GLTF_PBRMETALLICROUGHNESS_METALLICROUGHNESS_TEXTURE
    // The slot used for the metallic-roughness texture differs between assimp versions
    // The macro expands to "<texture type>, <index>"
    slots.push_back(
        { MeshTextureType::METALLIC_ROUGHNESS, AI_MATKEY_GLTF_PBRMETALLICROUGHNESS_METALLICROUGHNESS_TEXTURE });
#endif
    slots.push_back({ MeshTextureType::NORMAL, aiTextureType_NORMALS, 0 });
    slots.push_back({ MeshTextureType::OCCLUSION, aiTextureType_LIGHTMAP, 0 });
    slots.push_back({ MeshTextureType::EMISSIVE, aiTextureType_EMISSIVE, 0 });
  }
  else
  {
    slots.push_back({ MeshTextureType::BASE_COLOR, aiTextureType_DIFFUSE, 0 });
  }

  // UV vectors already extracted, by UV channel, so textures on the same channel share them
  std::map<unsigned, std::shared_ptr<const tesseract::common::VectorVector2d>> uv_channels;

  for (const auto& slot : slots)
  {
    if (mat->GetTextureCount(slot.ai_type) <= slot.ai_index)
      continue;

    aiString tex_path;
    aiTextureMapping mapping{ aiTextureMapping_UV };
    unsigned int uv_index{ 0 };
    if (mat->GetTexture(slot.ai_type, slot.ai_index, &tex_path, &mapping, &uv_index) != AI_SUCCESS)
      continue;

    if (uv_index >= AI_MAX_NUMBER_OF_TEXTURECOORDS || !a->HasTextureCoords(uv_index))
    {
      TESSERACT_LOG_DEBUG(
          "Mesh texture '{}' uses UV channel {}, which the mesh does not have", tex_path.C_Str(), uv_index);
      continue;
    }

    auto texture_image = loadTextureImage(scene, tex_path, resource);
    if (!texture_image)
      continue;

    auto& uvs = uv_channels[uv_index];
    if (!uvs)
    {
      auto channel = std::make_shared<tesseract::common::VectorVector2d>();
      channel->reserve(a->mNumVertices);
      const aiVector3D* tex_coords = a->mTextureCoords[uv_index];
      for (unsigned int k = 0; k < a->mNumVertices; ++k)
        channel->emplace_back(static_cast<double>(tex_coords[k].x), static_cast<double>(tex_coords[k].y));
      uvs = channel;
    }

    auto tex = std::make_shared<MeshTexture>(texture_image, uvs, slot.type, uv_index);

    int map_u{ aiTextureMapMode_Wrap };
    int map_v{ aiTextureMapMode_Wrap };
    mat->Get(AI_MATKEY_MAPPINGMODE_U(slot.ai_type, slot.ai_index), map_u);
    mat->Get(AI_MATKEY_MAPPINGMODE_V(slot.ai_type, slot.ai_index), map_v);
    tex->setWrap(toMeshTextureWrapMode(map_u), toMeshTextureWrapMode(map_v));
#if defined(TESSERACT_ASSIMP_USE_PBRMATERIAL) && defined(AI_MATKEY_GLTF_MAPPINGFILTER_MAG)
    int mag_filter{ 0 };
    int min_filter{ 0 };
    mat->Get(AI_MATKEY_GLTF_MAPPINGFILTER_MAG(slot.ai_type, slot.ai_index), mag_filter);
    mat->Get(AI_MATKEY_GLTF_MAPPINGFILTER_MIN(slot.ai_type, slot.ai_index), min_filter);
    tex->setFilters(toMeshTextureFilter(mag_filter), toMeshTextureFilter(min_filter));
#endif

    if (!textures)
      textures = std::make_shared<std::vector<MeshTexture::Ptr>>();
    textures->push_back(tex);
  }

  return { material, textures };
}
}  // namespace detail

/**
 * The template type must have a constructor as follows
 * Constructor(std::shared_ptr<tesseract::geometry::VectorVector3d> vertices, std::shared_ptr<std::vector<int>> faces,
 * int face_count)
 */

template <class T>
std::vector<std::shared_ptr<T>> extractMeshData(const aiScene* scene,
                                                const aiNode* node,
                                                const aiMatrix4x4& parent_transform,
                                                const Eigen::Vector3d& scale,
                                                tesseract::common::Resource::Ptr resource,
                                                bool normals,
                                                bool vertex_colors,
                                                bool material_and_texture)
{
  std::vector<std::shared_ptr<T>> meshes;
  meshes.reserve(node->mNumMeshes);

  aiMatrix4x4 transform = parent_transform;
  transform *= node->mTransformation;
  for (unsigned int j = 0; j < node->mNumMeshes; ++j)
  {
    auto vertices = std::make_shared<tesseract::common::VectorVector3d>();
    auto triangles = std::make_shared<Eigen::VectorXi>();
    std::shared_ptr<tesseract::common::VectorVector3d> vertex_normals = nullptr;
    std::shared_ptr<tesseract::common::VectorVector4d> vertex_colors_out = nullptr;
    MeshMaterial::Ptr material = nullptr;
    std::shared_ptr<std::vector<MeshTexture::Ptr>> textures = nullptr;

    const aiMesh* a = scene->mMeshes[node->mMeshes[j]];
    vertices->reserve(a->mNumVertices);
    for (unsigned int i = 0; i < a->mNumVertices; ++i)
    {
      aiVector3D v = transform * a->mVertices[i];
      vertices->emplace_back(static_cast<double>(v.x) * scale(0),
                             static_cast<double>(v.y) * scale(1),
                             static_cast<double>(v.z) * scale(2));
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
        TESSERACT_LOG_DEBUG("Mesh had a face with less than three vertices: {}", resource->getUrl());
      }
    }

    triangles->resize(static_cast<long>(local_triangles.size()));
    for (long i = 0; i < triangles->size(); ++i)
      (*triangles)[i] = local_triangles[static_cast<size_t>(i)];

    if (normals && a->HasNormals())
    {
      vertex_normals = std::make_shared<tesseract::common::VectorVector3d>();
      vertex_normals->reserve(a->mNumVertices);
      for (unsigned int i = 0; i < a->mNumVertices; ++i)
      {
        aiVector3D v = transform * a->mNormals[i];
        vertex_normals->emplace_back(static_cast<double>(v.x) * scale(0),
                                     static_cast<double>(v.y) * scale(1),
                                     static_cast<double>(v.z) * scale(2));
      }
    }

    if (vertex_colors && a->HasVertexColors(0))
    {
      vertex_colors_out = std::make_shared<tesseract::common::VectorVector4d>();
      vertex_colors_out->reserve(a->mNumVertices);
      for (unsigned int i = 0; i < a->mNumVertices; ++i)
      {
        aiColor4D v = a->mColors[0][i];
        vertex_colors_out->emplace_back(
            static_cast<double>(v.r), static_cast<double>(v.g), static_cast<double>(v.b), static_cast<double>(v.a));
      }
    }

    if (material_and_texture)
      std::tie(material, textures) = detail::extractMeshMaterialAndTextures(scene, a, resource);

    meshes.push_back(std::make_shared<T>(vertices,
                                         triangles,
                                         static_cast<int>(triangle_count),
                                         resource,
                                         scale,
                                         vertex_normals,
                                         vertex_colors_out,
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
                                                    tesseract::common::Resource::Ptr resource,
                                                    bool normals,
                                                    bool vertex_colors,
                                                    bool material_and_texture)
{
  if (!scene->HasMeshes())
  {
    TESSERACT_LOG_WARN("Assimp reports scene in {} has no meshes", resource->getUrl());
    return std::vector<std::shared_ptr<T>>();
  }
  std::vector<std::shared_ptr<T>> meshes = extractMeshData<T>(
      scene, scene->mRootNode, aiMatrix4x4(), scale, resource, normals, vertex_colors, material_and_texture);
  if (meshes.empty())
  {
    TESSERACT_LOG_WARN("There are no meshes in the scene {}", resource->getUrl());
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
                                                   const Eigen::Vector3d& scale = Eigen::Vector3d(1, 1, 1),
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
    TESSERACT_LOG_ERROR("Could not load mesh from \"{}\": {}", path, importer.GetErrorString());
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
std::vector<std::shared_ptr<T>> createMeshFromResource(tesseract::common::Resource::Ptr resource,
                                                       const Eigen::Vector3d& scale = Eigen::Vector3d(1, 1, 1),
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
    TESSERACT_LOG_ERROR("Could not load mesh from \"{}\": {}", resource->getUrl(), importer.GetErrorString());
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
                                                           const Eigen::Vector3d& scale = Eigen::Vector3d(1, 1, 1),
                                                           bool triangulate = false,
                                                           bool flatten = false,
                                                           bool normals = false,
                                                           bool vertex_colors = false,
                                                           bool material_and_texture = false)
{
  std::shared_ptr<tesseract::common::Resource> resource =
      std::make_shared<tesseract::common::BytesResource>(url, bytes, bytes_len);
  return tesseract::geometry::createMeshFromResource<T>(
      resource, scale, triangulate, flatten, normals, vertex_colors, material_and_texture);
}

}  // namespace tesseract::geometry

#endif
