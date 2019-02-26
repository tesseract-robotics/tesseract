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

#include <tesseract_scene_graph/macros.h>
TESSERACT_SCENE_GRAPH_IGNORE_WARNINGS_PUSH
#include <fstream>

#include <assimp/scene.h>
#include <assimp/Importer.hpp>
#include <assimp/postprocess.h>

#include <console_bridge/console.h>

#include <tesseract_geometry/impl/mesh.h>

TESSERACT_SCENE_GRAPH_IGNORE_WARNINGS_POP

namespace tesseract_scene_graph
{

  inline std::vector<tesseract_geometry::MeshPtr> extractMeshData(const aiScene* scene,
                                                                  const aiNode* node,
                                                                  const aiMatrix4x4& parent_transform,
                                                                  const Eigen::Vector3d& scale)
  {
    std::vector<tesseract_geometry::MeshPtr> meshes;

    aiMatrix4x4 transform = parent_transform;
    transform *= node->mTransformation;
    for (unsigned int j = 0; j < node->mNumMeshes; ++j)
    {
      std::shared_ptr<tesseract_geometry::VectorVector3d> vertices(new tesseract_geometry::VectorVector3d());
      std::shared_ptr<std::vector<int>> triangles(new std::vector<int>());
      int triangle_count = 0;

      const aiMesh* a = scene->mMeshes[node->mMeshes[j]];
      for (unsigned int i = 0; i < a->mNumVertices; ++i)
      {
        aiVector3D v = transform * a->mVertices[i];
        vertices->push_back(Eigen::Vector3d(static_cast<double>(v.x) * scale(0),
                                            static_cast<double>(v.y) * scale(1),
                                            static_cast<double>(v.z) * scale(2)));
      }

      for (unsigned int i = 0; i < a->mNumFaces; ++i)
      {
        assert(a->mFaces[i].mNumIndices >= 3);
        if (a->mFaces[i].mNumIndices > 3)
        {
          triangle_count += 1;
          triangles->push_back(3);
          triangles->push_back(static_cast<int>(a->mFaces[i].mIndices[0]));
          triangles->push_back(static_cast<int>(a->mFaces[i].mIndices[1]));
          triangles->push_back(static_cast<int>(a->mFaces[i].mIndices[2]));
          for (size_t k = 3; k < a->mFaces[i].mNumIndices; ++k)
          {
            triangle_count += 1;
            triangles->push_back(3);
            triangles->push_back(static_cast<int>(a->mFaces[i].mIndices[0]));
            triangles->push_back(static_cast<int>(a->mFaces[i].mIndices[k - 1]));
            triangles->push_back(static_cast<int>(a->mFaces[i].mIndices[k]));
          }
        }
        else if (a->mFaces[i].mNumIndices == 3)
        {
          triangle_count += 1;
          triangles->push_back(3);
          triangles->push_back(static_cast<int>(a->mFaces[i].mIndices[0]));
          triangles->push_back(static_cast<int>(a->mFaces[i].mIndices[1]));
          triangles->push_back(static_cast<int>(a->mFaces[i].mIndices[2]));
        }
      }

      meshes.push_back(tesseract_geometry::MeshPtr(new tesseract_geometry::Mesh(vertices, triangles, triangle_count)));
    }

    for (unsigned int n = 0; n < node->mNumChildren; ++n)
    {
      std::vector<tesseract_geometry::MeshPtr> child_meshes = extractMeshData(scene, node->mChildren[n], transform, scale);
      meshes.insert(meshes.end(), child_meshes.begin(), child_meshes.end());
    }
    return meshes;
  }

  inline std::vector<tesseract_geometry::MeshPtr> createMeshFromAsset(const aiScene* scene,
                                                                      const Eigen::Vector3d& scale,
                                                                      const std::string& path)
  {
    if (!scene->HasMeshes())
    {
      CONSOLE_BRIDGE_logWarn("Assimp reports scene in %s has no meshes", path.c_str());
      return std::vector<tesseract_geometry::MeshPtr>();
    }
    std::vector<tesseract_geometry::MeshPtr> meshes = extractMeshData(scene, scene->mRootNode, aiMatrix4x4(), scale);
    if (meshes.empty())
    {
      CONSOLE_BRIDGE_logWarn("There are no meshes in the scene %s", path.c_str());
      return std::vector<tesseract_geometry::MeshPtr>();
    }

    return meshes;
  }

  inline std::vector<tesseract_geometry::MeshPtr> createMeshFromPath(const std::string& path,  Eigen::Vector3d scale = Eigen::Vector3d(1, 1, 1))
  {
    // Create an instance of the Importer class
    Assimp::Importer importer;

    // Issue #38 fix: as part of the post-processing, we remove all other components in file but
    // the meshes, as anyway the resulting shapes:Mesh object just receives vertices and triangles.
    importer.SetPropertyInteger(AI_CONFIG_PP_RVC_FLAGS, aiComponent_NORMALS | aiComponent_TANGENTS_AND_BITANGENTS |
                                                            aiComponent_COLORS | aiComponent_TEXCOORDS |
                                                            aiComponent_BONEWEIGHTS | aiComponent_ANIMATIONS |
                                                            aiComponent_TEXTURES | aiComponent_LIGHTS |
                                                            aiComponent_CAMERAS | aiComponent_MATERIALS);

    // And have it read the given file with some post-processing
    const aiScene* scene = importer.ReadFile(path.c_str(), aiProcess_Triangulate | aiProcess_JoinIdenticalVertices | aiProcess_SortByPType | aiProcess_RemoveComponent);
    if (!scene)
      return std::vector<tesseract_geometry::MeshPtr>();

    // Assimp enforces Y_UP convention by rotating models with different conventions.
    // However, that behaviour is confusing and doesn't match the ROS convention
    // where the Z axis is pointing up.
    // Hopefully this doesn't undo legit use of the root node transformation...
    // Note that this is also what RViz does internally.
    scene->mRootNode->mTransformation = aiMatrix4x4();

    // These post processing steps flatten the root node transformation into child nodes,
    // so they must be delayed until after clearing the root node transform above.
    importer.ApplyPostProcessing(aiProcess_OptimizeMeshes | aiProcess_OptimizeGraph);

    return createMeshFromAsset(scene, scale, path);
  }

}
#endif
