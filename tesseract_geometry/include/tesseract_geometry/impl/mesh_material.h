/**
 * @file mesh_material.h
 * @brief Tesseract Mesh Material read from a mesh file
 *
 * @author John Wason
 * @date December 21, 2020
 *
 * @copyright Copyright (c) 2020, Wason Technology, LLC
 *
 * @par License
 * Software License Agreement (Apache License)
 * @par
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 * http://www.apache.org/licenses/LICENSE-2.0
 * @par
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */
#ifndef TESSERACT_GEOMETRY_MESH_MATERIAL_H
#define TESSERACT_GEOMETRY_MESH_MATERIAL_H

#include <tesseract_common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <Eigen/Core>
#include <memory>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract_geometry/geometry.h>
#include <tesseract_common/fwd.h>
#include <tesseract_common/eigen_types.h>

namespace tesseract::geometry
{
/**
 * @brief Represents material information extracted from a mesh file
 *
 * Mesh files contain material information. The mesh parser will
 * extract the material information and store it in a MeshMaterial instance.
 * The MeshMaterial class uses a subset PBR Metallic workflow, as specified
 * in the glTF 2.0 file format standard. The four parameters supported
 * are baseColorFactor, metallicFactor, roughnessFactor, and emmisiveFactor. (The
 * MeshTexture class stores diffuse textures that can be used for decals and
 * fiducial marks, and is stored separately from MeshMaterial.)
 * These four parameters and MeshTexture should be enough to display
 * "CAD quality" renderings in visualizers. The full mesh file should be used
 * when higher quality rendering is required.
 *
 * The MeshMaterial favors PBR materials extracted from glTF 2.0 files. COLLADA does
 * not support PBR. Only "Diffuse" and "Emissive" are read. "Specular" and "Ambient"
 * are ignored.
 *
 */
class MeshMaterial
{
public:
  // LCOV_EXCL_START
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  // LCOV_EXCL_STOP

  using Ptr = std::shared_ptr<MeshMaterial>;
  using ConstPtr = std::shared_ptr<const MeshMaterial>;

  /**
   * @brief Construct a new MeshMaterial
   *
   * @param base_color_factor The base color of the mesh
   * @param metallic_factor The metallic factor parameter (PBR parameter)
   * @param roughness_factor The roughness factor parameter (PBR parameter)
   * @param emissive_factor The emissivity of the mesh
   */
  MeshMaterial(const Eigen::Vector4d& base_color_factor,
               double metallic_factor,
               double roughness_factor,
               const Eigen::Vector4d& emissive_factor);

  MeshMaterial() = default;

  /**
   * @brief Get the base color of the mesh
   *
   * @return The base color in RGBA
   */
  Eigen::Vector4d getBaseColorFactor() const;

  /**
   * @brief Get the Metallic Factor of the mesh (PBR parameter)
   *
   * @return The metallic factor, between 0 and 1
   */
  double getMetallicFactor() const;

  /**
   * @brief Get the Roughness Factor of the mesh (PBR parameter)
   *
   * @return The roughness factor, between 0 and 1
   */
  double getRoughnessFactor() const;

  /**
   * @brief Get the emissive factor of the mesh
   *
   * "Emissive factor" is used to make the mesh "glow". How this is
   * interpreted depends on the rendering engine.
   *
   * @return The emissive factor in RGBA
   */
  Eigen::Vector4d getEmissiveFactor() const;

private:
  // Mesh material based on simplified glTF 2.0 pbrMetallicRoughness parameters
  Eigen::Vector4d base_color_factor_;
  double metallic_factor_ = 0;
  double roughness_factor_ = 0.5;
  Eigen::Vector4d emissive_factor_;
};

/**
 * @brief Represents a texture and UV coordinates extracted from a mesh file
 *
 * Mesh files contain (or reference) image files that form textures on the surface
 * of the mesh. UV coordinates specify how the image is applied to the mesh. The
 * MeshTexture structure contains a resource to the image, and the UV coordinates.
 * Currently only jpg and png image formats are supported.
 *
 * UV coordinates specify the location of each vertex in the mesh
 * on the texture. Each (u,v) coordinate is normalized to be
 * between 0 and 1.
 *
 */
class MeshTexture
{
public:
  using Ptr = std::shared_ptr<MeshTexture>;
  using ConstPtr = std::shared_ptr<const MeshTexture>;

  /**
   * @brief Construct a new MeshTexture
   *
   * @param texture_image Resource representing the texture image (jpg or png)
   * @param uvs UV coordinates for texture on mesh
   */
  MeshTexture(std::shared_ptr<tesseract::common::Resource> texture_image,
              std::shared_ptr<const tesseract::common::VectorVector2d> uvs);

  /**
   * @brief Get the texture image
   *
   * Must be jpg or png
   *
   * @return Resource to the texture image
   */
  std::shared_ptr<tesseract::common::Resource> getTextureImage() const;

  /**
   * @brief Get the texture UV coordinates
   *
   * @return UV coordinate vector
   */
  const std::shared_ptr<const tesseract::common::VectorVector2d>& getUVs();

private:
  std::shared_ptr<const tesseract::common::VectorVector2d> uvs_;

  // texture_image shall be jpg or png
  std::shared_ptr<tesseract::common::Resource> texture_image_;
};
}  // namespace tesseract::geometry

#endif
