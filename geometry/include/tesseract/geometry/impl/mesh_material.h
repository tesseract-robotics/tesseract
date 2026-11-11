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

#include <tesseract/common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <Eigen/Core>
#include <cstdint>
#include <memory>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract/geometry/geometry.h>
#include <tesseract/common/fwd.h>
#include <tesseract/common/eigen_types.h>

namespace tesseract::geometry
{
class MeshMaterial;
class MeshTexture;
template <class Archive>
void serialize(Archive& ar, MeshMaterial& obj);
template <class Archive>
void serialize(Archive& ar, MeshTexture& obj);

/**
 * @brief How the alpha channel of the base color is interpreted (glTF 2.0 material.alphaMode)
 */
enum class MeshMaterialAlphaMode : std::uint8_t
{
  /** @brief Alpha is ignored and the mesh is rendered fully opaque */
  OPAQUE = 0,
  /** @brief Fragments with alpha below the alpha cutoff are discarded, all others are opaque */
  MASK = 1,
  /** @brief Alpha is used to blend the mesh with the background */
  BLEND = 2
};

/**
 * @brief Represents material information extracted from a mesh file
 *
 * Mesh files contain material information. The mesh parser will
 * extract the material information and store it in a MeshMaterial instance.
 * The MeshMaterial class follows the PBR metallic-roughness workflow specified
 * in the glTF 2.0 file format standard. The parameters supported are:
 *
 * - baseColorFactor, metallicFactor, roughnessFactor and emissiveFactor (the core factors)
 * - alphaMode and alphaCutoff
 * - doubleSided
 * - the normal texture scale and the occlusion texture strength
 * - the emissive strength from the KHR_materials_emissive_strength extension
 *
 * Texture images are stored separately in MeshTexture instances, one per texture slot
 * (base color, metallic-roughness, normal, occlusion and emissive). The scalar parameters
 * here apply to those textures as specified by glTF 2.0.
 *
 * The MeshMaterial favors PBR materials extracted from glTF 2.0 files. COLLADA does
 * not support PBR. Only "Diffuse" and "Emissive" are read. "Specular" and "Ambient"
 * are ignored.
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

  /**
   * @brief Get the alpha mode, which controls how the base color alpha is interpreted
   * @return The alpha mode. The default is OPAQUE.
   */
  MeshMaterialAlphaMode getAlphaMode() const;

  /**
   * @brief Get the alpha cutoff, used only when the alpha mode is MASK
   * @return The alpha cutoff. The default is 0.5.
   */
  double getAlphaCutoff() const;

  /**
   * @brief Get whether back faces should be rendered (back-face culling disabled)
   * @return True if the material is double sided. The default is false.
   */
  bool getDoubleSided() const;

  /**
   * @brief Get the emissive strength multiplier (KHR_materials_emissive_strength)
   * @return The emissive strength. The default is 1.0.
   */
  double getEmissiveStrength() const;

  /**
   * @brief Get the scale applied to the normal texture's X and Y components
   * @return The normal scale. The default is 1.0.
   */
  double getNormalScale() const;

  /**
   * @brief Get the strength of the occlusion texture, between 0 (no occlusion) and 1 (full occlusion)
   * @return The occlusion strength. The default is 1.0.
   */
  double getOcclusionStrength() const;

  /** @brief Set the base color in RGBA */
  void setBaseColorFactor(const Eigen::Vector4d& base_color_factor);
  /** @brief Set the metallic factor, between 0 and 1 */
  void setMetallicFactor(double metallic_factor);
  /** @brief Set the roughness factor, between 0 and 1 */
  void setRoughnessFactor(double roughness_factor);
  /** @brief Set the emissive factor in RGBA */
  void setEmissiveFactor(const Eigen::Vector4d& emissive_factor);
  /** @brief Set the alpha mode */
  void setAlphaMode(MeshMaterialAlphaMode alpha_mode);
  /** @brief Set the alpha cutoff, used only when the alpha mode is MASK */
  void setAlphaCutoff(double alpha_cutoff);
  /** @brief Set whether the material is double sided */
  void setDoubleSided(bool double_sided);
  /** @brief Set the emissive strength multiplier */
  void setEmissiveStrength(double emissive_strength);
  /** @brief Set the normal texture scale */
  void setNormalScale(double normal_scale);
  /** @brief Set the occlusion texture strength */
  void setOcclusionStrength(double occlusion_strength);

  bool operator==(const MeshMaterial& rhs) const;
  bool operator!=(const MeshMaterial& rhs) const;

private:
  // Mesh material based on glTF 2.0 pbrMetallicRoughness parameters
  Eigen::Vector4d base_color_factor_{ Eigen::Vector4d::Zero() };
  double metallic_factor_ = 0;
  double roughness_factor_ = 0.5;
  Eigen::Vector4d emissive_factor_{ Eigen::Vector4d::Zero() };
  MeshMaterialAlphaMode alpha_mode_{ MeshMaterialAlphaMode::OPAQUE };
  double alpha_cutoff_ = 0.5;
  bool double_sided_ = false;
  double emissive_strength_ = 1.0;
  double normal_scale_ = 1.0;
  double occlusion_strength_ = 1.0;

  template <class Archive>
  friend void ::tesseract::geometry::serialize(Archive& ar, MeshMaterial& obj);
};

/**
 * @brief The material slot a texture is applied to (glTF 2.0 texture references)
 */
enum class MeshTextureType : std::uint8_t
{
  /** @brief Base color (albedo) in sRGB, multiplied by the base color factor. Alpha is the base alpha. */
  BASE_COLOR = 0,
  /** @brief Linear texture with roughness in the green channel and metalness in the blue channel */
  METALLIC_ROUGHNESS = 1,
  /** @brief Tangent space normal map */
  NORMAL = 2,
  /** @brief Ambient occlusion in the red channel */
  OCCLUSION = 3,
  /** @brief Emissive color in sRGB, multiplied by the emissive factor */
  EMISSIVE = 4
};

/** @brief Texture coordinate wrapping mode (glTF 2.0 sampler.wrapS / wrapT) */
enum class MeshTextureWrapMode : std::uint8_t
{
  REPEAT = 0,
  CLAMP_TO_EDGE = 1,
  MIRRORED_REPEAT = 2
};

/** @brief Texture filtering mode (glTF 2.0 sampler.magFilter / minFilter) */
enum class MeshTextureFilter : std::uint8_t
{
  /** @brief Not specified in the mesh file; the renderer chooses */
  DEFAULT = 0,
  NEAREST = 1,
  LINEAR = 2,
  NEAREST_MIPMAP_NEAREST = 3,
  LINEAR_MIPMAP_NEAREST = 4,
  NEAREST_MIPMAP_LINEAR = 5,
  LINEAR_MIPMAP_LINEAR = 6
};

/**
 * @brief Represents a texture and UV coordinates extracted from a mesh file
 *
 * Mesh files contain (or reference) image files that form textures on the surface
 * of the mesh. UV coordinates specify how the image is applied to the mesh. The
 * MeshTexture structure contains a resource to the image, the UV coordinates, the
 * material slot the texture is applied to, and the sampler settings.
 * Currently only jpg and png image formats are supported.
 *
 * UV coordinates specify the location of each vertex in the mesh
 * on the texture. Each (u,v) coordinate is normalized to be
 * between 0 and 1. Textures that use the same UV channel share the same UV vector.
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
   * @param type The material slot the texture is applied to
   * @param uv_channel The index of the UV channel in the source mesh that uvs were read from
   */
  MeshTexture(std::shared_ptr<tesseract::common::Resource> texture_image,
              std::shared_ptr<const tesseract::common::VectorVector2d> uvs,
              MeshTextureType type = MeshTextureType::BASE_COLOR,
              unsigned uv_channel = 0);

  MeshTexture() = default;

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
  const std::shared_ptr<const tesseract::common::VectorVector2d>& getUVs() const;

  /** @brief Get the material slot the texture is applied to */
  MeshTextureType getType() const;

  /** @brief Get the index of the UV channel the UV coordinates were read from */
  unsigned getUVChannel() const;

  /** @brief Get the wrap mode along U (S) */
  MeshTextureWrapMode getWrapU() const;
  /** @brief Get the wrap mode along V (T) */
  MeshTextureWrapMode getWrapV() const;
  /** @brief Get the magnification filter */
  MeshTextureFilter getMagFilter() const;
  /** @brief Get the minification filter */
  MeshTextureFilter getMinFilter() const;

  /** @brief Set the wrap modes along U (S) and V (T) */
  void setWrap(MeshTextureWrapMode wrap_u, MeshTextureWrapMode wrap_v);
  /** @brief Set the magnification and minification filters */
  void setFilters(MeshTextureFilter mag_filter, MeshTextureFilter min_filter);

  /**
   * @brief Equality compares the texture settings, the UV values and the image resource contents
   */
  bool operator==(const MeshTexture& rhs) const;
  bool operator!=(const MeshTexture& rhs) const;

private:
  std::shared_ptr<const tesseract::common::VectorVector2d> uvs_;

  // texture_image shall be jpg or png
  std::shared_ptr<tesseract::common::Resource> texture_image_;

  MeshTextureType type_{ MeshTextureType::BASE_COLOR };
  unsigned uv_channel_{ 0 };
  MeshTextureWrapMode wrap_u_{ MeshTextureWrapMode::REPEAT };
  MeshTextureWrapMode wrap_v_{ MeshTextureWrapMode::REPEAT };
  MeshTextureFilter mag_filter_{ MeshTextureFilter::DEFAULT };
  MeshTextureFilter min_filter_{ MeshTextureFilter::DEFAULT };

  template <class Archive>
  friend void ::tesseract::geometry::serialize(Archive& ar, MeshTexture& obj);
};
}  // namespace tesseract::geometry

#endif
