/**
 * @file mesh_material.cpp
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

#include <tesseract/geometry/impl/mesh_material.h>
#include <tesseract/common/resource_locator.h>
#include <tesseract/common/utils.h>

namespace tesseract::geometry
{
MeshMaterial::MeshMaterial(const Eigen::Vector4d& base_color_factor,  // NOLINT(modernize-pass-by-value)
                           double metallic_factor,
                           double roughness_factor,
                           const Eigen::Vector4d& emissive_factor)  // NOLINT(modernize-pass-by-value)
  : base_color_factor_(base_color_factor)
  , metallic_factor_(metallic_factor)
  , roughness_factor_(roughness_factor)
  , emissive_factor_(emissive_factor)
{
}

Eigen::Vector4d MeshMaterial::getBaseColorFactor() const { return base_color_factor_; }
double MeshMaterial::getMetallicFactor() const { return metallic_factor_; }
double MeshMaterial::getRoughnessFactor() const { return roughness_factor_; }
Eigen::Vector4d MeshMaterial::getEmissiveFactor() const { return emissive_factor_; }
MeshMaterialAlphaMode MeshMaterial::getAlphaMode() const { return alpha_mode_; }
double MeshMaterial::getAlphaCutoff() const { return alpha_cutoff_; }
bool MeshMaterial::getDoubleSided() const { return double_sided_; }
double MeshMaterial::getEmissiveStrength() const { return emissive_strength_; }
double MeshMaterial::getNormalScale() const { return normal_scale_; }
double MeshMaterial::getOcclusionStrength() const { return occlusion_strength_; }

void MeshMaterial::setBaseColorFactor(const Eigen::Vector4d& base_color_factor)
{
  base_color_factor_ = base_color_factor;
}
void MeshMaterial::setMetallicFactor(double metallic_factor) { metallic_factor_ = metallic_factor; }
void MeshMaterial::setRoughnessFactor(double roughness_factor) { roughness_factor_ = roughness_factor; }
void MeshMaterial::setEmissiveFactor(const Eigen::Vector4d& emissive_factor) { emissive_factor_ = emissive_factor; }
void MeshMaterial::setAlphaMode(MeshMaterialAlphaMode alpha_mode) { alpha_mode_ = alpha_mode; }
void MeshMaterial::setAlphaCutoff(double alpha_cutoff) { alpha_cutoff_ = alpha_cutoff; }
void MeshMaterial::setDoubleSided(bool double_sided) { double_sided_ = double_sided; }
void MeshMaterial::setEmissiveStrength(double emissive_strength) { emissive_strength_ = emissive_strength; }
void MeshMaterial::setNormalScale(double normal_scale) { normal_scale_ = normal_scale; }
void MeshMaterial::setOcclusionStrength(double occlusion_strength) { occlusion_strength_ = occlusion_strength; }

bool MeshMaterial::operator==(const MeshMaterial& rhs) const
{
  using tesseract::common::almostEqualRelativeAndAbs;
  bool equal = true;
  equal &= almostEqualRelativeAndAbs(base_color_factor_, rhs.base_color_factor_);
  equal &= almostEqualRelativeAndAbs(metallic_factor_, rhs.metallic_factor_);
  equal &= almostEqualRelativeAndAbs(roughness_factor_, rhs.roughness_factor_);
  equal &= almostEqualRelativeAndAbs(emissive_factor_, rhs.emissive_factor_);
  equal &= alpha_mode_ == rhs.alpha_mode_;
  equal &= almostEqualRelativeAndAbs(alpha_cutoff_, rhs.alpha_cutoff_);
  equal &= double_sided_ == rhs.double_sided_;
  equal &= almostEqualRelativeAndAbs(emissive_strength_, rhs.emissive_strength_);
  equal &= almostEqualRelativeAndAbs(normal_scale_, rhs.normal_scale_);
  equal &= almostEqualRelativeAndAbs(occlusion_strength_, rhs.occlusion_strength_);
  return equal;
}
bool MeshMaterial::operator!=(const MeshMaterial& rhs) const { return !operator==(rhs); }

MeshTexture::MeshTexture(std::shared_ptr<tesseract::common::Resource> texture_image,
                         std::shared_ptr<const tesseract::common::VectorVector2d> uvs,
                         MeshTextureType type,
                         unsigned uv_channel)
  : uvs_(std::move(uvs)), texture_image_(std::move(texture_image)), type_(type), uv_channel_(uv_channel)
{
}

std::shared_ptr<tesseract::common::Resource> MeshTexture::getTextureImage() const { return texture_image_; }
const std::shared_ptr<const tesseract::common::VectorVector2d>& MeshTexture::getUVs() const { return uvs_; }
MeshTextureType MeshTexture::getType() const { return type_; }
unsigned MeshTexture::getUVChannel() const { return uv_channel_; }
MeshTextureWrapMode MeshTexture::getWrapU() const { return wrap_u_; }
MeshTextureWrapMode MeshTexture::getWrapV() const { return wrap_v_; }
MeshTextureFilter MeshTexture::getMagFilter() const { return mag_filter_; }
MeshTextureFilter MeshTexture::getMinFilter() const { return min_filter_; }

void MeshTexture::setWrap(MeshTextureWrapMode wrap_u, MeshTextureWrapMode wrap_v)
{
  wrap_u_ = wrap_u;
  wrap_v_ = wrap_v;
}

void MeshTexture::setFilters(MeshTextureFilter mag_filter, MeshTextureFilter min_filter)
{
  mag_filter_ = mag_filter;
  min_filter_ = min_filter;
}

namespace
{
bool uvsEqual(const std::shared_ptr<const tesseract::common::VectorVector2d>& a,
              const std::shared_ptr<const tesseract::common::VectorVector2d>& b)
{
  if (a == b)
    return true;
  if (a == nullptr || b == nullptr || a->size() != b->size())
    return false;
  for (std::size_t i = 0; i < a->size(); ++i)
  {
    if (!tesseract::common::almostEqualRelativeAndAbs((*a)[i], (*b)[i]))
      return false;
  }
  return true;
}

bool imagesEqual(const std::shared_ptr<tesseract::common::Resource>& a,
                 const std::shared_ptr<tesseract::common::Resource>& b)
{
  if (a == b)
    return true;
  if (a == nullptr || b == nullptr)
    return false;
  return a->getResourceContents() == b->getResourceContents();
}
}  // namespace

bool MeshTexture::operator==(const MeshTexture& rhs) const
{
  bool equal = true;
  equal &= type_ == rhs.type_;
  equal &= uv_channel_ == rhs.uv_channel_;
  equal &= wrap_u_ == rhs.wrap_u_;
  equal &= wrap_v_ == rhs.wrap_v_;
  equal &= mag_filter_ == rhs.mag_filter_;
  equal &= min_filter_ == rhs.min_filter_;
  equal &= uvsEqual(uvs_, rhs.uvs_);
  equal &= equal && imagesEqual(texture_image_, rhs.texture_image_);  // Only read the images when all else matches
  return equal;
}
bool MeshTexture::operator!=(const MeshTexture& rhs) const { return !operator==(rhs); }

}  // namespace tesseract::geometry
