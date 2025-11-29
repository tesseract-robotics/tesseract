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

#include <tesseract_geometry/impl/mesh_material.h>
#include <tesseract_common/resource_locator.h>

namespace tesseract_geometry
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

MeshTexture::MeshTexture(std::shared_ptr<tesseract_common::Resource> texture_image,
                         std::shared_ptr<const tesseract_common::VectorVector2d> uvs)
  : uvs_(std::move(uvs)), texture_image_(std::move(texture_image))
{
}

std::shared_ptr<tesseract_common::Resource> MeshTexture::getTextureImage() const { return texture_image_; }

const std::shared_ptr<const tesseract_common::VectorVector2d>& MeshTexture::getUVs() { return uvs_; }

}  // namespace tesseract_geometry
