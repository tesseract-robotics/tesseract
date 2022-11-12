/*
 * Copyright (C) 2015 Open Source Robotics Foundation
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
 */
#ifndef TESSERACT_COMMON_MATERIAL_H
#define TESSERACT_COMMON_MATERIAL_H

#include <memory>
#include <string>

#include <tesseract_common/color.h>
#include <tesseract_common/image.h>
#include <tesseract_common/material.h>
#include <tesseract_common/shader_type.h>
#include <tesseract_common/shader_params.h>

namespace tesseract_common
{
//
/// \brief Default pbr material properties
//    static const common::Pbr kDefaultPbr;

enum class MaterialType
{
  /// \brief Classic shading, i.e. variants of Phong
  MT_CLASSIC = 0,

  /// \brief Physically Based Shading
  MT_PBS = 1
};

class Material
{
public:
  using Ptr = std::shared_ptr<Material>;
  using ConstPtr = std::shared_ptr<const Material>;

  Material(std::string name);

  virtual ~Material();

  // Documentation inherited
  virtual std::unique_ptr<Material> clone(const std::string& name = "") const;

  // Documentation inherited
  virtual std::string getName() const;

  // Documentation inherited
  virtual void setAmbient(double r, double g, double b, double a = 1.0);

  // Documentation inherited
  virtual void setAmbient(const Color& color);

  // Documentation inherited
  virtual void setDiffuse(double r, double g, double b, double a = 1.0);

  // Documentation inherited
  virtual void setDiffuse(const Color& color);

  // Documentation inherited
  virtual void setSpecular(double r, double g, double b, double a = 1.0);

  // Documentation inherited
  virtual void setSpecular(const Color& color);

  // Documentation inherited
  virtual void setEmissive(double r, double g, double b, double a = 1.0);

  // Documentation inherited
  virtual void setEmissive(const Color& color);

  // Documentation inherited
  virtual void setTransparency(double transparency);

  // Documentation inherited
  virtual void setAlphaFromTexture(bool enabled, double alpha = 0.5, bool two_sided = true);

  // Documentation inherited
  virtual void setShininess(double shininess);

  // Documentation inherited
  virtual void setReflectivity(double reflectivity);

  // Documentation inherited
  virtual void setCastShadows(bool cast_shadows);

  // Documentation inherited
  virtual void setReceiveShadows(bool receive_shadows);

  // Documentation inherited
  virtual void setReflectionEnabled(bool enabled);

  // Documentation inherited
  virtual void setLightingEnabled(bool enabled);

  // Documentation inherited.
  virtual void setDepthCheckEnabled(bool enabled);

  // Documentation inherited.
  virtual void setDepthWriteEnabled(bool enabled);

  // Documentation inherited
  virtual Color getAmbient() const;

  // Documentation inherited
  virtual Color getDiffuse() const;

  // Documentation inherited
  virtual Color getSpecular() const;

  // Documentation inherited
  virtual Color getEmissive() const;

  // Documentation inherited
  virtual double getTransparency() const;

  // Documentation inherited
  virtual double getReflectivity() const;

  // Documentation inherited
  virtual double getShininess() const;

  // Documentation inherited
  virtual bool castShadows() const;

  // Documentation inherited
  virtual bool receiveShadows() const;

  // Documentation inherited
  virtual bool lightingEnabled() const;

  // Documentation inherited
  virtual bool depthCheckEnabled() const;

  // Documentation inherited
  virtual bool depthWriteEnabled() const;

  // Documentation inherited
  virtual bool reflectionEnabled() const;

  // Documentation inherited
  virtual bool hasTexture() const;

  // Documentation inherited
  virtual std::string getTexture() const;

  // Documentation inherited
  virtual void setTexture(const std::string& texture, const Image::ConstPtr& img);

  // Documentation inherited
  virtual Image::ConstPtr getTextureData() const;

  // Documentation inherited
  virtual void clearTexture();

  // Documentation inherited
  virtual bool hasNormalMap() const;

  // Documentation inherited
  virtual std::string getNormalMap() const;

  // Documentation inherited
  virtual Image::ConstPtr getNormalMapData() const;

  // Documentation inherited
  virtual void setNormalMap(const std::string& normal_map, const Image::ConstPtr& img);

  // Documentation inherited
  virtual void clearNormalMap();

  // Documentation inherited
  virtual bool hasRoughnessMap() const;

  // Documentation inherited
  virtual std::string getRoughnessMap() const;

  // Documentation inherited
  virtual Image::ConstPtr getRoughnessMapData() const;

  // Documentation inherited
  virtual void setRoughnessMap(const std::string& roughness_map, const Image::ConstPtr& img);

  // Documentation inherited
  virtual void clearRoughnessMap();

  // Documentation inherited
  virtual bool hasMetalnessMap() const;

  // Documentation inherited
  virtual std::string getMetalnessMap() const;

  // Documentation inherited
  virtual Image::ConstPtr getMetalnessMapData() const;

  // Documentation inherited
  virtual void setMetalnessMap(const std::string& metalness_map, const Image::ConstPtr& img);

  // Documentation inherited
  virtual void clearMetalnessMap();

  // Documentation inherited
  virtual bool hasEnvironmentMap() const;

  // Documentation inherited
  virtual std::string getEnvironmentMap() const;

  // Documentation inherited
  virtual Image::ConstPtr getEnvironmentMapData() const;

  // Documentation inherited
  virtual void setEnvironmentMap(const std::string& environment_map, const Image::ConstPtr& img);

  // Documentation inherited
  virtual void clearEnvironmentMap();

  // Documentation inherited
  virtual bool hasEmissiveMap() const;

  // Documentation inherited
  virtual std::string getEmissiveMap() const;

  // Documentation inherited
  virtual Image::ConstPtr getEmissiveMapData() const;

  // Documentation inherited
  virtual void setEmissiveMap(const std::string& emissive_map, const Image::ConstPtr& img);

  // Documentation inherited
  virtual void clearEmissiveMap();

  // Documentation inherited
  virtual bool hasLightMap() const;

  // Documentation inherited
  virtual std::string getLightMap() const;

  // Documentation inherited
  virtual unsigned int getLightMapTexCoordSet() const;

  // Documentation inherited
  virtual void setLightMap(const std::string& light_map, const Image::ConstPtr& img, unsigned int uv_set = 0u);

  // Documentation inherited
  virtual Image::ConstPtr getLightMapData() const;

  // Documentation inherited
  virtual void clearLightMap();

  // Documentation inherited
  virtual void setRenderOrder(float render_order);

  // Documentation inherited
  virtual float getRenderOrder() const;

  // Documentation inherited
  virtual void setRoughness(float roughness);

  // Documentation inherited
  virtual float getRoughness() const;

  // Documentation inherited
  virtual void setMetalness(float metalness);

  // Documentation inherited
  virtual float getMetalness() const;

  // Documentation inherited
  virtual MaterialType getType() const;

  // Documentation inherited
  virtual void setShaderType(ShaderType type);

  // Documentation inherited
  virtual ShaderType getShaderType() const;

  // Documentation inherited.
  // \sa Material::SetDepthMaterial()
  virtual void setDepthMaterial(double far, double near);

  // Documentation inherited.
  // \sa Material::VertexShader() const
  virtual std::string getVertexShader() const;

  // Documentation inherited.
  // \sa Material::VertexShaderParams()
  virtual ShaderParams::Ptr getVertexShaderParams();

  // Documentation inherited.
  // \sa Material::SetVertexShader(const std::string &)
  virtual void setVertexShader(const std::string& path);

  // Documentation inherited.
  // \sa Material::FragmentShader() const
  virtual std::string getFragmentShader() const;

  // Documentation inherited.
  // \sa Material::FragmentShaderParams()
  virtual ShaderParams::Ptr getFragmentShaderParams();

  // Documentation inherited.
  // \sa Material::SetFragmentShader(const std::string &)
  virtual void setFragmentShader(const std::string& path);

  // Documentation inherited.
  virtual void copyFrom(Material::ConstPtr material);

  // Documentation inherited.
  virtual void copyFrom(const Material& material);

  // Documentation inherited
  bool textureAlphaEnabled() const;

  // Documentation inherited
  double getAlphaThreshold() const;

  // Documentation inherited
  bool twoSidedEnabled() const;

protected:
  virtual void reset();

  /// \brief Material name
  std::string name_;

  /// \brief Ambient color
  Color ambient_;

  /// \brief Diffuse color
  Color diffuse_;

  /// \brief Specular color
  Color specular_;

  /// \brief Emissive color
  Color emissive_;

  /// \brief Transparent. 1: fully transparent, 0: opaque
  double transparency_{ 0.0 };

  /// \brief Enable alpha channel based texture transparency
  bool texture_alpha_enabled_{ false };

  /// \brief Threshold for alpha channel rejection
  double alpha_threshold_{ 0.5 };

  /// \brief Enable two sided rendering
  bool two_sided_enabled_{ false };

  /// \brief Material render order
  double render_order_{ 0.0 };

  /// \brief Shininess factor
  double shininess_{ 0.0 };

  /// \brief Reflectivity
  double reflectivity_{ 0.0 };

  /// \brief Flag to indicate if dynamic lighting is enabled
  bool lighting_enabled_{ false };

  /// \brief Flag to indicate if depth buffer checking is enabled
  bool depth_check_enabled_{ true };

  /// \brief Flag to indicate if depth buffer writing is enabled
  bool depth_write_enabled_{ true };

  /// \brief Flag to indicate if reflection is enabled
  bool reflection_enabled_{ false };

  /// \brief True if material receives shadows
  bool receive_shadows_{ true };

  /// \brief Set to true to enable object with this material to cast shadows
  bool cast_shadows_{ true };
};

}  // namespace tesseract_common
#endif  // TESSERACT_COMMON_MATERIAL_H
