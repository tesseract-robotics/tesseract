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

#include <tesseract_common/material.h>

namespace tesseract_common
{
//////////////////////////////////////////////////
Material::Material(std::string name) : name_(std::move(name)) {}

//////////////////////////////////////////////////
Material::~Material() {}

//////////////////////////////////////////////////
std::string Material::getName() const { return name_; }

//////////////////////////////////////////////////
void Material::setAmbient(double r, double g, double b, double a) { setAmbient(Color(r, g, b, a)); }

//////////////////////////////////////////////////
void Material::setAmbient(const Color& color) { ambient_ = color; }

//////////////////////////////////////////////////
void Material::setDiffuse(double r, double g, double b, double a) { setDiffuse(Color(r, g, b, a)); }

//////////////////////////////////////////////////
void Material::setDiffuse(const Color& color) { diffuse_ = color; }

//////////////////////////////////////////////////
void Material::setSpecular(double r, double g, double b, double a) { setSpecular(Color(r, g, b, a)); }

//////////////////////////////////////////////////
void Material::setSpecular(const Color& color) { specular_ = color; }

//////////////////////////////////////////////////
void Material::setEmissive(double r, double g, double b, double a) { setEmissive(Color(r, g, b, a)); }

//////////////////////////////////////////////////
void Material::setEmissive(const Color& color) { emissive_ = color; }

//////////////////////////////////////////////////
void Material::setShininess(double shininess) { shininess_ = shininess; }

//////////////////////////////////////////////////
void Material::setTransparency(double transparency) { transparency_ = transparency; }

//////////////////////////////////////////////////
void Material::setAlphaFromTexture(bool enabled, double alpha, bool two_sided)
{
  texture_alpha_enabled_ = enabled;
  alpha_threshold_ = alpha;
  two_sided_enabled_ = two_sided;
}

//////////////////////////////////////////////////
bool Material::textureAlphaEnabled() const { return texture_alpha_enabled_; }

//////////////////////////////////////////////////
double Material::getAlphaThreshold() const { return alpha_threshold_; }

//////////////////////////////////////////////////
bool Material::twoSidedEnabled() const { return two_sided_enabled_; }

//////////////////////////////////////////////////
void Material::setReflectivity(double reflectivity) { reflectivity_ = reflectivity; }

//////////////////////////////////////////////////
void Material::setReflectionEnabled(bool enabled) { reflection_enabled_ = enabled; }

//////////////////////////////////////////////////
void Material::setLightingEnabled(bool enabled) { lighting_enabled_ = enabled; }

//////////////////////////////////////////////////
void Material::setDepthCheckEnabled(bool enabled) { depth_check_enabled_ = enabled; }

//////////////////////////////////////////////////
void Material::setDepthWriteEnabled(bool enabled) { depth_write_enabled_ = enabled; }

//////////////////////////////////////////////////
void Material::setCastShadows(bool cast_shadows) { cast_shadows_ = cast_shadows; }

//////////////////////////////////////////////////
void Material::setReceiveShadows(bool receive_shadows) { receive_shadows_ = receive_shadows; }

//////////////////////////////////////////////////
void Material::setRenderOrder(float render_order) { render_order_ = render_order; }

//////////////////////////////////////////////////
Color Material::getAmbient() const { return ambient_; }

//////////////////////////////////////////////////
Color Material::getDiffuse() const { return diffuse_; }

//////////////////////////////////////////////////
Color Material::getSpecular() const { return specular_; }

//////////////////////////////////////////////////
Color Material::getEmissive() const { return emissive_; }

//////////////////////////////////////////////////
double Material::getShininess() const { return shininess_; }

//////////////////////////////////////////////////
double Material::getTransparency() const { return transparency_; }

//////////////////////////////////////////////////
float Material::getRenderOrder() const { return render_order_; }

//////////////////////////////////////////////////
double Material::getReflectivity() const { return reflectivity_; }

//////////////////////////////////////////////////
bool Material::castShadows() const { return cast_shadows_; }

//////////////////////////////////////////////////
bool Material::receiveShadows() const { return receive_shadows_; }

//////////////////////////////////////////////////
bool Material::lightingEnabled() const { return lighting_enabled_; }

//////////////////////////////////////////////////
bool Material::depthCheckEnabled() const { return depth_check_enabled_; }

//////////////////////////////////////////////////
bool Material::depthWriteEnabled() const { return depth_write_enabled_; }

//////////////////////////////////////////////////
bool Material::reflectionEnabled() const { return reflection_enabled_; }

//////////////////////////////////////////////////
MaterialType Material::getType() const { return MaterialType::MT_CLASSIC; }

//////////////////////////////////////////////////
void Material::setShaderType(ShaderType /*type*/)
{
  // no op
}

//////////////////////////////////////////////////
enum ShaderType Material::getShaderType() const { return ShaderType::ST_PIXEL; }

//////////////////////////////////////////////////
std::string Material::getVertexShader() const { return {}; }

//////////////////////////////////////////////////
ShaderParams::Ptr Material::getVertexShaderParams() { return nullptr; }

//////////////////////////////////////////////////
void Material::setVertexShader(const std::string& /*path*/)
{
  // no op
}

//////////////////////////////////////////////////
std::string Material::getFragmentShader() const { return {}; }

//////////////////////////////////////////////////
ShaderParams::Ptr Material::getFragmentShaderParams() { return nullptr; }

//////////////////////////////////////////////////
void Material::setFragmentShader(const std::string& /*path*/)
{
  // no op
}

//////////////////////////////////////////////////
bool Material::hasTexture() const { return false; }

//////////////////////////////////////////////////
std::string Material::getTexture() const { return {}; }

//////////////////////////////////////////////////
void Material::setTexture(const std::string& texture, const Image::ConstPtr& img)
{
  // no op
}

//////////////////////////////////////////////////
Image::ConstPtr Material::getTextureData() const { return {}; }

//////////////////////////////////////////////////
void Material::clearTexture()
{
  // no op
}

//////////////////////////////////////////////////
bool Material::hasNormalMap() const { return false; }

//////////////////////////////////////////////////
std::string Material::getNormalMap() const { return {}; }

//////////////////////////////////////////////////
Image::ConstPtr Material::getNormalMapData() const { return {}; }

//////////////////////////////////////////////////
void Material::setNormalMap(const std::string& normal_map, const Image::ConstPtr& img)
{
  // no op
}

//////////////////////////////////////////////////
void Material::clearNormalMap()
{
  // no op
}

//////////////////////////////////////////////////
bool Material::hasRoughnessMap() const { return false; }

//////////////////////////////////////////////////
std::string Material::getRoughnessMap() const { return {}; }

//////////////////////////////////////////////////
Image::ConstPtr Material::getRoughnessMapData() const { return {}; }

//////////////////////////////////////////////////
void Material::setRoughnessMap(const std::string& roughness_map, const Image::ConstPtr& img)
{
  // no op
}

//////////////////////////////////////////////////
void Material::clearRoughnessMap()
{
  // no op
}

//////////////////////////////////////////////////
bool Material::hasMetalnessMap() const { return false; }

//////////////////////////////////////////////////
std::string Material::getMetalnessMap() const { return {}; }

//////////////////////////////////////////////////
Image::ConstPtr Material::getMetalnessMapData() const { return {}; }

//////////////////////////////////////////////////
void Material::setMetalnessMap(const std::string& metalness_map, const Image::ConstPtr& img)
{
  // no op
}

//////////////////////////////////////////////////
void Material::clearMetalnessMap()
{
  // no op
}

//////////////////////////////////////////////////
bool Material::hasEnvironmentMap() const { return false; }

//////////////////////////////////////////////////
std::string Material::getEnvironmentMap() const { return {}; }

//////////////////////////////////////////////////
Image::ConstPtr Material::getEnvironmentMapData() const { return {}; }

//////////////////////////////////////////////////
void Material::setEnvironmentMap(const std::string& environment_map, const Image::ConstPtr& img)
{
  // no op
}

//////////////////////////////////////////////////
void Material::clearEnvironmentMap()
{
  // no op
}

//////////////////////////////////////////////////
bool Material::hasEmissiveMap() const { return false; }

//////////////////////////////////////////////////
std::string Material::getEmissiveMap() const { return {}; }

//////////////////////////////////////////////////
Image::ConstPtr Material::getEmissiveMapData() const { return {}; }

//////////////////////////////////////////////////
void Material::setEmissiveMap(const std::string& emissive_map, const Image::ConstPtr& img)
{
  // no op
}

//////////////////////////////////////////////////
void Material::clearEmissiveMap()
{
  // no op
}

//////////////////////////////////////////////////
bool Material::hasLightMap() const { return false; }

//////////////////////////////////////////////////
std::string Material::getLightMap() const { return {}; }

//////////////////////////////////////////////////
unsigned int Material::getLightMapTexCoordSet() const { return 0u; }

//////////////////////////////////////////////////
void Material::setLightMap(const std::string& light_map, const Image::ConstPtr& img, unsigned int uv_set)
{
  // no op
}

//////////////////////////////////////////////////
Image::ConstPtr Material::getLightMapData() const { return {}; }

//////////////////////////////////////////////////
void Material::clearLightMap()
{
  // no op
}

//////////////////////////////////////////////////
void Material::setRoughness(float roughness)
{
  // no op
}

//////////////////////////////////////////////////
float Material::getRoughness() const { return 0.0f; }

//////////////////////////////////////////////////
void Material::setMetalness(float metalness)
{
  // no op
}

//////////////////////////////////////////////////
float Material::getMetalness() const { return 0.0f; }

//////////////////////////////////////////////////
std::unique_ptr<Material> Material::clone(const std::string& name) const
{
  //  auto baseShared = this->shared_from_this();

  //  auto thisShared =
  //      std::dynamic_pointer_cast<const Material>(baseShared);

  //  MaterialPtr material = T::Scene()->CreateMaterial(_name);
  //  material->CopyFrom(thisShared);
  //  return material;
}

//////////////////////////////////////////////////
void Material::copyFrom(ConstPtr material)
{
  //  this->SetLightingEnabled(_material->LightingEnabled());
  //  this->SetAmbient(_material->Ambient());
  //  this->SetDiffuse(_material->Diffuse());
  //  this->SetSpecular(_material->Specular());
  //  this->SetEmissive(_material->Emissive());
  //  this->SetRenderOrder(_material->RenderOrder());
  //  this->SetShininess(_material->Shininess());
  //  this->SetAlphaFromTexture(_material->TextureAlphaEnabled(),
  //      _material->AlphaThreshold(), _material->TwoSidedEnabled());
  //  // override transparency / blend setting after setting alpha from texture
  //  this->SetTransparency(_material->Transparency());
  //  // override depth check / depth write after setting transparency
  //  this->SetDepthCheckEnabled(_material->DepthCheckEnabled());
  //  this->SetDepthWriteEnabled(_material->DepthWriteEnabled());
  //  this->SetReflectivity(_material->Reflectivity());
  //  this->SetCastShadows(_material->CastShadows());
  //  this->SetReceiveShadows(_material->ReceiveShadows());
  //  this->SetReflectionEnabled(_material->ReflectionEnabled());
  //  this->SetTexture(_material->Texture(), _material->TextureData());
  //  this->SetNormalMap(_material->NormalMap(), _material->NormalMapData());
  //  this->SetRoughnessMap(_material->RoughnessMap(),
  //      _material->RoughnessMapData());
  //  this->SetMetalnessMap(_material->MetalnessMap(),
  //      _material->MetalnessMapData());
  //  this->SetRoughness(_material->Roughness());
  //  this->SetMetalness(_material->Metalness());
  //  this->SetEnvironmentMap(_material->EnvironmentMap(),
  //      _material->EnvironmentMapData());
  //  this->SetEmissiveMap(_material->EmissiveMap(),
  //      _material->EmissiveMapData());
  //  this->SetLightMap(_material->LightMap(), _material->LightMapData(),
  //      _material->LightMapTexCoordSet());
  //  this->SetShaderType(_material->ShaderType());
  //  this->SetVertexShader(_material->VertexShader());
  //  this->SetFragmentShader(_material->FragmentShader());
}

//////////////////////////////////////////////////
void Material::copyFrom(const Material& material)
{
  //  this->SetLightingEnabled(_material.Lighting());
  //  this->SetAmbient(_material.Ambient());
  //  this->SetDiffuse(_material.Diffuse());
  //  this->SetSpecular(_material.Specular());
  //  this->SetEmissive(_material.Emissive());
  //  this->SetShininess(_material.Shininess());
  //  this->SetTransparency(_material.Transparency());
  //  this->SetAlphaFromTexture(_material.TextureAlphaEnabled(),
  //      _material.AlphaThreshold(), _material.TwoSidedEnabled());
  //  this->SetRenderOrder(_material.RenderOrder());
  //  // TODO(anyone): update common::Material
  //  this->SetReflectivity(0);
  //  this->SetTexture(_material.TextureImage(), _material.TextureData());
  //  // TODO(anyone): update common::Material
  //  this->SetCastShadows(true);
  //  // TODO(anyone): update common::Material
  //  this->SetReceiveShadows(true);
  //  // TODO(anyone): update common::Material
  //  this->SetReflectionEnabled(true);
  //  // TODO(anyone): update common::Material
  //  this->ClearNormalMap();
  //  // TODO(anyone): update common::Material
  //  this->SetShaderType(ST_PIXEL);

  //  const common::Pbr *pbrMat = _material.PbrMaterial();
  //  if (!pbrMat)
  //    pbrMat = &kDefaultPbr;
  //  this->SetNormalMap(pbrMat->NormalMap(), pbrMat->NormalMapData());
  //  this->SetRoughnessMap(pbrMat->RoughnessMap(), pbrMat->RoughnessMapData());
  //  this->SetMetalnessMap(pbrMat->MetalnessMap(), pbrMat->MetalnessMapData());
  //  this->SetRoughness(pbrMat->Roughness());
  //  this->SetMetalness(pbrMat->Metalness());
  //  // TODO(anyone): update when pbrMat has EnvironmentMapData API
  //  this->SetEnvironmentMap(pbrMat->EnvironmentMap(), nullptr);
  //  this->SetEmissiveMap(pbrMat->EmissiveMap(), pbrMat->EmissiveMapData());
  //  this->SetLightMap(pbrMat->LightMap(), pbrMat->LightMapData(),
  //      pbrMat->LightMapTexCoordSet());
}

//////////////////////////////////////////////////
void Material::setDepthMaterial(double /*far*/, double /*near*/)
{
  // do nothing
}

//////////////////////////////////////////////////
void Material::reset()
{
  setLightingEnabled(true);
  setDepthCheckEnabled(true);
  setDepthWriteEnabled(true);
  setAmbient(0.3, 0.3, 0.3);
  setDiffuse(1.0, 1.0, 1.0);
  setSpecular(0.2, 0.2, 0.2);
  setEmissive(0, 0, 0);
  setRenderOrder(0);
  setShininess(1.5);
  setTransparency(0);
  setReflectivity(0);
  setCastShadows(true);
  setReceiveShadows(true);
  setReflectionEnabled(true);
  clearTexture();
  clearNormalMap();
  clearRoughnessMap();
  clearMetalnessMap();
  clearEmissiveMap();
  clearLightMap();
  //  setRoughness(kDefaultPbr.Roughness());
  //  setMetalness(kDefaultPbr.Metalness());
  setShaderType(ShaderType::ST_PIXEL);
}
}  // namespace tesseract_common
