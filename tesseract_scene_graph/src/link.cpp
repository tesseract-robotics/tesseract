/**
 * @file link.cpp
 * @brief Tesseract Link
 *
 * @author Levi Armstrong
 * @author Matthew Powelson
 * @date March 16, 2022
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

#include <tesseract_common/utils.h>
#include <tesseract_scene_graph/link.h>
#include <tesseract_geometry/geometry.h>

namespace tesseract_scene_graph
{
/*********************************************************/
/******                 Material                     *****/
/*********************************************************/
Material::Material(std::string name) : name_(std::move(name)) { this->clear(); }

std::shared_ptr<Material> Material::getDefaultMaterial()
{
  static auto default_material = std::make_shared<Material>("default_tesseract_material");
  return default_material;
}

const std::string& Material::getName() const { return name_; }

void Material::clear()
{
  color = Eigen::Vector4d(0.5, 0.5, 0.5, 1.0);
  texture_filename.clear();
}

bool Material::operator==(const Material& rhs) const
{
  bool equal = true;
  equal &= texture_filename == rhs.texture_filename;
  equal &= tesseract_common::almostEqualRelativeAndAbs(color, rhs.color);
  equal &= name_ == rhs.name_;

  return equal;
}
bool Material::operator!=(const Material& rhs) const { return !operator==(rhs); }

/*********************************************************/
/******                Inertial                      *****/
/*********************************************************/

void Inertial::clear()
{
  origin.setIdentity();
  mass = 0;
  ixx = ixy = ixz = iyy = iyz = izz = 0;
}

bool Inertial::operator==(const Inertial& rhs) const
{
  bool equal = true;
  equal &= origin.isApprox(rhs.origin, 1e-5);
  equal &= tesseract_common::almostEqualRelativeAndAbs(mass, rhs.mass);
  equal &= tesseract_common::almostEqualRelativeAndAbs(ixx, rhs.ixx);
  equal &= tesseract_common::almostEqualRelativeAndAbs(ixy, rhs.ixy);
  equal &= tesseract_common::almostEqualRelativeAndAbs(ixz, rhs.ixz);
  equal &= tesseract_common::almostEqualRelativeAndAbs(iyy, rhs.iyy);
  equal &= tesseract_common::almostEqualRelativeAndAbs(iyz, rhs.iyz);
  equal &= tesseract_common::almostEqualRelativeAndAbs(izz, rhs.izz);

  return equal;
}
bool Inertial::operator!=(const Inertial& rhs) const { return !operator==(rhs); }

/*********************************************************/
/******                 Visual                       *****/
/*********************************************************/

Visual::Visual() { this->clear(); }

void Visual::clear()
{
  origin.setIdentity();
  material = Material::getDefaultMaterial();
  geometry.reset();
  name.clear();
}

bool Visual::operator==(const Visual& rhs) const
{
  bool equal = true;
  equal &= origin.isApprox(rhs.origin, 1e-5);
  equal &= tesseract_common::pointersEqual(geometry, rhs.geometry);  /// @todo Make utility to check derived type
  equal &= tesseract_common::pointersEqual(material, rhs.material);
  equal &= name == rhs.name;

  return equal;
}
bool Visual::operator!=(const Visual& rhs) const { return !operator==(rhs); }

/*********************************************************/
/******                   Collision                  *****/
/*********************************************************/
Collision::Collision() { this->clear(); }

void Collision::clear()
{
  origin.setIdentity();
  geometry.reset();
  name.clear();
}

bool Collision::operator==(const Collision& rhs) const
{
  bool equal = true;
  equal &= origin.isApprox(rhs.origin, 1e-5);
  equal &= tesseract_common::pointersEqual(geometry, rhs.geometry);  /// @todo Make utility to check derived type
  equal &= name == rhs.name;

  return equal;
}
bool Collision::operator!=(const Collision& rhs) const { return !operator==(rhs); }

/*********************************************************/
/******                     Link                     *****/
/*********************************************************/
Link::Link(const std::string& name) : name_(name), hash_(std::hash<std::string>{}(name)) { this->clear(); }

const std::string& Link::getName() const { return name_; }

std::size_t Link::getHash() const { return hash_; }

void Link::clear()
{
  this->inertial.reset();
  this->collision.clear();
  this->visual.clear();
}

Link Link::clone() const { return clone(name_); }

Link Link::clone(const std::string& name) const
{
  Link ret(name);
  ret.visible = visible;
  ret.collision_enabled = collision_enabled;
  if (this->inertial)
  {
    ret.inertial = std::make_shared<Inertial>(*(this->inertial));
  }
  for (const auto& c : this->collision)
  {
    ret.collision.push_back(std::make_shared<Collision>(*c));
  }
  for (const auto& v : this->visual)
  {
    ret.visual.push_back(std::make_shared<Visual>(*v));
  }
  return ret;
}

bool Link::operator==(const Link& rhs) const
{
  using namespace tesseract_common;
  bool equal = true;
  equal &= tesseract_common::pointersEqual(inertial, rhs.inertial);
  equal &= isIdentical<Visual::Ptr>(visual,
                                    rhs.visual,
                                    false,
                                    tesseract_common::pointersEqual<Visual>,
                                    [](const Visual::Ptr& v1, const Visual::Ptr& v2) { return v1->name < v2->name; });
  equal &= isIdentical<Collision::Ptr>(
      collision,
      rhs.collision,
      false,
      tesseract_common::pointersEqual<Collision>,
      [](const Collision::Ptr& v1, const Collision::Ptr& v2) { return v1->name < v2->name; });
  equal &= name_ == rhs.name_;
  equal &= hash_ == rhs.hash_;
  equal &= visible == rhs.visible;
  equal &= collision_enabled == rhs.collision_enabled;
  return equal;
}
bool Link::operator!=(const Link& rhs) const { return !operator==(rhs); }

}  // namespace tesseract_scene_graph
