/**
 * @file link.cpp
 * @brief Tesseract Link
 *
 * @author Levi Armstrong
 * @author Matthew Powelson
 * @date March 16, 2022
 * @version TODO
 * @bug No known bugs
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

#include <tesseract_common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <boost/serialization/access.hpp>
#include <boost/serialization/nvp.hpp>
#include <boost/serialization/shared_ptr.hpp>
#include <boost/serialization/vector.hpp>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract_common/eigen_serialization.h>
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

template <class Archive>
void Material::serialize(Archive& ar, const unsigned int /*version*/)
{
  ar& BOOST_SERIALIZATION_NVP(texture_filename);
  ar& BOOST_SERIALIZATION_NVP(color);
  ar& BOOST_SERIALIZATION_NVP(name_);
}

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

template <class Archive>
void Inertial::serialize(Archive& ar, const unsigned int /*version*/)
{
  ar& BOOST_SERIALIZATION_NVP(origin);
  ar& BOOST_SERIALIZATION_NVP(mass);
  ar& BOOST_SERIALIZATION_NVP(ixx);
  ar& BOOST_SERIALIZATION_NVP(ixy);
  ar& BOOST_SERIALIZATION_NVP(ixz);
  ar& BOOST_SERIALIZATION_NVP(iyy);
  ar& BOOST_SERIALIZATION_NVP(iyz);
  ar& BOOST_SERIALIZATION_NVP(izz);
}

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

template <class Archive>
void Visual::serialize(Archive& ar, const unsigned int /*version*/)
{
  ar& BOOST_SERIALIZATION_NVP(origin);
  ar& BOOST_SERIALIZATION_NVP(geometry);
  ar& BOOST_SERIALIZATION_NVP(material);
  ar& BOOST_SERIALIZATION_NVP(name);
}

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

template <class Archive>
void Collision::serialize(Archive& ar, const unsigned int /*version*/)
{
  ar& BOOST_SERIALIZATION_NVP(origin);
  ar& BOOST_SERIALIZATION_NVP(geometry);
  ar& BOOST_SERIALIZATION_NVP(name);
}

/*********************************************************/
/******                     Link                     *****/
/*********************************************************/
Link::Link(std::string name) : name_(std::move(name)) { this->clear(); }

const std::string& Link::getName() const { return name_; }

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
  return equal;
}
bool Link::operator!=(const Link& rhs) const { return !operator==(rhs); }

template <class Archive>
void Link::serialize(Archive& ar, const unsigned int /*version*/)
{
  ar& BOOST_SERIALIZATION_NVP(inertial);
  ar& BOOST_SERIALIZATION_NVP(visual);
  ar& BOOST_SERIALIZATION_NVP(collision);
  ar& BOOST_SERIALIZATION_NVP(name_);
}

}  // namespace tesseract_scene_graph

#include <tesseract_common/serialization.h>
BOOST_CLASS_EXPORT_IMPLEMENT(tesseract_scene_graph::Material)
BOOST_CLASS_EXPORT_IMPLEMENT(tesseract_scene_graph::Inertial)
BOOST_CLASS_EXPORT_IMPLEMENT(tesseract_scene_graph::Visual)
BOOST_CLASS_EXPORT_IMPLEMENT(tesseract_scene_graph::Collision)
BOOST_CLASS_EXPORT_IMPLEMENT(tesseract_scene_graph::Link)

TESSERACT_SERIALIZE_ARCHIVES_INSTANTIATE(tesseract_scene_graph::Material)
TESSERACT_SERIALIZE_ARCHIVES_INSTANTIATE(tesseract_scene_graph::Inertial)
TESSERACT_SERIALIZE_ARCHIVES_INSTANTIATE(tesseract_scene_graph::Visual)
TESSERACT_SERIALIZE_ARCHIVES_INSTANTIATE(tesseract_scene_graph::Collision)
TESSERACT_SERIALIZE_ARCHIVES_INSTANTIATE(tesseract_scene_graph::Link)
