/**
 * @file cylinder_marker.cpp
 * @brief Cylinder marker
 *
 * @author Levi Armstrong
 * @date Dec 18, 2017
 * @version TODO
 * @bug No known bugs
 *
 * @copyright Copyright (c) 2017, Southwest Research Institute
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
#include <tesseract_rviz/markers/cylinder_marker.h>
#include <tesseract_rviz/markers/marker_selection_handler.h>
#include <tesseract_rviz/markers/utils.h>

#include <rviz/display_context.h>
#include <rviz/selection/selection_manager.h>

#include <rviz/ogre_helpers/shape.h>

#include <OgreSceneNode.h>
#include <OgreMatrix3.h>

namespace tesseract_rviz
{
CylinderMarker::CylinderMarker(const std::string& ns,
                               const int id,
                               rviz::DisplayContext* context,
                               Ogre::SceneNode* parent_node,
                               float radius,
                               float height)
  : MarkerBase(ns, id, context, parent_node)
  , shape_(nullptr)
  , scale_(Ogre::Vector3(1, 1, 1))
  , radius_(radius)
  , height_(height)
{
  shape_ = new rviz::Shape(rviz::Shape::Cylinder, context_->getSceneManager(), scene_node_);
  setScale(scale_);

  handler_.reset(new MarkerSelectionHandler(this, MarkerID(ns_, id_), context_));
  handler_->addTrackedObjects(shape_->getRootNode());
}

CylinderMarker::~CylinderMarker() { delete shape_; }

void CylinderMarker::setScale(Ogre::Vector3 scale)
{
  scale_ = scale;
  shape_->setScale(Ogre::Vector3(radius_ * scale_.x, radius_ * scale_.y, height_ * scale_.z));
}

Ogre::Vector3 CylinderMarker::getScale() const { return scale_; }

void CylinderMarker::setRadius(float radius)
{
  radius_ = radius;
  shape_->setScale(Ogre::Vector3(radius_ * scale_.x, radius_ * scale_.y, height_ * scale_.z));
}

float CylinderMarker::getRadius() const { return radius_; }

void CylinderMarker::setHeight(float height)
{
  height_ = height;
  shape_->setScale(Ogre::Vector3(radius_ * scale_.x, radius_ * scale_.y, height_ * scale_.z));
}

float CylinderMarker::getHeight() const { return height_; }

void CylinderMarker::setColor(Ogre::ColourValue color) { shape_->setColor(color.r, color.g, color.b, color.a); }

std::set<Ogre::MaterialPtr> CylinderMarker::getMaterials()
{
  std::set<Ogre::MaterialPtr> materials;
  extractMaterials(shape_->getEntity(), materials);
  return materials;
}

}  // namespace tesseract_rviz
