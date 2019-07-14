/**
 * @file cube_marker.cpp
 * @brief Cube marker
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
#include <tesseract_rviz/markers/cube_marker.h>
#include <tesseract_rviz/markers/marker_selection_handler.h>
#include <tesseract_rviz/markers/utils.h>

#include <rviz/display_context.h>
#include <rviz/selection/selection_manager.h>

#include <rviz/ogre_helpers/shape.h>

#include <OgreSceneNode.h>
#include <OgreMatrix3.h>

namespace tesseract_rviz
{
CubeMarker::CubeMarker(const std::string& ns,
                       const int id,
                       rviz::DisplayContext* context,
                       Ogre::SceneNode* parent_node,
                       float size)
  : MarkerBase(ns, id, context, parent_node), shape_(nullptr), scale_(Ogre::Vector3(1, 1, 1)), size_(size)
{
  shape_ = new rviz::Shape(rviz::Shape::Cube, context_->getSceneManager(), scene_node_);
  setScale(scale_);

  handler_.reset(new MarkerSelectionHandler(this, MarkerID(ns_, id_), context_));
  handler_->addTrackedObjects(shape_->getRootNode());
}

CubeMarker::~CubeMarker() { delete shape_; }

void CubeMarker::setScale(Ogre::Vector3 scale)
{
  scale_ = scale;
  shape_->setScale(size_ * scale_);
}

Ogre::Vector3 CubeMarker::getScale() const { return scale_; }

void CubeMarker::setSize(float size)
{
  size_ = size;
  shape_->setScale(size_ * scale_);
}

float CubeMarker::getSize() const { return size_; }

void CubeMarker::setColor(Ogre::ColourValue color) { shape_->setColor(color.r, color.g, color.b, color.a); }

std::set<Ogre::MaterialPtr> CubeMarker::getMaterials()
{
  std::set<Ogre::MaterialPtr> materials;
  extractMaterials(shape_->getEntity(), materials);
  return materials;
}

}  // namespace tesseract_rviz
