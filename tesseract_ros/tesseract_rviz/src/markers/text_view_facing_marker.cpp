/*
 * Copyright (c) 2010, Willow Garage, Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Willow Garage, Inc. nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#include <OgreSceneNode.h>
#include <OgreSceneManager.h>

#include <ros/assert.h>

#include "rviz/display_context.h"
#include "rviz/ogre_helpers/movable_text.h"
#include "rviz/selection/selection_manager.h"

#include <tesseract_rviz/markers/marker_selection_handler.h>
#include <tesseract_rviz/markers/text_view_facing_marker.h>

namespace tesseract_rviz
{
TextViewFacingMarker::TextViewFacingMarker(const std::string& ns,
                                           const int id,
                                           const std::string& caption,
                                           rviz::DisplayContext* context,
                                           Ogre::SceneNode* parent_node)

  : MarkerBase(ns, id, context, parent_node), text_(nullptr)
{
  text_ = new rviz::MovableText(caption);
  text_->setTextAlignment(rviz::MovableText::H_CENTER, rviz::MovableText::V_CENTER);
  text_->setCharacterHeight(0.15f);
  scene_node_->attachObject(text_);

  handler_.reset(new MarkerSelectionHandler(this, MarkerID(ns, id), context_));
  handler_->addTrackedObject(text_);
}

TextViewFacingMarker::~TextViewFacingMarker() { delete text_; }

void TextViewFacingMarker::setScale(Ogre::Vector3 scale)
{
  Ogre::Vector3 pos = getPosition();
  pos.x *= scale.x;
  pos.y *= scale.y;
  pos.z *= scale.z;
  scene_node_->setScale(scale);
  setPosition(pos);
}

Ogre::Vector3 TextViewFacingMarker::getScale() const { return scene_node_->getScale(); }

void TextViewFacingMarker::setColor(Ogre::ColourValue color) { text_->setColor(color); }

void TextViewFacingMarker::setText(const std::string& text) { text_->setCaption(text); }

std::set<Ogre::MaterialPtr> TextViewFacingMarker::getMaterials()
{
  std::set<Ogre::MaterialPtr> materials;
  if (text_->getMaterial().get())
  {
    materials.insert(text_->getMaterial());
  }
  return materials;
}

}  // namespace tesseract_rviz
