/*
 * Copyright (c) 2009, Willow Garage, Inc.
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

#include "rviz/default_plugin/marker_display.h"
#include "rviz/selection/selection_manager.h"
#include "rviz/uniform_string_stream.h"

#include "rviz/display_context.h"
#include "rviz/mesh_loader.h"

#include <OgreSceneNode.h>
#include <OgreSceneManager.h>
#include <OgreManualObject.h>
#include <OgreMaterialManager.h>
#include <OgreTextureManager.h>
#include <OgreTechnique.h>

#include <tesseract_rviz/markers/triangle_list_marker.h>
#include <tesseract_rviz/markers/marker_selection_handler.h>

namespace tesseract_rviz
{
static Ogre::NameGenerator tringle_list_generator("Tesseract_Triangle_List");
static Ogre::NameGenerator material_name_generator("Tesseract_Triangle_List_Material");

TriangleListMarker::TriangleListMarker(const std::string& ns,
                                       const int id,
                                       rviz::DisplayContext* context,
                                       Ogre::SceneNode* parent_node,
                                       const Ogre::ColourValue color,
                                       const std::vector<Ogre::Vector3>& points,
                                       const std::vector<Ogre::ColourValue>& colors)
  : MarkerBase(ns, id, context, parent_node)
  , has_vertex_colors_(false)
  , has_face_colors_(false)
  , any_vertex_has_alpha_(false)
  , manual_object_(nullptr)
{
  size_t num_points = points.size();
  if ((num_points % 3) != 0 || num_points == 0)
  {
    std::stringstream ss;
    if (num_points == 0)
    {
      ss << "TriMesh marker [" << getStringID() << "] has no points.";
    }
    else
    {
      ss << "TriMesh marker [" << getStringID() << "] has a point count which is not divisible by 3 [" << num_points
         << "]";
    }
    ROS_DEBUG("%s", ss.str().c_str());

    scene_node_->setVisible(false);
    return;
  }

  scene_node_->setVisible(true);

  if (!manual_object_)
  {
    manual_object_ = context_->getSceneManager()->createManualObject(tringle_list_generator.generate());
    scene_node_->attachObject(manual_object_);

    material_name_ = material_name_generator.generate();
    material_ = Ogre::MaterialManager::getSingleton().create(material_name_, "rviz");
    material_->setReceiveShadows(false);
    material_->getTechnique(0)->setLightingEnabled(true);
    material_->setCullingMode(Ogre::CULL_NONE);

    handler_.reset(new MarkerSelectionHandler(this, MarkerID(ns, id), context_));
  }

  manual_object_->clear();
  manual_object_->estimateVertexCount(num_points);
  manual_object_->begin(material_name_, Ogre::RenderOperation::OT_TRIANGLE_LIST);

  has_vertex_colors_ = colors.size() == num_points;
  has_face_colors_ = colors.size() == num_points / 3;

  for (size_t i = 0; i < num_points; i += 3)
  {
    std::vector<Ogre::Vector3> corners(3);
    for (size_t c = 0; c < 3; c++)
      corners[c] = points[i + c];

    Ogre::Vector3 normal = (corners[1] - corners[0]).crossProduct(corners[2] - corners[0]);
    normal.normalise();

    for (size_t c = 0; c < 3; c++)
    {
      manual_object_->position(corners[c]);
      manual_object_->normal(normal);
      if (has_vertex_colors_)
      {
        any_vertex_has_alpha_ = any_vertex_has_alpha_ || (colors[i + c].a < 0.9998f);
        manual_object_->colour(colors[i + c].r, colors[i + c].g, colors[i + c].b, color.a * colors[i + c].a);
      }
      else if (has_face_colors_)
      {
        any_vertex_has_alpha_ = any_vertex_has_alpha_ || (colors[i / 3].a < 0.9998f);
        manual_object_->colour(colors[i / 3].r, colors[i / 3].g, colors[i / 3].b, color.a * colors[i / 3].a);
      }
    }
  }

  manual_object_->end();

  setColor(color);

  handler_->addTrackedObject(manual_object_);
}

TriangleListMarker::~TriangleListMarker()
{
  if (manual_object_)
  {
    context_->getSceneManager()->destroyManualObject(manual_object_);
    material_->unload();
    Ogre::MaterialManager::getSingleton().remove(material_->getName());
  }
}

void TriangleListMarker::setScale(Ogre::Vector3 scale) { scene_node_->setScale(scale); }

Ogre::Vector3 TriangleListMarker::getScale() const { return scene_node_->getScale(); }

void TriangleListMarker::setColor(Ogre::ColourValue color)
{
  if (has_vertex_colors_ || has_face_colors_)
  {
    material_->getTechnique(0)->setLightingEnabled(false);
  }
  else
  {
    material_->getTechnique(0)->setLightingEnabled(true);
    material_->getTechnique(0)->setAmbient(color.r / 2, color.g / 2, color.b / 2);
    material_->getTechnique(0)->setDiffuse(color);
  }

  if ((!has_vertex_colors_ && color.a < 0.9998f) || (has_vertex_colors_ && any_vertex_has_alpha_))
  {
    material_->getTechnique(0)->setSceneBlending(Ogre::SBT_TRANSPARENT_ALPHA);
    material_->getTechnique(0)->setDepthWriteEnabled(false);
  }
  else
  {
    material_->getTechnique(0)->setSceneBlending(Ogre::SBT_REPLACE);
    material_->getTechnique(0)->setDepthWriteEnabled(true);
  }
}

std::set<Ogre::MaterialPtr> TriangleListMarker::getMaterials()
{
  std::set<Ogre::MaterialPtr> materials;
  materials.insert(material_);
  return materials;
}

}  // namespace tesseract_rviz
