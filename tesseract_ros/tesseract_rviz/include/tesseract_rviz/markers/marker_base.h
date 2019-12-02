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
#ifndef TESSERACT_RVIZ_MARKERS_MARKER_BASE_H
#define TESSERACT_RVIZ_MARKERS_MARKER_BASE_H

#include "rviz/selection/forwards.h"
#include "rviz/interactive_object.h"

#ifndef Q_MOC_RUN
#include <ros/time.h>
#include <boost/shared_ptr.hpp>
#endif

namespace Ogre
{
class SceneNode;
class Vector3;
class Quaternion;
class Entity;
}  // namespace Ogre

namespace rviz
{
class DisplayContext;
}

namespace tesseract_rviz
{
class MarkerSelectionHandler;
typedef std::pair<std::string, int32_t> MarkerID;

class MarkerBase
{
public:
  using Ptr = boost::shared_ptr<MarkerBase>;
  using ConstPtr = boost::shared_ptr<const MarkerBase>;

  MarkerBase(std::string ns, const int id, rviz::DisplayContext* context, Ogre::SceneNode* parent_node);
  virtual ~MarkerBase();

  MarkerID getID() { return MarkerID(ns_, id_); }

  std::string getStringID()
  {
    std::stringstream ss;
    ss << ns_ << "/" << id_;
    return ss.str();
  }

  /** @brief Associate an InteractiveObject with this MarkerBase. */
  void setInteractiveObject(rviz::InteractiveObjectWPtr object);

  virtual void setPosition(const Ogre::Vector3& position);
  virtual void setOrientation(const Ogre::Quaternion& orientation);
  const Ogre::Vector3& getPosition();
  const Ogre::Quaternion& getOrientation();

  virtual void setScale(Ogre::Vector3 scale) = 0;
  virtual Ogre::Vector3 getScale() const = 0;

  virtual void setColor(Ogre::ColourValue color) = 0;

  virtual std::set<Ogre::MaterialPtr> getMaterials() { return std::set<Ogre::MaterialPtr>(); }

protected:
  void extractMaterials(Ogre::Entity* entity, std::set<Ogre::MaterialPtr>& materials);

  std::string ns_;

  int id_;

  rviz::DisplayContext* context_;

  Ogre::SceneNode* scene_node_;

  boost::shared_ptr<MarkerSelectionHandler> handler_;
};

}  // namespace tesseract_rviz

#endif  // TESSERACT_RVIZ_INTERACTIVE_MARKER_MARKER_BASE_H
