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
#ifndef TESSERACT_RVIZ_MARKERS_TEXT_VIEW_FACING_MARKER_H
#define TESSERACT_RVIZ_MARKERS_TEXT_VIEW_FACING_MARKER_H

#include <tesseract_rviz/markers/marker_base.h>

namespace Ogre
{
class SceneNode;
}

namespace rviz
{
class MovableText;
}

namespace tesseract_rviz
{
class TextViewFacingMarker : public MarkerBase
{
public:
  using Ptr = boost::shared_ptr<TextViewFacingMarker>;
  using ConstPtr = boost::shared_ptr<const TextViewFacingMarker>;

  TextViewFacingMarker(const std::string& ns,
                       const int id,
                       const std::string& caption,
                       rviz::DisplayContext* context,
                       Ogre::SceneNode* parent_node);

  ~TextViewFacingMarker() override;

  void setText(const std::string& text);

  void setOrientation(const Ogre::Quaternion& /*orientation*/) override {}

  void setScale(Ogre::Vector3 scale) override;
  Ogre::Vector3 getScale() const override;

  void setColor(Ogre::ColourValue color) override;

  std::set<Ogre::MaterialPtr> getMaterials() override;

protected:
  rviz::MovableText* text_;
};

}  // namespace tesseract_rviz
#endif  // TESSERACT_RVIZ_MARKERS_TEXT_VIEW_FACING_MARKER_H
