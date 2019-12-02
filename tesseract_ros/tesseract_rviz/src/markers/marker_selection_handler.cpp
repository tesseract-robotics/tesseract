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

#include <OgreQuaternion.h>
#include <OgreVector3.h>

#include <tesseract_rviz/markers/marker_selection_handler.h>
#include <tesseract_rviz/markers/marker_base.h>
#include <tesseract_rviz/interactive_marker/interactive_marker_control.h>

namespace tesseract_rviz
{
MarkerSelectionHandler::MarkerSelectionHandler(const MarkerBase* marker, MarkerID id, rviz::DisplayContext* context)
  : SelectionHandler(context)
  , marker_(marker)
  , marker_id_(QString::fromStdString(id.first) + "/" + QString::number(id.second))
{
}

MarkerSelectionHandler::~MarkerSelectionHandler() = default;

void MarkerSelectionHandler::setPosition(const Ogre::Vector3& position) { position_ = position; }
void MarkerSelectionHandler::setOrientation(const Ogre::Quaternion& orientation) { orientation_ = orientation; }
Ogre::Vector3 MarkerSelectionHandler::getPosition() { return position_; }
Ogre::Quaternion MarkerSelectionHandler::getOrientation() { return orientation_; }

}  // namespace tesseract_rviz
