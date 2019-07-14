/*
 * Copyright (c) 2011, Willow Garage, Inc.
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

#ifndef TESSERACT_RVIZ_MARKERS_UTILS_H
#define TESSERACT_RVIZ_MARKERS_UTILS_H

#include <string>

namespace Ogre
{
class SceneNode;
class Quaternion;
class Vector3;
class ColourValue;
}  // namespace Ogre

namespace rviz
{
class DisplayContext;
}

namespace tesseract_rviz
{
class MarkerBase;
class ArrowMarker;
class InteractiveMarkerControl;
class InteractiveMarker;

/** @brief Make a sphere
 *  @param control  the interactive marker control that this will go into
 *  @param radius   The radius of the sphere
 */
void makeSphere(InteractiveMarkerControl& control, float radius);

/** @brief make a default-style arrow marker based on the properties of the given interactive marker
 *  @param control  the interactive marker control that this will go into
 *  @param pos          how far from the center should the arrow be, and on which side
 */
void makeArrow(InteractiveMarkerControl& control, float pos);

/** @brief make a default-style disc marker (e.g for rotating) based on the properties of the given interactive marker
 *  @param control  the interactive marker control that this will go into
 *  @param width    width of the disc, relative to its inner radius
 */
void makeDisc(InteractiveMarkerControl& control, float width = 0.3f);

///// @brief make a box which shows the given text and is view facing
///// @param msg      the interactive marker that this will go into
///// @param text     the text to display
// void makeViewFacingButton( const visualization_msgs::InteractiveMarker &msg,
//    visualization_msgs::InteractiveMarkerControl &control, std::string text );

/** @brief Get the default orientation */
Ogre::ColourValue getDefaultColor(const Ogre::Quaternion& quat);

/** @brief create a control which shows the description of the interactive marker */
void makeTitle(InteractiveMarkerControl& control, const std::string& text);

/** @brief Make a 6 dof interactive marker */
void make6Dof(InteractiveMarker& interactive_marker);

}  // namespace tesseract_rviz

#endif  // TESSERACT_RVIZ_RENDER_TOOLS_UTILS_H
