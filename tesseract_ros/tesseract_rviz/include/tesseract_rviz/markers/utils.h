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
}

namespace rviz
{
class DisplayContext;
}

namespace tesseract_rviz
{
class MarkerBase;

enum class MarkerType
{
  ARROW=0,
  CUBE=1,
  SPHERE=2,
  CYLINDER=3,
  LINE_STRIP=4,
  LINE_LIST=5,
  CUBE_LIST=6,
  SPHERE_LIST=7,
  POINTS=8,
  TEXT_VIEW_FACING=9,
  MESH_RESOURCE=10,
  TRIANGLE_LIST=11
};

/** Create a marker of given type as declared in visualization_messages::Marker */
MarkerBase* createMarker(const std::string &ns,
                         const int id,
                         MarkerType marker_type,
                         rviz::DisplayContext *context,
                         Ogre::SceneNode *parent_node);

///** @brief fill in default values & insert default controls when none are specified.
// *
// * This also calls uniqueifyControlNames().
// * @param msg      interactive marker to be completed */
//void autoComplete( visualization_msgs::InteractiveMarker &msg, bool enable_autocomplete_transparency = true );

///// @brief fill in default values & insert default controls when none are specified
///// @param msg      interactive marker which contains the control
///// @param control  the control to be completed
//void autoComplete( const visualization_msgs::InteractiveMarker &msg,
//    visualization_msgs::InteractiveMarkerControl &control, bool enable_autocomplete_transparency = true );

///** @brief Make sure all the control names are unique within the given msg.
// *
// * Appends _u0 _u1 etc to repeated names (not including the first of each).
// * This is called by autoComplete( visualization_msgs::InteractiveMarker &msg ). */
//void uniqueifyControlNames( visualization_msgs::InteractiveMarker& msg );

///// make a quaternion with a fixed local x axis.
///// The rotation around that axis will be chosen automatically.
///// @param x,y,z    the designated x axis
//geometry_msgs::Quaternion makeQuaternion( float x, float y, float z );


///// --- marker helpers ---

///// @brief make a default-style arrow marker based on the properties of the given interactive marker
///// @param msg      the interactive marker that this will go into
///// @param control  the control where to insert the arrow marker
///// @param pos      how far from the center should the arrow be, and on which side
//void makeArrow( const visualization_msgs::InteractiveMarker &msg,
//    visualization_msgs::InteractiveMarkerControl &control, float pos );

///// @brief make a default-style disc marker (e.g for rotating) based on the properties of the given interactive marker
///// @param msg      the interactive marker that this will go into
///// @param width    width of the disc, relative to its inner radius
//void makeDisc( const visualization_msgs::InteractiveMarker &msg,
//    visualization_msgs::InteractiveMarkerControl &control, float width = 0.3 );

///// @brief make a box which shows the given text and is view facing
///// @param msg      the interactive marker that this will go into
///// @param text     the text to display
//void makeViewFacingButton( const visualization_msgs::InteractiveMarker &msg,
//    visualization_msgs::InteractiveMarkerControl &control, std::string text );

///// assign an RGB value to the given marker based on the given orientation
//void assignDefaultColor(visualization_msgs::Marker &marker, const geometry_msgs::Quaternion &quat );

///// create a control which shows the description of the interactive marker
//visualization_msgs::InteractiveMarkerControl makeTitle( const visualization_msgs::InteractiveMarker &msg );


}

#endif // TESSERACT_RVIZ_RENDER_TOOLS_UTILS_H
