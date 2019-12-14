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

#include <tesseract_rviz/markers/utils.h>
#include <tesseract_rviz/markers/sphere_marker.h>
#include <tesseract_rviz/markers/arrow_marker.h>
#include <tesseract_rviz/markers/triangle_list_marker.h>
#include <tesseract_rviz/markers/text_view_facing_marker.h>
#include <tesseract_rviz/interactive_marker/interactive_marker.h>
#include <tesseract_rviz/interactive_marker/interactive_marker_control.h>

#include <rviz/display_context.h>
#include <rviz/ogre_helpers/shape.h>

#include <boost/make_shared.hpp>

namespace tesseract_rviz
{
void makeSphere(InteractiveMarkerControl& control, float radius)
{
  SphereMarker::Ptr marker = boost::make_shared<SphereMarker>(
      control.getName(), 0, control.getDisplayContext(), control.getMarkerSceneNode(), radius);
  marker->setScale(Ogre::Vector3(control.getSize(), control.getSize(), control.getSize()));
  marker->setColor(Ogre::ColourValue(1.0f, 1.0f, 0.0f, 0.5f));
  control.addMarker(marker);
}

void makeArrow(InteractiveMarkerControl& control, float pos)
{
  Ogre::ColourValue default_color = getDefaultColor(control.getControlOrientation());
  Ogre::Vector3 scale;
  scale.x = control.getSize() * 0.15f;  // aleeper: changed from 0.3 due to Rviz fix
  scale.y = control.getSize() * 0.25f;  // aleeper: changed from 0.5 due to Rviz fix
  scale.z = control.getSize() * 0.2f;

  float dist = std::abs(pos);
  float dir = pos > 0 ? 1 : -1;

  float inner = dist;
  float outer = inner + 0.4f;

  Ogre::Vector3 point1, point2;
  point1.x = dir * inner;
  point1.y = 0;
  point1.z = 0;

  point2.x = dir * outer;
  point2.y = 0;
  point2.z = 0;

  ArrowMarker::Ptr marker = boost::make_shared<ArrowMarker>(
      control.getName(), 0, point1, point2, control.getDisplayContext(), control.getMarkerSceneNode());
  marker->setColor(default_color);
  marker->setOrientation(control.getControlOrientation());
  float scale1 = control.getSize();
  marker->setScale(Ogre::Vector3(scale1, scale1, scale1));
  control.addMarker(marker);
}

void makeTitle(InteractiveMarkerControl& control, const std::string& text)
{
  Ogre::ColourValue default_color(1, 1, 1, 1);
  TextViewFacingMarker::Ptr marker = boost::make_shared<TextViewFacingMarker>(
      control.getName(), 0, text, control.getDisplayContext(), control.getMarkerSceneNode());
  marker->setColor(default_color);
  float scale = control.getSize();
  marker->setScale(Ogre::Vector3(scale, scale, scale));
  control.addMarker(marker);
}

void makeDisc(InteractiveMarkerControl& control, float width)
{
  Ogre::ColourValue default_color = getDefaultColor(control.getControlOrientation());

  // compute points on a circle in the y-z plane
  size_t steps = 36;
  std::vector<Ogre::Vector3> circle1, circle2;
  circle1.reserve(steps);
  circle2.reserve(steps);

  Ogre::Vector3 v1, v2;
  for (size_t i = 0; i < steps; i++)
  {
    auto a = static_cast<float>(double(i) / double(steps) * M_PI * 2.0);
    v1.x = 0;
    v1.y = (0.5f * std::cos(a));
    v1.z = (0.5f * std::sin(a));

    v2.x = 0;
    v2.y = ((1 + width) * v1.y);
    v2.z = ((1 + width) * v1.z);

    circle1.push_back(v1);
    circle2.push_back(v2);
  }

  std::vector<Ogre::Vector3> points;
  points.resize(6 * steps);

  Ogre::ColourValue color = default_color;

  std::vector<Ogre::ColourValue> colors;
  switch (control.getInteractionMode())
  {
    case InteractiveMode::ROTATE_AXIS:
    {
      colors.resize(2 * steps);
      Ogre::ColourValue base_color = color;
      for (size_t i = 0; i < steps; i++)
      {
        size_t i1 = i;
        size_t i2 = (i + 1) % steps;
        size_t i3 = (i + 2) % steps;

        size_t p = i * 6;
        size_t c = i * 2;

        points[p + 0] = circle1[i1];
        points[p + 1] = circle2[i2];
        points[p + 2] = circle1[i2];

        points[p + 3] = circle1[i2];
        points[p + 4] = circle2[i2];
        points[p + 5] = circle2[i3];

        float t = 0.6f + 0.4f * static_cast<float>((i % 2));
        color.r = base_color.r * t;
        color.g = base_color.g * t;
        color.b = base_color.b * t;

        colors[c] = color;
        colors[c + 1] = color;
      }
      break;
    }

    case InteractiveMode::MOVE_ROTATE:
    {
      colors.resize(2 * steps);
      Ogre::ColourValue base_color = color;
      for (size_t i = 0; i < steps - 1; i += 2)
      {
        size_t i1 = i;
        size_t i2 = (i + 1) % steps;
        size_t i3 = (i + 2) % steps;

        size_t p = i * 6;
        size_t c = i * 2;

        points[p + 0] = circle1[i1];
        points[p + 1] = circle2[i2];
        points[p + 2] = circle1[i2];

        points[p + 3] = circle1[i2];
        points[p + 4] = circle2[i2];
        points[p + 5] = circle1[i3];

        color.r = base_color.r * 0.6f;
        color.g = base_color.g * 0.6f;
        color.b = base_color.b * 0.6f;

        colors[c] = color;
        colors[c + 1] = color;

        p += 6;
        c += 2;

        points[p + 0] = circle2[i1];
        points[p + 1] = circle2[i2];
        points[p + 2] = circle1[i1];

        points[p + 3] = circle2[i2];
        points[p + 4] = circle2[i3];
        points[p + 5] = circle1[i3];

        colors[c] = base_color;
        colors[c + 1] = base_color;
      }
      break;
    }

    default:
      for (size_t i = 0; i < steps; i++)
      {
        size_t i1 = i;
        size_t i2 = (i + 1) % steps;

        size_t p = i * 6;

        points[p + 0] = circle1[i1];
        points[p + 1] = circle2[i1];
        points[p + 2] = circle1[i2];

        points[p + 3] = circle2[i1];
        points[p + 4] = circle2[i2];
        points[p + 5] = circle1[i2];
      }
      break;
  }

  TriangleListMarker::Ptr marker = boost::make_shared<TriangleListMarker>(
      control.getName(), 0, control.getDisplayContext(), control.getMarkerSceneNode(), default_color, points, colors);
  marker->setOrientation(control.getControlOrientation());
  float scale = control.getSize();
  marker->setScale(Ogre::Vector3(scale, scale, scale));
  control.addMarker(marker);
}

// void makeViewFacingButton( const visualization_msgs::InteractiveMarker &msg,
//    visualization_msgs::InteractiveMarkerControl &control, std::string text )
//{
//  control.orientation_mode = visualization_msgs::InteractiveMarkerControl::VIEW_FACING;
//  control.independent_marker_orientation = false;

//  visualization_msgs::Marker marker;

//  float base_scale = 0.25 * msg.scale;
//  float base_z = 1.2 * msg.scale;

//  marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
//  marker.scale.x = base_scale;
//  marker.scale.y = base_scale;
//  marker.scale.z = base_scale;
//  marker.color.r = 1.0;
//  marker.color.g = 1.0;
//  marker.color.b = 1.0;
//  marker.color.a = 1.0;
//  marker.pose.position.x = base_scale * -0.1;
//  marker.pose.position.z = base_z + base_scale * -0.1;
//  marker.text = text;

//  control.markers.push_back( marker );
//}

void make6Dof(InteractiveMarker& interactive_marker)
{
  InteractiveMarkerControl::Ptr control = interactive_marker.createInteractiveControl("move_rotate_3d",
                                                                                      "Move Rotate 3D",
                                                                                      InteractiveMode::MOVE_ROTATE_3D,
                                                                                      OrientationMode::INHERIT,
                                                                                      true,
                                                                                      Ogre::Quaternion(1, 0, 0, 0));
  makeSphere(*control, 0.5f);

  InteractiveMarkerControl::Ptr control1 = interactive_marker.createInteractiveControl("move_x",
                                                                                       "Move along X Axis",
                                                                                       InteractiveMode::MOVE_AXIS,
                                                                                       OrientationMode::INHERIT,
                                                                                       true,
                                                                                       Ogre::Quaternion(1, 1, 0, 0));
  makeArrow(*control1, 0.5);
  makeArrow(*control1, -0.5);

  InteractiveMarkerControl::Ptr control4 = interactive_marker.createInteractiveControl("rotate_x",
                                                                                       "Rotate around X Axis",
                                                                                       InteractiveMode::ROTATE_AXIS,
                                                                                       OrientationMode::INHERIT,
                                                                                       true,
                                                                                       Ogre::Quaternion(1, 1, 0, 0));
  makeDisc(*control4, 0.3f);

  InteractiveMarkerControl::Ptr control2 = interactive_marker.createInteractiveControl("move_y",
                                                                                       "Move along Y Axis",
                                                                                       InteractiveMode::MOVE_AXIS,
                                                                                       OrientationMode::INHERIT,
                                                                                       true,
                                                                                       Ogre::Quaternion(1, 0, 0, 1));
  makeArrow(*control2, 0.5);
  makeArrow(*control2, -0.5);

  InteractiveMarkerControl::Ptr control5 = interactive_marker.createInteractiveControl("rotate_y",
                                                                                       "Rotate around Y Axis",
                                                                                       InteractiveMode::ROTATE_AXIS,
                                                                                       OrientationMode::INHERIT,
                                                                                       true,
                                                                                       Ogre::Quaternion(1, 0, 0, 1));
  makeDisc(*control5, 0.3f);

  InteractiveMarkerControl::Ptr control3 = interactive_marker.createInteractiveControl("move_z",
                                                                                       "Move along Z Axis",
                                                                                       InteractiveMode::MOVE_AXIS,
                                                                                       OrientationMode::INHERIT,
                                                                                       true,
                                                                                       Ogre::Quaternion(1, 0, 1, 0));
  makeArrow(*control3, 0.5);
  makeArrow(*control3, -0.5);

  InteractiveMarkerControl::Ptr control6 = interactive_marker.createInteractiveControl("rotate_z",
                                                                                       "Rotate around Z Axis",
                                                                                       InteractiveMode::ROTATE_AXIS,
                                                                                       OrientationMode::INHERIT,
                                                                                       true,
                                                                                       Ogre::Quaternion(1, 0, 1, 0));
  makeDisc(*control6, 0.3f);
}

Ogre::ColourValue getDefaultColor(const Ogre::Quaternion& quat)
{
  Ogre::Vector3 bt_x_axis = quat * Ogre::Vector3(1, 0, 0);

  float x, y, z;
  x = std::fabs(bt_x_axis.x);
  y = std::fabs(bt_x_axis.y);
  z = std::fabs(bt_x_axis.z);

  float max_xy = x > y ? x : y;
  float max_yz = y > z ? y : z;
  float max_xyz = max_xy > max_yz ? max_xy : max_yz;

  Ogre::ColourValue color;
  color.r = x / max_xyz;
  color.g = y / max_xyz;
  color.b = z / max_xyz;
  color.a = 0.5f;

  return color;
}

}  // namespace tesseract_rviz
