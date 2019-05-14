/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2015, University of Colorado, Boulder
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the Univ of CO, Boulder nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

/* Author: Dave Coleman
   Desc:   Wraps a trajectory_visualization playback class for Rviz into a stand
   alone display
*/

#include <tesseract_environment/core/macros.h>
TESSERACT_ENVIRONMENT_IGNORE_WARNINGS_PUSH
#include <rviz/properties/string_property.h>
#include <ros/package.h>

#include <tesseract_environment/kdl/kdl_env.h>
#include <tesseract_scene_graph/parser/urdf_parser.h>
#include <tesseract_scene_graph/parser/srdf_parser.h>
#include <tesseract_scene_graph/utils.h>
#include <tesseract_rosutils/utils.h>
TESSERACT_ENVIRONMENT_IGNORE_WARNINGS_POP

#include <tesseract_rviz/tesseract_trajectory_plugin/tesseract_trajectory_display.h>

namespace tesseract_rviz
{

TesseractTrajectoryDisplay::TesseractTrajectoryDisplay() : Display(), load_env_(false)
{
  // The robot description property is only needed when using the trajectory
  // playback standalone (not within motion
  // planning plugin)
  urdf_description_property_ = new rviz::StringProperty("URDF Description",
                                                        "robot_description",
                                                        "The name of the ROS parameter where the URDF is loaded",
                                                        this,
                                                        SLOT(changedURDFDescription()),
                                                        this);

  trajectory_visual_.reset(new TrajectoryVisualization(this, this));
}

TesseractTrajectoryDisplay::~TesseractTrajectoryDisplay() {}
void TesseractTrajectoryDisplay::onInitialize()
{
  Display::onInitialize();

  trajectory_visual_->onInitialize(scene_node_, context_, update_nh_);
}

void TesseractTrajectoryDisplay::loadEnv()
{
  load_env_ = false;
  // Initial setup
  std::string urdf_xml_string, srdf_xml_string;
  nh_.getParam(urdf_description_property_->getString().toStdString(), urdf_xml_string);
  nh_.getParam(urdf_description_property_->getString().toStdString() + "_semantic", srdf_xml_string);

  // Load URDF model
  if (urdf_xml_string.empty())
  {
    setStatus(rviz::StatusProperty::Error, "TesseractState", "No URDF model loaded");
  }
  else
  {
    tesseract_scene_graph::ResourceLocatorFn locator = tesseract_rosutils::locateResource;
    tesseract_scene_graph::SceneGraphPtr g = tesseract_scene_graph::parseURDF(urdf::parseURDF(urdf_xml_string), locator);
    if (g != nullptr)
    {
      tesseract_scene_graph::SRDFModel srdf;
      bool success = srdf.initString(*g, srdf_xml_string);
      assert(success);

      // Populated the allowed collision matrix
      tesseract_scene_graph::processSRDFAllowedCollisions(*g, srdf);

      tesseract_environment::KDLEnvPtr env = std::make_shared<tesseract_environment::KDLEnv>();
      assert(env != nullptr);

      success = env->init(g);
      assert(success);

      if (success)
      {
        env_ = env;
        trajectory_visual_->onEnvLoaded(env_);
        trajectory_visual_->onEnable();

        setStatus(rviz::StatusProperty::Ok, "TesseractState", "Tesseract Environment Loaded Successfully");
      }
      else
      {
        setStatus(rviz::StatusProperty::Error, "TesseractState", "Tesseract Environment Failed to Load");
      }
    }
    else
    {
      setStatus(rviz::StatusProperty::Error, "TesseractState", "URDF file failed to parse");
    }
  }
}

void TesseractTrajectoryDisplay::reset()
{
  Display::reset();
  loadEnv();
  trajectory_visual_->reset();
}

void TesseractTrajectoryDisplay::onEnable()
{
  Display::onEnable();
  load_env_ = true;  // allow loading of robot model in update()
}

void TesseractTrajectoryDisplay::onDisable()
{
  Display::onDisable();
  trajectory_visual_->onDisable();
}

void TesseractTrajectoryDisplay::update(float wall_dt, float ros_dt)
{
  Display::update(wall_dt, ros_dt);

  if (load_env_)
    loadEnv();

  trajectory_visual_->update(wall_dt, ros_dt);
}

void TesseractTrajectoryDisplay::setName(const QString& name)
{
  BoolProperty::setName(name);
  trajectory_visual_->setName(name);
}

void TesseractTrajectoryDisplay::changedURDFDescription()
{
  if (isEnabled())
    reset();
  else
    loadEnv();
}

}  // namespace tesseract_rviz
