/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2012, Willow Garage, Inc.
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
 *   * Neither the name of Willow Garage nor the names of its
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

/* Author: Ioan Sucan */

#include <tesseract_environment/core/macros.h>
TESSERACT_ENVIRONMENT_IGNORE_WARNINGS_PUSH
#include <QApplication>
TESSERACT_ENVIRONMENT_IGNORE_WARNINGS_POP

#include "tesseract_rviz/render_tools/state_visualization.h"
#include "tesseract_rviz/render_tools/link_updater.h"
//#include <tesseract_ros/ros_tesseract_utils.h>

namespace tesseract_rviz
{

StateVisualization::StateVisualization(Ogre::SceneNode* root_node,
                                       rviz::DisplayContext* context,
                                       const std::string& name,
                                       rviz::Property* parent_property)
  : robot_(root_node, context, name, parent_property), visible_(true), visual_visible_(true), collision_visible_(false)
{

}

void StateVisualization::load(const tesseract_scene_graph::SceneGraphConstPtr& scene_graph,
                              bool visual,
                              bool collision,
                              bool show_active,
                              bool show_static)
{
  // clear previously loaded model
  clear();

  robot_.load(scene_graph, visual, collision, show_active, show_static);
  robot_.setVisualVisible(visual_visible_);
  robot_.setCollisionVisible(collision_visible_);
  robot_.setVisible(visible_);
}

void StateVisualization::clear() { robot_.clear(); }

void StateVisualization::update(const tesseract_environment::EnvironmentConstPtr& env,
                                const tesseract_environment::EnvStateConstPtr& state)
{
  updateHelper(env, state);
}

void StateVisualization::updateHelper(const tesseract_environment::EnvironmentConstPtr& env,
                                      const tesseract_environment::EnvStateConstPtr& state)
{
  // TODO Need to update based on environment change
  robot_.update(LinkUpdater(state));
  robot_.setVisualVisible(visual_visible_);
  robot_.setCollisionVisible(collision_visible_);
  robot_.setVisible(visible_);
}

void StateVisualization::setVisible(bool visible)
{
  visible_ = visible;
  robot_.setVisible(visible);
}

void StateVisualization::setVisualVisible(bool visible)
{
  visual_visible_ = visible;
  robot_.setVisualVisible(visible);
}

void StateVisualization::setCollisionVisible(bool visible)
{
  collision_visible_ = visible;
  robot_.setCollisionVisible(visible);
}

void StateVisualization::setAlpha(float alpha) { robot_.setAlpha(alpha); }
}
