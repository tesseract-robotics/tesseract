/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2008, Willow Garage, Inc.
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

/* Author: Dave Coleman */

#include <tesseract_environment/core/macros.h>
TESSERACT_ENVIRONMENT_IGNORE_WARNINGS_PUSH
#include <boost/algorithm/string.hpp>
#include <boost/algorithm/string/replace.hpp>

#include <rviz/display_context.h>
#include <rviz/properties/bool_property.h>
#include <rviz/properties/color_property.h>
#include <rviz/properties/editable_enum_property.h>
#include <rviz/properties/enum_property.h>
#include <rviz/properties/float_property.h>
#include <rviz/properties/int_property.h>
#include <rviz/properties/property.h>
#include <rviz/properties/ros_topic_property.h>
#include <rviz/properties/string_property.h>
#include <rviz/window_manager_interface.h>

#include <tesseract_rosutils/utils.h>
TESSERACT_ENVIRONMENT_IGNORE_WARNINGS_POP

#include "tesseract_rviz/render_tools/trajectory_visualization.h"
#include "tesseract_rviz/render_tools/env_visualization.h"
#include "tesseract_rviz/render_tools/env_link.h"
#include "tesseract_rviz/render_tools/env_link_updater.h"

namespace tesseract_rviz
{
TrajectoryVisualization::TrajectoryVisualization(rviz::Property* widget, rviz::Display* display)
  : widget_(widget)
  , display_(display)
  , animating_path_(false)
  , drop_displaying_trajectory_(false)
  , current_state_(-1)
  , trajectory_slider_panel_(nullptr)
  , trajectory_slider_dock_panel_(nullptr)
{
  trajectory_topic_property_ =
      new rviz::RosTopicProperty("Trajectory Topic",
                                 "/tesseract/display_tesseract_trajectory",
                                 ros::message_traits::datatype<tesseract_msgs::Trajectory>(),
                                 "The topic on which the tesseract_msgs::Trajectory messages are received",
                                 widget,
                                 SLOT(changedTrajectoryTopic()),
                                 this);

  display_path_visual_enabled_property_ =
      new rviz::BoolProperty("Show Visual",
                             true,
                             "Whether to display the visual representation of the path.",
                             widget,
                             SLOT(changedDisplayPathVisualEnabled()),
                             this);

  display_path_collision_enabled_property_ =
      new rviz::BoolProperty("Show Collision",
                             false,
                             "Whether to display the collision representation of the path.",
                             widget,
                             SLOT(changedDisplayPathCollisionEnabled()),
                             this);

  path_alpha_property_ = new rviz::FloatProperty(
      "Alpha", 0.5f, "Specifies the alpha for links with geometry", widget, SLOT(changedPathAlpha()), this);
  path_alpha_property_->setMin(0.0);
  path_alpha_property_->setMax(1.0);

  display_mode_property_ = new rviz::EnumProperty(
      "Display Mode", "Loop", "How to display the trajectoy.", widget, SLOT(changedDisplayMode()), this);
  display_mode_property_->addOptionStd("Single", 0);
  display_mode_property_->addOptionStd("Loop", 1);
  display_mode_property_->addOptionStd("Trail", 2);

  state_display_time_property_ = new rviz::EditableEnumProperty("State Display Time",
                                                                "0.05 s",
                                                                "The amount of wall-time to wait in between displaying "
                                                                "states along a received trajectory path",
                                                                widget,
                                                                SLOT(changedStateDisplayTime()),
                                                                this);
  state_display_time_property_->addOptionStd("REALTIME");
  state_display_time_property_->addOptionStd("0.05 s");
  state_display_time_property_->addOptionStd("0.1 s");
  state_display_time_property_->addOptionStd("0.5 s");

  trail_step_size_property_ = new rviz::IntProperty("Trail Step Size",
                                                    1,
                                                    "Specifies the step size of the samples "
                                                    "shown in the trajectory trail.",
                                                    widget,
                                                    SLOT(changedTrailStepSize()),
                                                    this);
  trail_step_size_property_->setMin(1);

  interrupt_display_property_ = new rviz::BoolProperty("Interrupt Display",
                                                       false,
                                                       "Immediately show newly planned trajectory, "
                                                       "interrupting the currently displayed one.",
                                                       widget);

  default_color_property_ = new rviz::ColorProperty(
      "Robot Color", QColor(150, 50, 150), "The color of the animated robot", widget, SLOT(changedColor()), this);

  enable_default_color_property_ = new rviz::BoolProperty(
      "Color Enabled", false, "Specifies whether custom coloring is enabled", widget, SLOT(enabledColor()), this);
}

TrajectoryVisualization::~TrajectoryVisualization()
{
  clearTrajectoryTrail();
  trajectory_message_to_display_.reset();
  displaying_trajectory_message_.reset();

  display_path_.reset();
  if (trajectory_slider_dock_panel_)
    delete trajectory_slider_dock_panel_;
}

void TrajectoryVisualization::onInitialize(Ogre::SceneNode* scene_node,
                                           rviz::DisplayContext* context,
                                           ros::NodeHandle update_nh)
{
  // Save pointers for later use
  scene_node_ = scene_node;
  context_ = context;
  update_nh_ = update_nh;

  // Load trajectory
  display_path_ = std::make_shared<EnvVisualization>(scene_node_, context_, "Planned Path", widget_);
  display_path_->setVisualVisible(display_path_visual_enabled_property_->getBool());
  display_path_->setCollisionVisible(display_path_collision_enabled_property_->getBool());
  display_path_->setVisible(true);

  previous_display_mode_ = display_mode_property_->getOptionInt();

  rviz::WindowManagerInterface* window_context = context_->getWindowManager();
  if (window_context)
  {
    trajectory_slider_panel_ = new TrajectoryPanel(window_context->getParentWindow());
    trajectory_slider_dock_panel_ =
        window_context->addPane(display_->getName() + " - Slider", trajectory_slider_panel_);
    trajectory_slider_dock_panel_->setIcon(display_->getIcon());
    connect(trajectory_slider_dock_panel_,
            SIGNAL(visibilityChanged(bool)),
            this,
            SLOT(trajectorySliderPanelVisibilityChange(bool)));
    trajectory_slider_panel_->onInitialize();
  }
}

void TrajectoryVisualization::setName(const QString& name)
{
  if (trajectory_slider_dock_panel_)
    trajectory_slider_dock_panel_->setWindowTitle(name + " - Slider");
}

void TrajectoryVisualization::onEnvLoaded(tesseract_environment::EnvironmentPtr env)
{
  env_ = env;

  // Error check
  if (env_ == nullptr)
  {
    ROS_ERROR_STREAM_NAMED("trajectory_visualization", "No environment found");
    return;
  }

  // Load rviz environment
  display_path_->load(env_->getSceneGraph());
  display_path_->update(EnvLinkUpdater(env_->getCurrentState()));
  enabledColor();  // force-refresh to account for saved display configuration
}

void TrajectoryVisualization::reset()
{
  clearTrajectoryTrail();
  trajectory_message_to_display_.reset();
  displaying_trajectory_message_.reset();
  animating_path_ = false;

  display_path_->setVisualVisible(display_path_visual_enabled_property_->getBool());
  display_path_->setCollisionVisible(display_path_collision_enabled_property_->getBool());
}

void TrajectoryVisualization::clearTrajectoryTrail()
{
  for (auto& link_pair : display_path_->getLinks())
    link_pair.second->clearTrajectory();
}

void TrajectoryVisualization::createTrajectoryTrail()
{
  clearTrajectoryTrail();

  tesseract_msgs::TrajectoryPtr t = trajectory_message_to_display_;
  if (!t)
    t = displaying_trajectory_message_;

  if (!t)
    return;

  int stepsize = trail_step_size_property_->getInt();
  // always include last trajectory point
  int num_waypoints = static_cast<int>(t->joint_trajectory.points.size());
  num_trajectory_waypoints_ = static_cast<size_t>(std::ceil(static_cast<float>(num_waypoints + stepsize - 1) / static_cast<float>(stepsize)));
  std::vector<tesseract_environment::EnvStatePtr> states_data;
  states_data.reserve(num_trajectory_waypoints_);
  for (std::size_t i = 0; i < num_trajectory_waypoints_; i++)
  {
    unsigned waypoint_i = static_cast<unsigned>(std::min(
        i * static_cast<size_t>(stepsize), static_cast<size_t>(num_waypoints - 1)));  // limit to last trajectory point

    std::unordered_map<std::string, double> joints;
    for (unsigned j = 0; j < t->joint_trajectory.joint_names.size(); ++j)
    {
      joints[t->joint_trajectory.joint_names[j]] = t->joint_trajectory.points[waypoint_i].positions[j];
    }

    states_data.push_back(env_->getState(joints));
  }

  for (const auto& link_name : env_->getActiveLinkNames())
  {
    std::vector<Eigen::Isometry3d> link_trajectory;
    link_trajectory.reserve(states_data.size());
    for (auto& state : states_data)
    {
      link_trajectory.push_back(state->transforms[link_name]);
    }
    display_path_->getLink(link_name)->setTrajectory(link_trajectory);
  }
}

void TrajectoryVisualization::changedDisplayMode()
{
  if (display_mode_property_->getOptionInt() != 2)
  {
    if (display_->isEnabled() && displaying_trajectory_message_ && animating_path_)
      return;

    clearTrajectoryTrail();

    if (trajectory_slider_panel_)
      trajectory_slider_panel_->pauseButton(false);
  }
  else
  {
    if (trajectory_slider_panel_)
      trajectory_slider_panel_->pauseButton(true);
  }
}

void TrajectoryVisualization::changedTrailStepSize()
{
  if (display_mode_property_->getOptionInt() == 2)
  {
    createTrajectoryTrail();
    if (trajectory_slider_panel_)
      trajectory_slider_panel_->update(static_cast<int>(num_trajectory_waypoints_));
  }
}

void TrajectoryVisualization::changedPathAlpha()
{
  display_path_->setAlpha(path_alpha_property_->getFloat());
}

void TrajectoryVisualization::changedTrajectoryTopic()
{
  trajectory_topic_sub_.shutdown();
  if (!trajectory_topic_property_->getStdString().empty())
  {
    trajectory_topic_sub_ = update_nh_.subscribe(
        trajectory_topic_property_->getStdString(), 5, &TrajectoryVisualization::incomingDisplayTrajectory, this);
  }
}

void TrajectoryVisualization::changedStateDisplayTime() {}
void TrajectoryVisualization::changedDisplayPathVisualEnabled()
{
  if (display_->isEnabled())
  {
    display_path_->setVisualVisible(display_path_visual_enabled_property_->getBool());
  }
}

void TrajectoryVisualization::changedDisplayPathCollisionEnabled()
{
  if (display_->isEnabled())
  {
    display_path_->setCollisionVisible(display_path_collision_enabled_property_->getBool());
  }
}

void TrajectoryVisualization::onEnable()
{
  changedPathAlpha();  // set alpha property

  display_path_->setVisualVisible(display_path_visual_enabled_property_->getBool());
  display_path_->setCollisionVisible(display_path_collision_enabled_property_->getBool());
  display_path_->setVisible(true);

  changedTrajectoryTopic();  // load topic at startup if default used
}

void TrajectoryVisualization::onDisable()
{
  display_path_->setVisible(false);

  displaying_trajectory_message_.reset();
  animating_path_ = false;

  if (trajectory_slider_panel_)
    trajectory_slider_panel_->onDisable();
}

void TrajectoryVisualization::interruptCurrentDisplay()
{
  // update() starts a new trajectory as soon as it is available
  // interrupting may cause the newly received trajectory to interrupt
  // hence, only interrupt when current_state_ already advanced past first
  if (current_state_ > 0)
    animating_path_ = false;
}

float TrajectoryVisualization::getStateDisplayTime()
{
  std::string tm = state_display_time_property_->getStdString();
  if (tm == "REALTIME")
    return -1.0;
  else
  {
    boost::replace_all(tm, "s", "");
    boost::trim(tm);
    float t = 0.05f;
    try
    {
      t = boost::lexical_cast<float>(tm);
    }
    catch (const boost::bad_lexical_cast& ex)
    {
      state_display_time_property_->setStdString("0.05 s");
    }
    return t;
  }
}

void TrajectoryVisualization::dropTrajectory() { drop_displaying_trajectory_ = true; }
void TrajectoryVisualization::update(float wall_dt, float /*ros_dt*/)
{
  if (drop_displaying_trajectory_)
  {
    animating_path_ = false;
    displaying_trajectory_message_.reset();
    trajectory_slider_panel_->update(0);
    drop_displaying_trajectory_ = false;
  }

  if (!animating_path_)
  {  // finished last animation?

    boost::mutex::scoped_lock lock(update_trajectory_message_);
    // new trajectory available to display?
    if (trajectory_message_to_display_ && !trajectory_message_to_display_->joint_trajectory.points.empty())
    {
      animating_path_ = true;

      if (display_mode_property_->getOptionInt() == 2)
        animating_path_ = false;

      displaying_trajectory_message_ = trajectory_message_to_display_;
      createTrajectoryTrail();
      if (trajectory_slider_panel_)
        trajectory_slider_panel_->update(static_cast<int>(num_trajectory_waypoints_));
    }
    else if (displaying_trajectory_message_)
    {
      if (display_mode_property_->getOptionInt() == 1)
      {
        animating_path_ = true;
      }
      else if (display_mode_property_->getOptionInt() == 0)
      {
        if (previous_display_mode_ != display_mode_property_->getOptionInt())
        {
          animating_path_ = true;
        }
        else
        {
          if (static_cast<unsigned>(trajectory_slider_panel_->getSliderPosition()) == (num_trajectory_waypoints_ - 1))
            animating_path_ = false;
          else
            animating_path_ = true;
        }
      }
      else
      {
        if (previous_display_mode_ != display_mode_property_->getOptionInt())
        {
          createTrajectoryTrail();
          if (trajectory_slider_panel_)
            trajectory_slider_panel_->update(static_cast<int>(num_trajectory_waypoints_));
        }

        animating_path_ = false;
      }
      previous_display_mode_ = display_mode_property_->getOptionInt();
    }
    trajectory_message_to_display_.reset();

    if (animating_path_)
    {
      current_state_ = -1;
      current_state_time_ = std::numeric_limits<float>::infinity();

      for (auto& link_pair : display_path_->getLinks())
        link_pair.second->showTrajectoryWaypointOnly(0);

      if (trajectory_slider_panel_)
        trajectory_slider_panel_->setSliderPosition(0);
    }
  }

  if (animating_path_)
  {
    float tm = getStateDisplayTime();
    if (tm < 0.0)  // if we should use realtime
    {
      ros::Duration d = displaying_trajectory_message_->joint_trajectory.points[static_cast<size_t>(current_state_) + 1]
                            .time_from_start;
      if (d.isZero())
        tm = 0;
      else
        tm = static_cast<float>(
            (d -
             displaying_trajectory_message_->joint_trajectory.points[static_cast<size_t>(current_state_)]
                 .time_from_start)
                .toSec());
    }

    if (current_state_time_ > tm)
    {
      if (trajectory_slider_panel_ && trajectory_slider_panel_->isVisible() && trajectory_slider_panel_->isPaused())
        current_state_ = trajectory_slider_panel_->getSliderPosition();
      else
        ++current_state_;

      if (current_state_ < num_trajectory_waypoints_)
      {
        if (trajectory_slider_panel_)
          trajectory_slider_panel_->setSliderPosition(current_state_);

        for (auto& link_pair : display_path_->getLinks())
          link_pair.second->showTrajectoryWaypointOnly(current_state_);

      }
      else
      {
        animating_path_ = false;  // animation finished
        if ((display_mode_property_->getOptionInt() != 1) && trajectory_slider_panel_)
          trajectory_slider_panel_->pauseButton(true);
      }
      current_state_time_ = 0.0f;
    }
    current_state_time_ += wall_dt;
  }
}

void TrajectoryVisualization::incomingDisplayTrajectory(const tesseract_msgs::Trajectory::ConstPtr& msg)
{
  // Error check
  if (!env_)
  {
    ROS_ERROR_STREAM_NAMED("trajectory_visualization", "No environment");
    return;
  }

  if (!msg->model_id.empty() && msg->model_id != env_->getSceneGraph()->getName())
    ROS_WARN("Received a trajectory to display for model '%s' but model '%s' "
             "was expected",
             msg->model_id.c_str(),
             env_->getSceneGraph()->getName().c_str());

  // Setup environment
  tesseract_rosutils::processMsg(env_, msg->trajectory_start);

  if (!msg->joint_trajectory.points.empty())
  {
    bool joints_equal = true;
    if (trajectory_message_to_display_)
    {
      if ((trajectory_message_to_display_->joint_trajectory.points.size() == msg->joint_trajectory.points.size()) &&
          (trajectory_message_to_display_->joint_trajectory.joint_names.size() ==
           msg->joint_trajectory.joint_names.size()))
      {
        for (unsigned i = 0; i < msg->joint_trajectory.points.size(); ++i)
        {
          for (unsigned j = 0; j < msg->joint_trajectory.joint_names.size(); ++j)
          {
            double delta = msg->joint_trajectory.points[i].positions[j] -
                           trajectory_message_to_display_->joint_trajectory.points[i].positions[j];
            joints_equal &= (std::abs(delta) < std::numeric_limits<double>::epsilon());
          }
        }
      }
      else
      {
        joints_equal = false;
      }
    }

    trajectory_message_to_display_.reset();
    if (!msg->joint_trajectory.points.empty() || !joints_equal)
    {
      boost::mutex::scoped_lock lock(update_trajectory_message_);
      trajectory_message_to_display_.reset(new tesseract_msgs::Trajectory(*msg));
      if (interrupt_display_property_->getBool())
        interruptCurrentDisplay();
    }
  }
  else
  {
    trajectory_message_to_display_.reset();
  }
}

void TrajectoryVisualization::changedColor()
{
  if (enable_default_color_property_->getBool())
    setColor(display_path_, default_color_property_->getColor());
}

void TrajectoryVisualization::enabledColor()
{
  if (enable_default_color_property_->getBool())
    setColor(display_path_, default_color_property_->getColor());
  else
    unsetColor(display_path_);
}

void TrajectoryVisualization::unsetColor(EnvVisualization::Ptr env)
{
  for (auto& link : env->getLinks())
    link.second->unsetColor();
}

void TrajectoryVisualization::setColor(EnvVisualization::Ptr env, const QColor& color)
{
  for (auto& link : env->getLinks())
    env->getLink(link.first)
        ->setColor(
            static_cast<float>(color.redF()), static_cast<float>(color.greenF()), static_cast<float>(color.blueF()));
}

void TrajectoryVisualization::trajectorySliderPanelVisibilityChange(bool enable)
{
  if (!trajectory_slider_panel_)
    return;

  if (enable)
    trajectory_slider_panel_->onEnable();
  else
    trajectory_slider_panel_->onDisable();
}

}  // namespace tesseract_rviz
