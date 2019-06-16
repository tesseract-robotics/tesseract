/**
 * @file manipulation_widget.h
 * @brief A manipulators widget for moving the robot around in rviz.
 *
 * @author Levi Armstrong
 * @date Dec 18, 2017
 * @version TODO
 * @bug No known bugs
 *
 * @copyright Copyright (c) 2017, Southwest Research Institute
 *
 * @par License
 * Software License Agreement (Apache License)
 * @par
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 * http://www.apache.org/licenses/LICENSE-2.0
 * @par
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */
#include <tesseract_environment/core/macros.h>
TESSERACT_ENVIRONMENT_IGNORE_WARNINGS_PUSH
#include <boost/algorithm/string.hpp>
#include <boost/algorithm/string/replace.hpp>

#include <rviz/display_context.h>
#include <rviz/properties/property.h>
#include <rviz/properties/enum_property.h>
#include <rviz/properties/ros_topic_property.h>
#include <rviz/properties/float_property.h>
#include <rviz/window_manager_interface.h>

#include <tesseract_rosutils/utils.h>
TESSERACT_ENVIRONMENT_IGNORE_WARNINGS_POP

#include <tesseract_rviz/markers/utils.h>

#include "tesseract_rviz/render_tools/manipulation_widget.h"
#include "tesseract_rviz/render_tools/visualization_widget.h"
#include "tesseract_rviz/render_tools/link_widget.h"

namespace tesseract_rviz
{
ManipulationWidget::ManipulationWidget(rviz::Property* widget, rviz::Display* display)
  : widget_(widget)
  , display_(display)
  , visualization_(nullptr)
  , tesseract_(nullptr)
  , root_interactive_node_(nullptr)
  , state_(ManipulatorState::START)
  , env_revision_(0)
  , env_state_(nullptr)
//  , trajectory_slider_panel_(nullptr)
//  , trajectory_slider_dock_panel_(nullptr)
{
  main_property_ = new ButtonProperty("Manipulation",
                                      "",
                                      "Tool for manipulating kinematics objects",
                                      widget_,
                                      SLOT(clickedResetToCurrentState()),
                                      this);

  main_property_->setCaptions("Reset");

  joint_state_topic_property_ =
      new rviz::RosTopicProperty("Topic",
                                 "/tesseract/manipulation_joint_states",
                                 ros::message_traits::datatype<sensor_msgs::JointState>(),
                                 "The topic on which the sensor_msgs::JointState messages are published",
                                 main_property_,
                                 SLOT(changedJointStateTopic()),
                                 this);

  manipulator_property_ = new rviz::EnumProperty(
      "Manipulator", "", "The manipulator to move around.", main_property_, SLOT(changedManipulator()), this);

  marker_scale_property_ = new rviz::FloatProperty("Marker Scale", 0.5, "Change the scale of the interactive marker", main_property_, SLOT(changedMarkerScale()), this);
  marker_scale_property_->setMin(0.001f);

}

ManipulationWidget::~ManipulationWidget()
{
  if (root_interactive_node_)
    context_->getSceneManager()->destroySceneNode(root_interactive_node_->getName());

//  if (trajectory_slider_dock_panel_)
//    delete trajectory_slider_dock_panel_;
}

void ManipulationWidget::onInitialize(Ogre::SceneNode* root_node,
                                      rviz::DisplayContext* context,
                                      VisualizationWidget::Ptr visualization,
                                      tesseract::Tesseract::Ptr tesseract,
                                      ros::NodeHandle update_nh,
                                      ManipulatorState state)
{
  // Save pointers for later use
  visualization_ = std::move(visualization);
  tesseract_ = std::move(tesseract);
  context_ = context;
  nh_ = update_nh;
  state_ = state;

  root_interactive_node_ = root_node->createChildSceneNode();
  if (tesseract_->isInitialized())
  {
    int cnt = 0;
    for (const auto& manip : tesseract_->getInvKinematicsManagerConst()->getAvailableInvKinematicsManipulators())
    {
      manipulator_property_->addOptionStd(manip, cnt);
      ++cnt;
    }
    env_revision_ = tesseract_->getEnvironmentConst()->getRevision();
    env_state_ = std::make_shared<tesseract_environment::EnvState>(*(tesseract_->getEnvironmentConst()->getCurrentState()));
    joints_ = env_state_->joints;
  }

  if (state_ == ManipulatorState::START)
  {
    main_property_->setName("Manipulate Start State");
    joint_state_topic_property_->setValue("/tesseract/manipulation_start_state");
  }
  else
  {
    main_property_->setName("Manipulate End State");
    joint_state_topic_property_->setValue("/tesseract/manipulation_end_state");
  }

  changedJointStateTopic();
  changedManipulator();



//  rviz::WindowManagerInterface* window_context = context_->getWindowManager();
//  if (window_context)
//  {
//    trajectory_slider_panel_ = new TrajectoryPanel(window_context->getParentWindow());
//    trajectory_slider_dock_panel_ =
//        window_context->addPane(display_->getName() + " - Slider", trajectory_slider_panel_);
//    trajectory_slider_dock_panel_->setIcon(display_->getIcon());
//    connect(trajectory_slider_dock_panel_,
//            SIGNAL(visibilityChanged(bool)),
//            this,
//            SLOT(trajectorySliderPanelVisibilityChange(bool)));
//    trajectory_slider_panel_->onInitialize();
//  }
}

void ManipulationWidget::onEnable()
{
  if (state_ == ManipulatorState::START)
    visualization_->setStartStateVisible(true);
  else
    visualization_->setEndStateVisible(true);

  changedJointStateTopic();  // load topic at startup if default used

  if (root_interactive_node_ && interactive_marker_)
  {
    interactive_marker_->setVisible(true);
  }
}

void ManipulationWidget::onDisable()
{
  if (state_ == ManipulatorState::START)
    visualization_->setStartStateVisible(false);
  else
    visualization_->setEndStateVisible(false);

  if (root_interactive_node_ && interactive_marker_)
  {
    interactive_marker_->setVisible(false);
  }

  env_state_ = nullptr;

//  if (trajectory_slider_panel_)
//    trajectory_slider_panel_->onDisable();
}

void ManipulationWidget::onReset()
{
  // Clear manipulators
  manipulator_property_->clearOptions();
}

void ManipulationWidget::onNameChange(const QString& name)
{
//  if (trajectory_slider_dock_panel_)
//    trajectory_slider_dock_panel_->setWindowTitle(name + " - Slider");
}

void ManipulationWidget::clickedResetToCurrentState()
{
  env_state_ = nullptr;
}

void ManipulationWidget::changedManipulator()
{
  if (tesseract_->isInitialized())
  {
    std::string manipulator = manipulator_property_->getStdString();
    inv_kin_ = tesseract_->getInvKinematicsManagerConst()->getInvKinematicSolver(manipulator);
    if (inv_kin_ == nullptr)
      return;

    std::vector<std::string> joint_names = inv_kin_->getJointNames();
    inv_seed_.resize(inv_kin_->numJoints());
    int i = 0;
    for (auto& j : joint_names)
      inv_seed_[i++] = joints_[j];

    // Need to update state information (transforms) because manipulator changes and
    env_state_ = tesseract_->getEnvironmentConst()->getState(joints_);

    LinkWidget* link = visualization_->getLink(inv_kin_->getTipLinkName());

    interactive_marker_ = boost::make_shared<InteractiveMarker>("Test", "Move Robot", tesseract_->getEnvironmentConst()->getRootLinkName(), root_interactive_node_, context_, true, marker_scale_property_->getFloat());
    make6Dof(*interactive_marker_);

    // Need to modify linkWidget to have a axis for each state even if it does not have visualization
    if (state_ == ManipulatorState::START)
      interactive_marker_->setPose(link->getStartVisualNode()->getPosition(), link->getStartVisualNode()->getOrientation(), "");
    else
      interactive_marker_->setPose(link->getEndVisualNode()->getPosition(), link->getEndVisualNode()->getOrientation(), "");

    interactive_marker_->setShowAxes(false);
    interactive_marker_->setShowVisualAids(false);
    interactive_marker_->setShowDescription(true);

    connect(interactive_marker_.get(),
            SIGNAL(userFeedback(std::string, Eigen::Isometry3d, Eigen::Vector3d, bool)),
            this,
            SLOT(markerFeedback(std::string, Eigen::Isometry3d, Eigen::Vector3d, bool)));

}
//  if (display_mode_property_->getOptionInt() != 2)
//  {
//    if (display_->isEnabled() && displaying_trajectory_message_ && animating_path_)
//      return;

//    clearTrajectoryTrail();

//    if (trajectory_slider_panel_)
//      trajectory_slider_panel_->pauseButton(false);
//  }
//  else
//  {
//    if (trajectory_slider_panel_)
//      trajectory_slider_panel_->pauseButton(true);
//  }
}

void ManipulationWidget::changedMarkerScale()
{
  if (interactive_marker_)
    interactive_marker_->setSize(marker_scale_property_->getFloat());
}


void ManipulationWidget::changedJointStateTopic()
{
  joint_state_pub_.shutdown();
  if (!joint_state_topic_property_->getStdString().empty())
  {
    joint_state_pub_ = nh_.advertise<sensor_msgs::JointState>(joint_state_topic_property_->getStdString(), 5);
  }
}

void ManipulationWidget::markerFeedback(std::string reference_frame, Eigen::Isometry3d transform, Eigen::Vector3d mouse_point, bool mouse_point_valid)
{
  if (inv_kin_ && env_state_)
  {
    const Eigen::Isometry3d& ref = env_state_->transforms[reference_frame];
    const Eigen::Isometry3d& base = env_state_->transforms[inv_kin_->getBaseLinkName()];

    Eigen::Isometry3d local_tf = base.inverse() * ref * transform;
    Eigen::VectorXd solutions;
    if (inv_kin_->calcInvKin(solutions, local_tf, inv_seed_))
    {
      inv_seed_ = solutions.head(inv_kin_->numJoints());
      int i = 0;
      for (auto j : inv_kin_->getJointNames())
        joints_[j] = inv_seed_[i++];

      tesseract_environment::EnvStatePtr env_state = tesseract_->getEnvironmentConst()->getState(joints_);
      if (state_ == ManipulatorState::START)
      {
        for (auto& link_pair : visualization_->getLinks())
        {
          LinkWidget* link = link_pair.second;
          auto it = env_state->transforms.find(link->getName());
          if (it != env_state->transforms.end())
          {
            link->setStartTransform(it->second);
          }
        }
      }
      else
      {
        for (auto& link_pair : visualization_->getLinks())
        {
          LinkWidget* link = link_pair.second;
          auto it = env_state->transforms.find(link->getName());
          if (it != env_state->transforms.end())
          {
            link->setEndTransform(it->second);
          }
        }
      }
      sensor_msgs::JointState joint_state;
      tesseract_rosutils::toMsg(joint_state, *env_state);
      joint_state_pub_.publish(joint_state);
    }
  }
}

void ManipulationWidget::onUpdate(float wall_dt)
{
  if (!tesseract_->isInitialized() || !visualization_)
    return;

  if (tesseract_->isInitialized())
  {
    if (env_revision_ != tesseract_->getEnvironmentConst()->getRevision() || !env_state_)
    {
      env_revision_ = tesseract_->getEnvironmentConst()->getRevision();
      env_state_ = std::make_shared<tesseract_environment::EnvState>(*(tesseract_->getEnvironmentConst()->getCurrentState()));
      joints_ = env_state_->joints;
      if (state_ == ManipulatorState::START)
      {
        for (auto& link_pair : visualization_->getLinks())
        {
          LinkWidget* link = link_pair.second;
          auto it = env_state_->transforms.find(link->getName());
          if (it != env_state_->transforms.end())
          {
            link->setStartTransform(it->second);
          }
        }
      }
      else
      {
        for (auto& link_pair : visualization_->getLinks())
        {
          LinkWidget* link = link_pair.second;
          auto it = env_state_->transforms.find(link->getName());
          if (it != env_state_->transforms.end())
          {
            link->setEndTransform(it->second);
          }
        }
      }
      if (inv_kin_)
      {
        LinkWidget* link = visualization_->getLink(inv_kin_->getTipLinkName());
        interactive_marker_->setPose(link->getPosition(), link->getOrientation(), "");
      }
    }

    if (!inv_kin_)
      changedManipulator();

    std::string current_manipulator = manipulator_property_->getStdString();
    std::vector<std::string> manipulators = tesseract_->getInvKinematicsManagerConst()->getAvailableInvKinematicsManipulators();

    if (manipulators_.size() != manipulators.size() && !manipulators.empty())
    {
      int cnt = 0;
      manipulator_property_->clearOptions();
      for (const auto& manip : tesseract_->getInvKinematicsManagerConst()->getAvailableInvKinematicsManipulators())
      {
        manipulator_property_->addOptionStd(manip, cnt);
        ++cnt;
      }
      auto it = std::find(manipulators.begin(), manipulators.end(), current_manipulator);

      if (it == manipulators.end())
        manipulator_property_->setStringStd(manipulators[0]);
      else
        manipulator_property_->setStringStd(current_manipulator);

      manipulators_ = manipulators;
    }
  }

  interactive_marker_->update(wall_dt);

}

//void TrajectoryMonitorWidget::trajectorySliderPanelVisibilityChange(bool enable)
//{
//  if (!trajectory_slider_panel_)
//    return;

//  if (enable)
//    trajectory_slider_panel_->onEnable();
//  else
//    trajectory_slider_panel_->onDisable();
//}

}  // namespace tesseract_rviz
