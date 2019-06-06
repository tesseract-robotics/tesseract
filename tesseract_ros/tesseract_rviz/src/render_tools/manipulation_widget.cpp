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
#include <rviz/properties/enum_property.h>
#include <rviz/properties/ros_topic_property.h>
#include <rviz/window_manager_interface.h>

#include <tesseract_rosutils/utils.h>
TESSERACT_ENVIRONMENT_IGNORE_WARNINGS_POP

#include <tesseract_rviz/markers/shape_marker.h>
#include <tesseract_rviz/markers/arrow_marker.h>
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
//  , trajectory_slider_panel_(nullptr)
//  , trajectory_slider_dock_panel_(nullptr)
{

  joint_state_topic_property_ =
      new rviz::RosTopicProperty("Manipulation Topic",
                                 "/tesseract/manipulation_joint_states",
                                 ros::message_traits::datatype<sensor_msgs::JointState>(),
                                 "The topic on which the sensor_msgs::JointState messages are published",
                                 widget_,
                                 SLOT(changedJointStateTopic()),
                                 this);

  manipulator_property_ = new rviz::EnumProperty(
      "Manipulator", "", "The manipulator to move around.", widget_, SLOT(changedManipulator()), this);
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
  }

  if (state_ == ManipulatorState::START)
  {
    joint_state_topic_property_->setName("Manipulation Start State Topic");
    joint_state_topic_property_->setValue("/tesseract/manipulation_start_state");
  }
  else
  {
    joint_state_topic_property_->setName("Manipulation End State Topic");
    joint_state_topic_property_->setValue("/tesseract/manipulation_end_state");
  }

  changedJointStateTopic();


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



void ManipulationWidget::changedManipulator()
{
  LinkWidget* link = visualization_->getLink("tool0");
  interactive_marker_ = boost::make_shared<InteractiveMarker>("Test", "Move Robot", root_interactive_node_, context_);

  InteractiveMarkerControl::Ptr control = interactive_marker_->createInteractiveControl("Test", "Move Rotate 3D",
                                                                                        InteractiveMode::MOVE_ROTATE_3D, OrientationMode::INHERIT,
                                                                                        true, link->getOrientation());
  makeSphere(*control, 0.35f);

  InteractiveMarkerControl::Ptr control1 = interactive_marker_->createInteractiveControl("Test1", "Move Axis",
                                                                                         InteractiveMode::MOVE_AXIS, OrientationMode::INHERIT,
                                                                                         true, link->getOrientation());
  makeArrow(*control1, 0.5);
  makeArrow(*control1, -0.5);

  InteractiveMarkerControl::Ptr control2 = interactive_marker_->createInteractiveControl("Test2", "Rotate Axis",
                                                                                         InteractiveMode::ROTATE_AXIS, OrientationMode::INHERIT,
                                                                                         true, link->getOrientation());
  makeDisc(*control2, 0.5);


  interactive_marker_->setShowAxes(false);
  interactive_marker_->setShowVisualAids(true);
  interactive_marker_->setPose(link->getPosition(), link->getOrientation(), "Test");

  connect(interactive_marker_.get(),
          SIGNAL(userFeedback(visualization_msgs::InteractiveMarkerFeedback&)),
          this,
          SLOT(markerFeedback(visualization_msgs::InteractiveMarkerFeedback&)));


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


void ManipulationWidget::changedJointStateTopic()
{
  joint_state_pub_.shutdown();
  if (!joint_state_topic_property_->getStdString().empty())
  {
    joint_state_pub_ = nh_.advertise<sensor_msgs::JointState>(joint_state_topic_property_->getStdString(), 5);
  }
}

void ManipulationWidget::markerFeedback(visualization_msgs::InteractiveMarkerFeedback &feedback)
{
  static int a = 0;
  ++a;
  ROS_ERROR_STREAM ("" << a);
}

void ManipulationWidget::onUpdate(float wall_dt)
{
  if (!tesseract_->isInitialized() || !visualization_)
    return;

  if (tesseract_->isInitialized())
  {
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
