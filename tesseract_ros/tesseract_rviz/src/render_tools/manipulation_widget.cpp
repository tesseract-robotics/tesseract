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
#include <tesseract_common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <boost/algorithm/string.hpp>
#include <boost/algorithm/string/replace.hpp>
#include <functional>

#include <rviz/display_context.h>
#include <rviz/properties/property.h>
#include <rviz/properties/enum_property.h>
#include <rviz/properties/ros_topic_property.h>
#include <rviz/properties/float_property.h>
#include <rviz/properties/bool_property.h>
#include <rviz/properties/string_property.h>

#include <rviz/window_manager_interface.h>

#include <tesseract_rosutils/utils.h>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract_rviz/markers/utils.h>

#include "tesseract_rviz/render_tools/manipulation_widget.h"
#include "tesseract_rviz/render_tools/visualization_widget.h"
#include "tesseract_rviz/render_tools/link_widget.h"
#include <tesseract_rviz/conversions.h>
#include <tesseract_kinematics/core/utils.h>

namespace tesseract_rviz
{
ManipulationWidget::ManipulationWidget(rviz::Property* widget, rviz::Display* display)
  : root_interactive_node_(nullptr)
  , widget_(widget)
  , display_(display)
  , visualization_(nullptr)
  , tesseract_(nullptr)
  , state_(ManipulatorState::START)
  , env_revision_(0)
  , env_state_(nullptr)
  , tcp_(Eigen::Isometry3d::Identity())
  , enabled_(false)
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

  joint_state_topic_property_ = new rviz::RosTopicProperty("Topic",
                                                           "/tesseract/manipulation_joint_states",
                                                           ros::message_traits::datatype<sensor_msgs::JointState>(),
                                                           "The topic on which the sensor_msgs::JointState messages "
                                                           "are published",
                                                           main_property_,
                                                           SLOT(changedJointStateTopic()),
                                                           this);

  manipulator_property_ = new rviz::EnumProperty(
      "Manipulator", "", "The manipulator to move around.", main_property_, SLOT(changedManipulator()), this);

  tcp_property_ =
      new rviz::EnumProperty("TCP Link", "", "The tool center point link", main_property_, SLOT(changedTCP()), this);

  cartesian_manipulation_property_ = new rviz::BoolProperty("Cartesian Manipulation",
                                                            true,
                                                            "Tool for cartesian manipulation of kinematics objects",
                                                            main_property_,
                                                            SLOT(changedCartesianManipulationEnabled()),
                                                            this);

  cartesian_marker_scale_property_ = new rviz::FloatProperty("Marker Scale",
                                                             0.5,
                                                             "Change the scale of the cartesian interactive marker",
                                                             cartesian_manipulation_property_,
                                                             SLOT(changedCartesianMarkerScale()),
                                                             this);
  cartesian_marker_scale_property_->setMin(0.001f);

  joint_manipulation_property_ = new rviz::BoolProperty("Joint Manipulation",
                                                        true,
                                                        "Tool for joint manipulation of kinematics objects",
                                                        main_property_,
                                                        SLOT(changedJointManipulationEnabled()),
                                                        this);

  joint_marker_scale_property_ = new rviz::FloatProperty("Marker Scale",
                                                         0.5,
                                                         "Change the scale of the joint interactive markers",
                                                         joint_manipulation_property_,
                                                         SLOT(changedJointMarkerScale()),
                                                         this);
  joint_marker_scale_property_->setMin(0.001f);

  joint_values_property_ =
      new rviz::Property("Joint Values", "", "Shows current joint values", main_property_, nullptr, this);
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
                                      const ros::NodeHandle& update_nh,
                                      ManipulatorState state,
                                      const QString& joint_state_topic)
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
    available_manipulators_.clear();
    for (const auto& manip : tesseract_->getInvKinematicsManagerConst()->getAvailableInvKinematicsManipulators())
    {
      available_manipulators_.push_back(QString::fromStdString(manip));
      manipulator_property_->addOptionStd(manip, cnt);
      ++cnt;
    }

    env_revision_ = tesseract_->getEnvironmentConst()->getRevision();
    env_state_ =
        std::make_shared<tesseract_environment::EnvState>(*(tesseract_->getEnvironmentConst()->getCurrentState()));
    joints_ = env_state_->joints;
  }

  joint_state_topic_property_->setValue(joint_state_topic);
  if (state_ == ManipulatorState::START)
    main_property_->setName("Manipulate Start State");
  else
    main_property_->setName("Manipulate End State");

  changedJointStateTopic();
  changedManipulator();
  Q_EMIT availableManipulatorsChanged(available_manipulators_);

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
  enabled_ = true;
  if (state_ == ManipulatorState::START)
    visualization_->setStartStateVisible(enabled_);
  else
    visualization_->setEndStateVisible(enabled_);

  changedJointStateTopic();  // load topic at startup if default used
  Q_EMIT availableManipulatorsChanged(available_manipulators_);

  if (root_interactive_node_ && interactive_marker_)
    interactive_marker_->setVisible(enabled_ && cartesian_manipulation_property_->getBool());

  for (auto& joint_marker : joint_interactive_markers_)
    joint_marker.second->setVisible(enabled_ && joint_manipulation_property_->getBool());

  env_state_ = nullptr;
}

void ManipulationWidget::onDisable()
{
  enabled_ = false;
  if (state_ == ManipulatorState::START)
    visualization_->setStartStateVisible(enabled_);
  else
    visualization_->setEndStateVisible(enabled_);

  if (root_interactive_node_ && interactive_marker_)
    interactive_marker_->setVisible(enabled_);

  for (auto& joint_marker : joint_interactive_markers_)
    joint_marker.second->setVisible(enabled_);

  //  if (trajectory_slider_panel_)
  //    trajectory_slider_panel_->onDisable();
}

void ManipulationWidget::onReset()
{
  // Clear manipulators
  manipulator_property_->clearOptions();
  available_manipulators_.clear();
  available_tcp_links_.clear();
}

void ManipulationWidget::onNameChange(const QString& /*name*/)
{
  //  if (trajectory_slider_dock_panel_)
  //    trajectory_slider_dock_panel_->setWindowTitle(name + " - Slider");
}

void ManipulationWidget::enableCartesianManipulation(bool enabled)
{
  interactive_marker_->setVisible(enabled_ && enabled);
  cartesian_manipulation_property_->setBool(enabled);
}

void ManipulationWidget::enableJointManipulation(bool enabled)
{
  for (auto& joint_marker : joint_interactive_markers_)
    joint_marker.second->setVisible(enabled_ && enabled);

  joint_manipulation_property_->setBool(enabled);
}

void ManipulationWidget::resetToCurrentState() { env_state_ = nullptr; }

bool ManipulationWidget::changeManipulator(const QString& manipulator)
{
  if (tesseract_->isInitialized())
  {
    inv_kin_ = tesseract_->getInvKinematicsManagerConst()->getInvKinematicSolver(manipulator.toStdString());
    if (inv_kin_ == nullptr)
      return false;

    manipulator_property_->setString(manipulator);

    const auto& scene_graph = tesseract_->getEnvironmentConst()->getSceneGraph();
    std::vector<std::string> joint_names = inv_kin_->getJointNames();
    const Eigen::MatrixX2d& limits = inv_kin_->getLimits();
    inv_seed_.resize(inv_kin_->numJoints());
    int i = 0;
    joint_values_property_->removeChildren();
    for (auto& j : joint_names)
    {
      inv_seed_[i] = joints_[j];
      const auto& joint = scene_graph->getJoint(j);
      QString joint_description =
          QString("Limits: [%1, %2]").arg(QString("%1").arg(limits(i, 0)), QString("%1").arg(limits(i, 1)));

      rviz::FloatProperty* joint_value_property = new rviz::FloatProperty(QString::fromStdString(j),
                                                                          static_cast<float>(joints_[j]),
                                                                          joint_description,
                                                                          nullptr,
                                                                          SLOT(userInputJointValuesChanged()),
                                                                          this);

      joint_value_property->setMin(static_cast<float>(limits(i, 0)));
      joint_value_property->setMax(static_cast<float>(limits(i, 1)));
      joint_values_property_->addChild(joint_value_property);
      ++i;
    }

    // Need to update state information (transforms) because manipulator changes and
    env_state_ = tesseract_->getEnvironmentConst()->getState(joints_);

    // Get available TCP's
    QString current_tcp = tcp_property_->getString();
    std::vector<std::string> tcp_links =
        tesseract_->getEnvironmentConst()->getSceneGraph()->getLinkChildrenNames(inv_kin_->getTipLinkName());
    tcp_links.push_back(inv_kin_->getTipLinkName());

    available_tcp_links_.clear();
    tcp_property_->clearOptions();
    for (const auto& tcp_link : tcp_links)
    {
      available_tcp_links_.push_back(QString::fromStdString(tcp_link));
      tcp_property_->addOptionStd(tcp_link);
    }

    if (current_tcp.isEmpty() || !available_tcp_links_.contains(current_tcp))
    {
      tcp_property_->setStdString(inv_kin_->getTipLinkName());
      tcp_ = Eigen::Isometry3d::Identity();
    }
    else
    {
      tcp_property_->setString(current_tcp);
      tcp_ = env_state_->transforms[inv_kin_->getTipLinkName()].inverse() *
             env_state_->transforms[current_tcp.toStdString()];
    }

    Q_EMIT availableTCPLinksChanged(available_tcp_links_);

    // Add 6 DOF interactive marker at the end of the manipulator
    interactive_marker_ = boost::make_shared<InteractiveMarker>("6DOF",
                                                                "Move Robot",
                                                                tesseract_->getEnvironmentConst()->getRootLinkName(),
                                                                root_interactive_node_,
                                                                context_,
                                                                true,
                                                                cartesian_marker_scale_property_->getFloat());
    make6Dof(*interactive_marker_);

    Eigen::Isometry3d pose = env_state_->transforms[inv_kin_->getTipLinkName()] * tcp_;
    Ogre::Vector3 position;
    Ogre::Quaternion orientation;
    toOgre(position, orientation, pose);
    interactive_marker_->setPose(position, orientation, "");

    interactive_marker_->setShowAxes(false);
    interactive_marker_->setShowVisualAids(false);
    interactive_marker_->setShowDescription(false);
    interactive_marker_->setVisible(enabled_ && cartesian_manipulation_property_->getBool());

    connect(interactive_marker_.get(),
            SIGNAL(userFeedback(std::string, Eigen::Isometry3d, Eigen::Vector3d, bool)),
            this,
            SLOT(markerFeedback(std::string, Eigen::Isometry3d, Eigen::Vector3d, bool)));

    // Add joint specific interactive marker
    joint_interactive_markers_.clear();
    for (const auto& joint_name : inv_kin_->getJointNames())
    {
      std::string name = joint_name + "_interactive_marker";
      std::string disc = "Move joint: " + joint_name;
      InteractiveMarker::Ptr interactive_marker =
          boost::make_shared<InteractiveMarker>(name,
                                                disc,
                                                tesseract_->getEnvironmentConst()->getRootLinkName(),
                                                root_interactive_node_,
                                                context_,
                                                true,
                                                joint_marker_scale_property_->getFloat());
      const auto& joint = scene_graph->getJoint(joint_name);

      switch (joint->type)
      {
        case tesseract_scene_graph::JointType::PRISMATIC:
        {
          Eigen::Vector3d disc_axis(1, 0, 0);
          Eigen::Quaternionf q = Eigen::Quaterniond::FromTwoVectors(disc_axis, joint->axis).cast<float>();

          InteractiveMarkerControl::Ptr control =
              interactive_marker->createInteractiveControl("move_" + joint_name,
                                                           "Move prismatic joint: " + joint_name,
                                                           InteractiveMode::MOVE_AXIS,
                                                           OrientationMode::INHERIT,
                                                           true,
                                                           Ogre::Quaternion(q.w(), q.x(), q.y(), q.z()));
          makeArrow(*control, 0.5);
          makeArrow(*control, -0.5);
          joint_interactive_markers_[joint_name] = interactive_marker;
          break;
        }
        case tesseract_scene_graph::JointType::REVOLUTE:
        {
          Eigen::Vector3d disc_axis(1, 0, 0);
          Eigen::Quaternionf q = Eigen::Quaterniond::FromTwoVectors(disc_axis, joint->axis).cast<float>();
          InteractiveMarkerControl::Ptr control =
              interactive_marker->createInteractiveControl("rotate_x",
                                                           "Rotate around X Axis",
                                                           InteractiveMode::ROTATE_AXIS,
                                                           OrientationMode::INHERIT,
                                                           true,
                                                           Ogre::Quaternion(q.w(), q.x(), q.y(), q.z()));
          makeDisc(*control, 0.3f);
          joint_interactive_markers_[joint_name] = interactive_marker;
          break;
        }
        case tesseract_scene_graph::JointType::CONTINUOUS:
        {
          Eigen::Vector3d disc_axis(1, 0, 0);
          Eigen::Quaternionf q = Eigen::Quaterniond::FromTwoVectors(disc_axis, joint->axis).cast<float>();
          InteractiveMarkerControl::Ptr control =
              interactive_marker->createInteractiveControl("rotate_x",
                                                           "Rotate around X Axis",
                                                           InteractiveMode::ROTATE_AXIS,
                                                           OrientationMode::INHERIT,
                                                           true,
                                                           Ogre::Quaternion(q.w(), q.x(), q.y(), q.z()));
          makeDisc(*control, 0.3f);
          joint_interactive_markers_[joint_name] = interactive_marker;
          break;
        }
        default:
          assert(false);
      }

      Eigen::Isometry3d pose = env_state_->transforms[joint->child_link_name];
      Ogre::Vector3 position;
      Ogre::Quaternion orientation;
      toOgre(position, orientation, pose);
      interactive_marker->setPose(position, orientation, "");
      interactive_marker->setShowAxes(false);
      interactive_marker->setShowVisualAids(false);
      interactive_marker->setShowDescription(false);
      interactive_marker->setVisible(enabled_ && joint_manipulation_property_->getBool());

      auto fn = std::bind(&tesseract_rviz::ManipulationWidget::jointMarkerFeedback,
                          this,
                          joint_name,
                          std::placeholders::_1,
                          std::placeholders::_2,
                          std::placeholders::_3,
                          std::placeholders::_4);

      connect(interactive_marker.get(), &tesseract_rviz::InteractiveMarker::userFeedback, this, fn);
    }
    return true;
  }

  return false;
}

bool ManipulationWidget::changeTCP(const QString& tcp_link)
{
  bool success = false;
  if (!env_state_ || tcp_link.isEmpty() || !available_tcp_links_.contains(tcp_link))
  {
    if (inv_kin_)
      tcp_property_->setStdString(inv_kin_->getTipLinkName());

    tcp_ = Eigen::Isometry3d::Identity();
    success = false;
  }
  else
  {
    tcp_property_->setString(tcp_link);
    tcp_ =
        env_state_->transforms[inv_kin_->getTipLinkName()].inverse() * env_state_->transforms[tcp_link.toStdString()];
    success = true;
  }

  if (inv_kin_ && env_state_ && interactive_marker_)
  {
    updateCartesianMarkerVisualization();
  }

  return success;
}

void ManipulationWidget::clickedResetToCurrentState() { resetToCurrentState(); }

void ManipulationWidget::changedManipulator()
{
  changeManipulator(manipulator_property_->getString());

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

void ManipulationWidget::changedTCP() { changeTCP(tcp_property_->getString()); }

void ManipulationWidget::changedCartesianMarkerScale()
{
  if (interactive_marker_)
    interactive_marker_->setSize(cartesian_marker_scale_property_->getFloat());
}

void ManipulationWidget::changedCartesianManipulationEnabled()
{
  enableCartesianManipulation(cartesian_manipulation_property_->getBool());
}

void ManipulationWidget::changedJointMarkerScale()
{
  for (auto& joint_marker : joint_interactive_markers_)
    joint_marker.second->setSize(joint_marker_scale_property_->getFloat());
}

void ManipulationWidget::changedJointManipulationEnabled()
{
  enableJointManipulation(joint_manipulation_property_->getBool());
}

void ManipulationWidget::changedJointStateTopic()
{
  joint_state_pub_.shutdown();
  if (!joint_state_topic_property_->getStdString().empty())
  {
    joint_state_pub_ = nh_.advertise<sensor_msgs::JointState>(joint_state_topic_property_->getStdString(), 5);
  }
}

void ManipulationWidget::markerFeedback(const std::string& reference_frame,
                                        const Eigen::Isometry3d& transform,
                                        const Eigen::Vector3d& /*mouse_point*/,
                                        bool /*mouse_point_valid*/)
{
  if (inv_kin_ && env_state_)
  {
    const Eigen::Isometry3d& ref = env_state_->transforms[reference_frame];
    const Eigen::Isometry3d& base = env_state_->transforms[inv_kin_->getBaseLinkName()];

    Eigen::Isometry3d local_tf = (base.inverse() * ref * transform) * tcp_.inverse();
    Eigen::VectorXd solutions;
    if (inv_kin_->calcInvKin(solutions, local_tf, inv_seed_))
    {
      Eigen::VectorXd temp_seed = solutions.head(inv_kin_->numJoints());
      if (!tesseract_kinematics::isWithinLimits<double>(temp_seed, inv_kin_->getLimits()))
        return;

      inv_seed_ = temp_seed;
      int i = 0;
      for (const auto& j : inv_kin_->getJointNames())
      {
        joints_[j] = inv_seed_[i];
        bool oldState = joint_values_property_->childAt(i)->blockSignals(true);
        joint_values_property_->childAt(i)->setValue(inv_seed_[i]);
        joint_values_property_->childAt(i)->blockSignals(oldState);

        ++i;
      }

      env_state_ = tesseract_->getEnvironmentConst()->getState(joints_);
      updateEnvironmentVisualization();
      updateCartesianMarkerVisualization();
      udpateJointMarkerVisualization();
      publishJointStates();
    }
    else
    {
      updateCartesianMarkerVisualization();
    }
  }
}

void ManipulationWidget::jointMarkerFeedback(const std::string& joint_name,
                                             const std::string& /*reference_frame*/,
                                             const Eigen::Isometry3d& transform,
                                             const Eigen::Vector3d& /*mouse_point*/,
                                             bool /*mouse_point_valid*/)
{
  const auto& scene_graph = tesseract_->getEnvironmentConst()->getSceneGraph();
  const auto& joint = scene_graph->getJoint(joint_name);
  double current_joint_value = env_state_->joints[joint_name];
  Eigen::Isometry3d child_pose = env_state_->transforms[joint->child_link_name];
  Eigen::Isometry3d delta_pose = child_pose.inverse() * transform;

  Eigen::Vector3d delta_axis;
  double delta_joint_value = 0;
  switch (joint->type)
  {
    case tesseract_scene_graph::JointType::PRISMATIC:
    {
      delta_axis = delta_pose.translation().normalized();
      delta_joint_value = delta_pose.translation().norm();
      break;
    }
    case tesseract_scene_graph::JointType::REVOLUTE:
    {
      Eigen::AngleAxisd delta_rotation;
      delta_rotation.fromRotationMatrix(delta_pose.rotation());

      delta_axis = delta_rotation.axis();
      delta_joint_value = delta_rotation.angle();
      break;
    }
    case tesseract_scene_graph::JointType::CONTINUOUS:
    {
      Eigen::AngleAxisd delta_rotation;
      delta_rotation.fromRotationMatrix(delta_pose.rotation());

      delta_axis = delta_rotation.axis();
      delta_joint_value = delta_rotation.angle();
      break;
    }
    default:
      assert(false);
  }

  double new_joint_value;
  if (delta_axis.dot(joint->axis) > 0)
    new_joint_value = current_joint_value + delta_joint_value;
  else
    new_joint_value = current_joint_value - delta_joint_value;

  const Eigen::MatrixX2d& limits = inv_kin_->getLimits();
  int i = 0;
  for (const auto& j : inv_kin_->getJointNames())
  {
    if (joint_name == j)
    {
      if (new_joint_value > limits(i, 1))
      {
        new_joint_value = limits(i, 1);
      }
      else if (new_joint_value < limits(i, 0))
      {
        new_joint_value = limits(i, 0);
      }

      joints_[j] = new_joint_value;
      inv_seed_[i] = new_joint_value;

      bool oldState = joint_values_property_->childAt(i)->blockSignals(true);
      joint_values_property_->childAt(i)->setValue(inv_seed_[i]);
      joint_values_property_->childAt(i)->blockSignals(oldState);
      break;
    }

    ++i;
  }

  env_state_ = tesseract_->getEnvironmentConst()->getState(joints_);
  updateEnvironmentVisualization();
  updateCartesianMarkerVisualization();
  udpateJointMarkerVisualization();
  publishJointStates();
}

void ManipulationWidget::userInputJointValuesChanged()
{
  if (joint_values_property_->numChildren() != static_cast<int>(inv_kin_->numJoints()))
    return;

  int i = 0;
  for (const auto& j : inv_kin_->getJointNames())
  {
    double new_joint_value = joint_values_property_->childAt(i)->getValue().toDouble();

    joints_[j] = new_joint_value;
    inv_seed_[i] = new_joint_value;

    ++i;
  }

  env_state_ = tesseract_->getEnvironmentConst()->getState(joints_);
  updateEnvironmentVisualization();
  updateCartesianMarkerVisualization();
  udpateJointMarkerVisualization();
  publishJointStates();
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
      env_state_ =
          std::make_shared<tesseract_environment::EnvState>(*(tesseract_->getEnvironmentConst()->getCurrentState()));
      joints_ = env_state_->joints;
      updateEnvironmentVisualization();

      if (inv_kin_)
      {
        inv_seed_.resize(inv_kin_->numJoints());
        int i = 0;
        for (const auto& j : inv_kin_->getJointNames())
        {
          inv_seed_[i] = joints_[j];
          bool oldState = joint_values_property_->childAt(i)->blockSignals(true);
          joint_values_property_->childAt(i)->setValue(inv_seed_[i]);
          joint_values_property_->childAt(i)->blockSignals(oldState);
          ++i;
        }

        updateCartesianMarkerVisualization();
        udpateJointMarkerVisualization();
      }
    }

    if (!inv_kin_)
      changedManipulator();

    std::string current_manipulator = manipulator_property_->getStdString();
    std::vector<std::string> manipulators =
        tesseract_->getInvKinematicsManagerConst()->getAvailableInvKinematicsManipulators();

    if (manipulators_.size() != manipulators.size() && !manipulators.empty())
    {
      int cnt = 0;
      manipulator_property_->clearOptions();
      available_manipulators_.clear();
      for (const auto& manip : tesseract_->getInvKinematicsManagerConst()->getAvailableInvKinematicsManipulators())
      {
        available_manipulators_.push_back(QString::fromStdString(manip));
        manipulator_property_->addOptionStd(manip, cnt);
        ++cnt;
      }
      auto it = std::find(manipulators.begin(), manipulators.end(), current_manipulator);

      if (it == manipulators.end())
        manipulator_property_->setStringStd(manipulators[0]);
      else
        manipulator_property_->setStringStd(current_manipulator);

      manipulators_ = manipulators;

      Q_EMIT availableManipulatorsChanged(available_manipulators_);
    }
  }

  interactive_marker_->update(wall_dt);

  for (auto& joint_marker : joint_interactive_markers_)
    joint_marker.second->update(wall_dt);
}

void ManipulationWidget::updateEnvironmentVisualization()
{
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
}

void ManipulationWidget::updateCartesianMarkerVisualization()
{
  Eigen::Isometry3d pose = env_state_->transforms[inv_kin_->getTipLinkName()] * tcp_;
  Ogre::Vector3 position;
  Ogre::Quaternion orientation;
  toOgre(position, orientation, pose);
  interactive_marker_->setPose(position, orientation, "");
}

void ManipulationWidget::udpateJointMarkerVisualization()
{
  const auto& scene_graph = tesseract_->getEnvironmentConst()->getSceneGraph();
  for (auto& joint_marker : joint_interactive_markers_)
  {
    const auto& joint = scene_graph->getJoint(joint_marker.first);
    Eigen::Isometry3d pose = env_state_->transforms[joint->child_link_name];
    Ogre::Vector3 position;
    Ogre::Quaternion orientation;
    toOgre(position, orientation, pose);

    joint_marker.second->setPose(position, orientation, "");
  }
}

void ManipulationWidget::publishJointStates()
{
  sensor_msgs::JointState joint_state;
  tesseract_rosutils::toMsg(joint_state, *env_state_);
  joint_state_pub_.publish(joint_state);
}

// void TrajectoryMonitorWidget::trajectorySliderPanelVisibilityChange(bool enable)
//{
//  if (!trajectory_slider_panel_)
//    return;

//  if (enable)
//    trajectory_slider_panel_->onEnable();
//  else
//    trajectory_slider_panel_->onDisable();
//}

}  // namespace tesseract_rviz
