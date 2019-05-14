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

/* Author: Ioan Sucan */
#include <tesseract_environment/core/macros.h>
TESSERACT_ENVIRONMENT_IGNORE_WARNINGS_PUSH
#include <ros/package.h>

#include <eigen_conversions/eigen_msg.h>
#include <octomap_msgs/conversions.h>
#include <rviz/display_context.h>
#include <rviz/frame_manager.h>
#include <rviz/properties/bool_property.h>
#include <rviz/properties/color_property.h>
#include <rviz/properties/float_property.h>
#include <rviz/properties/property.h>
#include <rviz/properties/ros_topic_property.h>
#include <rviz/properties/string_property.h>
#include <rviz/visualization_manager.h>
#include <tf/transform_listener.h>

#include <OgreSceneManager.h>
#include <OgreSceneNode.h>
TESSERACT_ENVIRONMENT_IGNORE_WARNINGS_POP

#include <tesseract_environment/kdl/kdl_env.h>
#include <tesseract_scene_graph/parser/urdf_parser.h>
#include <tesseract_scene_graph/parser/srdf_parser.h>
#include <tesseract_scene_graph/utils.h>
#include <tesseract_rosutils/utils.h>
#include <tesseract_rviz/render_tools/env_visualization.h>
#include <tesseract_rviz/render_tools/env_link.h>
#include <tesseract_rviz/render_tools/env_link_updater.h>
#include <tesseract_rviz/tesseract_state_plugin/tesseract_state_display.h>

namespace tesseract_rviz
{

const std::string DEFAULT_MODIFY_ENVIRONMENT_SERVICE = "modify_tesseract";

// ******************************************************************************************
// Base class contructor
// ******************************************************************************************
TesseractStateDisplay::TesseractStateDisplay() : Display(), update_state_(false), load_env_(false)
{
  urdf_description_property_ =
      new rviz::StringProperty("URDF Description",
                               "robot_description",
                               "The name of the ROS parameter where the URDF for the robot is loaded",
                               this,
                               SLOT(changedURDFDescription()),
                               this);

  joint_state_topic_property_ =
      new rviz::RosTopicProperty("Joint State Topic",
                                 "joint_states",
                                 ros::message_traits::datatype<sensor_msgs::JointState>(),
                                 "The topic on which the sensor_msgs::JointState messages are received",
                                 this,
                                 SLOT(changedJointStateTopic()),
                                 this);

  // Planning scene category
  // -------------------------------------------------------------------------------------------
  root_link_name_property_ = new rviz::StringProperty(
      "Root Link", "", "Shows the name of the root link for the urdf", this, SLOT(changedRootLinkName()), this);
  root_link_name_property_->setReadOnly(true);

  alpha_property_ = new rviz::FloatProperty(
      "Alpha", 1.0f, "Specifies the alpha for the links with geometry", this, SLOT(changedURDFSceneAlpha()), this);
  alpha_property_->setMin(0.0);
  alpha_property_->setMax(1.0);

  enable_link_highlight_ = new rviz::BoolProperty("Show Highlights",
                                                  true,
                                                  "Specifies whether link highlighting is enabled",
                                                  this,
                                                  SLOT(changedEnableLinkHighlight()),
                                                  this);
  enable_visual_visible_ = new rviz::BoolProperty("Show Visual",
                                                  true,
                                                  "Whether to display the visual representation of the environment.",
                                                  this,
                                                  SLOT(changedEnableVisualVisible()),
                                                  this);

  enable_collision_visible_ = new rviz::BoolProperty("Show Collision",
                                                     false,
                                                     "Whether to display the collision "
                                                     "representation of the environment.",
                                                     this,
                                                     SLOT(changedEnableCollisionVisible()),
                                                     this);

  show_all_links_ = new rviz::BoolProperty(
      "Show All Links", true, "Toggle all links visibility on or off.", this, SLOT(changedAllLinks()), this);

  modify_environment_server_ =
      nh_.advertiseService(DEFAULT_MODIFY_ENVIRONMENT_SERVICE, &TesseractStateDisplay::modifyEnvironmentCallback, this);
}

// ******************************************************************************************
// Deconstructor
// ******************************************************************************************
TesseractStateDisplay::~TesseractStateDisplay() {}
void TesseractStateDisplay::onInitialize()
{
  Display::onInitialize();
  visualization_ = std::make_shared<EnvVisualization>(scene_node_, context_, "Tesseract State", this);
  changedEnableVisualVisible();
  changedEnableCollisionVisible();
  visualization_->setVisible(false);
}

void TesseractStateDisplay::reset()
{
  visualization_->clear();
  //  rdf_loader_.reset();
  Display::reset();

  loadURDFModel();
}

void TesseractStateDisplay::changedAllLinks()
{
  Property* links_prop = subProp("Links");
  QVariant value(show_all_links_->getBool());

  for (int i = 0; i < links_prop->numChildren(); ++i)
  {
    Property* link_prop = links_prop->childAt(i);
    link_prop->setValue(value);
  }
}

//void TesseractStateDisplay::setHighlightedLink(const std::string& link_name, const std_msgs::ColorRGBA& color)
//{
//  RobotLink* link = state_->getRobot().getLink(link_name);
//  if (link)
//  {
//    link->setColor(color.r, color.g, color.b);
//    link->setRobotAlpha(color.a * alpha_property_->getFloat());
//  }
//}

//void TesseractStateDisplay::unsetHighlightedLink(const std::string& link_name)
//{
//  RobotLink* link = state_->getRobot().getLink(link_name);
//  if (link)
//  {
//    link->unsetColor();
//    link->setRobotAlpha(alpha_property_->getFloat());
//  }
//}

void TesseractStateDisplay::changedEnableLinkHighlight()
{
//  if (enable_link_highlight_->getBool())
//  {
//    for (std::map<std::string, std_msgs::ColorRGBA>::iterator it = highlights_.begin(); it != highlights_.end(); ++it)
//    {
//      setHighlightedLink(it->first, it->second);
//    }
//  }
//  else
//  {
//    for (std::map<std::string, std_msgs::ColorRGBA>::iterator it = highlights_.begin(); it != highlights_.end(); ++it)
//    {
//      unsetHighlightedLink(it->first);
//    }
//  }
}

void TesseractStateDisplay::changedEnableVisualVisible()
{
  visualization_->setVisualVisible(enable_visual_visible_->getBool());
}

void TesseractStateDisplay::changedEnableCollisionVisible()
{
  visualization_->setCollisionVisible(enable_collision_visible_->getBool());
}

static bool operator!=(const std_msgs::ColorRGBA& a, const std_msgs::ColorRGBA& b)
{
  return a.r != b.r || a.g != b.g || a.b != b.b || a.a != b.a;
}

//void TesseractStateDisplay::setHighlightedLinks(const tesseract_msgs::TesseractState::_highlight_links_type& highlight_links)
//{
//  if (highlight_links.empty() && highlights_.empty())
//    return;

//  std::map<std::string, std_msgs::ColorRGBA> highlights;
//  for (tesseract_msgs::TesseractState::_highlight_links_type::const_iterator it = highlight_links.begin();
//       it != highlight_links.end();
//       ++it)
//  {
//    highlights[it->name] = it->visual[0];
//  }

//  if (enable_link_highlight_->getBool())
//  {
//    std::map<std::string, std_msgs::ColorRGBA>::iterator ho = highlights_.begin();
//    std::map<std::string, std_msgs::ColorRGBA>::iterator hn = highlights.begin();
//    while (ho != highlights_.end() || hn != highlights.end())
//    {
//      if (ho == highlights_.end())
//      {
//        setHighlightedLink(hn->first, hn->second);
//        ++hn;
//      }
//      else if (hn == highlights.end())
//      {
//        unsetHighlightedLink(ho->first);
//        ++ho;
//      }
//      else if (hn->first < ho->first)
//      {
//        setHighlightedLink(hn->first, hn->second);
//        ++hn;
//      }
//      else if (hn->first > ho->first)
//      {
//        unsetHighlightedLink(ho->first);
//        ++ho;
//      }
//      else if (hn->second != ho->second)
//      {
//        setHighlightedLink(hn->first, hn->second);
//        ++ho;
//        ++hn;
//      }
//      else
//      {
//        ++ho;
//        ++hn;
//      }
//    }
//  }

//  swap(highlights, highlights_);
//}

void TesseractStateDisplay::changedURDFDescription()
{
  if (isEnabled())
    reset();
}

void TesseractStateDisplay::changedRootLinkName() {}
void TesseractStateDisplay::changedURDFSceneAlpha()
{
  if (visualization_)
  {
    visualization_->setAlpha(alpha_property_->getFloat());
    update_state_ = true;
  }
}

bool TesseractStateDisplay::modifyEnvironmentCallback(tesseract_msgs::ModifyTesseractEnvRequest& req,
                                                      tesseract_msgs::ModifyTesseractEnvResponse& res)
{
  update_state_ = true;
  for (const auto& command : req.commands)
  {
    switch (command.command)
    {
      case tesseract_msgs::EnvironmentCommand::ADD:
      {
        tesseract_scene_graph::Link link = tesseract_rosutils::fromMsg(command.add_link);
        tesseract_scene_graph::Joint joint = tesseract_rosutils::fromMsg(command.add_joint);
        if (!visualization_->addLink(link) || !visualization_->addJoint(joint))
          return false;

        if (!env_->addLink(link, joint))
          return false;

        return true;
      }
      case tesseract_msgs::EnvironmentCommand::MOVE_LINK:
      {
        tesseract_scene_graph::Joint joint = tesseract_rosutils::fromMsg(command.move_link_joint);

        std::vector<tesseract_scene_graph::JointConstPtr> joints = env_->getSceneGraph()->getInboundJoints(joint.child_link_name);
        assert(joints.size() == 1);

        if (!visualization_->removeJoint(joints[0]->getName()) || !visualization_->addJoint(joint))
          return false;

        if (!env_->moveLink(joint))
          return false;

        return true;
      }
      case tesseract_msgs::EnvironmentCommand::MOVE_JOINT:
      {
        if (!visualization_->moveJoint(command.move_joint_name, command.move_joint_parent_link))
          return false;

        if (!env_->moveJoint(command.move_joint_name, command.move_joint_parent_link))
          return false;

        return true;
      }
      case tesseract_msgs::EnvironmentCommand::REMOVE_LINK:
      {
        if (env_->getLink(command.remove_link) == nullptr)
        {
          ROS_WARN("Tried to remove link (%s) that does not exist", command.remove_link.c_str());
          return false;
        }

        std::vector<tesseract_scene_graph::JointConstPtr> joints = env_->getSceneGraph()->getInboundJoints(command.remove_link);
        assert(joints.size() <= 1);

        // get child link names to remove
        std::vector<std::string> child_link_names = env_->getSceneGraph()->getLinkChildrenNames(command.remove_link);

        if (!visualization_->removeLink(command.remove_link))
          return false;

        if (!visualization_->removeJoint(joints[0]->getName()))
          return false;

        for (const auto& link_name : child_link_names)
        {
          if (!visualization_->removeLink(link_name))
            return false;

          std::vector<tesseract_scene_graph::JointConstPtr> joints = env_->getSceneGraph()->getInboundJoints(link_name);
          if (joints.size() == 1)
          {
            if (!visualization_->removeJoint(joints[0]->getName()))
              return false;
          }
        }

        if(!env_->removeLink(command.remove_link))
          return false;

        return true;
      }
      case tesseract_msgs::EnvironmentCommand::REMOVE_JOINT:
      {
        if (!visualization_->removeJoint(command.remove_joint))
          return false;

        if (!env_->removeJoint(command.remove_joint))
          return false;

        return true;
      }
      case tesseract_msgs::EnvironmentCommand::CHANGE_LINK_ORIGIN:
      {
        assert(false);
      }
      case tesseract_msgs::EnvironmentCommand::CHANGE_JOINT_ORIGIN:
      {
        assert(false);
      }
      case tesseract_msgs::EnvironmentCommand::CHANGE_LINK_COLLISION_ENABLED:
      {
        visualization_->setLinkCollisionEnabled(command.change_link_collision_enabled_name, command.change_link_collision_enabled_value);

        env_->setLinkCollisionEnabled(command.change_link_collision_enabled_name, command.change_link_collision_enabled_value);
        return true;
      }
      case tesseract_msgs::EnvironmentCommand::CHANGE_LINK_VISIBILITY:
      {
        // TODO:: Need to update visualization.
        env_->setLinkVisibility(command.change_link_visibility_name, command.change_link_visibility_value);
        return true;
      }
      case tesseract_msgs::EnvironmentCommand::ADD_ALLOWED_COLLISION:
      {
        visualization_->addAllowedCollision(command.add_allowed_collision.link_1, command.add_allowed_collision.link_2, command.add_allowed_collision.reason);

        env_->addAllowedCollision(command.add_allowed_collision.link_1, command.add_allowed_collision.link_2, command.add_allowed_collision.reason);
        return true;
      }
      case tesseract_msgs::EnvironmentCommand::REMOVE_ALLOWED_COLLISION:
      {
        visualization_->removeAllowedCollision(command.add_allowed_collision.link_1, command.add_allowed_collision.link_2);

        env_->removeAllowedCollision(command.add_allowed_collision.link_1, command.add_allowed_collision.link_2);
        return true;
      }
      case tesseract_msgs::EnvironmentCommand::REMOVE_ALLOWED_COLLISION_LINK:
      {
        visualization_->removeAllowedCollision(command.remove_allowed_collision_link);

        env_->removeAllowedCollision(command.remove_allowed_collision_link);
        return true;
      }
      case tesseract_msgs::EnvironmentCommand::UPDATE_JOINT_STATE:
      {
        return tesseract_rosutils::processMsg(*env_, command.joint_state);
      }
    }
  }
  return true;
}

void TesseractStateDisplay::changedJointStateTopic()
{
  joint_state_subscriber_.shutdown();

  joint_state_subscriber_ = nh_.subscribe(
      joint_state_topic_property_->getStdString(), 10, &TesseractStateDisplay::newJointStateCallback, this);
}

void TesseractStateDisplay::newJointStateCallback(const sensor_msgs::JointStateConstPtr& joint_state_msg)
{
  if (!env_)
    return;

  tesseract_rosutils::processMsg(env_, *joint_state_msg);
  update_state_ = true;
}

//void TesseractStateDisplay::setLinkColor(const tesseract_msgs::TesseractState::_object_colors_type& link_colors)
//{
//  assert(false);
//  for (tesseract_msgs::TesseractState::_object_colors_type::const_iterator it = link_colors.begin();
//       it != link_colors.end();
//       ++it)
//  {
//    setLinkColor(it->name,
//                 QColor(static_cast<int>(it->visual[0].r * 255),
//                        static_cast<int>(it->visual[0].g * 255),
//                        static_cast<int>(it->visual[0].b * 255)));
//  }
//}

//void TesseractStateDisplay::setLinkColor(const std::string& link_name, const QColor& color)
//{
//  setLinkColor(&state_->getRobot(), link_name, color);
//}

//void TesseractStateDisplay::unsetLinkColor(const std::string& link_name)
//{
//  unsetLinkColor(&state_->getRobot(), link_name);
//}

//void TesseractStateDisplay::setLinkColor(Robot* robot, const std::string& link_name, const QColor& color)
//{
//  RobotLink* link = robot->getLink(link_name);

//  // Check if link exists
//  if (link)
//    link->setColor(
//        static_cast<float>(color.redF()), static_cast<float>(color.greenF()), static_cast<float>(color.blueF()));
//}

//void TesseractStateDisplay::unsetLinkColor(Robot* robot, const std::string& link_name)
//{
//  RobotLink* link = robot->getLink(link_name);

//  // Check if link exists
//  if (link)
//    link->unsetColor();
//}

// ******************************************************************************************
// Load
// ******************************************************************************************
void TesseractStateDisplay::loadURDFModel()
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
        visualization_->clear();
        visualization_->load(env_->getSceneGraph(), true, true, true, true);
        bool oldState = root_link_name_property_->blockSignals(true);
        root_link_name_property_->setStdString(env_->getRootLinkName());
        root_link_name_property_->blockSignals(oldState);
        update_state_ = true;
        setStatus(rviz::StatusProperty::Ok, "TesseractState", "Tesseract Environment Loaded Successfully");

        changedEnableVisualVisible();
        changedEnableCollisionVisible();
        visualization_->setVisible(true);
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

  highlights_.clear();
}

void TesseractStateDisplay::onEnable()
{
  Display::onEnable();
  load_env_ = true;  // allow loading of robot model in update()
  calculateOffsetPosition();
}

// ******************************************************************************************
// Disable
// ******************************************************************************************
void TesseractStateDisplay::onDisable()
{
  joint_state_subscriber_.shutdown();
  if (visualization_)
    visualization_->setVisible(false);
  Display::onDisable();
}

void TesseractStateDisplay::update(float wall_dt, float ros_dt)
{
  Display::update(wall_dt, ros_dt);

  if (load_env_)
  {
    loadURDFModel();
    changedJointStateTopic();
  }

  calculateOffsetPosition();
  if (visualization_ && update_state_ && env_)
  {
    update_state_ = false;
    visualization_->update(EnvLinkUpdater(env_->getCurrentState()));
  }
}

// ******************************************************************************************
// Calculate Offset Position
// ******************************************************************************************
void TesseractStateDisplay::calculateOffsetPosition()
{
  if (!env_)
    return;

  Ogre::Vector3 position;
  Ogre::Quaternion orientation;

  context_->getFrameManager()->getTransform(env_->getRootLinkName(), ros::Time(0), position, orientation);
  scene_node_->setPosition(position);
  scene_node_->setOrientation(orientation);
}

void TesseractStateDisplay::fixedFrameChanged()
{
  Display::fixedFrameChanged();
  calculateOffsetPosition();
}

}  // namespace tesseract_rviz
