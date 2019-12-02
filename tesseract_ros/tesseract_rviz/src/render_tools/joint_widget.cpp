/*
 * Copyright (c) 2013, Willow Garage, Inc.
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

#include "tesseract_rviz/render_tools/joint_widget.h"
#include "tesseract_rviz/render_tools/visualization_widget.h"
#include "tesseract_rviz/render_tools/link_widget.h"

#include <tesseract_common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <OgreSceneNode.h>

#include "rviz/load_resource.h"
#include "rviz/ogre_helpers/arrow.h"
#include "rviz/ogre_helpers/axes.h"
#include "rviz/properties/float_property.h"
#include "rviz/properties/quaternion_property.h"
#include "rviz/properties/string_property.h"
#include "rviz/properties/vector_property.h"
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract_rviz/conversions.h>

namespace tesseract_rviz
{
JointWidget::JointWidget(VisualizationWidget* env, const tesseract_scene_graph::Joint& joint)
  : env_(env)
  , name_(joint.getName())
  , parent_link_name_(joint.parent_link_name)
  , child_link_name_(joint.child_link_name)
  , has_decendent_links_with_geometry_(true)
  , doing_set_checkbox_(false)
  , axes_(nullptr)
  , axis_(nullptr)
{
  joint_property_ = new rviz::Property(name_.c_str(), true, "", nullptr, SLOT(updateChildVisibility()), this);
  joint_property_->setIcon(rviz::loadPixmap("package://rviz/icons/classes/RobotJoint.png"));

  details_ = new rviz::Property("Details", QVariant(), "", nullptr);

  axes_property_ = new rviz::Property(
      "Show Axes", false, "Enable/disable showing the axes of this joint.", joint_property_, SLOT(updateAxes()), this);

  position_property_ = new rviz::VectorProperty("Position",
                                                Ogre::Vector3::ZERO,
                                                "Position of this joint, in the current Fixed Frame.  (Not editable)",
                                                joint_property_);
  position_property_->setReadOnly(true);

  orientation_property_ = new rviz::QuaternionProperty("Orientation",
                                                       Ogre::Quaternion::IDENTITY,
                                                       "Orientation of this joint, in the current Fixed Frame.  (Not "
                                                       "editable)",
                                                       joint_property_);
  orientation_property_->setReadOnly(true);

  std::string type;
  if (joint.type == tesseract_scene_graph::JointType::UNKNOWN)
    type = "unknown";
  else if (joint.type == tesseract_scene_graph::JointType::REVOLUTE)
    type = "revolute";
  else if (joint.type == tesseract_scene_graph::JointType::CONTINUOUS)
    type = "continuous";
  else if (joint.type == tesseract_scene_graph::JointType::PRISMATIC)
    type = "prismatic";
  else if (joint.type == tesseract_scene_graph::JointType::FLOATING)
    type = "floating";
  else if (joint.type == tesseract_scene_graph::JointType::PLANAR)
    type = "planar";
  else if (joint.type == tesseract_scene_graph::JointType::FIXED)
    type = "fixed";

  type_property_ = new rviz::StringProperty(
      "Type", QString::fromStdString(type), "Type of this joint.  (Not editable)", joint_property_);
  type_property_->setReadOnly(true);

  if (joint.limits)
  {
    // continuous joints have lower limit and upper limits of zero,
    // which means this isn't very useful but show it anyhow.
    lower_limit_property_ = new rviz::FloatProperty("Lower Limit",
                                                    static_cast<float>(joint.limits->lower),
                                                    "Lower limit of this joint.  (Not editable)",
                                                    joint_property_);
    lower_limit_property_->setReadOnly(true);

    upper_limit_property_ = new rviz::FloatProperty("Upper Limit",
                                                    static_cast<float>(joint.limits->upper),
                                                    "Upper limit of this joint.  (Not editable)",
                                                    joint_property_);
    upper_limit_property_->setReadOnly(true);
  }

  if ((type == "continuous") || (type == "revolute") || (type == "prismatic") || (type == "planar"))
  {
    show_axis_property_ = new rviz::Property("Show Joint Axis",
                                             false,
                                             "Enable/disable showing the axis of this joint.",
                                             joint_property_,
                                             SLOT(updateAxis()),
                                             this);

    axis_property_ = new rviz::VectorProperty("Joint Axis",
                                              Ogre::Vector3(static_cast<float>(joint.axis(0)),
                                                            static_cast<float>(joint.axis(1)),
                                                            static_cast<float>(joint.axis(2))),
                                              "Axis of this joint.  (Not editable)",
                                              joint_property_);
    axis_property_->setReadOnly(true);
  }

  joint_property_->collapse();

  const Eigen::Vector3d& pos = joint.parent_to_joint_origin_transform.translation();
  Eigen::Quaterniond rot(joint.parent_to_joint_origin_transform.linear());
  joint_origin_pos_ = Ogre::Vector3(static_cast<float>(pos(0)), static_cast<float>(pos(1)), static_cast<float>(pos(2)));
  joint_origin_rot_ = Ogre::Quaternion(static_cast<float>(rot.w()),
                                       static_cast<float>(rot.x()),
                                       static_cast<float>(rot.y()),
                                       static_cast<float>(rot.z()));
}

JointWidget::~JointWidget()
{
  delete axes_;
  delete axis_;
  delete details_;
  delete joint_property_;
}

void JointWidget::setJointPropertyDescription()
{
  int links_with_geom;
  int links_with_geom_checked;
  int links_with_geom_unchecked;
  getChildLinkState(links_with_geom, links_with_geom_checked, links_with_geom_unchecked, true);

  std::stringstream desc;
  desc << "Joint <b>" << name_ << "</b> with parent link <b>" << parent_link_name_ << "</b> and child link <b>"
       << child_link_name_ << "</b>.";

  if (links_with_geom == 0)
  {
    desc << "  This joint's descendents have NO geometry.";
    setJointCheckbox(QVariant());
    has_decendent_links_with_geometry_ = false;
  }
  else if (styleIsTree())
  {
    desc << "  Check/uncheck to show/hide all links descended from this joint.";
    setJointCheckbox(links_with_geom_unchecked == 0);
    has_decendent_links_with_geometry_ = true;
  }
  else
  {
    getChildLinkState(links_with_geom, links_with_geom_checked, links_with_geom_unchecked, false);
    if (links_with_geom == 0)
    {
      desc << "  This joint's child link has NO geometry.";
      setJointCheckbox(QVariant());
      has_decendent_links_with_geometry_ = false;
    }
    else
    {
      desc << "  Check/uncheck to show/hide this joint's child link.";
      setJointCheckbox(links_with_geom_unchecked == 0);
      has_decendent_links_with_geometry_ = true;
    }
  }

  joint_property_->setDescription(desc.str().c_str());
}

void JointWidget::setJointCheckbox(const QVariant& val)
{
  // setting doing_set_checkbox_ to true prevents updateChildVisibility() from
  // updating child link enables.
  doing_set_checkbox_ = true;
  joint_property_->setValue(val);
  doing_set_checkbox_ = false;
}

void JointWidget::calculateJointCheckboxesRecursive(int& links_with_geom,
                                                    int& links_with_geom_checked,
                                                    int& links_with_geom_unchecked)
{
  links_with_geom_checked = 0;
  links_with_geom_unchecked = 0;

  LinkWidget* link = env_->getLink(child_link_name_);
  if (link && link->hasGeometry())
  {
    bool checked = link->getLinkProperty()->getValue().toBool();
    links_with_geom_checked += checked ? 1 : 0;
    links_with_geom_unchecked += checked ? 0 : 1;
  }
  links_with_geom = links_with_geom_checked + links_with_geom_unchecked;

  if (!styleIsTree())
  {
    if (!links_with_geom)
    {
      setJointCheckbox(QVariant());
    }
    else
    {
      setJointCheckbox(links_with_geom_unchecked == 0);
    }
  }

  JointWidget* child_joint = env_->findChildJoint(link);
  while (child_joint != nullptr)
  {
    int child_links_with_geom;
    int child_links_with_geom_checked;
    int child_links_with_geom_unchecked;
    child_joint->calculateJointCheckboxesRecursive(
        child_links_with_geom, child_links_with_geom_checked, child_links_with_geom_unchecked);
    links_with_geom_checked += child_links_with_geom_checked;
    links_with_geom_unchecked += child_links_with_geom_unchecked;

    link = env_->getLink(child_joint->getChildLinkName());
    child_joint = env_->findChildJoint(link);
  }

  links_with_geom = links_with_geom_checked + links_with_geom_unchecked;

  if (styleIsTree())
  {
    if (!links_with_geom)
    {
      setJointCheckbox(QVariant());
    }
    else
    {
      setJointCheckbox(links_with_geom_unchecked == 0);
    }
  }
}

void JointWidget::getChildLinkState(int& links_with_geom,
                                    int& links_with_geom_checked,
                                    int& links_with_geom_unchecked,
                                    bool recursive) const
{
  links_with_geom_checked = 0;
  links_with_geom_unchecked = 0;

  LinkWidget* link = env_->getLink(child_link_name_);
  if (link && link->hasGeometry())
  {
    bool checked = link->getLinkProperty()->getValue().toBool();
    links_with_geom_checked += checked ? 1 : 0;
    links_with_geom_unchecked += checked ? 0 : 1;
  }

  if (recursive)
  {
    JointWidget* child_joint = env_->findChildJoint(link);
    while (child_joint != nullptr)
    {
      int child_links_with_geom;
      int child_links_with_geom_checked;
      int child_links_with_geom_unchecked;
      child_joint->calculateJointCheckboxesRecursive(
          child_links_with_geom, child_links_with_geom_checked, child_links_with_geom_unchecked);
      links_with_geom_checked += child_links_with_geom_checked;
      links_with_geom_unchecked += child_links_with_geom_unchecked;

      link = env_->getLink(child_joint->getChildLinkName());
      child_joint = env_->findChildJoint(link);
    }
  }

  links_with_geom = links_with_geom_checked + links_with_geom_unchecked;
}

bool JointWidget::getEnabled() const
{
  if (!hasDescendentLinksWithGeometry())
    return true;
  return joint_property_->getValue().toBool();
}

bool JointWidget::styleIsTree() const { return details_->getParent() != nullptr; }

void JointWidget::updateChildVisibility()
{
  if (doing_set_checkbox_)
    return;

  if (!hasDescendentLinksWithGeometry())
    return;

  bool visible = getEnabled();

  LinkWidget* link = env_->getLink(child_link_name_);
  if (link)
  {
    if (link->hasGeometry())
    {
      link->getLinkProperty()->setValue(visible);
    }

    if (styleIsTree())
    {
      JointWidget* child_joint = env_->findChildJoint(link);
      while (child_joint != nullptr)
      {
        child_joint->getJointProperty()->setValue(visible);
        link = env_->getLink(child_joint->getChildLinkName());
        child_joint = env_->findChildJoint(link);
      }
    }
  }
}

void JointWidget::updateAxes()
{
  if (axes_property_->getValue().toBool())
  {
    if (!axes_)
    {
      axes_ = new rviz::Axes(env_->getSceneManager(), env_->getOtherNode(), 0.1f, 0.01f);
      axes_->getSceneNode()->setVisible(getEnabled());

      axes_->setPosition(position_property_->getVector());
      axes_->setOrientation(orientation_property_->getQuaternion());
    }
  }
  else
  {
    if (axes_)
    {
      delete axes_;
      axes_ = nullptr;
    }
  }
}

void JointWidget::updateAxis()
{
  if (show_axis_property_->getValue().toBool())
  {
    if (!axis_)
    {
      axis_ = new rviz::Arrow(env_->getSceneManager(), env_->getOtherNode(), 0.15f, 0.05f, 0.05f, 0.08f);
      axis_->getSceneNode()->setVisible(getEnabled());

      axis_->setPosition(position_property_->getVector());
      axis_->setOrientation(orientation_property_->getQuaternion());

      // TODO(lucasw) store an Ogre::ColorValue and set it according to
      // joint type.
      axis_->setColor(0.0f, 0.8f, 0.0f, 1.0f);
    }
  }
  else
  {
    if (axis_)
    {
      delete axis_;
      axis_ = nullptr;
    }
  }
}

void JointWidget::setTransforms(const Ogre::Vector3& parent_link_position,
                                const Ogre::Quaternion& parent_link_orientation)
{
  Ogre::Vector3 position = parent_link_position + parent_link_orientation * joint_origin_pos_;
  Ogre::Quaternion orientation = parent_link_orientation * joint_origin_rot_;

  position_property_->setVector(position);
  orientation_property_->setQuaternion(orientation);

  if (axes_)
  {
    axes_->setPosition(position);
    axes_->setOrientation(orientation);
  }
  if (axis_)
  {
    axis_->setPosition(position);
    axis_->setOrientation(orientation);
    axis_->setDirection(parent_link_orientation * axis_property_->getVector());
  }
}

void JointWidget::hideSubProperties(bool hide)
{
  position_property_->setHidden(hide);
  orientation_property_->setHidden(hide);
  axes_property_->setHidden(hide);
  show_axis_property_->setHidden(hide);
  axis_property_->setHidden(hide);
}

Ogre::Vector3 JointWidget::getPosition() { return position_property_->getVector(); }
Ogre::Quaternion JointWidget::getOrientation() { return orientation_property_->getQuaternion(); }
void JointWidget::setParentProperty(rviz::Property* new_parent)
{
  rviz::Property* old_parent = joint_property_->getParent();
  if (old_parent)
    old_parent->takeChild(joint_property_);

  if (new_parent)
    new_parent->addChild(joint_property_);
}

// if use_detail:
//    - all sub properties become children of details_ property.
//    - details_ property becomes a child of joint_property_
// else (!use_detail)
//    - all sub properties become children of joint_property_.
//    details_ property does not have a parent.
void JointWidget::useDetailProperty(bool use_detail)
{
  rviz::Property* old_parent = details_->getParent();
  if (old_parent)
    old_parent->takeChild(details_);

  if (use_detail)
  {
    while (joint_property_->numChildren() > 0)
    {
      rviz::Property* child = joint_property_->childAt(0);
      joint_property_->takeChild(child);
      details_->addChild(child);
    }

    joint_property_->addChild(details_);
  }
  else
  {
    while (details_->numChildren() > 0)
    {
      rviz::Property* child = details_->childAt(0);
      details_->takeChild(child);
      joint_property_->addChild(child);
    }
  }
}

void JointWidget::expandDetails(bool expand)
{
  rviz::Property* parent = details_->getParent() ? details_ : joint_property_;
  if (expand)
  {
    parent->expand();
  }
  else
  {
    parent->collapse();
  }
}

}  // namespace tesseract_rviz
