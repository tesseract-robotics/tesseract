/**
 * @file joint.cpp
 * @brief Parse joint from xml string
 *
 * @author Levi Armstrong
 * @date September 1, 2019
 * @version TODO
 * @bug No known bugs
 *
 * @copyright Copyright (c) 2019, Southwest Research Institute
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
#include <stdexcept>
#include <tesseract_common/utils.h>
#include <Eigen/Geometry>
#include <boost/algorithm/string.hpp>
#include <tinyxml2.h>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract_urdf/joint.h>
#include <tesseract_urdf/calibration.h>
#include <tesseract_urdf/dynamics.h>
#include <tesseract_urdf/limits.h>
#include <tesseract_urdf/mimic.h>
#include <tesseract_urdf/origin.h>
#include <tesseract_urdf/safety_controller.h>
#include <tesseract_scene_graph/joint.h>

tesseract_scene_graph::Joint::Ptr tesseract_urdf::parseJoint(const tinyxml2::XMLElement* xml_element, int version)
{
  // get joint name
  std::string joint_name;
  if (tesseract_common::QueryStringAttribute(xml_element, "name", joint_name) != tinyxml2::XML_SUCCESS)
    std::throw_with_nested(std::runtime_error("Joint: Missing or failed parsing attribute 'name'!"));

  // create joint
  auto j = std::make_shared<tesseract_scene_graph::Joint>(joint_name);

  // get joint origin
  const tinyxml2::XMLElement* origin = xml_element->FirstChildElement("origin");
  if (origin != nullptr)
  {
    try
    {
      j->parent_to_joint_origin_transform = parseOrigin(origin, version);
    }
    catch (...)
    {
      std::throw_with_nested(
          std::runtime_error("Joint: Error parsing 'origin' element for joint '" + joint_name + "'!"));
    }
  }

  // get parent link
  const tinyxml2::XMLElement* parent = xml_element->FirstChildElement("parent");
  if (parent == nullptr)
    std::throw_with_nested(std::runtime_error("Joint: Missing element 'parent' for joint '" + joint_name + "'!"));

  if (tesseract_common::QueryStringAttribute(parent, "link", j->parent_link_name) != tinyxml2::XML_SUCCESS)
    std::throw_with_nested(
        std::runtime_error("Joint: Failed parsing element 'parent' attribute 'link' for joint '" + joint_name + "'!"));

  // get child link
  const tinyxml2::XMLElement* child = xml_element->FirstChildElement("child");
  if (child == nullptr)
    std::throw_with_nested(std::runtime_error("Joint: Missing element 'child' for joint '" + joint_name + "'!"));

  if (tesseract_common::QueryStringAttribute(child, "link", j->child_link_name) != tinyxml2::XML_SUCCESS)
    std::throw_with_nested(
        std::runtime_error("Joint: Failed parsing element 'child' attribute 'link' for joint '" + joint_name + "'!"));

  // get joint type
  std::string joint_type;
  if (tesseract_common::QueryStringAttribute(xml_element, "type", joint_type) != tinyxml2::XML_SUCCESS)
    std::throw_with_nested(std::runtime_error("Joint: Missing element 'type' for joint '" + joint_name + "'!"));

  if (joint_type == "planar")
    j->type = tesseract_scene_graph::JointType::PLANAR;
  else if (joint_type == "floating")
    j->type = tesseract_scene_graph::JointType::FLOATING;
  else if (joint_type == "revolute")
    j->type = tesseract_scene_graph::JointType::REVOLUTE;
  else if (joint_type == "continuous")
    j->type = tesseract_scene_graph::JointType::CONTINUOUS;
  else if (joint_type == "prismatic")
    j->type = tesseract_scene_graph::JointType::PRISMATIC;
  else if (joint_type == "fixed")
    j->type = tesseract_scene_graph::JointType::FIXED;
  else
    std::throw_with_nested(
        std::runtime_error("Joint: Invalid joint type '" + joint_type + "' for joint '" + joint_name + "'!"));

  // get joint axis
  if (j->type != tesseract_scene_graph::JointType::FLOATING && j->type != tesseract_scene_graph::JointType::FIXED)
  {
    const tinyxml2::XMLElement* axis = xml_element->FirstChildElement("axis");
    if (axis == nullptr)
    {
      j->axis = Eigen::Vector3d(1.0, 0.0, 0.0);
    }
    else
    {
      std::string axis_str;
      if (tesseract_common::QueryStringAttribute(axis, "xyz", axis_str) != tinyxml2::XML_SUCCESS)
        std::throw_with_nested(
            std::runtime_error("Joint: Failed parsing element 'axis' attribute 'xyz' for joint '" + joint_name + "'!"));

      std::vector<std::string> tokens;
      boost::split(tokens, axis_str, boost::is_any_of(" "), boost::token_compress_on);
      if (tokens.size() != 3 || !tesseract_common::isNumeric(tokens))
        std::throw_with_nested(std::runtime_error("Joint: Failed parsing element 'axis' attribute 'xyz' string for "
                                                  "joint '" +
                                                  joint_name + "'!"));

      double ax{ 0 }, ay{ 0 }, az{ 0 };
      // No need to check return values because the tokens are verified above
      tesseract_common::toNumeric<double>(tokens[0], ax);
      tesseract_common::toNumeric<double>(tokens[1], ay);
      tesseract_common::toNumeric<double>(tokens[2], az);

      j->axis = Eigen::Vector3d(ax, ay, az);
    }
  }

  // get joint limits
  if (j->type == tesseract_scene_graph::JointType::REVOLUTE || j->type == tesseract_scene_graph::JointType::PRISMATIC ||
      j->type == tesseract_scene_graph::JointType::CONTINUOUS)
  {
    const tinyxml2::XMLElement* limits = xml_element->FirstChildElement("limit");
    if (limits == nullptr && j->type != tesseract_scene_graph::JointType::CONTINUOUS)
      std::throw_with_nested(std::runtime_error("Joint: Missing element 'limits' for joint '" + joint_name + "'!"));

    if (limits == nullptr && j->type == tesseract_scene_graph::JointType::CONTINUOUS)
    {
      j->limits = std::make_shared<tesseract_scene_graph::JointLimits>();
    }
    else
    {
      try
      {
        j->limits = parseLimits(limits, version);
      }
      catch (...)
      {
        std::throw_with_nested(
            std::runtime_error("Joint: Failed parsing element 'limits' for joint '" + joint_name + "'!"));
      }
    }
  }

  // get joint safety if exists
  const tinyxml2::XMLElement* safety = xml_element->FirstChildElement("safety_controller");
  if (safety != nullptr)
  {
    try
    {
      j->safety = parseSafetyController(safety, version);
    }
    catch (...)
    {
      std::throw_with_nested(
          std::runtime_error("Joint: Failed parsing element 'safety_controller' for joint '" + joint_name + "'!"));
    }
  }

  // get joint calibration if exists
  const tinyxml2::XMLElement* calibration = xml_element->FirstChildElement("calibration");
  if (calibration != nullptr)
  {
    try
    {
      j->calibration = parseCalibration(calibration, version);
    }
    catch (...)
    {
      std::throw_with_nested(
          std::runtime_error("Joint: Failed parsing element 'calibration' for joint '" + joint_name + "'!"));
    }
  }

  // get mimic joint if exists
  const tinyxml2::XMLElement* mimic = xml_element->FirstChildElement("mimic");
  if (mimic != nullptr)
  {
    try
    {
      j->mimic = parseMimic(mimic, version);
    }
    catch (...)
    {
      std::throw_with_nested(
          std::runtime_error("Joint: Failed parsing element 'mimic' for joint '" + joint_name + "'!"));
    }
  }

  // get dynamics if exists
  const tinyxml2::XMLElement* dynamics = xml_element->FirstChildElement("dynamics");
  if (dynamics != nullptr)
  {
    try
    {
      j->dynamics = parseDynamics(dynamics, version);
    }
    catch (...)
    {
      std::throw_with_nested(
          std::runtime_error("Joint: Failed parsing element 'dynamics' for joint '" + joint_name + "'!"));
    }
  }

  return j;
}

tinyxml2::XMLElement* tesseract_urdf::writeJoint(const std::shared_ptr<const tesseract_scene_graph::Joint>& joint,
                                                 tinyxml2::XMLDocument& doc)
{
  if (joint == nullptr)
    std::throw_with_nested(std::runtime_error("Joint is nullptr and cannot be converted to XML"));
  tinyxml2::XMLElement* xml_element = doc.NewElement("joint");

  // Set the joint name
  xml_element->SetAttribute("name", joint->getName().c_str());

  // Set joint origin
  tinyxml2::XMLElement* xml_origin = writeOrigin(joint->parent_to_joint_origin_transform, doc);
  xml_element->InsertEndChild(xml_origin);

  // Set parent link
  tinyxml2::XMLElement* xml_parent = doc.NewElement("parent");
  xml_parent->SetAttribute("link", joint->parent_link_name.c_str());
  xml_element->InsertEndChild(xml_parent);

  // Set child link
  tinyxml2::XMLElement* xml_child = doc.NewElement("child");
  xml_child->SetAttribute("link", joint->child_link_name.c_str());
  xml_element->InsertEndChild(xml_child);

  // Set joint type
  if (joint->type == tesseract_scene_graph::JointType::PLANAR)
    xml_element->SetAttribute("type", "planar");
  else if (joint->type == tesseract_scene_graph::JointType::FLOATING)
    xml_element->SetAttribute("type", "floating");
  else if (joint->type == tesseract_scene_graph::JointType::REVOLUTE)
    xml_element->SetAttribute("type", "revolute");
  else if (joint->type == tesseract_scene_graph::JointType::CONTINUOUS)
    xml_element->SetAttribute("type", "continuous");
  else if (joint->type == tesseract_scene_graph::JointType::PRISMATIC)
    xml_element->SetAttribute("type", "prismatic");
  else if (joint->type == tesseract_scene_graph::JointType::FIXED)
    xml_element->SetAttribute("type", "fixed");
  else
    std::throw_with_nested(std::runtime_error("Joint: Invalid joint type for joint '" + joint->getName() + "'!"));

  // Set joint axis
  if (joint->type != tesseract_scene_graph::JointType::FLOATING &&
      joint->type != tesseract_scene_graph::JointType::FIXED)
  {
    tinyxml2::XMLElement* xml_axis = doc.NewElement("axis");
    std::string axis_str =
        std::to_string(joint->axis.x()) + " " + std::to_string(joint->axis.y()) + " " + std::to_string(joint->axis.z());
    xml_axis->SetAttribute("xyz", axis_str.c_str());
    xml_element->InsertEndChild(xml_axis);
  }

  // Set joint limits
  if (joint->type == tesseract_scene_graph::JointType::REVOLUTE ||
      joint->type == tesseract_scene_graph::JointType::PRISMATIC ||
      joint->type == tesseract_scene_graph::JointType::CONTINUOUS)
  {
    if (joint->limits == nullptr)
      std::throw_with_nested(std::runtime_error("Joint: Missing limits for joint '" + joint->getName() + "'!"));
    tinyxml2::XMLElement* xml_limits = writeLimits(joint->limits, doc);
    xml_element->InsertEndChild(xml_limits);
  }

  // Set joint safety if it exists
  if (joint->safety != nullptr)
  {
    tinyxml2::XMLElement* xml_safety = writeSafetyController(joint->safety, doc);
    xml_element->InsertEndChild(xml_safety);
  }

  // Set joint calibration if it exists
  if (joint->calibration != nullptr)
  {
    tinyxml2::XMLElement* xml_calibration = writeCalibration(joint->calibration, doc);
    xml_element->InsertEndChild(xml_calibration);
  }

  // Set mimic joint it it exists
  if (joint->mimic != nullptr)
  {
    tinyxml2::XMLElement* xml_mimic = writeMimic(joint->mimic, doc);
    xml_element->InsertEndChild(xml_mimic);
  }

  // Set dynamics if exists
  if (joint->dynamics != nullptr)
  {
    tinyxml2::XMLElement* xml_dynamics = writeDynamics(joint->dynamics, doc);
    xml_element->InsertEndChild(xml_dynamics);
  }

  return xml_element;
}
