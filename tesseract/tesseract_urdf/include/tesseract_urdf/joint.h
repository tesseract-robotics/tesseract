/**
 * @file joint.h
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
#ifndef TESSERACT_URDF_JOINT_H
#define TESSERACT_URDF_JOINT_H

#include <tesseract_common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <tesseract_common/status_code.h>
#include <tesseract_common/utils.h>
#include <Eigen/Geometry>
#include <tinyxml2.h>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract_scene_graph/joint.h>
#include <tesseract_urdf/calibration.h>
#include <tesseract_urdf/dynamics.h>
#include <tesseract_urdf/limits.h>
#include <tesseract_urdf/mimic.h>
#include <tesseract_urdf/origin.h>
#include <tesseract_urdf/safety_controller.h>
#include <tesseract_urdf/utils.h>

namespace tesseract_urdf
{
class JointStatusCategory : public tesseract_common::StatusCategory
{
public:
  JointStatusCategory(std::string joint_name = "") : name_("JointStatusCategory"), joint_name_(std::move(joint_name)) {}
  const std::string& name() const noexcept override { return name_; }
  std::string message(int code) const override
  {
    switch (code)
    {
      case Success:
        return "Sucessfully parsed joint '" + joint_name_ + "'!";
      case ErrorMissingAttributeName:
        return "Missing or failed to parse joint attribute 'name' for joint '" + joint_name_ + "'!";
      case ErrorMissingOriginElement:
        return "Missing joint element 'origin' for joint '" + joint_name_ + "'!";
      case ErrorParsingOrigin:
        return "Failed parsing joint element 'origin' for joint '" + joint_name_ + "'!";
      case ErrorMissingParentElement:
        return "Missing joint element 'parent' for joint '" + joint_name_ + "'!";
      case ErrorParsingParentAttributeLink:
        return "Failed parsing joint element 'parent' attribute 'link' for joint '" + joint_name_ + "'!";
      case ErrorMissingChildElement:
        return "Missing joint element 'child' for joint '" + joint_name_ + "'!";
      case ErrorParsingChildAttributeLink:
        return "Failed parsing joint element 'child' attribute 'link' for joint '" + joint_name_ + "'!";
      case ErrorMissingTypeElement:
        return "Missing joint element 'type' for joint '" + joint_name_ + "'!";
      case ErrorInvalidType:
        return "Invalid joint type for joint '" + joint_name_ + "'!";
      case ErrorMissingAxisElement:
        return "Missing joint element 'axes' for joint '" + joint_name_ + "'!";
      case ErrorParsingAxisAttributeXYZ:
        return "Failed parsing joint element 'axis' attribute 'xyz' for joint '" + joint_name_ + "'!";
      case ErrorMissingLimitsElement:
        return "Missing joint element 'limits' for joint '" + joint_name_ + "'!";
      case ErrorParsingLimits:
        return "Failed parsing joint element 'limits' for joint '" + joint_name_ + "'!";
      case ErrorParsingSafetyController:
        return "Failed parsing joint element 'safety_controller' for joint '" + joint_name_ + "'!";
      case ErrorParsingCalibration:
        return "Failed parsing joint element 'calibration' for joint '" + joint_name_ + "'!";
      case ErrorParsingMimic:
        return "Failed parsing joint element 'mimic' for joint '" + joint_name_ + "'!";
      case ErrorParsingDynamics:
        return "Failed parsing joint element 'dynamics' for joint '" + joint_name_ + "'!";
      default:
        return "Invalid error code for " + name_ + "!";
    }
  }

  enum
  {
    Success = 0,
    ErrorMissingAttributeName = -1,
    ErrorMissingOriginElement = -2,
    ErrorParsingOrigin = -3,
    ErrorMissingParentElement = -4,
    ErrorParsingParentAttributeLink = -5,
    ErrorMissingChildElement = -6,
    ErrorParsingChildAttributeLink = -7,
    ErrorMissingTypeElement = -8,
    ErrorInvalidType = -9,
    ErrorMissingAxisElement = -10,
    ErrorParsingAxisAttributeXYZ = -11,
    ErrorMissingLimitsElement = -12,
    ErrorParsingLimits = -13,
    ErrorParsingSafetyController = -14,
    ErrorParsingCalibration = -15,
    ErrorParsingMimic = -16,
    ErrorParsingDynamics = -17
  };

private:
  std::string name_;
  std::string joint_name_;
};

inline tesseract_common::StatusCode::Ptr parse(tesseract_scene_graph::Joint::Ptr& joint,
                                               const tinyxml2::XMLElement* xml_element,
                                               const int version)
{
  joint = nullptr;
  auto status_cat = std::make_shared<JointStatusCategory>();

  // get joint name
  std::string joint_name;
  if (QueryStringAttribute(xml_element, "name", joint_name) != tinyxml2::XML_SUCCESS)
    return std::make_shared<tesseract_common::StatusCode>(JointStatusCategory::ErrorMissingAttributeName, status_cat);

  status_cat = std::make_shared<JointStatusCategory>(joint_name);

  // create joint
  auto j = std::make_shared<tesseract_scene_graph::Joint>(joint_name);

  // get joint origin
  const tinyxml2::XMLElement* origin = xml_element->FirstChildElement("origin");
  if (origin != nullptr)
  {
    auto status = parse(j->parent_to_joint_origin_transform, origin, version);
    if (!(*status))
      return std::make_shared<tesseract_common::StatusCode>(
          JointStatusCategory::ErrorParsingOrigin, status_cat, status);
  }

  // get parent link
  const tinyxml2::XMLElement* parent = xml_element->FirstChildElement("parent");
  if (parent == nullptr)
    return std::make_shared<tesseract_common::StatusCode>(JointStatusCategory::ErrorMissingParentElement, status_cat);

  if (QueryStringAttribute(parent, "link", j->parent_link_name) != tinyxml2::XML_SUCCESS)
    return std::make_shared<tesseract_common::StatusCode>(JointStatusCategory::ErrorParsingParentAttributeLink,
                                                          status_cat);

  // get child link
  const tinyxml2::XMLElement* child = xml_element->FirstChildElement("child");
  if (child == nullptr)
    return std::make_shared<tesseract_common::StatusCode>(JointStatusCategory::ErrorMissingChildElement, status_cat);

  if (QueryStringAttribute(child, "link", j->child_link_name) != tinyxml2::XML_SUCCESS)
    return std::make_shared<tesseract_common::StatusCode>(JointStatusCategory::ErrorParsingChildAttributeLink,
                                                          status_cat);

  // get joint type
  std::string joint_type;
  if (QueryStringAttribute(xml_element, "type", joint_type) != tinyxml2::XML_SUCCESS)
    return std::make_shared<tesseract_common::StatusCode>(JointStatusCategory::ErrorMissingTypeElement, status_cat);

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
    return std::make_shared<tesseract_common::StatusCode>(JointStatusCategory::ErrorInvalidType, status_cat);

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
      if (QueryStringAttribute(axis, "xyz", axis_str) != tinyxml2::XML_SUCCESS)
        return std::make_shared<tesseract_common::StatusCode>(JointStatusCategory::ErrorParsingAxisAttributeXYZ,
                                                              status_cat);

      std::vector<std::string> tokens;
      boost::split(tokens, axis_str, boost::is_any_of(" "), boost::token_compress_on);
      if (tokens.size() != 3 || !tesseract_common::isNumeric(tokens))
        return std::make_shared<tesseract_common::StatusCode>(JointStatusCategory::ErrorParsingAxisAttributeXYZ,
                                                              status_cat);

      double ax, ay, az;
      if (!tesseract_common::toNumeric<double>(tokens[0], ax))
        return std::make_shared<tesseract_common::StatusCode>(JointStatusCategory::ErrorParsingAxisAttributeXYZ,
                                                              status_cat);

      if (!tesseract_common::toNumeric<double>(tokens[1], ay))
        return std::make_shared<tesseract_common::StatusCode>(JointStatusCategory::ErrorParsingAxisAttributeXYZ,
                                                              status_cat);

      if (!tesseract_common::toNumeric<double>(tokens[2], az))
        return std::make_shared<tesseract_common::StatusCode>(JointStatusCategory::ErrorParsingAxisAttributeXYZ,
                                                              status_cat);

      j->axis = Eigen::Vector3d(ax, ay, az);
    }
  }

  // get joint limits
  if (j->type == tesseract_scene_graph::JointType::REVOLUTE || j->type == tesseract_scene_graph::JointType::PRISMATIC ||
      j->type == tesseract_scene_graph::JointType::CONTINUOUS)
  {
    const tinyxml2::XMLElement* limits = xml_element->FirstChildElement("limit");
    if (limits == nullptr && j->type != tesseract_scene_graph::JointType::CONTINUOUS)
    {
      return std::make_shared<tesseract_common::StatusCode>(JointStatusCategory::ErrorMissingLimitsElement, status_cat);
    }

    if (limits == nullptr && j->type == tesseract_scene_graph::JointType::CONTINUOUS)
    {
      j->limits = std::make_shared<tesseract_scene_graph::JointLimits>();
    }
    else
    {
      auto status = parse(j->limits, limits, version);
      if (!(*status))
        return std::make_shared<tesseract_common::StatusCode>(
            JointStatusCategory::ErrorParsingLimits, status_cat, status);
    }
  }

  // get joint safety if exists
  const tinyxml2::XMLElement* safety = xml_element->FirstChildElement("safety_controller");
  if (safety != nullptr)
  {
    auto status = parse(j->safety, safety, version);
    if (!(*status))
      return std::make_shared<tesseract_common::StatusCode>(
          JointStatusCategory::ErrorParsingSafetyController, status_cat, status);
  }

  // get joint calibration if exists
  const tinyxml2::XMLElement* calibration = xml_element->FirstChildElement("calibration");
  if (calibration != nullptr)
  {
    auto status = parse(j->calibration, calibration, version);
    if (!(*status))
      return std::make_shared<tesseract_common::StatusCode>(
          JointStatusCategory::ErrorParsingCalibration, status_cat, status);
  }

  // get mimic joint if exists
  const tinyxml2::XMLElement* mimic = xml_element->FirstChildElement("mimic");
  if (mimic != nullptr)
  {
    auto status = parse(j->mimic, mimic, version);
    if (!(*status))
      return std::make_shared<tesseract_common::StatusCode>(JointStatusCategory::ErrorParsingMimic, status_cat, status);
  }

  // get dynamics if exists
  const tinyxml2::XMLElement* dynamics = xml_element->FirstChildElement("dynamics");
  if (dynamics != nullptr)
  {
    auto status = parse(j->dynamics, dynamics, version);
    if (!(*status))
      return std::make_shared<tesseract_common::StatusCode>(
          JointStatusCategory::ErrorParsingDynamics, status_cat, status);
  }

  joint = std::move(j);
  return std::make_shared<tesseract_common::StatusCode>(JointStatusCategory::Success, status_cat);
}
}  // namespace tesseract_urdf
#endif  // TESSERACT_URDF_JOINT_H
