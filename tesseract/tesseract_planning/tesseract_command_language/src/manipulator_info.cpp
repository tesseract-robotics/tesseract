/**
 * @file manipulator_info.cpp
 * @brief
 *
 * @author Levi Armstrong
 * @date June 15, 2020
 * @version TODO
 * @bug No known bugs
 *
 * @copyright Copyright (c) 2020, Southwest Research Institute
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
#include <iostream>
#include <boost/algorithm/string.hpp>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract_command_language/manipulator_info.h>
#include <tesseract_common/utils.h>

namespace tesseract_planning
{
ToolCenterPoint::ToolCenterPoint(const std::string& name, bool external) : type_(1), name_(name), external_(external) {}

ToolCenterPoint::ToolCenterPoint(const Eigen::Isometry3d& transform, bool external)
  : type_(2), transform_(transform), external_(external)
{
}

bool ToolCenterPoint::empty() const { return (type_ == 0); }
bool ToolCenterPoint::isString() const { return (type_ == 1); }
bool ToolCenterPoint::isTransform() const { return (type_ == 2); }
bool ToolCenterPoint::isExternal() const { return external_; }
void ToolCenterPoint::setExternal(bool value) { external_ = value; }

const std::string& ToolCenterPoint::getString() const
{
  assert(type_ == 1);
  return name_;
}
const Eigen::Isometry3d& ToolCenterPoint::getTransform() const
{
  assert(type_ == 2);
  return transform_;
}

ManipulatorInfo::ManipulatorInfo(std::string manipulator) : manipulator(std::move(manipulator)) {}
ManipulatorInfo::ManipulatorInfo(const tinyxml2::XMLElement& xml_element)
{
  const tinyxml2::XMLElement* manipulator_element = xml_element.FirstChildElement("Manipulator");
  const tinyxml2::XMLElement* manipulator_ik_solver_element = xml_element.FirstChildElement("ManipulatorIKSolver");
  const tinyxml2::XMLElement* working_frame_element = xml_element.FirstChildElement("WorkingFrame");
  const tinyxml2::XMLElement* tcp_element = xml_element.FirstChildElement("TCP");

  if (manipulator_element != nullptr)
  {
    tinyxml2::XMLError status = tesseract_common::QueryStringText(manipulator_element, manipulator);
    if (status != tinyxml2::XML_NO_ATTRIBUTE && status != tinyxml2::XML_SUCCESS)
      throw std::runtime_error("ManipulatorInfo: Error parsing Manipulator string");
  }

  if (manipulator_ik_solver_element != nullptr)
  {
    tinyxml2::XMLError status = tesseract_common::QueryStringText(manipulator_ik_solver_element, manipulator_ik_solver);
    if (status != tinyxml2::XML_NO_ATTRIBUTE && status != tinyxml2::XML_SUCCESS)
      throw std::runtime_error("ManipulatorInfo: Error parsing ManipulatorIKSolver string");
  }

  if (working_frame_element != nullptr)
  {
    tinyxml2::XMLError status = tesseract_common::QueryStringText(working_frame_element, working_frame);
    if (status != tinyxml2::XML_NO_ATTRIBUTE && status != tinyxml2::XML_SUCCESS)
      throw std::runtime_error("ManipulatorInfo: Error parsing WorkingFrame string");
  }

  if (tcp_element != nullptr)
  {
    if (tcp_element->Attribute("xyz") == nullptr && tcp_element->Attribute("rpy") == nullptr &&
        tcp_element->Attribute("wxyz") == nullptr && tcp_element->Attribute("name") == nullptr)
      throw std::runtime_error("ManipulatorInfo: TCP Missing Required Attributes");

    if (tcp_element->Attribute("name") == nullptr)
    {
      Eigen::Isometry3d local_tcp{ Eigen::Isometry3d::Identity() };
      std::string xyz_string, rpy_string, wxyz_string;
      tinyxml2::XMLError status = tesseract_common::QueryStringAttribute(tcp_element, "xyz", xyz_string);
      if (status != tinyxml2::XML_NO_ATTRIBUTE && status != tinyxml2::XML_SUCCESS)
        throw std::runtime_error("ManipulatorInfo: Error parsing TCP attribute xyz.");

      if (status != tinyxml2::XML_NO_ATTRIBUTE)
      {
        std::vector<std::string> tokens;
        boost::split(tokens, xyz_string, boost::is_any_of(" "), boost::token_compress_on);
        if (tokens.size() != 3 || !tesseract_common::isNumeric(tokens))
          throw std::runtime_error("ManipulatorInfo: Error parsing TCP attribute xyz string.");

        double x{ 0 }, y{ 0 }, z{ 0 };
        // No need to check return values because the tokens are verified above
        tesseract_common::toNumeric<double>(tokens[0], x);
        tesseract_common::toNumeric<double>(tokens[1], y);
        tesseract_common::toNumeric<double>(tokens[2], z);

        local_tcp.translation() = Eigen::Vector3d(x, y, z);
      }

      if (tcp_element->Attribute("wxyz") == nullptr)
      {
        status = tesseract_common::QueryStringAttribute(tcp_element, "rpy", rpy_string);
        if (status != tinyxml2::XML_NO_ATTRIBUTE && status != tinyxml2::XML_SUCCESS)
          throw std::runtime_error("ManipulatorInfo: Error parsing TCP attribute rpy.");

        if (status != tinyxml2::XML_NO_ATTRIBUTE)
        {
          std::vector<std::string> tokens;
          boost::split(tokens, rpy_string, boost::is_any_of(" "), boost::token_compress_on);
          if (tokens.size() != 3 || !tesseract_common::isNumeric(tokens))
            throw std::runtime_error("ManipulatorInfo: Error parsing TCP attribute rpy string.");

          double r{ 0 }, p{ 0 }, y{ 0 };
          // No need to check return values because the tokens are verified above
          tesseract_common::toNumeric<double>(tokens[0], r);
          tesseract_common::toNumeric<double>(tokens[1], p);
          tesseract_common::toNumeric<double>(tokens[2], y);

          Eigen::AngleAxisd rollAngle(r, Eigen::Vector3d::UnitX());
          Eigen::AngleAxisd pitchAngle(p, Eigen::Vector3d::UnitY());
          Eigen::AngleAxisd yawAngle(y, Eigen::Vector3d::UnitZ());

          Eigen::Quaterniond rpy = yawAngle * pitchAngle * rollAngle;

          local_tcp.linear() = rpy.toRotationMatrix();
        }
      }
      else
      {
        status = tesseract_common::QueryStringAttribute(tcp_element, "wxyz", wxyz_string);
        if (status != tinyxml2::XML_NO_ATTRIBUTE && status != tinyxml2::XML_SUCCESS)
          throw std::runtime_error("ManipulatorInfo: Error parsing TCP attribute wxyz.");

        if (status != tinyxml2::XML_NO_ATTRIBUTE)
        {
          std::vector<std::string> tokens;
          boost::split(tokens, wxyz_string, boost::is_any_of(" "), boost::token_compress_on);
          if (tokens.size() != 4 || !tesseract_common::isNumeric(tokens))
            throw std::runtime_error("ManipulatorInfo: Error parsing TCP attribute wxyz string.");

          double qw{ 0 }, qx{ 0 }, qy{ 0 }, qz{ 0 };
          // No need to check return values because the tokens are verified above
          tesseract_common::toNumeric<double>(tokens[0], qw);
          tesseract_common::toNumeric<double>(tokens[1], qx);
          tesseract_common::toNumeric<double>(tokens[2], qy);
          tesseract_common::toNumeric<double>(tokens[3], qz);

          Eigen::Quaterniond q(qw, qx, qy, qz);
          q.normalize();

          local_tcp.linear() = q.toRotationMatrix();
        }
      }
      tcp = local_tcp;
    }
    else
    {
      std::string tcp_name;
      tinyxml2::XMLError status = tesseract_common::QueryStringAttribute(tcp_element, "name", tcp_name);
      if (status != tinyxml2::XML_SUCCESS)
        throw std::runtime_error("ManipulatorInfo: Error parsing TCP attribute name.");

      tcp = tcp_name;
    }

    if (tcp_element->Attribute("external") != nullptr)
    {
      bool external = false;
      tinyxml2::XMLError status = tcp_element->QueryBoolAttribute("external", &external);

      if (status != tinyxml2::XML_SUCCESS)
        throw std::runtime_error("ManipulatorInfo: Error parsing TCP attribute external.");

      tcp.setExternal(external);
    }
  }
}

ManipulatorInfo ManipulatorInfo::getCombined(const ManipulatorInfo& manip_info_override) const
{
  ManipulatorInfo combined = *this;

  if (!manip_info_override.manipulator.empty())
    combined.manipulator = manip_info_override.manipulator;

  if (!manip_info_override.manipulator_ik_solver.empty())
    combined.manipulator_ik_solver = manip_info_override.manipulator_ik_solver;

  if (!manip_info_override.working_frame.empty())
    combined.working_frame = manip_info_override.working_frame;

  if (!manip_info_override.tcp.empty())
    combined.tcp = manip_info_override.tcp;

  return combined;
}

bool ManipulatorInfo::empty() const
{
  if (!manipulator.empty())
    return false;

  if (!manipulator_ik_solver.empty())
    return false;

  if (!working_frame.empty())
    return false;

  if (!tcp.empty())
    return false;

  return true;
}

tinyxml2::XMLElement* ManipulatorInfo::toXML(tinyxml2::XMLDocument& doc) const
{
  Eigen::IOFormat eigen_format(Eigen::StreamPrecision, Eigen::DontAlignCols, " ", " ");
  tinyxml2::XMLElement* xml_manip_info = doc.NewElement("ManipulatorInfo");

  tinyxml2::XMLElement* xml_manipulator = doc.NewElement("Manipulator");
  xml_manipulator->SetText(manipulator.c_str());
  xml_manip_info->InsertEndChild(xml_manipulator);

  tinyxml2::XMLElement* xml_manipulator_solver = doc.NewElement("ManipulatorIKSolver");
  xml_manipulator_solver->SetText(manipulator_ik_solver.c_str());
  xml_manip_info->InsertEndChild(xml_manipulator_solver);

  tinyxml2::XMLElement* xml_working_frame = doc.NewElement("WorkingFrame");
  xml_working_frame->SetText(working_frame.c_str());
  xml_manip_info->InsertEndChild(xml_working_frame);

  if (tcp.isString())
  {
    tinyxml2::XMLElement* xml_tcp = doc.NewElement("TCP");
    xml_tcp->SetAttribute("name", tcp.getString().c_str());
    xml_tcp->SetAttribute("external", tcp.isExternal());
    xml_manip_info->InsertEndChild(xml_tcp);
  }
  else if (tcp.isTransform())
  {
    const Eigen::Isometry3d& local_tcp = tcp.getTransform();
    tinyxml2::XMLElement* xml_tcp = doc.NewElement("TCP");
    std::stringstream xyz_string;
    xyz_string << local_tcp.translation().format(eigen_format);
    xml_tcp->SetAttribute("xyz", xyz_string.str().c_str());

    std::stringstream wxyz_string;
    Eigen::Quaterniond q(local_tcp.linear());
    wxyz_string << Eigen::Vector4d(q.w(), q.x(), q.y(), q.z()).format(eigen_format);

    xml_tcp->SetAttribute("wxyz", wxyz_string.str().c_str());
    xml_tcp->SetAttribute("external", tcp.isExternal());
    xml_manip_info->InsertEndChild(xml_tcp);
  }

  return xml_manip_info;
}
}  // namespace tesseract_planning
