/**
 * @file state_waypoint.cpp
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
#include <boost/algorithm/string.hpp>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract_command_language/state_waypoint.h>
#include <tesseract_command_language/waypoint_type.h>
#include <tesseract_common/utils.h>

namespace tesseract_planning
{
StateWaypoint::StateWaypoint(std::vector<std::string> joint_names, const Eigen::Ref<const Eigen::VectorXd>& position)
  : joint_names(std::move(joint_names)), position(position)
{
}

StateWaypoint::StateWaypoint(const tinyxml2::XMLElement& xml_element)
{
  const tinyxml2::XMLElement* names_element = xml_element.FirstChildElement("Names");
  const tinyxml2::XMLElement* position_element = xml_element.FirstChildElement("Position");
  const tinyxml2::XMLElement* velocity_element = xml_element.FirstChildElement("Velocity");
  const tinyxml2::XMLElement* acceleration_element = xml_element.FirstChildElement("Acceleration");
  const tinyxml2::XMLElement* time_element = xml_element.FirstChildElement("TimeFromStart");

  if (!names_element)
    throw std::runtime_error("StateWaypoint: Must have Names element.");

  if (!position_element)
    throw std::runtime_error("StateWaypoint: Must have Position element.");

  if (!time_element)
    throw std::runtime_error("StateWaypoint: Must have TimeFromStart element.");

  std::vector<std::string> names_tokens, position_tokens;
  std::string names_string;
  tinyxml2::XMLError status = tesseract_common::QueryStringText(names_element, names_string);
  if (status != tinyxml2::XML_NO_ATTRIBUTE && status != tinyxml2::XML_SUCCESS)
    throw std::runtime_error("JointWaypoint: Error parsing Names string");

  std::string position_string;
  status = tesseract_common::QueryStringText(position_element, position_string);
  if (status != tinyxml2::XML_NO_ATTRIBUTE && status != tinyxml2::XML_SUCCESS)
    throw std::runtime_error("JointWaypoint: Error parsing Position string");

  boost::split(names_tokens, names_string, boost::is_any_of(" "), boost::token_compress_on);
  boost::split(position_tokens, position_string, boost::is_any_of(" "), boost::token_compress_on);

  if (names_tokens.empty() || position_tokens.empty())
    throw std::runtime_error("StateWaypoint: Names or Position elements are empty.");

  if (names_tokens.size() != position_tokens.size())
    throw std::runtime_error("StateWaypoint: Names and Position are not the same size.");

  if (!tesseract_common::isNumeric(position_tokens))
    throw std::runtime_error("StateWaypoint: Positions are not all numeric values.");

  joint_names = names_tokens;
  position.resize(static_cast<long>(names_tokens.size()));
  for (std::size_t i = 0; i < position_tokens.size(); ++i)
    tesseract_common::toNumeric<double>(position_tokens[i], position[static_cast<long>(i)]);

  std::string time_string;
  status = tesseract_common::QueryStringText(time_element, time_string);
  if (status != tinyxml2::XML_NO_ATTRIBUTE && status != tinyxml2::XML_SUCCESS)
    throw std::runtime_error("StateWaypoint: Error parsing TimeFromStart string");

  if (!tesseract_common::isNumeric(time_string))
    throw std::runtime_error("StateWaypoint: TimeFromStart is not a numeric values.");

  tesseract_common::toNumeric<double>(time_string, time);

  if (velocity_element)
  {
    std::vector<std::string> velocity_tokens;
    std::string velocity_string;
    status = tesseract_common::QueryStringText(velocity_element, velocity_string);
    if (status != tinyxml2::XML_NO_ATTRIBUTE && status != tinyxml2::XML_SUCCESS)
      throw std::runtime_error("JointWaypoint: Error parsing Velocity string");

    boost::split(velocity_tokens, velocity_string, boost::is_any_of(" "), boost::token_compress_on);
    if (names_tokens.size() != velocity_tokens.size())
      throw std::runtime_error("StateWaypoint: Velocity size is incorrect.");

    if (!tesseract_common::isNumeric(velocity_tokens))
      throw std::runtime_error("StateWaypoint: Velocity are not all numeric values.");

    velocity.resize(static_cast<long>(names_tokens.size()));
    for (std::size_t i = 0; i < velocity_tokens.size(); ++i)
      tesseract_common::toNumeric<double>(velocity_tokens[i], velocity[static_cast<long>(i)]);
  }

  if (acceleration_element)
  {
    std::vector<std::string> acceleration_tokens;
    std::string acceleration_string;
    status = tesseract_common::QueryStringText(acceleration_element, acceleration_string);
    if (status != tinyxml2::XML_NO_ATTRIBUTE && status != tinyxml2::XML_SUCCESS)
      throw std::runtime_error("JointWaypoint: Error parsing Acceleration string");

    boost::split(acceleration_tokens, acceleration_string, boost::is_any_of(" "), boost::token_compress_on);
    if (names_tokens.size() != acceleration_tokens.size())
      throw std::runtime_error("StateWaypoint: Acceleration size is incorrect.");

    if (!tesseract_common::isNumeric(acceleration_tokens))
      throw std::runtime_error("StateWaypoint: Acceleration are not all numeric values.");

    acceleration.resize(static_cast<long>(names_tokens.size()));
    for (std::size_t i = 0; i < acceleration_tokens.size(); ++i)
      tesseract_common::toNumeric<double>(acceleration_tokens[i], acceleration[static_cast<long>(i)]);
  }
}

int StateWaypoint::getType() const { return static_cast<int>(WaypointType::STATE_WAYPOINT); }

void StateWaypoint::print(const std::string& prefix) const
{
  std::cout << prefix << "State WP: Pos=" << position.transpose() << std::endl;
}

tinyxml2::XMLElement* StateWaypoint::toXML(tinyxml2::XMLDocument& doc) const
{
  Eigen::IOFormat eigen_format(Eigen::StreamPrecision, Eigen::DontAlignCols, " ", " ");
  tinyxml2::XMLElement* xml_waypoint = doc.NewElement("Waypoint");
  xml_waypoint->SetAttribute("type", std::to_string(getType()).c_str());

  tinyxml2::XMLElement* xml_joint_waypoint = doc.NewElement("StateWaypoint");
  tinyxml2::XMLElement* xml_joint_names = doc.NewElement("Names");
  if (!joint_names.empty())
  {
    std::string jn = joint_names[0];
    for (std::size_t i = 1; i < joint_names.size(); ++i)
      jn += " " + joint_names[i];

    xml_joint_names->SetText(jn.c_str());
  }
  xml_joint_waypoint->InsertEndChild(xml_joint_names);

  std::stringstream position_string;
  position_string << position.format(eigen_format);

  tinyxml2::XMLElement* xml_joint_position = doc.NewElement("Position");
  xml_joint_position->SetText(position_string.str().c_str());
  xml_joint_waypoint->InsertEndChild(xml_joint_position);

  if (velocity.size() > 0)
  {
    std::stringstream velocity_string;
    velocity_string << velocity.format(eigen_format);

    tinyxml2::XMLElement* xml_joint_velocity = doc.NewElement("Velocity");
    xml_joint_velocity->SetText(velocity_string.str().c_str());
    xml_joint_waypoint->InsertEndChild(xml_joint_velocity);
  }

  if (acceleration.size() > 0)
  {
    std::stringstream acceleration_string;
    acceleration_string << acceleration.format(eigen_format);

    tinyxml2::XMLElement* xml_joint_acceleration = doc.NewElement("Acceleration");
    xml_joint_acceleration->SetText(acceleration_string.str().c_str());
    xml_joint_waypoint->InsertEndChild(xml_joint_acceleration);
  }

  tinyxml2::XMLElement* xml_time = doc.NewElement("TimeFromStart");
  xml_time->SetText(std::to_string(time).c_str());
  xml_joint_waypoint->InsertEndChild(xml_time);

  xml_waypoint->InsertEndChild(xml_joint_waypoint);

  return xml_waypoint;
}
}  // namespace tesseract_planning
