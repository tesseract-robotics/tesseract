/**
 * @file contact_monitor_node.cpp
 * @brief Node that instantiates a contact_monitor
 *
 * @author David Merz, Jr.
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

#include <tesseract_monitoring/contact_monitor.h>

#include <tesseract_common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <ros/ros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <boost/thread/thread.hpp>

#include <tesseract/tesseract.h>
#include <tesseract_scene_graph/graph.h>
#include <tesseract_scene_graph/utils.h>
#include <tesseract_scene_graph/parser/srdf_parser.h>
#include <tesseract_urdf/urdf_parser.h>
#include <tesseract_rosutils/utils.h>

// Stuff for the contact monitor
const static std::string ROBOT_DESCRIPTION_PARAM = "robot_description"; /**< Default ROS parameter for robot description */
const static double DEFAULT_CONTACT_DISTANCE = 0.1;

int main(int argc, char** argv)
{
  ros::init(argc, argv, "tesseract_contact_monitoring");
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");

  tesseract_scene_graph::SceneGraph::Ptr scene_graph;
  tesseract_scene_graph::SRDFModel::Ptr srdf_model;
  std::string robot_description;
  bool publish_environment;
  bool publish_markers;

  pnh.param<std::string>("robot_description", robot_description, ROBOT_DESCRIPTION_PARAM);
  pnh.param<bool>("publish_environment", publish_environment, false);
  pnh.param<bool>("publish_markers", publish_markers, false);

  // Initial setup
  std::string urdf_xml_string, srdf_xml_string;
  if (!nh.hasParam(robot_description))
  {
    ROS_ERROR("Failed to find parameter: %s", robot_description.c_str());
    return -1;
  }

  if (!nh.hasParam(robot_description + "_semantic"))
  {
    ROS_ERROR("Failed to find parameter: %s", (robot_description + "_semantic").c_str());
    return -1;
  }

  nh.getParam(robot_description, urdf_xml_string);
  nh.getParam(robot_description + "_semantic", srdf_xml_string);

  tesseract::Tesseract::Ptr tess = std::make_shared<tesseract::Tesseract>();
  tesseract_scene_graph::ResourceLocator::Ptr locator = std::make_shared<tesseract_rosutils::ROSResourceLocator>();
  if (!tess->init(urdf_xml_string, srdf_xml_string, locator))
  {
    ROS_ERROR("Failed to initialize environment.");
    return -1;
  }

  double contact_distance;
  pnh.param<double>("contact_distance", contact_distance, DEFAULT_CONTACT_DISTANCE);

  std::vector<std::string> monitored_link_names;
  monitored_link_names = tess->getEnvironment()->getLinkNames();
  if (pnh.hasParam("monitor_links"))
    pnh.getParam("monitor_links", monitored_link_names);

  if (monitored_link_names.empty())
    monitored_link_names = tess->getEnvironment()->getLinkNames();

  int contact_test_type = 2;
  if (pnh.hasParam("contact_test_type"))
    pnh.getParam("contact_test_type", contact_test_type);

  if (contact_test_type < 0 || contact_test_type > 3)
  {
    ROS_WARN("Request type must be 0, 1, 2 or 3. Setting to 2(ALL)!");
    contact_test_type = 2;
  }
  tesseract_collision::ContactTestType type = static_cast<tesseract_collision::ContactTestType>(contact_test_type);

  contact_monitor::ContactMonitor cm(tess,
                                     nh,
                                     pnh,
                                     monitored_link_names,
                                     type,
                                     contact_distance,
                                     publish_environment,
                                     publish_markers);

  boost::thread t (&contact_monitor::ContactMonitor::computeCollisionReportThread, &cm);

  ROS_INFO("Contact Monitor Running!");

  ros::spin();

  return 0;
}
