/**
 * @file utils.i
 * @brief SWIG interface file for tesseract_rosutils/utils.h
 *
 * @author John Wason
 * @date December 10, 2019
 * @version TODO
 * @bug No known bugs
 *
 * @copyright Copyright (c) 2019, Wason Technology, LLC
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

%{
#include <tesseract_rosutils/utils.h>
%}

%shared_ptr(tesseract_rosutils::ROSResourceLocator)

namespace tesseract_rosutils
{
class ROSResourceLocator : public tesseract_scene_graph::ResourceLocator
{
public:
  ROSResourceLocator();
  virtual tesseract_common::Resource::Ptr locateResource(const std::string& url) override;
};
}

%define CONVERT_MSG_TYPE(name, tesseract_type, msg_type)
%inline %{
void name ## ToMsg(msg_type& msg, const tesseract_type& t)
{
  if (!tesseract_rosutils::toMsg(msg, t))
  {
    throw std::runtime_error(#name "ToMsg failed");
  }  
}

tesseract_type::Ptr name ## FromMsg(const msg_type& msg)
{
  tesseract_type::Ptr t;
  if (!tesseract_rosutils::fromMsg(t,msg))
  {
    throw std::runtime_error(#name "ToMsg failed");
  }
  return t;
}
%}
%enddef

%define CONVERT_MSG_TYPE2(name, tesseract_type, msg_type)
%inline %{
void name ## ToMsg(msg_type& msg, const tesseract_type::Ptr& t)
{
  if (!tesseract_rosutils::toMsg(msg, t))
  {
    throw std::runtime_error(#name "ToMsg failed");
  }  
}

tesseract_type::Ptr name ## FromMsg(const msg_type& msg)
{
  tesseract_type::Ptr t;
  if (!tesseract_rosutils::fromMsg(t,msg))
  {
    throw std::runtime_error(#name "ToMsg failed");
  }
  return t;
}
%}
%enddef

%define CONVERT_MSG_TYPE3(name, tesseract_type, msg_type)
%inline %{
void name ## ToMsg(msg_type& msg, const tesseract_type& t)
{  
  if (!tesseract_rosutils::toMsg(msg, t))
  {
    throw std::runtime_error(#name "ToMsg failed");
  }  
}

tesseract_type name ## FromMsg(const msg_type& msg)
{  
  return tesseract_rosutils::fromMsg(msg);  
}
%}
%enddef

%define CONVERT_MSG_TYPE4(name, tesseract_type, msg_type)
%inline %{
void name ## ToMsg(msg_type& msg, const tesseract_type& t)
{
  if (!tesseract_rosutils::toMsg(msg, t))
  {
    throw std::runtime_error(#name "ToMsg failed");
  }  
}
%}
%enddef

%define CONVERT_MSG_TYPE5(name, tesseract_type, msg_type)
%inline %{
void name ## ToMsg(msg_type& msg, const tesseract_type& t)
{
  tesseract_rosutils::toMsg(msg, t);
}
%}
%enddef

%define CONVERT_MSG_TYPE6(name, tesseract_type, msg_type)
%inline %{
void name ## ToMsg(msg_type& msg, const tesseract_type& t)
{
  if (!tesseract_rosutils::toMsg(msg, t))
  {
    throw std::runtime_error(#name "ToMsg failed");
  }  
}

tesseract_type name ## FromMsg(const msg_type& msg)
{
  tesseract_type t;
  if (!tesseract_rosutils::fromMsg(t,msg))
  {
    throw std::runtime_error(#name "ToMsg failed");
  }
  return t;
}
%}
%enddef

// Macro to convert messages with move-only semantics
%define CONVERT_MSG_TYPE7(name, tesseract_type, msg_type)
%inline %{
void name ## ToMsg(msg_type& msg, const tesseract_type& t)
{
  if (!tesseract_rosutils::toMsg(msg, t))
  {
    throw std::runtime_error(#name "ToMsg failed");
  }
}

tesseract_type::Ptr name ## FromMsg(const msg_type& msg)
{
  tesseract_type::Ptr t = std::make_shared<tesseract_type>(std::move(tesseract_rosutils::fromMsg(msg)));
  return t;
}
%}
%enddef


%define PROCESS_MSG(name, tesseract_type, msg_type)
%inline %{
void name ## ProcessMsg(tesseract_type& t, const msg_type& msg)
{
  if (!tesseract_rosutils::processMsg(t,msg))
  {
    throw std::runtime_error(#name "ProcessMsg failed");
  }  
}
%}
%enddef

%rosmsg_typemaps(tesseract_msgs::Geometry)
%rosmsg_typemaps(tesseract_msgs::Material)
%rosmsg_typemaps(tesseract_msgs::Inertial)
%rosmsg_typemaps(tesseract_msgs::VisualGeometry)
%rosmsg_typemaps(tesseract_msgs::CollisionGeometry)
%rosmsg_typemaps(tesseract_msgs::Link)
%rosmsg_typemaps(tesseract_msgs::JointCalibration)
%rosmsg_typemaps(tesseract_msgs::JointDynamics)
%rosmsg_typemaps(tesseract_msgs::JointLimits)
%rosmsg_typemaps(tesseract_msgs::JointMimic)
%rosmsg_typemaps(tesseract_msgs::JointSafety)
%rosmsg_typemaps(tesseract_msgs::Joint)
%rosmsg_typemaps(sensor_msgs::JointState)
%rosmsg_typemaps(tesseract_msgs::EnvironmentCommand)
%rosmsg_vector_typemaps(tesseract_msgs::EnvironmentCommand)
%rosmsg_typemaps(tesseract_msgs::TesseractState)
%rosmsg_typemaps(trajectory_msgs::JointTrajectory)
%rosmsg_typemaps(tesseract_msgs::ContactResult)
%rosmsg_typemaps(geometry_msgs::Pose)

// tesseract_geometry
CONVERT_MSG_TYPE(geometry, tesseract_geometry::Geometry, tesseract_msgs::Geometry)

// tesseract_scene_graph
CONVERT_MSG_TYPE2(materials, tesseract_scene_graph::Material, tesseract_msgs::Material)
CONVERT_MSG_TYPE2(inertial, tesseract_scene_graph::Inertial, tesseract_msgs::Inertial)
CONVERT_MSG_TYPE(visualGeometry, tesseract_scene_graph::Visual, tesseract_msgs::VisualGeometry)
CONVERT_MSG_TYPE(collisionGeometry, tesseract_scene_graph::Collision, tesseract_msgs::CollisionGeometry)
CONVERT_MSG_TYPE7(link, tesseract_scene_graph::Link, tesseract_msgs::Link)
CONVERT_MSG_TYPE2(jointCalibration, tesseract_scene_graph::JointCalibration, tesseract_msgs::JointCalibration)
CONVERT_MSG_TYPE2(jointDynamics, tesseract_scene_graph::JointDynamics, tesseract_msgs::JointDynamics)
CONVERT_MSG_TYPE2(jointLimits, tesseract_scene_graph::JointLimits, tesseract_msgs::JointLimits)
CONVERT_MSG_TYPE2(jointMimic, tesseract_scene_graph::JointMimic, tesseract_msgs::JointMimic)
CONVERT_MSG_TYPE2(jointSafety, tesseract_scene_graph::JointSafety, tesseract_msgs::JointSafety)
CONVERT_MSG_TYPE7(joint, tesseract_scene_graph::Joint, tesseract_msgs::Joint)

// tesseract_environment
CONVERT_MSG_TYPE5(tesseractEnvStateJoints, tesseract_environment::EnvState, sensor_msgs::JointState)
CONVERT_MSG_TYPE4(environmentCommand, tesseract_environment::Command, tesseract_msgs::EnvironmentCommand)
%inline %{
void environmentCommandsToMsg(std::vector<tesseract_msgs::EnvironmentCommand>& commands_msg,
                  const tesseract_environment::Commands& commands,
                  const unsigned long past_revision)
{
  if (!tesseract_rosutils::toMsg(commands_msg,commands,past_revision))
  {
    throw std::runtime_error("environmentCommandsToMsg failed");
  }
}
%}
CONVERT_MSG_TYPE5(environment, tesseract_environment::Environment, tesseract_msgs::TesseractState)

// trajectory
%inline %{
void trajectoryToMsg(trajectory_msgs::JointTrajectory& traj_msg,
           const tesseract_environment::EnvState& start_state,
           const std::vector<std::string>& joint_names,
           const Eigen::Ref<const tesseract_common::TrajArray>& traj)
{
  tesseract_rosutils::toMsg(traj_msg, start_state, joint_names, traj);
}
%}

CONVERT_MSG_TYPE5(trajectory, tesseract_common::JointTrajectory, trajectory_msgs::JointTrajectory)

// tesseract_collision
CONVERT_MSG_TYPE5(contactResult, tesseract_collision::ContactResult, tesseract_msgs::ContactResult)

// tesseract_motion_planners
CONVERT_MSG_TYPE6(isometry, Eigen::Isometry3d, geometry_msgs::Pose)
CONVERT_MSG_TYPE4(waypoint, tesseract_motion_planners::Waypoint, sensor_msgs::JointState)
CONVERT_MSG_TYPE4(waypoints, std::vector<tesseract_motion_planners::Waypoint::Ptr>, geometry_msgs::PoseArray)
CONVERT_MSG_TYPE4(transforms, tesseract_common::VectorIsometry3d, geometry_msgs::PoseArray)

// process_msg

PROCESS_MSG(environmentJointStateMsg, tesseract_environment::Environment, sensor_msgs::JointState)
PROCESS_MSG(environmentCommandsMsg, tesseract_environment::Environment, std::vector<tesseract_msgs::EnvironmentCommand>)
PROCESS_MSG(environmentTesseractStateMsg, tesseract_environment::Environment, tesseract_msgs::TesseractState)
