/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2012, Willow Garage, Inc.
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
#include <ros/console.h>
#include <geometric_shapes/solid_primitive_dims.h>
#include <tesseract_msgs/TesseractState.h>
#include <tesseract_msgs/ModifyTesseractEnv.h>
#include <tesseract_msgs/EnvironmentCommand.h>
#include <ros/service.h>
TESSERACT_ENVIRONMENT_IGNORE_WARNINGS_POP

//#include <tesseract_ros/ros_tesseract_utils.h>
//#include <tesseract_collision/core/collision_shapes.h>

static const std::string ROBOT_DESCRIPTION = "robot_description";

static ros::ServiceClient modify_env;

void sendSphere()
{
  tesseract_msgs::ModifyTesseractEnv update_env;

  // Create add command
  tesseract_msgs::EnvironmentCommand add_sphere_command;
  add_sphere_command.command = tesseract_msgs::EnvironmentCommand::ADD;

  // Create the link
  add_sphere_command.add_link.name = "sphere_attached";

  tesseract_msgs::VisualGeometry visual;
  visual.origin.position.x = 0.5;
  visual.origin.position.y = 0;
  visual.origin.position.z = 0.55;

  visual.geometry.type = tesseract_msgs::Geometry::SPHERE;
  visual.geometry.sphere_radius = 0.15;

  add_sphere_command.add_link.visual.push_back(visual);

  tesseract_msgs::CollisionGeometry collision;
  collision.origin.position.x = 0.5;
  collision.origin.position.y = 0;
  collision.origin.position.z = 0.55;

  collision.geometry.type = tesseract_msgs::Geometry::SPHERE;
  collision.geometry.sphere_radius = 0.15;

  add_sphere_command.add_link.collision.push_back(collision);

  // Create the Joint
  add_sphere_command.add_joint.type = tesseract_msgs::Joint::FIXED;
  add_sphere_command.add_joint.name = "sphere_attached_joint";
  add_sphere_command.add_joint.parent_link_name = "base_link";
  add_sphere_command.add_joint.child_link_name = "sphere_attached";

  update_env.request.id = 0;
  update_env.request.commands.push_back(add_sphere_command);

  if (modify_env.call(update_env))
  {
    ROS_INFO("Environment updated!");
  }
  else
  {
    ROS_INFO("Failed to update environment");
  }

}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "demo", ros::init_options::AnonymousName);
  ros::NodeHandle nh;

  ros::AsyncSpinner spinner(1);
  spinner.start();

  modify_env = nh.serviceClient<tesseract_msgs::ModifyTesseractEnv>("modify_tesseract", 10);

  sendSphere();

  ros::waitForShutdown();

  return 0;
}
