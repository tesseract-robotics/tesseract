/**
 * @file plotting.h
 * @brief Tesseract ROS Basic plotting functions.
 *
 * @author Levi Armstrong
 * @date April 15, 2018
 * @version TODO
 * @bug No known bugs
 *
 * @copyright Copyright (c) 2013, Southwest Research Institute
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
#ifndef TESSERACT_ROSUTILS_PLOTTING_H
#define TESSERACT_ROSUTILS_PLOTTING_H

#include <tesseract_common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <ros/console.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <ros/publisher.h>
#include <tesseract_msgs/Trajectory.h>
#include <tesseract_msgs/TesseractState.h>
#include <tesseract_motion_planners/core/waypoint.h>
#include <Eigen/Geometry>
#include <ros/ros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract_visualization/visualization.h>
#include <tesseract_rosutils/utils.h>
#include <tesseract_environment/core/environment.h>

namespace tesseract_rosutils
{
/** @brief The BasicPlotting class */
class ROSPlotting : public tesseract_visualization::Visualization
{
public:
  ROSPlotting(tesseract_environment::Environment::ConstPtr env) : env_(std::move(env))
  {
    ros::NodeHandle nh;

    trajectory_pub_ = nh.advertise<tesseract_msgs::Trajectory>("/trajopt/display_tesseract_trajectory", 1, true);
    collisions_pub_ = nh.advertise<visualization_msgs::MarkerArray>("/trajopt/display_collisions", 1, true);
    arrows_pub_ = nh.advertise<visualization_msgs::MarkerArray>("/trajopt/display_arrows", 1, true);
    axes_pub_ = nh.advertise<visualization_msgs::MarkerArray>("/trajopt/display_axes", 1, true);
  }

  void plotTrajectory(const std::vector<std::string>& joint_names,
                      const Eigen::Ref<const tesseract_common::TrajArray>& traj) override
  {
    tesseract_msgs::Trajectory msg;

    // Set tesseract state information
    toMsg(msg.tesseract_state, *env_);

    // Set the joint trajectory message
    toMsg(msg.joint_trajectory, *(env_->getCurrentState()), joint_names, traj);

    trajectory_pub_.publish(msg);
  }

  /**
   * @brief Plots waypoints according to their type. Currently only CARTESIAN_WAYPOINT is implemented.
   * @param waypoints A vector of waypoint pointers.
   */
  void plotWaypoints(const std::vector<tesseract_motion_planners::Waypoint::Ptr>& waypoints)
  {
    for (auto& waypoint : waypoints)
    {
      if (waypoint->getType() == tesseract_motion_planners::WaypointType::CARTESIAN_WAYPOINT)
      {
        auto cart_wp = std::static_pointer_cast<const tesseract_motion_planners::CartesianWaypoint>(waypoint);
        if (!cart_wp->getParentLinkName().empty() && cart_wp->getParentLinkName() != env_->getSceneGraph()->getRoot())
        {
          if (env_->getLink(cart_wp->getParentLinkName()) != nullptr)
          {
            Eigen::Isometry3d root_to_parent = env_->getLinkTransform(cart_wp->getParentLinkName());
            plotAxis(root_to_parent * cart_wp->getTransform(), 0.05);
          }
          else
            ROS_WARN("Unable to plot waypoint. Parent link '%s' not found", cart_wp->getParentLinkName().c_str());
        }
        else
        {
          plotAxis(cart_wp->getTransform(), 0.05);
        }
      }
    }
  }

  static visualization_msgs::MarkerArray
  getContactResultsMarkerArrayMsg(int& id_counter,
                                  const std::string& frame_id,
                                  const std::string& ns,
                                  const ros::Time& time_stamp,
                                  const std::vector<std::string>& link_names,
                                  const tesseract_collision::ContactResultVector& dist_results,
                                  const Eigen::Ref<const Eigen::VectorXd>& safety_distances)
  {
    visualization_msgs::MarkerArray msg;
    for (unsigned i = 0; i < dist_results.size(); ++i)
    {
      const tesseract_collision::ContactResult& dist = dist_results[i];
      const double& safety_distance = safety_distances[i];

      Eigen::Vector4d rgba;
      if (dist.distance < 0)
      {
        rgba << 1.0, 0.0, 0.0, 1.0;
      }
      else if (dist.distance < safety_distance)
      {
        rgba << 1.0, 1.0, 0.0, 1.0;
      }
      else
      {
        rgba << 0.0, 1.0, 0.0, 1.0;
      }

      if (dist.cc_type[0] == tesseract_collision::ContinuousCollisionType::CCType_Between)
      {
        Eigen::Vector4d cc_rgba;
        cc_rgba << 0.0, 0.0, 1.0, 1.0;
        auto marker = getMarkerArrowMsg(id_counter,
                                        frame_id,
                                        ns,
                                        time_stamp,
                                        dist.transform[0] * dist.nearest_points_local[0],
                                        dist.cc_transform[0] * dist.nearest_points_local[0],
                                        cc_rgba,
                                        0.01);
        msg.markers.push_back(marker);
      }

      if (dist.cc_type[1] == tesseract_collision::ContinuousCollisionType::CCType_Between)
      {
        Eigen::Vector4d cc_rgba;
        cc_rgba << 0.0, 0.0, 0.5, 1.0;
        auto marker = getMarkerArrowMsg(id_counter,
                                        frame_id,
                                        ns,
                                        time_stamp,
                                        dist.transform[1] * dist.nearest_points_local[1],
                                        dist.cc_transform[1] * dist.nearest_points_local[1],
                                        cc_rgba,
                                        0.01);
        msg.markers.push_back(marker);
      }

      auto it0 = std::find(link_names.begin(), link_names.end(), dist.link_names[0]);
      auto it1 = std::find(link_names.begin(), link_names.end(), dist.link_names[1]);

      if (it0 != link_names.end() && it1 != link_names.end())
      {
        auto marker0 = getMarkerArrowMsg(
            id_counter, frame_id, ns, time_stamp, dist.nearest_points[0], dist.nearest_points[1], rgba, 0.01);
        msg.markers.push_back(marker0);

        auto marker1 = getMarkerArrowMsg(
            id_counter, frame_id, ns, time_stamp, dist.nearest_points[1], dist.nearest_points[0], rgba, 0.01);
        msg.markers.push_back(marker1);
      }
      else if (it0 != link_names.end())
      {
        auto marker = getMarkerArrowMsg(
            id_counter, frame_id, ns, time_stamp, dist.nearest_points[1], dist.nearest_points[0], rgba, 0.01);
        msg.markers.push_back(marker);
      }
      else
      {
        auto marker = getMarkerArrowMsg(
            id_counter, frame_id, ns, time_stamp, dist.nearest_points[0], dist.nearest_points[1], rgba, 0.01);
        msg.markers.push_back(marker);
      }
    }

    return msg;
  }

  void plotContactResults(const std::vector<std::string>& link_names,
                          const tesseract_collision::ContactResultVector& dist_results,
                          const Eigen::Ref<const Eigen::VectorXd>& safety_distances) override
  {
    visualization_msgs::MarkerArray msg = getContactResultsMarkerArrayMsg(marker_counter_,
                                                                          env_->getSceneGraph()->getRoot(),
                                                                          "trajopt",
                                                                          ros::Time::now(),
                                                                          link_names,
                                                                          dist_results,
                                                                          safety_distances);
    if (!dist_results.empty())
    {
      collisions_pub_.publish(msg);
    }
  }

  void plotArrow(const Eigen::Ref<const Eigen::Vector3d>& pt1,
                 const Eigen::Ref<const Eigen::Vector3d>& pt2,
                 const Eigen::Ref<const Eigen::Vector4d>& rgba,
                 double scale) override
  {
    visualization_msgs::MarkerArray msg;
    auto marker = getMarkerArrowMsg(
        marker_counter_, env_->getSceneGraph()->getRoot(), "trajopt", ros::Time::now(), pt1, pt2, rgba, scale);
    msg.markers.push_back(marker);
    arrows_pub_.publish(msg);
  }

  void plotAxis(const Eigen::Isometry3d& axis, double scale) override
  {
    visualization_msgs::MarkerArray msg;
    Eigen::Vector3d x_axis = axis.matrix().block<3, 1>(0, 0);
    Eigen::Vector3d y_axis = axis.matrix().block<3, 1>(0, 1);
    Eigen::Vector3d z_axis = axis.matrix().block<3, 1>(0, 2);
    Eigen::Vector3d position = axis.matrix().block<3, 1>(0, 3);

    auto marker = getMarkerCylinderMsg(marker_counter_,
                                       env_->getSceneGraph()->getRoot(),
                                       "trajopt",
                                       ros::Time::now(),
                                       position,
                                       position + x_axis,
                                       Eigen::Vector4d(1, 0, 0, 1),
                                       scale);
    msg.markers.push_back(marker);

    marker = getMarkerCylinderMsg(marker_counter_,
                                  env_->getSceneGraph()->getRoot(),
                                  "trajopt",
                                  ros::Time::now(),
                                  position,
                                  position + y_axis,
                                  Eigen::Vector4d(0, 1, 0, 1),
                                  scale);
    msg.markers.push_back(marker);

    marker = getMarkerCylinderMsg(marker_counter_,
                                  env_->getSceneGraph()->getRoot(),
                                  "trajopt",
                                  ros::Time::now(),
                                  position,
                                  position + z_axis,
                                  Eigen::Vector4d(0, 0, 1, 1),
                                  scale);
    msg.markers.push_back(marker);

    axes_pub_.publish(msg);
  }

  void clear() override
  {
    // Remove old arrows
    marker_counter_ = 0;
    visualization_msgs::MarkerArray msg;
    visualization_msgs::Marker marker;
    marker.header.frame_id = env_->getSceneGraph()->getRoot();
    marker.header.stamp = ros::Time();
    marker.ns = "trajopt";
    marker.id = 0;
    marker.type = visualization_msgs::Marker::ARROW;
    marker.action = visualization_msgs::Marker::DELETEALL;
    msg.markers.push_back(marker);
    collisions_pub_.publish(msg);
    arrows_pub_.publish(msg);
    axes_pub_.publish(msg);

    ros::Duration(0.5).sleep();
  }

  void waitForInput() override
  {
    ROS_ERROR("Hit enter key to step optimization!");
    std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
  }

  static visualization_msgs::Marker getMarkerArrowMsg(int& id_counter,
                                                      const std::string& frame_id,
                                                      const std::string& ns,
                                                      const ros::Time& time_stamp,
                                                      const Eigen::Ref<const Eigen::Vector3d>& pt1,
                                                      const Eigen::Ref<const Eigen::Vector3d>& pt2,
                                                      const Eigen::Ref<const Eigen::Vector4d>& rgba,
                                                      const double scale)
  {
    visualization_msgs::Marker marker;
    marker.header.frame_id = frame_id;
    marker.header.stamp = time_stamp;
    marker.ns = ns;
    marker.id = ++id_counter;
    marker.type = visualization_msgs::Marker::ARROW;
    marker.action = visualization_msgs::Marker::ADD;

    Eigen::Vector3d x, y, z;
    x = (pt2 - pt1).normalized();
    marker.pose.position.x = pt1(0);
    marker.pose.position.y = pt1(1);
    marker.pose.position.z = pt1(2);

    y = x.unitOrthogonal();
    z = (x.cross(y)).normalized();
    Eigen::Matrix3d rot;
    rot.col(0) = x;
    rot.col(1) = y;
    rot.col(2) = z;
    Eigen::Quaterniond q(rot);
    q.normalize();
    marker.pose.orientation.x = q.x();
    marker.pose.orientation.y = q.y();
    marker.pose.orientation.z = q.z();
    marker.pose.orientation.w = q.w();

    marker.scale.x = std::abs((pt2 - pt1).norm());
    marker.scale.y = scale;
    marker.scale.z = scale;

    marker.color.r = static_cast<float>(rgba(0));
    marker.color.g = static_cast<float>(rgba(1));
    marker.color.b = static_cast<float>(rgba(2));
    marker.color.a = static_cast<float>(rgba(3));

    return marker;
  }

  static visualization_msgs::Marker getMarkerCylinderMsg(int& id_counter,
                                                         const std::string& frame_id,
                                                         const std::string& ns,
                                                         const ros::Time& time_stamp,
                                                         const Eigen::Ref<const Eigen::Vector3d>& pt1,
                                                         const Eigen::Ref<const Eigen::Vector3d>& pt2,
                                                         const Eigen::Ref<const Eigen::Vector4d>& rgba,
                                                         const double scale)
  {
    visualization_msgs::Marker marker;
    marker.header.frame_id = frame_id;
    marker.header.stamp = time_stamp;
    marker.ns = ns;
    marker.id = ++id_counter;
    marker.type = visualization_msgs::Marker::CYLINDER;
    marker.action = visualization_msgs::Marker::ADD;

    double length = scale * std::abs((pt2 - pt1).norm());
    Eigen::Vector3d x, y, z, center;
    z = (pt2 - pt1).normalized();
    center = pt1 + (length / 2.0) * z;
    marker.pose.position.x = center(0);
    marker.pose.position.y = center(1);
    marker.pose.position.z = center(2);

    y = z.unitOrthogonal();
    x = (y.cross(z)).normalized();
    Eigen::Matrix3d rot;
    rot.col(0) = x;
    rot.col(1) = y;
    rot.col(2) = z;
    Eigen::Quaterniond q(rot);
    q.normalize();
    marker.pose.orientation.x = q.x();
    marker.pose.orientation.y = q.y();
    marker.pose.orientation.z = q.z();
    marker.pose.orientation.w = q.w();

    marker.scale.x = length / 20.0;
    marker.scale.y = length / 20.0;
    marker.scale.z = length;

    marker.color.r = static_cast<float>(rgba(0));
    marker.color.g = static_cast<float>(rgba(1));
    marker.color.b = static_cast<float>(rgba(2));
    marker.color.a = static_cast<float>(rgba(3));

    return marker;
  }

private:
  tesseract_environment::Environment::ConstPtr env_; /**< The Env */
  int marker_counter_;                               /**< Counter when plotting */
  ros::Publisher scene_pub_;                         /**< Scene publisher */
  ros::Publisher trajectory_pub_;                    /**< Trajectory publisher */
  ros::Publisher collisions_pub_;                    /**< Collision Data publisher */
  ros::Publisher arrows_pub_;                        /**< Used for publishing arrow markers */
  ros::Publisher axes_pub_;                          /**< Used for publishing axis markers */
};
using ROSPlottingPtr = std::shared_ptr<ROSPlotting>;
using ROSPlottingConstPtr = std::shared_ptr<const ROSPlotting>;
}  // namespace tesseract_rosutils

#endif
