/**
 * @file car_seat_example.h
 * @brief An example of a robot on a rail installing a seat in a car.
 *
 * @author Levi Armstrong
 * @date July 22, 2018
 * @version TODO
 * @bug No known bugs
 *
 * @copyright Copyright (c) 2017, Southwest Research Institute
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
#ifndef TESSERACT_ROS_EXAMPLES_CAR_SEAT_EXAMPLE_H
#define TESSERACT_ROS_EXAMPLES_CAR_SEAT_EXAMPLE_H

#include <tesseract_common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <trajopt/problem_description.hpp>
#include <string>
#include <vector>
#include <memory>
#include <ros/ros.h>
#include <Eigen/Geometry>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract_kinematics/core/forward_kinematics.h>
#include <tesseract_ros_examples/example.h>

namespace tesseract_ros_examples
{
/**
 * @brief An example of a robot on a rail installing a seat in a car.
 */
class CarSeatExample : public Example
{
public:
  CarSeatExample(const ros::NodeHandle& nh, bool plotting, bool rviz);
  ~CarSeatExample() override = default;
  CarSeatExample(const CarSeatExample&) = default;
  CarSeatExample& operator=(const CarSeatExample&) = default;
  CarSeatExample(CarSeatExample&&) = default;
  CarSeatExample& operator=(CarSeatExample&&) = default;

  bool run() override;

private:
  ros::NodeHandle nh_;
  int env_current_revision_;
  tesseract_scene_graph::ResourceLocator::Ptr locator_;
  std::unordered_map<std::string, std::unordered_map<std::string, double>> saved_positions_;

  std::shared_ptr<trajopt::ProblemConstructionInfo> cppMethod(const std::string& start, const std::string& finish);
  void addSeats();
  void addCar();
  std::unordered_map<std::string, std::unordered_map<std::string, double>> getPredefinedPosition();
  std::vector<double> getPositionVector(const tesseract_kinematics::ForwardKinematics::ConstPtr& kin,
                                        const std::unordered_map<std::string, double>& pos);
  Eigen::VectorXd getPositionVectorXd(const tesseract_kinematics::ForwardKinematics::ConstPtr& kin,
                                      const std::unordered_map<std::string, double>& pos);
};

}  // namespace tesseract_ros_examples

#endif  // TESSERACT_ROS_EXAMPLES_CAR_SEAT_EXAMPLE_H
