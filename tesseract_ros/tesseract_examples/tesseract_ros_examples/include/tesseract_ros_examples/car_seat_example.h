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

class CarSeatExample : public Example
{
public:
  CarSeatExample(ros::NodeHandle nh, bool plotting, bool rviz);
  ~CarSeatExample() = default;

  bool run() override;

private:
  ros::NodeHandle nh_;
  int env_current_revision_;
  tesseract_scene_graph::ResourceLocatorFn locator_;
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

}

#endif // TESSERACT_ROS_EXAMPLES_CAR_SEAT_EXAMPLE_H
