/**
 * @file tesseract_ignition_vizualization.h
 * @brief A tesseract vizualization implementation leveraging Ignition Robotics
 *
 * @author Levi Armstrong
 * @date May 14, 2020
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
#ifndef TESSERACT_VISUALIZATION_IGNITION_TESSERACT_IGNITION_VISUALIZATION_H
#define TESSERACT_VISUALIZATION_IGNITION_TESSERACT_IGNITION_VISUALIZATION_H

#include <tesseract_common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <ignition/msgs/scene.pb.h>
#include <ignition/msgs/boolean.pb.h>
#include <ignition/transport/Node.hh>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract_visualization/visualization.h>
#include <tesseract_visualization/ignition/entity_manager.h>
#include <tesseract_environment/core/environment.h>
#include <tesseract_visualization/ignition/visibility_control.h>

namespace tesseract_visualization
{
/** @brief The Tesseract Ignition Vizualization class */
class TESSERACT_VISUALIZATION_IGNITION_PUBLIC TesseractIgnitionVisualization
  : public tesseract_visualization::Visualization
{
public:
  using Ptr = std::shared_ptr<TesseractIgnitionVisualization>;
  using ConstPtr = std::shared_ptr<const TesseractIgnitionVisualization>;

  TesseractIgnitionVisualization();

  bool init(tesseract::Tesseract::ConstPtr thor) override;

  void plotEnvironment(tesseract_environment::Environment::ConstPtr env = nullptr) override;

  void plotEnvironmentState(tesseract_environment::EnvState::ConstPtr state = nullptr) override;

  bool isConnected() const override;

  void waitForConnection(long seconds = 0) const override;

  void plotTrajectory(const std::vector<std::string>& joint_names,
                      const Eigen::Ref<const tesseract_common::TrajArray>& traj) override;

  void plotTrajectory(const tesseract_common::JointTrajectory& traj) override;

  void plotTrajectory(const tesseract_planning::Instruction& instruction) override;

  void plotToolPath(const tesseract_planning::Instruction& instruction) override;

  void plotContactResults(const std::vector<std::string>& link_names,
                          const tesseract_collision::ContactResultVector& dist_results,
                          const Eigen::Ref<const Eigen::VectorXd>& safety_distances) override;

  void plotArrow(const Eigen::Ref<const Eigen::Vector3d>& pt1,
                 const Eigen::Ref<const Eigen::Vector3d>& pt2,
                 const Eigen::Ref<const Eigen::Vector4d>& rgba,
                 double scale) override;

  void plotAxis(const Eigen::Isometry3d& axis, double scale) override;

  void clear() override;

  void waitForInput() override;

private:
  tesseract::Tesseract::ConstPtr thor_;               /**< Tesseract object */
  tesseract_environment::Environment::ConstPtr env_;  /**< Tesseract Environment Object */
  ignition::transport::Node node_;                    /**< Ignition communication node. */
  ignition::transport::Node::Publisher scene_pub_;    /**< Scene publisher */
  ignition::transport::Node::Publisher pose_pub_;     /**< Pose publisher */
  ignition::transport::Node::Publisher deletion_pub_; /**< Deletion publisher */
  EntityManager entity_manager_;

  /**
   * @brief Helper function for sending state to visualization tool
   * @param env_state Environment state
   */
  void sendEnvState(const tesseract_environment::EnvState::ConstPtr& env_state);
};

}  // namespace tesseract_visualization

#endif  // TESSERACT_VISUALIZATION_IGNITION_TESSERACT_IGNITION_VISUALIZATION_H
