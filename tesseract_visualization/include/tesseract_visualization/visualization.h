/**
 * @file visualization.h
 * @brief Visualization Class.
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
#ifndef TESSERACT_VISUALIZATION_VISUALIZATION_H
#define TESSERACT_VISUALIZATION_VISUALIZATION_H

#include <tesseract_common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <any>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract_common/joint_state.h>
#include <tesseract_collision/core/types.h>
#include <tesseract_scene_graph/graph.h>
#include <tesseract_environment/environment.h>
#include <tesseract_visualization/markers/marker.h>

// clang-format off
#define TESSERACT_ADD_VISUALIZATION_PLUGIN(DERIVED_CLASS, ALIAS)                                                       \
  TESSERACT_ADD_PLUGIN_SECTIONED(DERIVED_CLASS, ALIAS, Plotter)
// clang-format on

namespace tesseract_visualization
{
/** @brief The Vizualization class */
class Visualization
{
public:
  using Ptr = std::shared_ptr<Visualization>;
  using ConstPtr = std::shared_ptr<const Visualization>;

  Visualization() = default;
  virtual ~Visualization() = default;
  Visualization(const Visualization&) = default;
  Visualization& operator=(const Visualization&) = default;
  Visualization(Visualization&&) = default;
  Visualization& operator=(Visualization&&) = default;

  /**
   * @brief Some plotters may require connecting to external software.
   * @return True if connected, otherwise false
   */
  virtual bool isConnected() const = 0;

  /**
   * @brief Wait for connection
   * @param seconds The number of seconds to wait before returning, if zero it waits indefinitely
   */
  virtual void waitForConnection(long seconds = 0) const = 0;

  /**
   * @brief Plot environment
   * @param env The environment.
   */
  virtual void plotEnvironment(const tesseract_environment::Environment& env, std::string ns = "") = 0;

  /**
   * @brief Plot state of the environment
   * @param state The state of the environment.
   */
  virtual void plotEnvironmentState(const tesseract_scene_graph::SceneState& state, std::string ns = "") = 0;

  /**
   * @brief Plot a JointTrajectory
   * @param state_solver The environment
   * @param trajectory JointTrajectory to be plotted
   */
  virtual void plotTrajectory(const tesseract_common::JointTrajectory& traj,
                              const tesseract_scene_graph::StateSolver& state_solver,
                              std::string ns = "") = 0;

  /**
   * @brief Plot marker
   * @param marker The marker to plot
   * @param ns The namespace to plot the object under
   */
  virtual void plotMarker(const Marker& marker, std::string ns = "") = 0;

  /**
   * @brief Plot a vector of markers under a given namespace
   * @param markers The markers to plot
   * @param ns The namespace to plot the objects under
   */
  virtual void plotMarkers(const std::vector<Marker::Ptr>& markers, std::string ns = "") = 0;

  /**
   * @brief This is called at the start of the plotting for each iteration
   *        to clear previous iteration graphics if necessary.
   */
  virtual void clear(std::string ns = "") = 0;

  /** @brief Pause code and wait for enter key in terminal*/
  virtual void waitForInput(std::string message = "Hit enter key to continue!") = 0;
};

}  // namespace tesseract_visualization

#endif  // TESSERACT_VISUALIZATION_VISUALIZATION_H
