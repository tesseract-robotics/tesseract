/**
 * @file state_solver.h
 * @brief Tesseract Environment State Solver Interface.
 *
 * @author Levi Armstrong
 * @date Dec 18, 2017
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
#ifndef TESSERACT_ENVIRONMENT_STATE_SOLVER_H
#define TESSERACT_ENVIRONMENT_STATE_SOLVER_H

#include <tesseract_common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <vector>
#include <string>
#include <memory>
#include <unordered_map>
#include <Eigen/Geometry>
#include <Eigen/Core>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract_scene_graph/graph.h>
#include <tesseract_environment/core/types.h>
#include <tesseract_environment/core/commands.h>

namespace tesseract_environment
{
class StateSolver
{
public:
  using Ptr = std::shared_ptr<StateSolver>;
  using ConstPtr = std::shared_ptr<const StateSolver>;

  StateSolver() = default;
  virtual ~StateSolver() = default;
  StateSolver(const StateSolver&) = delete;
  StateSolver& operator=(const StateSolver&) = delete;
  StateSolver(StateSolver&&) = delete;
  StateSolver& operator=(StateSolver&&) = delete;

  virtual bool init(tesseract_scene_graph::SceneGraph::ConstPtr scene_graph) = 0;

  /**
   * @brief Set the current state of the solver
   *
   * After updating the current state these function must call currentStateChanged() which
   * will update the contact managers transforms
   *
   */
  virtual void setState(const std::unordered_map<std::string, double>& joints) = 0;
  virtual void setState(const std::vector<std::string>& joint_names, const std::vector<double>& joint_values) = 0;
  virtual void setState(const std::vector<std::string>& joint_names,
                        const Eigen::Ref<const Eigen::VectorXd>& joint_values) = 0;

  /**
   * @brief Get the state of the environment for a given set or subset of joint values.
   *
   * This does not change the internal state of the environment.
   *
   * @param joints A map of joint names to joint values to change.
   * @return A the state of the environment
   */
  virtual EnvState::Ptr getState(const std::unordered_map<std::string, double>& joints) const = 0;
  virtual EnvState::Ptr getState(const std::vector<std::string>& joint_names,
                                 const std::vector<double>& joint_values) const = 0;
  virtual EnvState::Ptr getState(const std::vector<std::string>& joint_names,
                                 const Eigen::Ref<const Eigen::VectorXd>& joint_values) const = 0;

  /**
   * @brief Get the current state of the environment
   * @return
   */
  virtual EnvState::ConstPtr getCurrentState() const = 0;

  /**
   * @brief This should clone the object so it may be used in a multi threaded application where each thread would
   * clone the solver.
   * @return A clone of the object.
   */
  virtual Ptr clone() const = 0;

protected:
  /**
   * @brief This is to only be used by the environment
   * @param commands
   */
  virtual void onEnvironmentChanged(const Commands& commands) = 0;

  friend class Environment;
};
}  // namespace tesseract_environment

#endif  // TESSERACT_ENVIRONMENT_STATE_SOLVER_H
