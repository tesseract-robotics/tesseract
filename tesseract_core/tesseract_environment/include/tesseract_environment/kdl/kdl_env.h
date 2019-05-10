/**
 * @file kdl_env.h
 * @brief Tesseract environment kdl implementation.
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
#ifndef TESSERACT_ENVIRONMENT_KDL_ENV_H
#define TESSERACT_ENVIRONMENT_KDL_ENV_H

#include <tesseract_environment/core/macros.h>
TESSERACT_ENVIRONMENT_IGNORE_WARNINGS_PUSH
#include <kdl/tree.hpp>
#include <kdl_parser/kdl_parser.hpp>
#include <kdl/jntarray.hpp>

#include <tesseract_collision/core/discrete_contact_manager.h>
#include <tesseract_collision/core/continuous_contact_manager.h>
#include <tesseract_scene_graph/graph.h>
TESSERACT_ENVIRONMENT_IGNORE_WARNINGS_POP

#include <tesseract_environment/core/environment.h>

namespace tesseract_environment
{

class KDLEnv : public Environment
{
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  KDLEnv() : Environment() {}

  bool init(tesseract_scene_graph::SceneGraphPtr scene_graph) override;

  void setState(const std::unordered_map<std::string, double>& joints) override;
  void setState(const std::vector<std::string>& joint_names, const std::vector<double>& joint_values) override;
  void setState(const std::vector<std::string>& joint_names,
                const Eigen::Ref<const Eigen::VectorXd>& joint_values) override;

  EnvStatePtr getState(const std::unordered_map<std::string, double>& joints) const override;
  EnvStatePtr getState(const std::vector<std::string>& joint_names,
                       const std::vector<double>& joint_values) const override;
  EnvStatePtr getState(const std::vector<std::string>& joint_names,
                       const Eigen::Ref<const Eigen::VectorXd>& joint_values) const override;

  bool addLink(tesseract_scene_graph::Link link, tesseract_scene_graph::Joint joint) override;

  bool removeLink(const std::string& name) override;

  bool moveLink(tesseract_scene_graph::Joint joint) override;

  bool moveJoint(const std::string& joint_name, const std::string& parent_link) override;

private:
  std::shared_ptr<KDL::Tree> kdl_tree_;                        /**< KDL tree object */
  std::unordered_map<std::string, unsigned int> joint_to_qnr_; /**< Map between joint name and kdl q index */
  KDL::JntArray kdl_jnt_array_;                                /**< The kdl joint array */


  void calculateTransforms(TransformMap& transforms,
                           const KDL::JntArray& q_in,
                           const KDL::SegmentMap::const_iterator& it,
                           const Eigen::Isometry3d& parent_frame) const;

  void calculateTransformsHelper(TransformMap& transforms,
                                 const KDL::JntArray& q_in,
                                 const KDL::SegmentMap::const_iterator& it,
                                 const Eigen::Isometry3d& parent_frame) const;

  bool setJointValuesHelper(KDL::JntArray& q, const std::string& joint_name, const double& joint_value) const;

  void createKDETree();

};
typedef std::shared_ptr<KDLEnv> KDLEnvPtr;
typedef std::shared_ptr<const KDLEnv> KDLEnvConstPtr;
}

#endif  // TESSERACT_ENVIRONMENT_KDL_ENV_H
