/**
 * @file kdl_utils.h
 * @brief Tesseract KDL utility functions.
 *
 * @author Levi Armstrong
 * @date May 27, 2018
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
#ifndef TESSERACT_KINEMATICS_KDL_UTILS_H
#define TESSERACT_KINEMATICS_KDL_UTILS_H

#include <tesseract_common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <kdl/frames.hpp>
#include <kdl/jntarray.hpp>
#include <Eigen/Eigen>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract_common/types.h>
#include <tesseract_common/utils.h>
#include <tesseract_scene_graph/graph.h>
#include <tesseract_scene_graph/kdl_parser.h>
#include <tesseract_kinematics/core/utils.h>
#include <tesseract_kinematics/core/types.h>

namespace tesseract_kinematics
{
/**
 * @brief Convert KDL::Frame to Eigen::Isometry3d
 * @param frame Input KDL Frame
 * @param transform Output Eigen transform (Isometry3d)
 */
void KDLToEigen(const KDL::Frame& frame, Eigen::Isometry3d& transform);

/**
 * @brief Convert Eigen::Isometry3d to KDL::Frame
 * @param transform Input Eigen transform (Isometry3d)
 * @param frame Output KDL Frame
 */
void EigenToKDL(const Eigen::Isometry3d& transform, KDL::Frame& frame);

/**
 * @brief Convert KDL::Jacobian to Eigen::Matrix
 * @param jacobian Input KDL Jacobian
 * @param matrix Output Eigen MatrixXd
 */
void KDLToEigen(const KDL::Jacobian& jacobian, Eigen::Ref<Eigen::MatrixXd> matrix);

/**
 * @brief Convert a subset of KDL::Jacobian to Eigen::Matrix
 * @param jacobian Input KDL Jacobian
 * @param q_nrs Input the columns to use
 * @param matrix Output Eigen MatrixXd
 */
void KDLToEigen(const KDL::Jacobian& jacobian, const std::vector<int>& q_nrs, Eigen::Ref<Eigen::MatrixXd> matrix);

/**
 * @brief Convert Eigen::Vector to KDL::JntArray
 * @param vec Input Eigen vector
 * @param joints Output KDL joint array
 */
void EigenToKDL(const Eigen::Ref<const Eigen::VectorXd>& vec, KDL::JntArray& joints);

/**
 * @brief Convert KDL::JntArray to Eigen::Vector
 * @param joints Input KDL joint array
 * @param vec Output Eigen vector
 */
void KDLToEigen(const KDL::JntArray& joints, Eigen::Ref<Eigen::VectorXd> vec);

/**
 * @brief The KDLChainData struct
 *
 * This contains common data extracted when parsing
 * a kdl chain from the scene graph
 */
struct KDLChainData
{
  KDL::Chain robot_chain;                   /**< @brief KDL Chain object */
  KDL::Tree kdl_tree;                       /**< @brief KDL tree object */
  std::vector<std::string> joint_names;     /**< @brief List of joint names */
  std::string base_link_name;               /**< @brief Link name of first link in the kinematic object */
  std::string tip_link_name;                /**< @brief Link name of last kink in the kinematic object */
  std::map<std::string, int> segment_index; /**< @brief A map from chain link name to kdl chain segment number */
  std::vector<std::pair<std::string, std::string>> chains; /**< The chains used to create the object */
};

/**
 * @brief Parse KDL chain data from the scene graph
 * @param results KDL Chain data
 * @param scene_graph The Scene Graph
 * @param chains A vector of link pairs that represent a chain which all get concatenated together. The firts link
 *        should be the base link and the second link should the tip link of the chain.
 * @return True if successful otherwise false
 */
bool parseSceneGraph(KDLChainData& results,
                     const tesseract_scene_graph::SceneGraph& scene_graph,
                     const std::vector<std::pair<std::string, std::string>>& chains);

/**
 * @brief Parse KDL chain data from the scene graph
 * @param results KDL Chain data
 * @param scene_graph The Scene Graph
 * @param base_name The base link name of chain
 * @param tip_name The tip link name of chain
 * @return True if successful otherwise false
 */
bool parseSceneGraph(KDLChainData& results,
                     const tesseract_scene_graph::SceneGraph& scene_graph,
                     const std::string& base_name,
                     const std::string& tip_name);
}  // namespace tesseract_kinematics
#endif  // TESSERACT_KINEMATICS_KDL_UTILS_H
