/**
 * @file kdl_utils.cpp
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

#include <tesseract_kinematics/kdl/kdl_utils.h>

namespace tesseract_kinematics
{
void KDLToEigen(const KDL::Frame& frame, Eigen::Isometry3d& transform)
{
  transform.setIdentity();

  // translation
  for (int i = 0; i < 3; ++i)
    transform(i, 3) = frame.p[i];

  // rotation matrix
  for (int i = 0; i < 9; ++i)
    transform(i / 3, i % 3) = frame.M.data[i];
}

void EigenToKDL(const Eigen::Isometry3d& transform, KDL::Frame& frame)
{
  frame.Identity();

  for (int i = 0; i < 3; ++i)
    frame.p[i] = transform(i, 3);

  for (int i = 0; i < 9; ++i)
    frame.M.data[i] = transform(i / 3, i % 3);
}

void KDLToEigen(const KDL::Jacobian& jacobian, Eigen::Ref<Eigen::MatrixXd> matrix)
{
  assert(matrix.rows() == jacobian.rows());
  assert(matrix.cols() == jacobian.columns());

  for (unsigned i = 0; i < jacobian.rows(); ++i)
    for (unsigned j = 0; j < jacobian.columns(); ++j)
      matrix(i, j) = jacobian(i, j);
}

void KDLToEigen(const KDL::Jacobian& jacobian, const std::vector<int>& q_nrs, Eigen::Ref<Eigen::MatrixXd> matrix)
{
  assert(matrix.rows() == jacobian.rows());
  assert(static_cast<unsigned>(matrix.cols()) == q_nrs.size());

  for (int i = 0; i < static_cast<int>(jacobian.rows()); ++i)
    for (int j = 0; j < static_cast<int>(q_nrs.size()); ++j)
      matrix(i, j) = jacobian(static_cast<unsigned>(i), static_cast<unsigned>(q_nrs[static_cast<size_t>(j)]));
}

void EigenToKDL(const Eigen::Ref<const Eigen::VectorXd>& vec, KDL::JntArray& joints) { joints.data = vec; }

void KDLToEigen(const KDL::JntArray& joints, Eigen::Ref<Eigen::VectorXd> vec) { vec = joints.data; }

bool parseSceneGraph(KDLChainData& results,
                     const tesseract_scene_graph::SceneGraph& scene_graph,
                     const std::vector<std::pair<std::string, std::string>>& chains)
{
  try
  {
    tesseract_scene_graph::KDLTreeData data = tesseract_scene_graph::parseSceneGraph(scene_graph);
    results.kdl_tree = data.tree;
  }
  catch (...)
  {
    CONSOLE_BRIDGE_logError("Failed to parse KDL tree from Scene Graph");
    return false;
  }

  results.chains = chains;
  results.base_link_name = chains.front().first;
  for (const auto& chain : chains)
  {
    KDL::Chain sub_chain;
    if (!results.kdl_tree.getChain(chain.first, chain.second, sub_chain))
    {
      CONSOLE_BRIDGE_logError(
          "Failed to initialize KDL between links: '%s' and '%s'", chain.first.c_str(), chain.second.c_str());
      return false;
    }
    results.robot_chain.addChain(sub_chain);
  }
  results.tip_link_name = chains.back().second;

  results.joint_names.clear();
  results.joint_names.resize(results.robot_chain.getNrOfJoints());

  results.segment_index.clear();
  results.segment_index[results.base_link_name] = 0;
  results.segment_index[results.tip_link_name] = static_cast<int>(results.robot_chain.getNrOfSegments());

  for (unsigned i = 0, j = 0; i < results.robot_chain.getNrOfSegments(); ++i)
  {
    const KDL::Segment& seg = results.robot_chain.getSegment(i);
    const KDL::Joint& jnt = seg.getJoint();

    if (jnt.getType() == KDL::Joint::None)
      continue;

    // KDL segments does not contain the the base link in this list. When calling function that take segmentNr, like
    // JntToCart to get the base link transform you would pass an index of zero and for subsequent links it is
    // index + 1. This was determined through testing which is captured in this packages unit tests.
    results.segment_index[seg.getName()] = static_cast<int>(i + 1);

    results.joint_names[j] = jnt.getName();

    ++j;
  }

  return true;
}

bool parseSceneGraph(KDLChainData& results,
                     const tesseract_scene_graph::SceneGraph& scene_graph,
                     const std::string& base_name,
                     const std::string& tip_name)
{
  std::vector<std::pair<std::string, std::string>> chains;
  chains.emplace_back(base_name, tip_name);
  return parseSceneGraph(results, scene_graph, chains);
}
}  // namespace tesseract_kinematics
