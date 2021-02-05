/**
 * @file ofkt_node.h
 * @brief A implementation of the Optimized Forward Kinematic Tree Node.
 *
 * This is based on the paper "A Forward Kinematics Data Structure for Efficient Evolutionary Inverse Kinematics".
 *
 * Starke, S., Hendrich, N., & Zhang, J. (2018). A Forward Kinematics Data Structure for Efficient Evolutionary Inverse
 * Kinematics. In Computational Kinematics (pp. 560-568). Springer, Cham.
 *
 * @author Levi Armstrong
 * @date August 24, 2020
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
#ifndef TESSERACT_ENVIRONMENT_OFKT_NODE_H
#define TESSERACT_ENVIRONMENT_OFKT_NODE_H

#include <tesseract_common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <Eigen/Geometry>
#include <memory>
#include <string>
#include <vector>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract_scene_graph/joint.h>

namespace tesseract_environment
{
/**
 * @brief The OFKT node is contains multiple trasformation which are described below.
 *
 *   - Static Transformation: (S)
 *   - Joint Transformation: (J) this if a fuction of the joint value if not fixed
 *   - Local Transformation: L = S * J
 *   - World Transformation: W = W(parent) * L
 *
 */
class OFKTNode
{
public:
  // LCOV_EXCL_START
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  // LCOV_EXCL_STOP

  using UPtr = std::unique_ptr<OFKTNode>;
  using Ptr = std::shared_ptr<OFKTNode>;
  using ConstPtr = std::shared_ptr<const OFKTNode>;

  OFKTNode() = default;
  virtual ~OFKTNode() = default;
  OFKTNode(const OFKTNode&) = delete;
  OFKTNode& operator=(const OFKTNode&) = delete;
  OFKTNode(OFKTNode&&) = delete;
  OFKTNode& operator=(OFKTNode&&) = delete;

  /**
   * @brief Get the type of joint associated with the node
   * @return The Joint Type
   */
  virtual tesseract_scene_graph::JointType getType() const = 0;

  /**
   * @brief Set the parent node
   *
   *  This should indicate that updateWorldTransformationRequired() = true
   *
   * @param parent The parent node
   */
  virtual void setParent(OFKTNode* parent) = 0;

  /**
   * @brief Get the parent node
   * @return The parent node
   */
  virtual OFKTNode* getParent() = 0;

  /**
   * @brief Get the parent node (const)
   * @return The parent node
   */
  virtual const OFKTNode* getParent() const = 0;

  /**
   * @brief Get the link name associated with the node
   * @return The link name
   */
  virtual const std::string& getLinkName() const = 0;

  /**
   * @brief Get the joint name associated with the node
   * @return The link name
   */
  virtual const std::string& getJointName() const = 0;

  /**
   * @brief Set the nodes joint value if it has one.
   *
   * This should indicate hasJointValueChanged() = true
   *
   * @param joint_value The joint value
   */
  virtual void storeJointValue(double joint_value) = 0;

  /**
   * @brief Get the current joint value
   * @return The current joint value
   */
  virtual double getJointValue() const = 0;

  /**
   * @brief Indicates that the joint value has changed and that local and world transformation need to be recomputed.
   * @return If true, local and world transformation need to be recompute
   */
  virtual bool hasJointValueChanged() const = 0;

  /**
   * @brief Set the static transformation.
   *
   * This should recompute the local transformation and updateWorldTransformationRequired() = true.
   *
   * @param static_tf The new static transformation
   */
  virtual void setStaticTransformation(Eigen::Isometry3d static_tf) = 0;

  /**
   * @brief Get the nodes static transformation
   * @return The static transformation
   */
  virtual const Eigen::Isometry3d& getStaticTransformation() const = 0;

  /**
   * @brief Compute and save the local transformation 'L = S * J(Joint Value)'
   *
   * This should reset the flag such that hasJointValueChanged() returns false
   */
  virtual void computeAndStoreLocalTransformation() = 0;

  /**
   * @brief Get the local transformation: 'L = S * J'
   * @return The nodes local transformation
   */
  virtual const Eigen::Isometry3d& getLocalTransformation() const = 0;

  /**
   * @brief Compute the local tranformation but do not save.
   *
   * This provides a const method for computing the local transform.
   *
   * @param joint_value The joint value for calculating the local transformation
   * @return The local transformation for the provided joint value
   */
  virtual Eigen::Isometry3d computeLocalTransformation(double joint_value) const = 0;

  /**
   * @brief Compute and store the nodes world transformation
   *
   * This should reset the flag such that updateWorldTransformationRequired() returns false
   */
  virtual void computeAndStoreWorldTransformation() = 0;

  /**
   * @brief Get the nodes world transformation
   * @return The nodes world transformation
   */
  virtual const Eigen::Isometry3d& getWorldTransformation() const = 0;

  /**
   * @brief Indicates if an update of the world transformation is required.
   *
   * This should get set to true when either of the following happen.
   *     - If setParent() was called
   *     - If setStaticTransformation() was called
   *
   * Note: setJointValue() is not handle here because that is captured by hasJointValueChanged()
   * @return If true, the world transformation need to be recalculated.
   */
  virtual bool updateWorldTransformationRequired() const = 0;

  /**
   * @brief Add a child node
   * @param node The node which is a child of this node
   */
  virtual void addChild(OFKTNode* node) = 0;

  /**
   * @brief Remove a child node assiciated with this node
   * @param node The child node to be removed
   */
  virtual void removeChild(const OFKTNode* node) = 0;

  /**
   * @brief Get a vector of child nodes associated with this node
   * @return A vector of child nodes
   */
  virtual std::vector<OFKTNode*>& getChildren() = 0;

  /**
   * @brief Get a vector of child nodes associated with this node (Const)
   * @return A vector of child nodes
   */
  virtual const std::vector<const OFKTNode*>& getChildren() const = 0;
};
}  // namespace tesseract_environment

#endif  // TESSERACT_ENVIRONMENT_OFKT_NODE_H
