/**
 * @file ofkt_nodes.h
 * @brief A implementation of the Optimized Forward Kinematic Tree Nodes.
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
#ifndef TESSERACT_ENVIRONMENT_OFKT_NODES_H
#define TESSERACT_ENVIRONMENT_OFKT_NODES_H

#include <tesseract_environment/ofkt/ofkt_node.h>

namespace tesseract_environment
{
/*********************************************************************/
/*************************** BASE NODE *******************************/
/*********************************************************************/
class OFKTBaseNode : public OFKTNode
{
public:
  // LCOV_EXCL_START
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  // LCOV_EXCL_STOP

  OFKTBaseNode(tesseract_scene_graph::JointType type, OFKTNode* parent, std::string link_name);

  OFKTBaseNode(tesseract_scene_graph::JointType type,
               OFKTNode* parent,
               std::string link_name,
               std::string joint_name,
               Eigen::Isometry3d static_tf);

  tesseract_scene_graph::JointType getType() const override;

  void setParent(OFKTNode* parent) override;
  OFKTNode* getParent() override;
  const OFKTNode* getParent() const override;

  const std::string& getLinkName() const override;
  const std::string& getJointName() const override;

  void storeJointValue(double joint_value) override;

  double getJointValue() const override;

  bool hasJointValueChanged() const override;

  void setStaticTransformation(Eigen::Isometry3d static_tf) override;

  const Eigen::Isometry3d& getStaticTransformation() const override;
  const Eigen::Isometry3d& getLocalTransformation() const override;
  Eigen::Isometry3d computeLocalTransformation(double joint_value) const override;

  void computeAndStoreWorldTransformation() override;
  const Eigen::Isometry3d& getWorldTransformation() const override;
  bool updateWorldTransformationRequired() const override;

  void addChild(OFKTNode* node) override;
  void removeChild(const OFKTNode* node) override;
  std::vector<OFKTNode*>& getChildren() override;
  const std::vector<const OFKTNode*>& getChildren() const override;

protected:
  tesseract_scene_graph::JointType type_;
  OFKTNode* parent_{ nullptr };
  std::string link_name_;
  std::string joint_name_;
  Eigen::Isometry3d static_tf_{ Eigen::Isometry3d::Identity() };
  Eigen::Isometry3d joint_tf_{ Eigen::Isometry3d::Identity() };
  Eigen::Isometry3d local_tf_{ Eigen::Isometry3d::Identity() };
  Eigen::Isometry3d world_tf_{ Eigen::Isometry3d::Identity() };

  double joint_value_{ 0 };
  bool joint_value_changed_{ false };

  std::vector<OFKTNode*> children_;
  std::vector<const OFKTNode*> children_const_;

  bool update_world_required_{ true };

  friend class OFKTStateSolver;
};

/*********************************************************************/
/*************************** ROOT NODE *******************************/
/*********************************************************************/
class OFKTRootNode : public OFKTBaseNode
{
public:
  // LCOV_EXCL_START
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  // LCOV_EXCL_STOP

  /**
   * @brief This should only be used for the root node of the tree
   * @param link_name The link name associated with the node
   */
  OFKTRootNode(std::string link_name);

  void setParent(OFKTNode* parent) override;
  void storeJointValue(double joint_value) override;
  void setStaticTransformation(Eigen::Isometry3d static_tf) override;
  void computeAndStoreLocalTransformation() override;
  void computeAndStoreWorldTransformation() override;
  bool updateWorldTransformationRequired() const override;

  Eigen::Isometry3d computeLocalTransformation(double joint_value) const override;

private:
  friend class OFKTStateSolver;
};

/**********************************************************************/
/*************************** FIXED NODE *******************************/
/**********************************************************************/
class OFKTFixedNode : public OFKTBaseNode
{
public:
  // LCOV_EXCL_START
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  // LCOV_EXCL_STOP

  OFKTFixedNode(OFKTNode* parent, std::string link_name, std::string joint_name, Eigen::Isometry3d static_tf);

  void storeJointValue(double joint_value) override;
  double getJointValue() const override;
  void setStaticTransformation(Eigen::Isometry3d static_tf) override;
  void computeAndStoreLocalTransformation() override;
  Eigen::Isometry3d computeLocalTransformation(double joint_value) const override;

private:
  friend class OFKTStateSolver;
};

/*********************************************************************/
/************************* REVOLUTE NODE *****************************/
/*********************************************************************/
class OFKTRevoluteNode : public OFKTBaseNode
{
public:
  // LCOV_EXCL_START
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  // LCOV_EXCL_STOP

  OFKTRevoluteNode(OFKTNode* parent,
                   std::string link_name,
                   std::string joint_name,
                   Eigen::Isometry3d static_tf,
                   Eigen::Vector3d axis);

  void storeJointValue(double joint_value) override;
  void computeAndStoreLocalTransformation() override;
  Eigen::Isometry3d computeLocalTransformation(double joint_value) const override;
  const Eigen::Vector3d& getAxis() const;

private:
  Eigen::Vector3d axis_;

  void computeAndStoreLocalTransformationImpl();

  friend class OFKTStateSolver;
};

/*********************************************************************/
/************************ CONTINUOUS NODE ****************************/
/*********************************************************************/
class OFKTContinuousNode : public OFKTBaseNode
{
public:
  // LCOV_EXCL_START
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  // LCOV_EXCL_STOP

  OFKTContinuousNode(OFKTNode* parent,
                     std::string link_name,
                     std::string joint_name,
                     Eigen::Isometry3d static_tf,
                     Eigen::Vector3d axis);

  void computeAndStoreLocalTransformation() override;
  Eigen::Isometry3d computeLocalTransformation(double joint_value) const override;
  const Eigen::Vector3d& getAxis() const;

private:
  Eigen::Vector3d axis_;

  void computeAndStoreLocalTransformationImpl();

  friend class OFKTStateSolver;
};

/*********************************************************************/
/************************* PRISMATIC NODE ****************************/
/*********************************************************************/
class OFKTPrismaticNode : public OFKTBaseNode
{
public:
  // LCOV_EXCL_START
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  // LCOV_EXCL_STOP

  OFKTPrismaticNode(OFKTNode* parent,
                    std::string link_name,
                    std::string joint_name,
                    Eigen::Isometry3d static_tf,
                    Eigen::Vector3d axis);

  void computeAndStoreLocalTransformation() override;
  Eigen::Isometry3d computeLocalTransformation(double joint_value) const override;
  const Eigen::Vector3d& getAxis() const;

private:
  Eigen::Vector3d axis_;

  void computeAndStoreLocalTransformationImpl();

  friend class OFKTStateSolver;
};
}  // namespace tesseract_environment

#endif  // TESSERACT_ENVIRONMENT_OFKT_NODES_H
