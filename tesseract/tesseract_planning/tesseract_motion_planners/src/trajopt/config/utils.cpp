/**
 * @file utils.cpp
 * @brief Utilities for creating TrajOpt term information
 *
 * @author Michael Ripperger
 * @date September 16, 2019
 * @version TODO
 * @bug No known bugs
 *
 * @copyright Copyright (c) 2019, Southwest Research Institute
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
#include <tesseract_motion_planners/trajopt/config/utils.h>

namespace tesseract_motion_planners
{
trajopt::TermInfo::Ptr createJointWaypointTermInfo(const JointWaypoint::ConstPtr& waypoint,
                                                   int ind,
                                                   const std::vector<std::string>& joint_names,
                                                   double coeff,
                                                   const std::string& name)
{
  std::shared_ptr<trajopt::JointPosTermInfo> jv = std::make_shared<trajopt::JointPosTermInfo>();
  const Eigen::VectorXd& coeffs = waypoint->getCoefficients(joint_names);
  const Eigen::VectorXd position = waypoint->getPositions(joint_names);
  if (static_cast<std::size_t>(coeffs.size()) != joint_names.size())
    jv->coeffs = std::vector<double>(joint_names.size(), coeff);  // Default value
  else
    jv->coeffs = std::vector<double>(coeffs.data(), coeffs.data() + coeffs.rows() * coeffs.cols());
  jv->targets = std::vector<double>(position.data(), position.data() + position.rows() * position.cols());
  jv->first_step = static_cast<int>(ind);
  jv->last_step = static_cast<int>(ind);
  jv->name = name + "_" + std::to_string(ind);
  jv->term_type = waypoint->isCritical() ? trajopt::TT_CNT : trajopt::TT_COST;

  return jv;
}

trajopt::TermInfo::Ptr createJointTolerancedWaypointTermInfo(const JointTolerancedWaypoint::ConstPtr& waypoint,
                                                             int ind,
                                                             const std::vector<std::string>& joint_names,
                                                             double coeff,
                                                             const std::string& name)
{
  // For a toleranced waypoint we add an inequality term and a smaller equality term. This acts as a "leaky"
  // hinge to keep the problem numerically stable.
  // Equality cost with coeffs much smaller than inequality
  std::shared_ptr<trajopt::JointPosTermInfo> jv_equal = std::make_shared<trajopt::JointPosTermInfo>();
  const Eigen::VectorXd& coeffs = waypoint->getCoefficients(joint_names);
  const Eigen::VectorXd position = waypoint->getPositions(joint_names);
  if (static_cast<std::size_t>(coeffs.size()) != joint_names.size())
    jv_equal->coeffs = std::vector<double>(joint_names.size(), coeff);  // Default value
  else
    jv_equal->coeffs = std::vector<double>(coeffs.data(), coeffs.data() + coeffs.rows() * coeffs.cols());
  jv_equal->targets = std::vector<double>(position.data(), position.data() + position.rows() * position.cols());
  jv_equal->first_step = static_cast<int>(ind);
  jv_equal->last_step = static_cast<int>(ind);
  jv_equal->name = name + "_" + std::to_string(ind);
  // If this was a CNT, then the inequality tolernce would not do anything
  jv_equal->term_type = trajopt::TT_COST;

  return jv_equal;
}

trajopt::TermInfo::Ptr createCartesianWaypointTermInfo(const CartesianWaypoint::ConstPtr& waypoint,
                                                       int ind,
                                                       const std::string& link,
                                                       const Eigen::Isometry3d& tcp,
                                                       const std::string& name)
{
  std::shared_ptr<trajopt::CartPoseTermInfo> pose = std::make_shared<trajopt::CartPoseTermInfo>();
  pose->term_type = waypoint->isCritical() ? trajopt::TT_CNT : trajopt::TT_COST;
  pose->name = name + "_" + std::to_string(ind);

  pose->link = link;
  pose->tcp = tcp;

  pose->timestep = ind;
  pose->xyz = waypoint->getPosition();
  pose->wxyz = waypoint->getOrientation();
  pose->target = waypoint->getParentLinkName();

  const Eigen::VectorXd& coeffs = waypoint->getCoefficients();
  assert(coeffs.size() == 6);
  pose->pos_coeffs = coeffs.head<3>();
  pose->rot_coeffs = coeffs.tail<3>();

  return pose;
}

trajopt::TermInfo::Ptr createDynamicCartesianWaypointTermInfo(const CartesianWaypoint::ConstPtr& waypoint,
                                                              int ind,
                                                              const std::string& link,
                                                              const Eigen::Isometry3d& tcp,
                                                              const std::string& name)
{
  std::shared_ptr<trajopt::DynamicCartPoseTermInfo> pose = std::make_shared<trajopt::DynamicCartPoseTermInfo>();
  pose->term_type = waypoint->isCritical() ? trajopt::TT_CNT : trajopt::TT_COST;
  pose->name = name + "_" + std::to_string(ind);
  pose->timestep = ind;

  pose->link = link;
  pose->tcp = tcp;

  pose->target = waypoint->getParentLinkName();
  pose->target_tcp = waypoint->getTransform();

  const Eigen::VectorXd& coeffs = waypoint->getCoefficients();
  assert(coeffs.size() == 6);
  pose->pos_coeffs = coeffs.head<3>();
  pose->rot_coeffs = coeffs.tail<3>();

  return pose;
}

WaypointTermInfo createWaypointTermInfo(const Waypoint::ConstPtr& waypoint,
                                        int ind,
                                        const std::vector<std::string>& joint_names,
                                        const std::vector<std::string>& adjacency_map_links,
                                        const std::string& link,
                                        const Eigen::Isometry3d& tcp)
{
  WaypointTermInfo term_info;
  switch (waypoint->getType())
  {
    case WaypointType::JOINT_WAYPOINT:
    {
      JointWaypoint::ConstPtr joint_waypoint = std::static_pointer_cast<const JointWaypoint>(waypoint);
      trajopt::TermInfo::Ptr info = createJointWaypointTermInfo(joint_waypoint, ind, joint_names);
      joint_waypoint->isCritical() ? term_info.cnt.push_back(info) : term_info.cost.push_back(info);
      break;
    }
    case WaypointType::JOINT_TOLERANCED_WAYPOINT:
    {
      // Create a regular joint term info
      JointWaypoint::ConstPtr joint_waypoint = std::static_pointer_cast<const JointWaypoint>(waypoint);
      trajopt::TermInfo::Ptr info =
          createJointWaypointTermInfo(joint_waypoint, ind, joint_names, 1.0, "target_joint_toleranced_position");
      joint_waypoint->isCritical() ? term_info.cnt.push_back(info) : term_info.cost.push_back(info);

      // Create the tolerance joint term info
      JointTolerancedWaypoint::ConstPtr joint_tol_waypoint =
          std::static_pointer_cast<const JointTolerancedWaypoint>(waypoint);
      trajopt::TermInfo::Ptr info_tol =
          createJointTolerancedWaypointTermInfo(joint_tol_waypoint, ind, joint_names, 0.1);
      term_info.cost.push_back(info);

      break;
    }
    case WaypointType::CARTESIAN_WAYPOINT:
    {
      CartesianWaypoint::ConstPtr cart_waypoint = std::static_pointer_cast<const CartesianWaypoint>(waypoint);
      trajopt::TermInfo::Ptr info;

      /* Check if this cartesian waypoint is dynamic
       * (i.e. defined relative to a frame that will move with the kinematic chain)
       */
      auto it = std::find(adjacency_map_links.begin(), adjacency_map_links.end(), cart_waypoint->getParentLinkName());
      if (it != adjacency_map_links.end())
      {
        // This point is a dynamic cartesian waypoint
        info = createDynamicCartesianWaypointTermInfo(cart_waypoint, ind, link, tcp);
      }
      else
      {
        // This point is a static cartesian waypoint
        info = createCartesianWaypointTermInfo(cart_waypoint, ind, link, tcp);
      }

      // Add the cost/constraint based on the waypoint criticality
      cart_waypoint->isCritical() ? term_info.cnt.push_back(info) : term_info.cost.push_back(info);
      break;
    }
  }

  return term_info;
}

trajopt::TermInfo::Ptr createConfigurationTermInfo(const JointWaypoint::ConstPtr& configuration,
                                                   const std::vector<std::string>& joint_names,
                                                   int n_steps,
                                                   double coeff,
                                                   const std::string& name)
{
  assert(static_cast<std::size_t>(configuration->getPositions().size()) == joint_names.size());
  //  JointWaypoint::ConstPtr joint_waypoint = configuration;
  std::shared_ptr<trajopt::JointPosTermInfo> jp = std::make_shared<trajopt::JointPosTermInfo>();
  const Eigen::VectorXd& coeffs = configuration->getCoefficients();
  assert(std::equal(joint_names.begin(), joint_names.end(), configuration->getNames().begin()));
  if (static_cast<std::size_t>(coeffs.size()) != joint_names.size())
    jp->coeffs = std::vector<double>(joint_names.size(), coeff);  // Default value
  else
    jp->coeffs = std::vector<double>(coeffs.data(), coeffs.data() + coeffs.rows() * coeffs.cols());
  jp->targets = std::vector<double>(configuration->getPositions().data(),
                                    configuration->getPositions().data() + configuration->getPositions().size());
  jp->first_step = 0;
  jp->last_step = n_steps - 1;
  jp->name = name;
  jp->term_type = trajopt::TT_COST;

  return jp;
}

trajopt::TermInfo::Ptr createCollisionTermInfo(int n_steps,
                                               double collision_safety_margin,
                                               bool collision_continuous,
                                               double coeff,
                                               tesseract_collision::ContactTestType contact_test_type,
                                               double longest_valid_segment_length,
                                               const std::string& name)
{
  std::shared_ptr<trajopt::CollisionTermInfo> collision = std::make_shared<trajopt::CollisionTermInfo>();
  collision->name = name;
  collision->term_type = trajopt::TT_COST;
  collision->continuous = collision_continuous;
  collision->first_step = 0;
  collision->last_step = n_steps - 1;
  collision->contact_test_type = contact_test_type;
  collision->longest_valid_segment_length = longest_valid_segment_length;
  collision->info = trajopt::createSafetyMarginDataVector(n_steps, collision_safety_margin, coeff);
  return collision;
}

trajopt::TermInfo::Ptr createSmoothVelocityTermInfo(int n_steps, int n_joints, double coeff, const std::string& name)
{
  std::shared_ptr<trajopt::JointVelTermInfo> jv = std::make_shared<trajopt::JointVelTermInfo>();
  jv->coeffs = std::vector<double>(static_cast<std::size_t>(n_joints), coeff);
  jv->targets = std::vector<double>(static_cast<std::size_t>(n_joints), 0.0);
  jv->first_step = 0;
  jv->last_step = n_steps - 1;
  jv->name = name;
  jv->term_type = trajopt::TT_COST;
  return jv;
}

trajopt::TermInfo::Ptr createSmoothVelocityTermInfo(int n_steps,
                                                    const Eigen::Ref<const Eigen::VectorXd>& coeff,
                                                    const std::string& name)
{
  std::shared_ptr<trajopt::JointVelTermInfo> jv = std::make_shared<trajopt::JointVelTermInfo>();
  jv->coeffs = std::vector<double>(coeff.data(), coeff.data() + coeff.size());
  jv->targets = std::vector<double>(static_cast<std::size_t>(coeff.size()), 0.0);
  jv->first_step = 0;
  jv->last_step = n_steps - 1;
  jv->name = name;
  jv->term_type = trajopt::TT_COST;
  return jv;
}

trajopt::TermInfo::Ptr
createSmoothAccelerationTermInfo(int n_steps, int n_joints, double coeff, const std::string& name)
{
  std::shared_ptr<trajopt::JointAccTermInfo> ja = std::make_shared<trajopt::JointAccTermInfo>();
  ja->coeffs = std::vector<double>(static_cast<std::size_t>(n_joints), coeff);
  ja->targets = std::vector<double>(static_cast<std::size_t>(n_joints), 0.0);
  ja->first_step = 0;
  ja->last_step = n_steps - 1;
  ja->name = name;
  ja->term_type = trajopt::TT_COST;
  return ja;
}

trajopt::TermInfo::Ptr createSmoothAccelerationTermInfo(int n_steps,
                                                        const Eigen::Ref<const Eigen::VectorXd>& coeff,
                                                        const std::string& name)
{
  std::shared_ptr<trajopt::JointAccTermInfo> ja = std::make_shared<trajopt::JointAccTermInfo>();
  ja->coeffs = std::vector<double>(coeff.data(), coeff.data() + coeff.size());
  ja->targets = std::vector<double>(static_cast<std::size_t>(coeff.size()), 0.0);
  ja->first_step = 0;
  ja->last_step = n_steps - 1;
  ja->name = name;
  ja->term_type = trajopt::TT_COST;
  return ja;
}

trajopt::TermInfo::Ptr createSmoothJerkTermInfo(int n_steps, int n_joints, double coeff, const std::string& name)
{
  std::shared_ptr<trajopt::JointJerkTermInfo> jj = std::make_shared<trajopt::JointJerkTermInfo>();
  jj->coeffs = std::vector<double>(static_cast<std::size_t>(n_joints), coeff);
  jj->targets = std::vector<double>(static_cast<std::size_t>(n_joints), 0.0);
  jj->first_step = 0;
  jj->last_step = n_steps - 1;
  jj->name = name;
  jj->term_type = trajopt::TT_COST;
  return jj;
}

trajopt::TermInfo::Ptr createSmoothJerkTermInfo(int n_steps,
                                                const Eigen::Ref<const Eigen::VectorXd>& coeff,
                                                const std::string& name)
{
  std::shared_ptr<trajopt::JointJerkTermInfo> jj = std::make_shared<trajopt::JointJerkTermInfo>();
  jj->coeffs = std::vector<double>(coeff.data(), coeff.data() + coeff.size());
  jj->targets = std::vector<double>(static_cast<std::size_t>(coeff.size()), 0.0);
  jj->first_step = 0;
  jj->last_step = n_steps - 1;
  jj->name = name;
  jj->term_type = trajopt::TT_COST;
  return jj;
}

trajopt::TermInfo::Ptr createUserDefinedTermInfo(int n_steps,
                                                 sco::VectorOfVector::func error_function,
                                                 sco::MatrixOfVector::func jacobian_function,
                                                 const std::string& name)
{
  if (error_function == nullptr)
  {
    throw std::runtime_error("TrajOpt Planner Config constraint from error function recieved nullptr!");
  }

  auto ef = std::make_shared<trajopt::UserDefinedTermInfo>();
  ef->name = name;
  ef->term_type = trajopt::TT_COST;
  ef->first_step = 0;
  ef->last_step = n_steps - 1;
  ef->error_function = std::move(error_function);
  ef->jacobian_function = std::move(jacobian_function);

  return ef;
}

}  // namespace tesseract_motion_planners
