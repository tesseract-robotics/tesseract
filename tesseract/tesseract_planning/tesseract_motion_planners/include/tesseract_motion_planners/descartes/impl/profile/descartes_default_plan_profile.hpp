/**
 * @file descartes_default_plan_profile.hpp
 * @brief
 *
 * @author Levi Armstrong
 * @date June 18, 2020
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
#ifndef TESSERACT_MOTION_PLANNERS_DESCARTES_IMPL_DESCARTES_DEFAULT_PLAN_PROFILE_HPP
#define TESSERACT_MOTION_PLANNERS_DESCARTES_IMPL_DESCARTES_DEFAULT_PLAN_PROFILE_HPP

#include <tesseract_command_language/move_instruction.h>
#include <tesseract_command_language/plan_instruction.h>

#include <tesseract_motion_planners/descartes/profile/descartes_default_plan_profile.h>
#include <tesseract_motion_planners/descartes/descartes_robot_sampler.h>
#include <tesseract_motion_planners/descartes/descartes_collision.h>
#include <tesseract_motion_planners/descartes/descartes_collision_edge_evaluator.h>

#include <descartes_light/interface/collision_interface.h>
#include <descartes_samplers/evaluators/euclidean_distance_edge_evaluator.h>
#include <descartes_samplers/evaluators/compound_edge_evaluator.h>
#include <descartes_samplers/samplers/fixed_joint_pose_sampler.h>

#include <tesseract_kinematics/core/utils.h>

namespace tesseract_planning
{
template <typename FloatType>
DescartesDefaultPlanProfile<FloatType>::DescartesDefaultPlanProfile(const tinyxml2::XMLElement& xml_element)
{
  const tinyxml2::XMLElement* vertex_collisions_element = xml_element.FirstChildElement("VertexCollisions");
  const tinyxml2::XMLElement* edge_collisions_element = xml_element.FirstChildElement("EdgeCollisions");
  const tinyxml2::XMLElement* num_threads_element = xml_element.FirstChildElement("NumberThreads");
  const tinyxml2::XMLElement* allow_collisions_element = xml_element.FirstChildElement("AllowCollisions");
  const tinyxml2::XMLElement* debug_element = xml_element.FirstChildElement("Debug");

  tinyxml2::XMLError status;

  if (vertex_collisions_element)
  {
    const tinyxml2::XMLElement* enabled_element = vertex_collisions_element->FirstChildElement("Enabled");
    const tinyxml2::XMLElement* coll_safety_margin_element = vertex_collisions_element->FirstChildElement("CollisionSaf"
                                                                                                          "etyMargin");

    if (enabled_element)
    {
      status = enabled_element->QueryBoolText(&enable_collision);
      if (status != tinyxml2::XML_NO_ATTRIBUTE && status != tinyxml2::XML_SUCCESS)
        throw std::runtime_error("DescartesPlanProfile: VertexCollisions: Error parsing Enabled string");
    }

    if (coll_safety_margin_element)
    {
      std::string coll_safety_margin_string;
      status = tesseract_common::QueryStringText(coll_safety_margin_element, coll_safety_margin_string);
      if (status != tinyxml2::XML_NO_ATTRIBUTE && status != tinyxml2::XML_SUCCESS)
        throw std::runtime_error("DescartesPlanProfile: VertexCollisions:: Error parsing CollisionSafetyMargin string");

      if (!tesseract_common::isNumeric(coll_safety_margin_string))
        throw std::runtime_error("DescartesPlanProfile: VertexCollisions:: CollisionSafetyMargin is not a numeric "
                                 "values.");

      tesseract_common::toNumeric<double>(coll_safety_margin_string, collision_safety_margin);
    }
  }

  if (edge_collisions_element)
  {
    const tinyxml2::XMLElement* enabled_element = edge_collisions_element->FirstChildElement("Enabled");
    const tinyxml2::XMLElement* coll_safety_margin_element = edge_collisions_element->FirstChildElement("CollisionSafet"
                                                                                                        "yMargin");
    const tinyxml2::XMLElement* long_valid_seg_len_element = edge_collisions_element->FirstChildElement("LongestValidSe"
                                                                                                        "gmentLength");

    if (enabled_element)
    {
      status = enabled_element->QueryBoolText(&enable_edge_collision);
      if (status != tinyxml2::XML_NO_ATTRIBUTE && status != tinyxml2::XML_SUCCESS)
        throw std::runtime_error("DescartesPlanProfile: EdgeCollisions: Error parsing Enabled string");
    }

    if (coll_safety_margin_element)
    {
      std::string coll_safety_margin_string;
      status = tesseract_common::QueryStringText(coll_safety_margin_element, coll_safety_margin_string);
      if (status != tinyxml2::XML_NO_ATTRIBUTE && status != tinyxml2::XML_SUCCESS)
        throw std::runtime_error("DescartesPlanProfile: EdgeCollisions: Error parsing CollisionSafetyMargin string");

      if (!tesseract_common::isNumeric(coll_safety_margin_string))
        throw std::runtime_error("DescartesPlanProfile: EdgeCollisions: CollisionSafetyMargin is not a numeric "
                                 "values.");

      /** @todo Update XML */
      //      tesseract_common::toNumeric<double>(coll_safety_margin_string, edge_collision_saftey_margin);
    }

    if (long_valid_seg_len_element)
    {
      std::string long_valid_seg_len_string;
      status = tesseract_common::QueryStringText(long_valid_seg_len_element, long_valid_seg_len_string);
      if (status != tinyxml2::XML_NO_ATTRIBUTE && status != tinyxml2::XML_SUCCESS)
        throw std::runtime_error("DescartesPlanProfile: EdgeCollisions: Error parsing LongestValidSegmentLength "
                                 "string");

      if (!tesseract_common::isNumeric(long_valid_seg_len_string))
        throw std::runtime_error("DescartesPlanProfile: EdgeCollisions: LongestValidSegmentLength is not a numeric "
                                 "values.");

      /** @todo Update XML */
      //      tesseract_common::toNumeric<double>(long_valid_seg_len_string, edge_longest_valid_segment_length);
    }
  }

  if (num_threads_element)
  {
    std::string num_threads_string;
    status = tesseract_common::QueryStringText(num_threads_element, num_threads_string);
    if (status != tinyxml2::XML_NO_ATTRIBUTE && status != tinyxml2::XML_SUCCESS)
      throw std::runtime_error("DescartesPlanProfile: Error parsing NumberThreads string");

    if (!tesseract_common::isNumeric(num_threads_string))
      throw std::runtime_error("DescartesPlanProfile: NumberThreads is not a numeric values.");

    tesseract_common::toNumeric<int>(num_threads_string, num_threads);
  }

  if (allow_collisions_element)
  {
    status = allow_collisions_element->QueryBoolText(&allow_collision);
    if (status != tinyxml2::XML_NO_ATTRIBUTE && status != tinyxml2::XML_SUCCESS)
      throw std::runtime_error("DescartesPlanProfile: Error parsing AllowCollisions string");
  }

  if (debug_element)
  {
    status = debug_element->QueryBoolText(&debug);
    if (status != tinyxml2::XML_NO_ATTRIBUTE && status != tinyxml2::XML_SUCCESS)
      throw std::runtime_error("DescartesPlanProfile: Error parsing Debug string");
  }
}

template <typename FloatType>
void DescartesDefaultPlanProfile<FloatType>::apply(DescartesProblem<FloatType>& prob,
                                                   const Eigen::Isometry3d& cartesian_waypoint,
                                                   const Instruction& parent_instruction,
                                                   const ManipulatorInfo& manip_info,
                                                   const std::vector<std::string>& active_links,
                                                   int index) const
{
  assert(isPlanInstruction(parent_instruction));
  const auto* base_instruction = parent_instruction.cast_const<PlanInstruction>();
  assert(!(manip_info.empty() && base_instruction->getManipulatorInfo().empty()));
  ManipulatorInfo mi = manip_info.getCombined(base_instruction->getManipulatorInfo());
  Eigen::Isometry3d tcp = prob.tesseract->findTCP(mi);

  /* Check if this cartesian waypoint is dynamic
   * (i.e. defined relative to a frame that will move with the kinematic chain) */
  auto it = std::find(active_links.begin(), active_links.end(), prob.manip_inv_kin->getBaseLinkName());
  if (it != active_links.end() && prob.manip_inv_kin->getBaseLinkName() != mi.working_frame)
    throw std::runtime_error("DescartesDefaultPlanProfile: Assigned dynamic waypoint but parent instruction working is "
                             "not set to the base link of manipulator!");

  typename descartes_light::CollisionInterface<FloatType>::Ptr ci = nullptr;
  if (enable_collision)
    ci = std::make_shared<DescartesCollision<FloatType>>(prob.tesseract->getEnvironment(),
                                                         active_links,
                                                         prob.manip_inv_kin->getJointNames(),
                                                         collision_safety_margin,
                                                         debug);

  Eigen::Isometry3d manip_baselink_to_waypoint = Eigen::Isometry3d::Identity();
  if (it == active_links.end())
  {
    // Check if the waypoint is not relative to the manipulator base coordinate system
    Eigen::Isometry3d world_to_waypoint = cartesian_waypoint;
    if (!mi.working_frame.empty())
      world_to_waypoint = prob.env_state->link_transforms.at(mi.working_frame) * cartesian_waypoint;

    Eigen::Isometry3d world_to_base_link = prob.env_state->link_transforms.at(prob.manip_inv_kin->getBaseLinkName());
    manip_baselink_to_waypoint = world_to_base_link.inverse() * world_to_waypoint;
  }

  // Add vertex evaluator
  std::shared_ptr<descartes_light::PositionSampler<FloatType>> sampler;
  if (vertex_evaluator == nullptr)
  {
    auto ve =
        std::make_shared<DescartesJointLimitsVertexEvaluator<FloatType>>(prob.manip_inv_kin->getLimits().joint_limits);
    sampler = std::make_shared<DescartesRobotSampler<FloatType>>(
        manip_baselink_to_waypoint, target_pose_sampler, prob.manip_inv_kin, ci, tcp, allow_collision, ve);
  }
  else
  {
    sampler = std::make_shared<DescartesRobotSampler<FloatType>>(manip_baselink_to_waypoint,
                                                                 target_pose_sampler,
                                                                 prob.manip_inv_kin,
                                                                 ci,
                                                                 tcp,
                                                                 allow_collision,
                                                                 vertex_evaluator(prob));
  }
  prob.samplers.push_back(std::move(sampler));

  if (index != 0)
  {
    // Add edge Evaluator
    if (edge_evaluator == nullptr)
    {
      if (enable_edge_collision)
      {
        auto compound_evaluator =
            std::make_shared<descartes_light::CompoundEdgeEvaluator<FloatType>>(prob.manip_inv_kin->numJoints());
        compound_evaluator->evaluators.push_back(
            std::make_shared<descartes_light::EuclideanDistanceEdgeEvaluator<FloatType>>(
                prob.manip_inv_kin->numJoints()));
        compound_evaluator->evaluators.push_back(
            std::make_shared<DescartesCollisionEdgeEvaluator<FloatType>>(prob.tesseract->getEnvironment(),
                                                                         active_links,
                                                                         prob.manip_inv_kin->getJointNames(),
                                                                         edge_collision_check_config,
                                                                         allow_collision,
                                                                         debug));
        prob.edge_evaluators.push_back(compound_evaluator);
      }
      else
      {
        prob.edge_evaluators.push_back(std::make_shared<descartes_light::EuclideanDistanceEdgeEvaluator<FloatType>>(
            prob.manip_inv_kin->numJoints()));
      }
    }
    else
    {
      prob.edge_evaluators.push_back(edge_evaluator(prob));
    }
  }

  prob.num_threads = num_threads;
}

template <typename FloatType>
void DescartesDefaultPlanProfile<FloatType>::apply(DescartesProblem<FloatType>& prob,
                                                   const Eigen::VectorXd& joint_waypoint,
                                                   const Instruction& /*parent_instruction*/,
                                                   const ManipulatorInfo& /*manip_info*/,
                                                   const std::vector<std::string>& active_links,
                                                   int index) const
{
  std::vector<FloatType> joint_pose(joint_waypoint.data(),
                                    joint_waypoint.data() + joint_waypoint.rows() * joint_waypoint.cols());
  auto sampler = std::make_shared<descartes_light::FixedJointPoseSampler<FloatType>>(joint_pose);
  prob.samplers.push_back(std::move(sampler));

  if (index != 0)
  {
    // Add edge Evaluator
    if (edge_evaluator == nullptr)
    {
      if (enable_edge_collision)
      {
        auto compound_evaluator =
            std::make_shared<descartes_light::CompoundEdgeEvaluator<FloatType>>(prob.manip_inv_kin->numJoints());
        compound_evaluator->evaluators.push_back(
            std::make_shared<descartes_light::EuclideanDistanceEdgeEvaluator<FloatType>>(
                prob.manip_inv_kin->numJoints()));
        compound_evaluator->evaluators.push_back(
            std::make_shared<DescartesCollisionEdgeEvaluator<FloatType>>(prob.tesseract->getEnvironment(),
                                                                         active_links,
                                                                         prob.manip_inv_kin->getJointNames(),
                                                                         edge_collision_check_config,
                                                                         allow_collision,
                                                                         debug));
        prob.edge_evaluators.push_back(compound_evaluator);
      }
      else
      {
        prob.edge_evaluators.push_back(std::make_shared<descartes_light::EuclideanDistanceEdgeEvaluator<FloatType>>(
            prob.manip_inv_kin->numJoints()));
      }
    }
    else
    {
      prob.edge_evaluators.push_back(edge_evaluator(prob));
    }
  }

  prob.num_threads = num_threads;
}

template <typename FloatType>
tinyxml2::XMLElement* DescartesDefaultPlanProfile<FloatType>::toXML(tinyxml2::XMLDocument& doc) const
{
  tinyxml2::XMLElement* xml_planner = doc.NewElement("Planner");
  xml_planner->SetAttribute("type", std::to_string(3).c_str());

  tinyxml2::XMLElement* xml_descartes = doc.NewElement("DescartesPlanProfile");

  tinyxml2::XMLElement* vertex_collisions = doc.NewElement("VertexCollisions");
  tinyxml2::XMLElement* vertex_collisions_enabled = doc.NewElement("Enabled");
  vertex_collisions_enabled->SetText(enable_collision);
  vertex_collisions->InsertEndChild(vertex_collisions_enabled);

  tinyxml2::XMLElement* vertex_collisions_safety_margin = doc.NewElement("CollisionSafetyMargin");
  vertex_collisions_safety_margin->SetText(collision_safety_margin);
  vertex_collisions->InsertEndChild(vertex_collisions_safety_margin);

  xml_descartes->InsertEndChild(vertex_collisions);

  tinyxml2::XMLElement* edge_collisions = doc.NewElement("EdgeCollisions");
  tinyxml2::XMLElement* edge_collisions_enabled = doc.NewElement("Enabled");
  edge_collisions_enabled->SetText(enable_edge_collision);
  edge_collisions->InsertEndChild(edge_collisions_enabled);

  /** @todo Update XML */
  //  tinyxml2::XMLElement* edge_collisions_safety_margin = doc.NewElement("CollisionSafetyMargin");
  //  edge_collisions_safety_margin->SetText(edge_collision_saftey_margin);
  //  edge_collisions->InsertEndChild(edge_collisions_safety_margin);

  //  tinyxml2::XMLElement* edge_collisions_long_valid_seg_len = doc.NewElement("LongestValidSegmentLength");
  //  edge_collisions_long_valid_seg_len->SetText(edge_longest_valid_segment_length);
  //  edge_collisions->InsertEndChild(edge_collisions_long_valid_seg_len);

  xml_descartes->InsertEndChild(edge_collisions);

  tinyxml2::XMLElement* number_threads = doc.NewElement("NumberThreads");
  number_threads->SetText(num_threads);
  xml_descartes->InsertEndChild(number_threads);

  tinyxml2::XMLElement* allow_collision_element = doc.NewElement("AllowCollisions");
  allow_collision_element->SetText(allow_collision);
  xml_descartes->InsertEndChild(allow_collision_element);

  tinyxml2::XMLElement* debug_element = doc.NewElement("Debug");
  debug_element->SetText(debug);
  xml_descartes->InsertEndChild(debug_element);

  xml_planner->InsertEndChild(xml_descartes);

  // TODO: Add Edge Evaluator and IsValidFn?

  return xml_planner;
}

}  // namespace tesseract_planning

#endif  // TESSERACT_MOTION_PLANNERS_DESCARTES_IMPL_DESCARTES_DEFAULT_PLAN_PROFILE_HPP
