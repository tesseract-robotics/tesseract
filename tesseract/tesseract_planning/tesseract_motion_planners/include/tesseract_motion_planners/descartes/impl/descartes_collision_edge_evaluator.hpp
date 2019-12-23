/**
 * @file descartes_collision_edge_evaluator.hpp
 * @brief Tesseract Descartes Collision Edge Evaluator Implementation
 *
 * @author Levi Armstrong
 * @date December 18, 2019
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
#ifndef TESSERACT_MOTION_PLANNERS_IMPL_DESCARTES_COLLISION_EDGE_EVALUATOR_HPP
#define TESSERACT_MOTION_PLANNERS_IMPL_DESCARTES_COLLISION_EDGE_EVALUATOR_HPP

#include <tesseract/tesseract.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <numeric>
#include <thread>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract_motion_planners/descartes/descartes_collision_edge_evaluator.h>
#include <tesseract_environment/core/utils.h>

namespace tesseract_motion_planners
{
template <typename FloatType>
DescartesCollisionEdgeEvaluator<FloatType>::DescartesCollisionEdgeEvaluator(
    const tesseract_environment::Environment::ConstPtr& collision_env,
    std::vector<std::string> active_links,
    std::vector<std::string> joint_names,
    double collision_safety_margin,
    double longest_valid_segment_length,
    bool allow_collision,
    bool debug)
  : state_solver_(collision_env->getStateSolver())
  , acm_(*(collision_env->getAllowedCollisionMatrix()))
  , active_link_names_(std::move(active_links))
  , joint_names_(std::move(joint_names))
  , discrete_contact_manager_(collision_env->getDiscreteContactManager())
  , continuous_contact_manager_(collision_env->getContinuousContactManager())
  , collision_safety_margin_(collision_safety_margin)
  , longest_valid_segment_length_(longest_valid_segment_length)
  , allow_collision_(allow_collision)
  , debug_(debug)
  , dof_(joint_names_.size())
{
  discrete_contact_manager_->setActiveCollisionObjects(active_link_names_);
  discrete_contact_manager_->setContactDistanceThreshold(collision_safety_margin_);
  discrete_contact_manager_->setIsContactAllowedFn(
      std::bind(&tesseract_motion_planners::DescartesCollisionEdgeEvaluator<FloatType>::isContactAllowed,
                this,
                std::placeholders::_1,
                std::placeholders::_2));

  continuous_contact_manager_->setActiveCollisionObjects(active_link_names_);
  continuous_contact_manager_->setContactDistanceThreshold(collision_safety_margin_);
  continuous_contact_manager_->setIsContactAllowedFn(
      std::bind(&tesseract_motion_planners::DescartesCollisionEdgeEvaluator<FloatType>::isContactAllowed,
                this,
                std::placeholders::_1,
                std::placeholders::_2));
}

template <typename FloatType>
bool DescartesCollisionEdgeEvaluator<FloatType>::evaluate(
    const descartes_light::Rung_<FloatType>& from,
    const descartes_light::Rung_<FloatType>& to,
    std::vector<typename descartes_light::LadderGraph<FloatType>::EdgeList>& edges)
{
  assert(from.data.size() % dof_ == 0);
  assert(to.data.size() % dof_ == 0);
  const auto n_start = from.data.size() / dof_;
  const auto n_end = to.data.size() / dof_;

  // Allocate
  edges.resize(n_start);

  for (std::size_t i = 0; i < n_start; ++i)
  {
    const auto* start_vertex = from.data.data() + dof_ * i;
    for (std::size_t j = 0; j < n_end; ++j)
    {
      const auto* end_vertex = to.data.data() + dof_ * j;

      // Consider the edge:
      considerEdge(edges[i], start_vertex, end_vertex, j, allow_collision_);
    }
  }

  for (const auto& rung : edges)
    if (!rung.empty())
      return true;

  return false;
}

template <typename FloatType>
void DescartesCollisionEdgeEvaluator<FloatType>::considerEdge(
    typename descartes_light::LadderGraph<FloatType>::EdgeList& out,
    const FloatType* start,
    const FloatType* end,
    std::size_t next_idx,
    bool find_best)
{
  // Happens in two phases:
  // 1. Compute the transform of all objects
  tesseract_common::TrajArray segment(2, dof_);
  for (size_t i = 0; i < dof_; ++i)
  {
    segment(0, static_cast<long>(i)) = start[i];
    segment(1, static_cast<long>(i)) = end[i];
  }

  std::vector<tesseract_collision::ContactResultMap> discrete_results;
  std::vector<tesseract_collision::ContactResultMap> continuous_results;
  bool discrete_in_contact = discreteCollisionCheck(discrete_results, segment, find_best);
  bool continuous_in_contact = continuousCollisionCheck(continuous_results, segment, find_best);

  if (!discrete_in_contact && !continuous_in_contact)
    out.emplace_back(0, next_idx);
  else if (!discrete_in_contact && continuous_in_contact && find_best)
    out.emplace_back(collision_safety_margin_ - continuous_results.begin()->begin()->second[0].distance, next_idx);
  else if (discrete_in_contact && !continuous_in_contact && find_best)
    out.emplace_back(collision_safety_margin_ - discrete_results.begin()->begin()->second[0].distance, next_idx);
  else if (discrete_in_contact && continuous_in_contact && find_best)
  {
    double d = collision_safety_margin_ - discrete_results.begin()->begin()->second[0].distance;
    double c = collision_safety_margin_ - continuous_results.begin()->begin()->second[0].distance;
    out.emplace_back(std::max(d, c), next_idx);
  }
}

template <typename FloatType>
bool DescartesCollisionEdgeEvaluator<FloatType>::isContactAllowed(const std::string& a, const std::string& b) const
{
  return acm_.isCollisionAllowed(a, b);
}

template <typename FloatType>
bool DescartesCollisionEdgeEvaluator<FloatType>::continuousCollisionCheck(
    std::vector<tesseract_collision::ContactResultMap>& results,
    const tesseract_common::TrajArray& segment,
    bool find_best)
{
  // It was time using chronos time elapsed and it was faster to cache the contact manager
  unsigned long int hash = std::hash<std::thread::id>{}(std::this_thread::get_id());
  tesseract_collision::ContinuousContactManager::Ptr cm;
  tesseract_environment::StateSolver::Ptr ss;
  mutex_.lock();
  auto it = continuous_contact_managers_.find(hash);
  if (it == continuous_contact_managers_.end())
  {
    cm = continuous_contact_manager_->clone();
    continuous_contact_managers_[hash] = cm;

    ss = state_solver_->clone();
    state_solver_managers_[hash] = ss;
  }
  else
  {
    cm = it->second;
    ss = state_solver_managers_[hash];
  }
  mutex_.unlock();

  return tesseract_environment::checkTrajectory(results,
                                                *cm,
                                                *ss,
                                                joint_names_,
                                                segment,
                                                longest_valid_segment_length_,
                                                (find_best) ? tesseract_collision::ContactTestType::CLOSEST :
                                                              tesseract_collision::ContactTestType::FIRST,
                                                debug_);
}

template <typename FloatType>
bool DescartesCollisionEdgeEvaluator<FloatType>::discreteCollisionCheck(
    std::vector<tesseract_collision::ContactResultMap>& results,
    const tesseract_common::TrajArray& segment,
    bool find_best)
{
  // It was time using chronos time elapsed and it was faster to cache the contact manager
  unsigned long int hash = std::hash<std::thread::id>{}(std::this_thread::get_id());
  tesseract_collision::DiscreteContactManager::Ptr cm;
  tesseract_environment::StateSolver::Ptr ss;
  mutex_.lock();
  auto it = discrete_contact_managers_.find(hash);
  if (it == discrete_contact_managers_.end())
  {
    cm = discrete_contact_manager_->clone();
    discrete_contact_managers_[hash] = cm;

    ss = state_solver_->clone();
    state_solver_managers_[hash] = ss;
  }
  else
  {
    cm = it->second;
    ss = state_solver_managers_[hash];
  }
  mutex_.unlock();

  return tesseract_environment::checkTrajectory(results,
                                                *cm,
                                                *ss,
                                                joint_names_,
                                                segment,
                                                longest_valid_segment_length_,
                                                (find_best) ? tesseract_collision::ContactTestType::CLOSEST :
                                                              tesseract_collision::ContactTestType::FIRST,
                                                debug_);
}

}  // namespace tesseract_motion_planners

#endif  // TESSERACT_MOTION_PLANNERS_IMPL_DESCARTES_COLLISION_EDGE_EVALUATOR_HPP
