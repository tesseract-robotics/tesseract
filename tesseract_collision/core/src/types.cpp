/**
 * @file types.cpp
 * @brief Tesseracts Collision Common Types
 *
 * @author Levi Armstrong
 * @date January 18, 2018
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

#include <tesseract_collision/core/types.h>

namespace tesseract_collision
{
void ContactResult::clear()
{
  distance = std::numeric_limits<double>::max();
  nearest_points[0].setZero();
  nearest_points[1].setZero();
  nearest_points_local[0].setZero();
  nearest_points_local[1].setZero();
  transform[0] = Eigen::Isometry3d::Identity();
  transform[1] = Eigen::Isometry3d::Identity();
  link_names[0] = "";
  link_names[1] = "";
  shape_id[0] = -1;
  shape_id[1] = -1;
  subshape_id[0] = -1;
  subshape_id[1] = -1;
  type_id[0] = 0;
  type_id[1] = 0;
  normal.setZero();
  cc_time[0] = -1;
  cc_time[1] = -1;
  cc_type[0] = ContinuousCollisionType::CCType_None;
  cc_type[1] = ContinuousCollisionType::CCType_None;
  cc_transform[0] = Eigen::Isometry3d::Identity();
  cc_transform[1] = Eigen::Isometry3d::Identity();
  single_contact_point = false;
}

ContactRequest::ContactRequest(ContactTestType type) : type(type) {}

std::size_t flattenMoveResults(ContactResultMap&& m, ContactResultVector& v)
{
  v.clear();
  v.reserve(m.size());
  for (const auto& mv : m)
    std::move(mv.second.begin(), mv.second.end(), std::back_inserter(v));

  return v.size();
}

std::size_t flattenCopyResults(const ContactResultMap& m, ContactResultVector& v)
{
  v.clear();
  v.reserve(m.size());
  for (const auto& mv : m)
    std::copy(mv.second.begin(), mv.second.end(), std::back_inserter(v));

  return v.size();
}

ContactTestData::ContactTestData(const std::vector<std::string>& active,
                                 CollisionMarginData collision_margin_data,
                                 IsContactAllowedFn fn,
                                 ContactRequest req,
                                 ContactResultMap& res)
  : active(&active)
  , collision_margin_data(std::move(collision_margin_data))
  , fn(std::move(fn))
  , req(std::move(req))
  , res(&res)
{
}

ContactManagerConfig::ContactManagerConfig(double default_margin)
  : margin_data_override_type(CollisionMarginOverrideType::OVERRIDE_DEFAULT_MARGIN), margin_data(default_margin)
{
}

CollisionCheckConfig::CollisionCheckConfig(double default_margin,
                                           ContactRequest request,
                                           CollisionEvaluatorType type,
                                           double longest_valid_segment_length)
  : contact_manager_config(default_margin)
  , contact_request(std::move(request))
  , type(type)
  , longest_valid_segment_length(longest_valid_segment_length)
{
}

}  // namespace tesseract_collision
