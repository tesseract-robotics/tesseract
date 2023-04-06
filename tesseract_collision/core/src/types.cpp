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

ContactResult& ContactResultMap::addContactResult(const KeyType& key, ContactResult result)
{
  ++count_;
  auto& cv = data_[key];
  return cv.emplace_back(std::move(result));
}

ContactResult& ContactResultMap::addContactResult(const KeyType& key, const MappedType& results)
{
  assert(!results.empty());
  count_ += static_cast<long>(results.size());
  auto& cv = data_[key];
  cv.reserve(cv.size() + results.size());
  cv.insert(cv.end(), results.begin(), results.end());
  return cv.back();
}

ContactResult& ContactResultMap::setContactResult(const KeyType& key, ContactResult result)
{
  auto& cv = data_[key];
  count_ += (1 - static_cast<long>(cv.size()));
  assert(count_ >= 0);
  cv.clear();

  return cv.emplace_back(std::move(result));
}

ContactResult& ContactResultMap::setContactResult(const KeyType& key, const MappedType& results)
{
  assert(!results.empty());
  auto& cv = data_[key];
  count_ += (static_cast<long>(results.size()) - static_cast<long>(cv.size()));
  assert(count_ >= 0);
  cv.clear();
  cv.reserve(results.size());
  cv.insert(cv.end(), results.begin(), results.end());
  return cv.back();
}

void ContactResultMap::addInterpolatedCollisionResults(ContactResultMap& sub_segment_results,
                                                       long sub_segment_index,
                                                       long sub_segment_last_index,
                                                       const std::vector<std::string>& active_link_names,
                                                       double segment_dt,
                                                       bool discrete,
                                                       const tesseract_collision::ContactResultMap::FilterFn& filter)
{
  for (auto& pair : sub_segment_results.data_)
  {
    // Update cc_time and cc_type
    for (auto& r : pair.second)
    {
      // Iterate over the two time values in r.cc_time
      for (size_t j = 0; j < 2; ++j)
      {
        if (std::find(active_link_names.begin(), active_link_names.end(), r.link_names[j]) != active_link_names.end())
        {
          r.cc_time[j] = (r.cc_time[j] < 0) ?
                             (static_cast<double>(sub_segment_index) * segment_dt) :
                             (static_cast<double>(sub_segment_index) * segment_dt) + (r.cc_time[j] * segment_dt);
          assert(r.cc_time[j] >= 0.0 && r.cc_time[j] <= 1.0);
          if (sub_segment_index == 0 &&
              (r.cc_type[j] == tesseract_collision::ContinuousCollisionType::CCType_Time0 || discrete))
            r.cc_type[j] = tesseract_collision::ContinuousCollisionType::CCType_Time0;
          else if (sub_segment_index == sub_segment_last_index &&
                   (r.cc_type[j] == tesseract_collision::ContinuousCollisionType::CCType_Time1 || discrete))
            r.cc_type[j] = tesseract_collision::ContinuousCollisionType::CCType_Time1;
          else
            r.cc_type[j] = tesseract_collision::ContinuousCollisionType::CCType_Between;

          // If discrete set cc_transform for discrete continuous
          if (discrete)
            r.cc_transform = r.transform;
        }
      }
    }

    // Filter results
    if (filter != nullptr)
      filter(pair);

    if (!pair.second.empty())
    {
      // Add results to the full segment results
      count_ += static_cast<long>(pair.second.size());
      auto it = data_.find(pair.first);
      if (it == data_.end())
      {
        data_.insert(pair);
      }
      else
      {
        assert(it != data_.end());
        // Note: Must include all contacts throughout the trajectory so the optimizer has all the information
        //      to understand how to adjust the start and end state to move it out of collision. Originally tried
        //      keeping the worst case only but ran into edge cases where this does not work in the units tests.

        it->second.reserve(it->second.size() + pair.second.size());
        it->second.insert(it->second.end(), pair.second.begin(), pair.second.end());
      }
    }
  }
}

long ContactResultMap::count() const { return count_; }

std::size_t ContactResultMap::size() const
{
  if (count_ == 0)
    return 0;

  std::size_t cnt{ 0 };
  for (const auto& pair : data_)
  {
    if (!pair.second.empty())
      ++cnt;
  }

  return cnt;
}

bool ContactResultMap::empty() const { return (count_ == 0); }

void ContactResultMap::clear()
{
  if (count_ == 0)
    return;

  // Only clear the vectors so the capacity stays the same
  for (auto& cv : data_)
    cv.second.clear();

  count_ = 0;
}

void ContactResultMap::shrinkToFit()
{
  // Erase members that satisfy needs_removing(itr)
  for (auto it = data_.cbegin(); it != data_.cend();)
    it = it->second.empty() ? data_.erase(it) : std::next(it);
}

void ContactResultMap::release()
{
  data_.clear();
  count_ = 0;
}

const ContactResultMap::ContainerType& ContactResultMap::getContainer() const { return data_; }

ContactResultMap::ConstIteratorType ContactResultMap::begin() const { return data_.begin(); }

ContactResultMap::ConstIteratorType ContactResultMap::end() const { return data_.end(); }

ContactResultMap::ConstIteratorType ContactResultMap::cbegin() const { return data_.cbegin(); }

ContactResultMap::ConstIteratorType ContactResultMap::cend() const { return data_.cend(); }

const ContactResultVector& ContactResultMap::at(const KeyType& key) const { return data_.at(key); }

ContactResultMap::ConstIteratorType ContactResultMap::find(const KeyType& key) const { return data_.find(key); }

void ContactResultMap::flattenMoveResults(ContactResultVector& v)
{
  v.clear();
  v.reserve(static_cast<std::size_t>(count_));
  for (auto& mv : data_)
  {
    std::move(mv.second.begin(), mv.second.end(), std::back_inserter(v));
    mv.second.clear();
  }
  count_ = 0;
}

void ContactResultMap::flattenCopyResults(ContactResultVector& v) const
{
  v.clear();
  v.reserve(static_cast<std::size_t>(count_));
  for (const auto& mv : data_)
    std::copy(mv.second.begin(), mv.second.end(), std::back_inserter(v));
}

void ContactResultMap::flattenWrapperResults(std::vector<std::reference_wrapper<ContactResult>>& v)
{
  v.clear();
  v.reserve(static_cast<std::size_t>(count_));
  for (auto& mv : data_)
    v.insert(v.end(), mv.second.begin(), mv.second.end());
}

void ContactResultMap::flattenWrapperResults(std::vector<std::reference_wrapper<const ContactResult>>& v) const
{
  v.clear();
  v.reserve(static_cast<std::size_t>(count_));
  for (const auto& mv : data_)
    v.insert(v.end(), mv.second.begin(), mv.second.end());
}

void ContactResultMap::filter(const FilterFn& filter)
{
  std::size_t removed_cnt{ 0 };
  for (auto& pair : data_)
  {
    std::size_t current_cnt = pair.second.size();
    filter(pair);
    removed_cnt += (current_cnt - pair.second.size());
  }
  count_ -= static_cast<long>(removed_cnt);
  assert(count_ >= 0);
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
                                           double longest_valid_segment_length,
                                           CollisionCheckProgramType check_program_mode)
  : contact_manager_config(default_margin)
  , contact_request(std::move(request))
  , type(type)
  , longest_valid_segment_length(longest_valid_segment_length)
  , check_program_mode(check_program_mode)
{
}

}  // namespace tesseract_collision
