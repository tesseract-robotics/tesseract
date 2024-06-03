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
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <iomanip>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

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
  assert(tesseract_common::makeOrderedLinkPair(key.first, key.second) == key);
  ++count_;
  auto& cv = data_[key];
  return cv.emplace_back(std::move(result));
}

ContactResult& ContactResultMap::addContactResult(const KeyType& key, const MappedType& results)
{
  assert(!results.empty());
  assert(tesseract_common::makeOrderedLinkPair(key.first, key.second) == key);
  count_ += static_cast<long>(results.size());
  auto& cv = data_[key];
  cv.reserve(cv.size() + results.size());
  cv.insert(cv.end(), results.begin(), results.end());
  return cv.back();
}

ContactResult& ContactResultMap::setContactResult(const KeyType& key, ContactResult result)
{
  assert(tesseract_common::makeOrderedLinkPair(key.first, key.second) == key);
  auto& cv = data_[key];
  count_ += (1 - static_cast<long>(cv.size()));
  assert(count_ >= 0);
  cv.clear();

  return cv.emplace_back(std::move(result));
}

ContactResult& ContactResultMap::setContactResult(const KeyType& key, const MappedType& results)
{
  assert(tesseract_common::makeOrderedLinkPair(key.first, key.second) == key);
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
    assert(tesseract_common::makeOrderedLinkPair(pair.first.first, pair.first.second) == pair.first);
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

ContactTrajectorySubstepResults::ContactTrajectorySubstepResults(int substep_number,
                                                                 const Eigen::VectorXd& start_state,  // NOLINT
                                                                 const Eigen::VectorXd& end_state)    // NOLINT
  : substep(substep_number), state0(start_state), state1(end_state)
{
}

ContactTrajectorySubstepResults::ContactTrajectorySubstepResults(int substep_number, const Eigen::VectorXd& state)
  : substep(substep_number), state0(state), state1(state)
{
}

int ContactTrajectorySubstepResults::numContacts() const { return static_cast<int>(contacts.size()); }

tesseract_collision::ContactResultVector ContactTrajectorySubstepResults::worstCollision() const
{
  tesseract_collision::ContactResultVector worst_collision;
  double worst_distance = std::numeric_limits<double>::max();
  for (const auto& collision : contacts)
  {
    if (collision.second.front().distance < worst_distance)
    {
      worst_distance = collision.second.front().distance;
      worst_collision = collision.second;
    }
  }
  return worst_collision;
}

ContactTrajectoryStepResults::ContactTrajectoryStepResults(int step_number,
                                                           const Eigen::VectorXd& start_state,  // NOLINT
                                                           const Eigen::VectorXd& end_state,    // NOLINT
                                                           int num_substeps)
  : step(step_number), state0(start_state), state1(end_state), total_substeps(num_substeps)
{
  substeps.resize(static_cast<std::size_t>(num_substeps));
}

ContactTrajectoryStepResults::ContactTrajectoryStepResults(int step_number, const Eigen::VectorXd& state)
  : step(step_number), state0(state), state1(state), total_substeps(2)
{
  substeps.resize(static_cast<std::size_t>(2));
}

void ContactTrajectoryStepResults::resize(int num_substeps)
{
  total_substeps = num_substeps;
  substeps.resize(static_cast<std::size_t>(num_substeps));
}

int ContactTrajectoryStepResults::numSubsteps() const { return static_cast<int>(substeps.size()); }

int ContactTrajectoryStepResults::numContacts() const
{
  int num_contacts = 0;
  for (const auto& substep : substeps)
    num_contacts += substep.numContacts();
  return num_contacts;
}

ContactTrajectorySubstepResults ContactTrajectoryStepResults::worstSubstep() const
{
  ContactTrajectorySubstepResults worst_substep;
  double worst_distance = std::numeric_limits<double>::max();
  for (const auto& substep : substeps)
  {
    tesseract_collision::ContactResultVector substep_worst_collision = substep.worstCollision();
    if (substep_worst_collision.front().distance < worst_distance)
    {
      worst_distance = substep_worst_collision.front().distance;
      worst_substep = substep;
    }
  }
  return worst_substep;
}

tesseract_collision::ContactResultVector ContactTrajectoryStepResults::worstCollision() const
{
  tesseract_collision::ContactResultVector worst_collision = worstSubstep().worstCollision();
  return worst_collision;
}

ContactTrajectorySubstepResults ContactTrajectoryStepResults::mostCollisionsSubstep() const
{
  int most_contacts = 0;
  ContactTrajectorySubstepResults most_collisions_substep;
  for (const auto& substep : substeps)
  {
    if (substep.numContacts() > most_contacts)
    {
      most_contacts = substep.numContacts();
      most_collisions_substep = substep;
    }
  }
  return most_collisions_substep;
}

ContactTrajectoryResults::ContactTrajectoryResults(std::vector<std::string> j_names) : joint_names(std::move(j_names))
{
}

ContactTrajectoryResults::ContactTrajectoryResults(std::vector<std::string> j_names, int num_steps)
  : joint_names(std::move(j_names)), total_steps(num_steps)
{
  steps.resize(static_cast<std::size_t>(num_steps));
}

void ContactTrajectoryResults::resize(int num_steps)
{
  total_steps = num_steps;
  steps.resize(static_cast<std::size_t>(num_steps));
}

int ContactTrajectoryResults::numSteps() const { return static_cast<int>(steps.size()); }

int ContactTrajectoryResults::numContacts() const
{
  int num_contacts = 0;
  for (const auto& step : steps)
    num_contacts += step.numContacts();
  return num_contacts;
}

ContactTrajectoryStepResults ContactTrajectoryResults::worstStep() const
{
  ContactTrajectoryStepResults worst_step;
  double worst_distance = std::numeric_limits<double>::max();
  for (const auto& step : steps)
  {
    tesseract_collision::ContactResultVector step_worst_collision = step.worstCollision();
    if (step_worst_collision.front().distance < worst_distance)
    {
      worst_distance = step_worst_collision.front().distance;
      worst_step = step;
    }
  }
  return worst_step;
}

tesseract_collision::ContactResultVector ContactTrajectoryResults::worstCollision() const
{
  tesseract_collision::ContactResultVector worst_collision = worstStep().worstCollision();
  return worst_collision;
}

ContactTrajectoryStepResults ContactTrajectoryResults::mostCollisionsStep() const
{
  int most_contacts = 0;
  ContactTrajectoryStepResults most_collisions_step;
  for (const auto& step : steps)
  {
    if (step.numContacts() > most_contacts)
    {
      most_contacts = step.numContacts();
      most_collisions_step = step;
    }
  }
  return most_collisions_step;
}
std::stringstream ContactTrajectoryResults::trajectoryCollisionResultsTable() const
{
  // Possible multiple contacts for every substep
  // For every contact need to display contact distance, link1, link2
  // For every substep with a contact need to display substep #/total, all contacts
  // For every step need to display joint names, state0, state1, all substeps
  // No seperation between collision lines
  // Dashed  (---) line seperating substeps
  // Star (***) line seperating steps
  std::stringstream ss;

  if (numContacts() == 0)
  {
    ss << "No contacts detected" << std::endl;
    return ss;
  }

  int step_details_width = 0;
  int substep_details_width = 0;

  // First need to determine the width of every column, should be a space on either side of each
  // Step is displayed as (step)/(total number of steps), example: 2/23
  std::string step_title = "STEP";
  int longest_steps_width = 2 + static_cast<int>(step_title.size());
  int number_steps_digits = static_cast<int>(std::log10(steps.back().step)) + 1;
  // *2 for either side, plus 1 for '/', plus 2 for spaces
  int width_steps_display = number_steps_digits * 2 + 3;
  if (width_steps_display > longest_steps_width)
    longest_steps_width = width_steps_display;

  step_details_width += longest_steps_width;

  // Joint Names can vary widely
  std::string joint_name_title = "JOINT NAMES";
  int longest_joint_name_width = static_cast<int>(joint_name_title.size()) + 2;
  for (const auto& name : joint_names)
  {
    if (static_cast<int>(name.size()) + 2 > longest_joint_name_width)
      longest_joint_name_width = static_cast<int>(name.size()) + 2;
  }

  step_details_width += longest_joint_name_width;

  // State0 and State1 we will truncate all values to be to 4 decimals of precision,
  // important to add 1 to the length of negative values to account for the sign
  std::string state0_title = "STATE0";
  std::string state1_title = "STATE1";
  int longest_state0_width = 9;  // Default negative sign, number, decimal point, four places, plus space either side
  int longest_state1_width = 9;
  for (const auto& step : steps)
  {
    for (int i = 0; i < static_cast<int>(step.state0.size()); i++)
    {
      double state0_value = step.state0(i);
      if (state0_value < 0)
      {
        state0_value *= -1;
      }
      int state0_number_digits_left_decimal = static_cast<int>(std::log10(state0_value)) + 1;
      if (state0_number_digits_left_decimal + 7 > longest_state0_width)
        longest_state0_width =
            state0_number_digits_left_decimal + 7;  // + 4 after decimal + 2 for spaces either side + 1 for decimal

      double state1_value = step.state1(i);
      if (state1_value < 0)
      {
        state1_value *= -1;
      }
      int state1_number_digits_left_decimal = static_cast<int>(std::log10(state1_value)) + 1;
      if (state1_number_digits_left_decimal + 7 > longest_state1_width)
        longest_state1_width =
            state1_number_digits_left_decimal + 7;  // + 4 after decimal + 2 for spaces either side + 1 for decimal
    }
  }

  step_details_width += longest_state0_width;
  step_details_width += longest_state1_width;

  // Substep will almost certainly be the width of substep, but still check
  std::string substep_title = "SUBSTEP";
  int longest_substep_width = 2 + static_cast<int>(substep_title.size());
  for (const auto& step : steps)
  {
    // Check to make sure there are value, could be empty if checking for first collision
    if (step.numSubsteps() == 0)
      continue;

    // Substep is displayed as (substep)/(total number of substeps), example: 5/7
    // so length will be 2*(max substep width) + 1
    int number_digits = static_cast<int>(std::log10(step.substeps.size())) + 1;
    int width = 2 * number_digits + 3;
    if (width > longest_substep_width)
      longest_substep_width = width;
  }

  substep_details_width += longest_substep_width;

  // Link1 and Link2 will each be the width of the widest link name in that calumn
  std::string link1_title = "LINK1";
  std::string link2_title = "LINK2";
  int longest_link1_width = static_cast<int>(link1_title.size()) + 2;
  int longest_link2_width = static_cast<int>(link2_title.size()) + 2;
  for (const auto& step : steps)
  {
    for (const auto& substep : step.substeps)
    {
      if (!substep.contacts.empty())
      {
        for (const auto& collision : substep.contacts)
        {
          if (collision.second.empty())
            continue;
          std::string link1_name = collision.second.front().link_names[0];
          if (static_cast<int>(link1_name.size()) + 2 > longest_link1_width)
            longest_link1_width = static_cast<int>(link1_name.size()) + 2;

          std::string link2_name = collision.second.front().link_names[1];
          if (static_cast<int>(link2_name.size()) + 2 > longest_link2_width)
            longest_link2_width = static_cast<int>(link2_name.size()) + 2;
        }
      }
    }
  }

  substep_details_width += longest_link1_width;
  substep_details_width += longest_link2_width;

  // Distance will also be truncated at 4 decimal points of precision, shouldn't need more
  // than 0.1 mm of precision
  // Assumming "DISTANCE" is the widest text, also doesn't matter because this is the last column
  std::string distance_title = "DISTANCE";
  int longest_distance_width = static_cast<int>(distance_title.size()) + 2;

  substep_details_width += longest_distance_width;

  // Construct strings for displaying info on a new state and new substate
  std::string new_step_string(static_cast<std::size_t>(step_details_width), '*');
  new_step_string += "|";
  new_step_string += std::string(static_cast<std::size_t>(substep_details_width), '*');
  std::string new_substep_string(static_cast<std::size_t>(substep_details_width), '-');

  // Start making the table
  // Start on new line to avoid offset by anythnig on previous line
  ss << std::endl;
  // Make the header
  ss << std::setw(longest_steps_width) << step_title << std::setw(longest_joint_name_width) << joint_name_title
     << std::setw(longest_state0_width) << state0_title << std::setw(longest_state1_width) << state1_title << "|"
     << std::setw(longest_substep_width) << substep_title << std::setw(longest_link1_width) << link1_title
     << std::setw(longest_link2_width) << link2_title << std::setw(longest_distance_width) << distance_title
     << std::endl;

  ss << new_step_string << std::endl;

  for (const auto& step : steps)
  {
    // Check if there are contacts in this step
    if (step.numContacts() == 0)
      continue;

    // Create string for stating the step number, repeated on every line of this step. example: 2/23
    std::string step_number_string = std::to_string(step.step) + "/" + std::to_string(total_steps);
    int line_number = 0;
    for (const auto& substep : step.substeps)
    {
      if (substep.contacts.empty())
        continue;

      // Check if there are contacts in this substep
      if (substep.numContacts() == 0)
        continue;

      // Create string for stating the substep number, repeated on every line of this substep
      std::string substep_string = std::to_string(substep.substep) + "/" + std::to_string(step.total_substeps);

      // Iterate over every collision in this substep
      for (const auto& collision : substep.contacts)
      {
        if (collision.second.empty())
          continue;
        // Write the current substep string
        ss << std::setw(longest_steps_width) << step_number_string;

        // Check if we still need to be adding to the joint state information
        if (line_number < static_cast<int>(joint_names.size()))
        {
          ss << std::setprecision(4) << std::fixed;
          ss << std::setw(longest_joint_name_width) << joint_names[static_cast<std::size_t>(line_number)];
          ss << std::setw(longest_state0_width) << step.state0(line_number);
          ss << std::setw(longest_state1_width) << step.state1(line_number);
        }
        else
        {
          // Add blank spaces once done writing joint states
          ss << std::setw(longest_joint_name_width) << " " << std::setw(longest_state0_width) << " "
             << std::setw(longest_state1_width) << " ";
        }
        // Add vertical bar
        ss << "|";

        // Add specific contact information
        ss << std::setw(longest_substep_width) << substep_string;
        ss << std::setw(longest_link1_width) << collision.second.front().link_names[0];
        ss << std::setw(longest_link2_width) << collision.second.front().link_names[1];
        ss << std::setw(longest_distance_width) << collision.second.front().distance;
        ss << std::endl;
        line_number++;
      }

      // Make new line for seperator between substates
      ss << std::setw(longest_steps_width) << step_number_string;
      if (line_number < static_cast<int>(joint_names.size()))
      {
        ss << std::setw(longest_joint_name_width) << joint_names[static_cast<std::size_t>(line_number)];
        ss << std::setw(longest_state0_width) << step.state0(line_number);
        ss << std::setw(longest_state1_width) << step.state1(line_number);
      }
      else
      {
        ss << std::setw(longest_joint_name_width) << " " << std::setw(longest_state0_width) << " "
           << std::setw(longest_state1_width) << " ";
      }
      ss << "|";
      ss << new_substep_string;
      ss << std::endl;
      line_number++;
    }

    // Finish writing joint state if necessary
    while (line_number < static_cast<int>(joint_names.size()))
    {
      ss << std::setw(longest_steps_width) << step_number_string;
      ss << std::setw(longest_joint_name_width) << joint_names[static_cast<std::size_t>(line_number)];
      ss << std::setw(longest_state0_width) << step.state0(line_number);
      ss << std::setw(longest_state1_width) << step.state1(line_number);
      ss << "|" << std::endl;
      line_number++;
    }
    ss << new_step_string << std::endl;
  }
  return ss;
}

std::stringstream ContactTrajectoryResults::collisionFrequencyPerLink() const
{
  // Count all links that experienced a collision
  std::unordered_map<std::string, int> link_collision_count;
  for (const auto& step : steps)
  {
    for (const auto& substep : step.substeps)
    {
      for (const auto& contact_pair : substep.contacts.getContainer())
      {
        const auto& link_pair = contact_pair.first;
        link_collision_count[link_pair.first]++;
        link_collision_count[link_pair.second]++;
      }
    }
  }

  // Determine the maximum width for the link name column to have a clean output
  size_t max_link_name_length = 0;
  for (const auto& entry : link_collision_count)
  {
    if (entry.first.size() > max_link_name_length)
      max_link_name_length = entry.first.size();
  }

  // Adjust the width to have some extra space after the longest link name
  const size_t column_width = max_link_name_length + 2;

  // Create a vector of pairs and sort it by frequency in descending order
  std::vector<std::pair<std::string, int>> sorted_collisions(link_collision_count.begin(), link_collision_count.end());
  std::sort(sorted_collisions.begin(), sorted_collisions.end(), [](const auto& a, const auto& b) {
    return b.second < a.second;
  });

  // Create a string stream to store the table
  std::stringstream ss;
  ss << std::left << std::setw(static_cast<int>(column_width)) << "Link Name"
     << "Collisions" << std::endl;
  ss << std::string(column_width + 10, '-') << std::endl;
  for (const auto& entry : sorted_collisions)
  {
    ss << std::left << std::setw(static_cast<int>(column_width)) << entry.first << entry.second << std::endl;
  }

  return ss;
}

}  // namespace tesseract_collision
