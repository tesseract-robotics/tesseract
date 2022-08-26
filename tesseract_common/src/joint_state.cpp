/**
 * @file joint_state.cpp
 * @brief Tesseract Joint State
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
#include <tesseract_common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <boost/serialization/nvp.hpp>
#include <boost/serialization/string.hpp>
#include <boost/serialization/vector.hpp>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract_common/utils.h>
#include <tesseract_common/eigen_serialization.h>
#include <tesseract_common/joint_state.h>

namespace tesseract_common
{
JointState::JointState(std::vector<std::string> joint_names, const Eigen::Ref<const Eigen::VectorXd>& position)
  : joint_names(std::move(joint_names)), position(position)
{
}

bool JointState::operator==(const JointState& other) const
{
  bool ret_val = true;
  ret_val &= (joint_names == other.joint_names);
  ret_val &= (position.isApprox(other.position, 1e-5));
  ret_val &= (velocity.isApprox(other.velocity, 1e-5));
  ret_val &= (acceleration.isApprox(other.acceleration, 1e-5));
  ret_val &= (effort.isApprox(other.effort, 1e-5));
  ret_val &= (tesseract_common::almostEqualRelativeAndAbs(time, other.time, 1e-5));
  return ret_val;
}

bool JointState::operator!=(const JointState& rhs) const { return !operator==(rhs); }

template <class Archive>
void JointState::serialize(Archive& ar, const unsigned int /*version*/)  // NOLINT
{
  ar& BOOST_SERIALIZATION_NVP(joint_names);
  ar& BOOST_SERIALIZATION_NVP(position);
  ar& BOOST_SERIALIZATION_NVP(velocity);
  ar& BOOST_SERIALIZATION_NVP(acceleration);
  ar& BOOST_SERIALIZATION_NVP(effort);
  ar& BOOST_SERIALIZATION_NVP(time);
}

JointTrajectory::JointTrajectory(std::string description) : description(std::move(description)) {}

JointTrajectory::JointTrajectory(std::vector<JointState> states, std::string description)
  : states(std::move(states)), description(std::move(description))
{
}

bool JointTrajectory::operator==(const JointTrajectory& other) const
{
  bool ret_val = true;
  ret_val &= (description == other.description);
  ret_val &= (states == other.states);
  return ret_val;
}

bool JointTrajectory::operator!=(const JointTrajectory& rhs) const { return !operator==(rhs); }

///////////////
// Iterators //
///////////////
JointTrajectory::iterator JointTrajectory::begin() { return states.begin(); }
JointTrajectory::const_iterator JointTrajectory::begin() const { return states.begin(); }
JointTrajectory::iterator JointTrajectory::end() { return states.end(); }
JointTrajectory::const_iterator JointTrajectory::end() const { return states.end(); }
JointTrajectory::reverse_iterator JointTrajectory::rbegin() { return states.rbegin(); }
JointTrajectory::const_reverse_iterator JointTrajectory::rbegin() const { return states.rbegin(); }
JointTrajectory::reverse_iterator JointTrajectory::rend() { return states.rend(); }
JointTrajectory::const_reverse_iterator JointTrajectory::rend() const { return states.rend(); }
JointTrajectory::const_iterator JointTrajectory::cbegin() const { return states.cbegin(); }
JointTrajectory::const_iterator JointTrajectory::cend() const { return states.cend(); }
JointTrajectory::const_reverse_iterator JointTrajectory::crbegin() const { return states.crbegin(); }
JointTrajectory::const_reverse_iterator JointTrajectory::crend() const { return states.crend(); }

//////////////
// Capacity //
//////////////
bool JointTrajectory::empty() const { return states.empty(); }
JointTrajectory::size_type JointTrajectory::size() const { return states.size(); }
JointTrajectory::size_type JointTrajectory::max_size() const { return states.max_size(); }
void JointTrajectory::reserve(size_type n) { states.reserve(n); }
JointTrajectory::size_type JointTrajectory::capacity() const { return states.capacity(); }
void JointTrajectory::shrink_to_fit() { states.shrink_to_fit(); }

////////////////////
// Element Access //
////////////////////
JointTrajectory::reference JointTrajectory::front() { return states.front(); }
JointTrajectory::const_reference JointTrajectory::front() const { return states.front(); }
JointTrajectory::reference JointTrajectory::back() { return states.back(); }
JointTrajectory::const_reference JointTrajectory::back() const { return states.back(); }
JointTrajectory::reference JointTrajectory::at(size_type n) { return states.at(n); }
JointTrajectory::const_reference JointTrajectory::at(size_type n) const { return states.at(n); }
JointTrajectory::pointer JointTrajectory::data() { return states.data(); }
JointTrajectory::const_pointer JointTrajectory::data() const { return states.data(); }
JointTrajectory::reference JointTrajectory::operator[](size_type pos) { return states[pos]; }
JointTrajectory::const_reference JointTrajectory::operator[](size_type pos) const { return states[pos]; };

///////////////
// Modifiers //
///////////////
void JointTrajectory::clear() { states.clear(); }
JointTrajectory::iterator JointTrajectory::insert(const_iterator p, const value_type& x) { return states.insert(p, x); }
JointTrajectory::iterator JointTrajectory::insert(const_iterator p, value_type&& x) { return states.insert(p, x); }
JointTrajectory::iterator JointTrajectory::insert(const_iterator p, std::initializer_list<value_type> l)
{
  return states.insert(p, l);
}

template <class... Args>
JointTrajectory::iterator JointTrajectory::emplace(const_iterator pos, Args&&... args)
{
  return states.emplace(pos, std::forward<Args>(args)...);
}

JointTrajectory::iterator JointTrajectory::erase(const_iterator p) { return states.erase(p); }
JointTrajectory::iterator JointTrajectory::erase(const_iterator first, const_iterator last)
{
  return states.erase(first, last);
}
void JointTrajectory::push_back(const value_type& x) { states.push_back(x); }
void JointTrajectory::push_back(const value_type&& x) { states.push_back(x); }

template <typename... Args>
#if __cplusplus > 201402L
JointTrajectory::reference JointTrajectory::emplace_back(Args&&... args)
{
  return states.emplace_back(std::forward<Args>(args)...);
}
#else
void JointTrajectory::emplace_back(Args&&... args)
{
  container_.emplace_back(std::forward<Args>(args)...);
}
#endif

void JointTrajectory::pop_back() { states.pop_back(); }
void JointTrajectory::swap(std::vector<value_type>& other) { states.swap(other); }

template <class Archive>
void JointTrajectory::serialize(Archive& ar, const unsigned int version)  // NOLINT
{
  ar& BOOST_SERIALIZATION_NVP(states);
  ar& BOOST_SERIALIZATION_NVP(description);
}

}  // namespace tesseract_common

#include <tesseract_common/serialization.h>
TESSERACT_SERIALIZE_ARCHIVES_INSTANTIATE(tesseract_common::JointState)
TESSERACT_SERIALIZE_ARCHIVES_INSTANTIATE(tesseract_common::JointTrajectory)
