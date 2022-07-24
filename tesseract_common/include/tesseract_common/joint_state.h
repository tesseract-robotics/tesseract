/**
 * @file joint_state.h
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
#ifndef TESSERACT_COMMON_JOINT_STATE_H
#define TESSERACT_COMMON_JOINT_STATE_H

#include <tesseract_common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <Eigen/Dense>
#include <vector>
#include <boost/serialization/base_object.hpp>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

namespace tesseract_common
{
class JointState
{
public:
  JointState() = default;
  JointState(std::vector<std::string> joint_names, Eigen::VectorXd position);

  /** @brief The joint corresponding to the position vector. */
  std::vector<std::string> joint_names;

  /** @brief The joint position at the waypoint */
  Eigen::VectorXd position;

  /** @brief The velocity at the waypoint (optional) */
  Eigen::VectorXd velocity;

  /** @brief The Acceleration at the waypoint (optional) */
  Eigen::VectorXd acceleration;

  /** @brief The Effort at the waypoint (optional) */
  Eigen::VectorXd effort;

  /** @brief The Time from start at the waypoint (optional) */
  double time{ 0 };

  bool operator==(const JointState& other) const;

  bool operator!=(const JointState& rhs) const;

private:
  friend class boost::serialization::access;
  template <class Archive>
  void serialize(Archive& ar, const unsigned int version);  // NOLINT
};

}  // namespace tesseract_common

namespace tesseract_common
{
/** @brief Represents a joint trajectory */
class JointTrajectory
{
public:
  JointTrajectory(std::string description = "");
  JointTrajectory(std::vector<JointState> states, std::string description = "");

  std::vector<JointState> states;
  std::string description;

  bool operator==(const JointTrajectory& other) const;

  bool operator!=(const JointTrajectory& rhs) const;

  ///////////////////////////
  // C++ container support //
  ///////////////////////////

  /** value_type */
  using value_type = JointState;
  /** pointer */
  using pointer = typename std::vector<value_type>::pointer;
  /** const_pointer */
  using const_pointer = typename std::vector<value_type>::const_pointer;
  /** reference */
  using reference = typename std::vector<value_type>::reference;
  /** const_reference */
  using const_reference = typename std::vector<value_type>::const_reference;
  /** size_type */
  using size_type = typename std::vector<value_type>::size_type;
  /** difference_type */
  using difference_type = typename std::vector<value_type>::difference_type;
  /** iterator */
  using iterator = typename std::vector<value_type>::iterator;
  /** const_iterator */
  using const_iterator = typename std::vector<value_type>::const_iterator;
  /** reverse_iterator */
  using reverse_iterator = typename std::vector<value_type>::reverse_iterator;
  /** const_reverse_iterator */
  using const_reverse_iterator = typename std::vector<value_type>::const_reverse_iterator;

  template <class InputIt>
  JointTrajectory(InputIt first, InputIt last) : states(first, last)
  {
  }

  ///////////////
  // Iterators //
  ///////////////
  /** @brief returns an iterator to the beginning */
  iterator begin();
  /** @brief returns an iterator to the beginning */
  const_iterator begin() const;
  /** @brief returns an iterator to the end */
  iterator end();
  /** @brief returns an iterator to the end */
  const_iterator end() const;
  /** @brief returns a reverse iterator to the beginning */
  reverse_iterator rbegin();
  /** @brief returns a reverse iterator to the beginning */
  const_reverse_iterator rbegin() const;
  /** @brief returns a reverse iterator to the end */
  reverse_iterator rend();
  /** @brief returns a reverse iterator to the end */
  const_reverse_iterator rend() const;
  /** @brief returns an iterator to the beginning */
  const_iterator cbegin() const;
  /** @brief returns an iterator to the end */
  const_iterator cend() const;
  /** @brief returns a reverse iterator to the beginning */
  const_reverse_iterator crbegin() const;
  /** @brief returns a reverse iterator to the end */
  const_reverse_iterator crend() const;

  //////////////
  // Capacity //
  //////////////
  /** @brief checks whether the container is empty */
  bool empty() const;
  /** @brief returns the number of elements */
  size_type size() const;
  /** @brief returns the maximum possible number of elements */
  size_type max_size() const;
  /** @brief reserve number of elements */
  void reserve(size_type n);
  /** @brief returns the number of elements that can be held in currently allocated storage */
  size_type capacity() const;
  /** @brief reduces memory usage by freeing unused memory  */
  void shrink_to_fit();

  ////////////////////
  // Element Access //
  ////////////////////
  /** @brief access the first element */
  reference front();
  /** @brief access the first element */
  const_reference front() const;
  /** @brief access the last element */
  reference back();
  /** @brief access the last element */
  const_reference back() const;
  /** @brief access specified element with bounds checking */
  reference at(size_type n);
  /** @brief access specified element with bounds checking */
  const_reference at(size_type n) const;
  /** @brief direct access to the underlying array  */
  pointer data();
  /** @brief direct access to the underlying array  */
  const_pointer data() const;
  /** @brief access specified element */
  reference operator[](size_type pos);
  /** @brief access specified element */
  const_reference operator[](size_type pos) const;

  ///////////////
  // Modifiers //
  ///////////////
  /** @brief clears the contents */
  void clear();
  /** @brief inserts element */
  iterator insert(const_iterator p, const value_type& x);
  iterator insert(const_iterator p, value_type&& x);
  iterator insert(const_iterator p, std::initializer_list<value_type> l);
  template <class InputIt>
  void insert(const_iterator pos, InputIt first, InputIt last)
  {
    states.insert(pos, first, last);
  }

  /** @brief constructs element in-place */
  template <class... Args>
  iterator emplace(const_iterator pos, Args&&... args);

  /** @brief erases element */
  iterator erase(const_iterator p);
  iterator erase(const_iterator first, const_iterator last);
  /** @brief adds an element to the end */
  void push_back(const value_type& x);
  void push_back(const value_type&& x);

  /** @brief constructs an element in-place at the end  */
  template <typename... Args>
#if __cplusplus > 201402L
  reference emplace_back(Args&&... args);
#else
  void emplace_back(Args&&... args);
#endif

  /** @brief removes the last element */
  void pop_back();
  /** @brief swaps the contents  */
  void swap(std::vector<value_type>& other);

private:
  friend class boost::serialization::access;
  template <class Archive>
  void serialize(Archive& ar, const unsigned int version);  // NOLINT
};

}  // namespace tesseract_common

#endif  // TESSERACT_COMMON_JOINT_STATE_H
