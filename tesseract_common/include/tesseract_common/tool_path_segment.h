/**
 * @file tool_path_segment.h
 * @brief Common Tesseract Tool Path Segment
 *
 * @author Levi Armstrong
 * @date Nov 16, 2022
 * @version TODO
 * @bug No known bugs
 *
 * @copyright Copyright (c) 2022, Levi Armstrong
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
#ifndef TESSERACT_COMMON_TOOL_PATH_SEGMENT_H
#define TESSERACT_COMMON_TOOL_PATH_SEGMENT_H

#include <string>
#include <boost/uuid/uuid.hpp>
#include <tesseract_common/types.h>

namespace tesseract_common
{
class ToolPathSegment
{
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  ToolPathSegment(std::string description = "");
  ToolPathSegment(boost::uuids::uuid uuid, std::string description = "");
  virtual ~ToolPathSegment() = default;

  /** @brief Get the uuid */
  boost::uuids::uuid getUUID() const;

  /** @brief Regenerate uuid */
  void regenerateUUID();

  /**
   * @brief Get the parent uuid
   * @details This can be null
   */
  const boost::uuids::uuid& getParentUUID() const;

  /**
   * @brief Set the parent uuid
   * @details This can be used in cases were a segment is split during a filter process and want tracability.
   * @param uuid
   */
  void setParentUUID(const boost::uuids::uuid& uuid);

  /** @brief Set the segment description */
  void setDescription(const std::string& desc);

  /** @brief Get the segment description */
  const std::string& getDescription() const;

  bool operator==(const ToolPathSegment& rhs) const;
  bool operator!=(const ToolPathSegment& rhs) const;

  // LCOV_EXCL_START

  ///////////////////////////
  // C++ container support //
  ///////////////////////////
  /** pointer */
  using pointer = typename VectorIsometry3d::pointer;
  /** const_pointer */
  using const_pointer = typename VectorIsometry3d::const_pointer;
  /** reference */
  using reference = typename VectorIsometry3d::reference;
  /** const_reference */
  using const_reference = typename VectorIsometry3d::const_reference;
  /** size_type */
  using size_type = typename VectorIsometry3d::size_type;
  /** difference_type */
  using difference_type = typename VectorIsometry3d::difference_type;
  /** iterator */
  using iterator = typename VectorIsometry3d::iterator;
  /** const_iterator */
  using const_iterator = typename VectorIsometry3d::const_iterator;
  /** reverse_iterator */
  using reverse_iterator = typename VectorIsometry3d::reverse_iterator;
  /** const_reverse_iterator */
  using const_reverse_iterator = typename VectorIsometry3d::const_reverse_iterator;

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
  iterator insert(const_iterator p, const Eigen::Isometry3d& x);
  iterator insert(const_iterator p, Eigen::Isometry3d&& x);
  iterator insert(const_iterator p, std::initializer_list<Eigen::Isometry3d> l);
  template <class InputIt>
  void insert(const_iterator pos, InputIt first, InputIt last)
  {
    container_.insert(pos, first, last);
  }

  /** @brief constructs element in-place */
  template <class... Args>
  iterator emplace(const_iterator pos, Args&&... args);

  /** @brief erases element */
  iterator erase(const_iterator p);
  iterator erase(const_iterator first, const_iterator last);

  /** Append element to container */
  void push_back(const Eigen::Isometry3d& x);
  void push_back(const Eigen::Isometry3d&& x);

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
  void swap(VectorIsometry3d& other);

  // LCOV_EXCL_STOP

protected:
  /** @brief The uuid */
  boost::uuids::uuid uuid_{};

  /** @brief The parent uuid */
  boost::uuids::uuid parent_uuid_{};

  /** @brief The description */
  std::string description_;

  /** @brief The container */
  VectorIsometry3d container_;

  friend class boost::serialization::access;
  template <class Archive>
  void serialize(Archive& ar, const unsigned int version);  // NOLINT
};

}  // namespace tesseract_common
#endif  // TESSERACT_COMMON_TOOL_PATH_SEGMENT_H
