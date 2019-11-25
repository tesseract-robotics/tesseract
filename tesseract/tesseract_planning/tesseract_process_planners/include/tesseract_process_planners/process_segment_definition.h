/**
 * @file process_segment_definition.h
 * @brief Tesseract process segment definition
 *
 * @author Levi Armstrong
 * @date April 18, 2018
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
#ifndef TESSERACT_PROCESS_PLANNERS_PROCESS_SEGMENT_DEFINITION_H
#define TESSERACT_PROCESS_PLANNERS_PROCESS_SEGMENT_DEFINITION_H

#include <tesseract_common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <vector>
#include <cassert>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract_motion_planners/core/waypoint.h>

namespace tesseract_process_planners
{
/**
 * @class tesseract_process_planners::ProcessSegmentDefinition
 * @details
 * A process segment definition is assumed to have three components, an approach, process and departure.
 *
 */
class ProcessSegmentDefinition
{
public:
  class Iterator;
  /**
   * @brief The approach defined as a series of waypoints.
   *
   * If empty, the approach is skipped
   */
  std::vector<tesseract_motion_planners::Waypoint::Ptr> approach;

  /**
   * @brief The process is defined as a series of waypoints.
   *
   * This should contain a minimum of two waypoints.
   */
  std::vector<tesseract_motion_planners::Waypoint::Ptr> process;

  /**
   * @brief The departure is defined as a series of waypoints.
   *
   * If empty, the approach is skipped
   */
  std::vector<tesseract_motion_planners::Waypoint::Ptr> departure;

  /**
   * @brief Get the size = approach.size() + process.size() + departure.size()
   * @return The size of the process segement definition
   */
  std::size_t size() const noexcept;

  /**
   * @brief Given an index, check if it is part of the approach
   * @param index The index to check
   * @return True if associated with the approach, otherwise false.
   */
  bool isApproach(std::size_t index) const noexcept;

  /**
   * @brief Given an index, check if it is part of the process
   * @param index The index to check
   * @return True if associated with the process, otherwise false.
   */
  bool isProcess(std::size_t index) const noexcept;

  /**
   * @brief Given an index, check if it is part of the departure
   * @param index The index to check
   * @return True if associated with the departure, otherwise false.
   */
  bool isDeparture(std::size_t index) const noexcept;

  /**
   * @brief Get the iterator to the beginning of the process segment definition
   * @return Iterator
   */
  Iterator begin() noexcept;

  /**
   * @brief Get the iterator to the end of the process segment definition
   * @return Iterator
   */
  Iterator end() noexcept;

  /**
   * @brief Get waypoint by index
   * @param index The index
   * @return Waypoint
   */
  tesseract_motion_planners::Waypoint::Ptr& operator[](std::size_t index) noexcept;

  /**
   * @brief Get const waypoint by index
   * @param index The index
   * @return Const Waypoint
   */
  const tesseract_motion_planners::Waypoint::Ptr& operator[](std::size_t index) const noexcept;

  /**
   * @brief Get waypoint by index
   * @param index The index
   * @return Waypoint
   * @throw  std::out_of_range  If @a index is an invalid index.
   *
   *  This function provides for safer data access.  The parameter
   *  is first checked that it is in the range of the vector.  The
   *  function throws out_of_range if the check fails.
   */
  tesseract_motion_planners::Waypoint::Ptr& at(std::size_t index);

  /**
   * @brief Get const waypoint by index
   * @param index The index
   * @return Const Waypoint
   * @throw  std::out_of_range  If @a index is an invalid index.
   *
   *  This function provides for safer data access.  The parameter
   *  is first checked that it is in the range of the vector.  The
   *  function throws out_of_range if the check fails.
   */
  const tesseract_motion_planners::Waypoint::Ptr& at(std::size_t index) const;

  /**
   * @brief Erase waypoint given iterator
   * @param pos The iterator associated with the waypoint to remove
   * @return The iterator
   */
  Iterator erase(Iterator pos);

  /**
   * @brief Erase waypoints given iterator
   * @param pos The iterator associated with the waypoint to remove
   * @return The iterator
   */
  Iterator erase(Iterator first, Iterator last);

  /** @brief The Iterator class ProcessSegmentDefinition */
  class Iterator : public std::iterator<std::random_access_iterator_tag, tesseract_motion_planners::Waypoint::Ptr>
  {
  public:
    Iterator(ProcessSegmentDefinition& container, std::size_t pos);

    tesseract_motion_planners::Waypoint::Ptr& operator*();

    const tesseract_motion_planners::Waypoint::Ptr& operator*() const;

    const Iterator& operator++();

    Iterator operator++(int);

    const Iterator& operator--();

    Iterator operator--(int);

    difference_type operator-(const Iterator& rhs) const;

    Iterator operator+(difference_type rhs) const;

    Iterator operator-(difference_type rhs) const;

    bool operator==(const Iterator& rhs) const;

    bool operator!=(const Iterator& rhs) const;

    bool operator>(const Iterator& rhs) const;

    bool operator<(const Iterator& rhs) const;

    bool operator>=(const Iterator& rhs) const;

    bool operator<=(const Iterator& rhs) const;

  private:
    std::size_t pos_;
    ProcessSegmentDefinition* container_;
  };
};

}  // namespace tesseract_process_planners
#endif  // TESSERACT_PROCESS_PLANNERS_PROCESS_SEGMENT_DEFINITION_H
