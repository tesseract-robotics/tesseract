/**
 * @file process_segment_definition.cpp
 * @brief Tesseract process segment definition implementation
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

#include <tesseract_process_planners/process_segment_definition.h>

namespace tesseract_process_planners
{
std::size_t ProcessSegmentDefinition::size() const noexcept
{
  return approach.size() + process.size() + departure.size();
}

bool ProcessSegmentDefinition::isApproach(std::size_t index) const noexcept { return (index < approach.size()); }

bool ProcessSegmentDefinition::isProcess(std::size_t index) const noexcept
{
  return ((index >= approach.size()) && (index < (approach.size() + process.size())));
}

bool ProcessSegmentDefinition::isDeparture(std::size_t index) const noexcept
{
  return ((index >= (approach.size() + process.size())) &&
          (index < (approach.size() + process.size() + departure.size())));
}

ProcessSegmentDefinition::Iterator ProcessSegmentDefinition::begin() noexcept { return Iterator{ *this, size_t(0) }; }

ProcessSegmentDefinition::Iterator ProcessSegmentDefinition::end() noexcept { return Iterator{ *this, size() }; }

tesseract_motion_planners::Waypoint::Ptr& ProcessSegmentDefinition::operator[](std::size_t index) noexcept
{
  assert(index < size());
  if (index < approach.size())
    return approach[index];

  if (index < (approach.size() + process.size()))
    return process[index - approach.size()];

  return departure[index - approach.size() - process.size()];
}

const tesseract_motion_planners::Waypoint::Ptr& ProcessSegmentDefinition::operator[](std::size_t index) const noexcept
{
  assert(index < size());
  if (index < approach.size())
    return approach[index];

  if (index < (approach.size() + process.size()))
    return process[index - approach.size()];

  return departure[index - approach.size() - process.size()];
}

tesseract_motion_planners::Waypoint::Ptr& ProcessSegmentDefinition::at(std::size_t index)
{
  assert(index < size());
  if (index < approach.size())
    return approach.at(index);

  if (index < (approach.size() + process.size()))
    return process.at(index - approach.size());

  return departure.at(index - approach.size() - process.size());
}

const tesseract_motion_planners::Waypoint::Ptr& ProcessSegmentDefinition::at(std::size_t index) const
{
  assert(index < size());
  if (index < approach.size())
    return approach.at(index);

  if (index < (approach.size() + process.size()))
    return process.at(index - approach.size());

  return departure.at(index - approach.size() - process.size());
}

ProcessSegmentDefinition::Iterator ProcessSegmentDefinition::erase(Iterator pos)
{
  long index = std::distance(begin(), pos);
  Iterator ret = ++pos;

  if (index < static_cast<long>(approach.size()))
    approach.erase(approach.begin() + index);
  else if (index < static_cast<long>(approach.size() + process.size()))
    process.erase(process.begin() + index - static_cast<long>(approach.size()));
  else if (index < static_cast<long>(approach.size() + process.size() + departure.size()))
    departure.erase(departure.begin() + index - static_cast<long>(approach.size() - process.size()));

  return ret;
}

ProcessSegmentDefinition::Iterator ProcessSegmentDefinition::erase(Iterator first, Iterator last)
{
  auto index = static_cast<std::size_t>(std::distance(begin(), first));
  auto length = static_cast<std::size_t>(std::distance(first, last));
  Iterator ret = ++last;
  for (std::size_t i = 0; i < length; ++i)
  {
    if (index < approach.size())
      approach.erase(approach.begin() + static_cast<long>(index));
    else if (index < (approach.size() + process.size()))
      process.erase(process.begin() + static_cast<long>(index - approach.size()));
    else if (index < (approach.size() + process.size() + departure.size()))
      departure.erase(departure.begin() + static_cast<long>(index - approach.size() - process.size()));
    else
      throw std::runtime_error("Index is out of range!");
  }

  return ret;
}

ProcessSegmentDefinition::Iterator::Iterator(ProcessSegmentDefinition& container, size_t pos)
  : pos_(pos), container_(&container)
{
}

tesseract_motion_planners::Waypoint::Ptr& ProcessSegmentDefinition::Iterator::operator*()
{
  if (pos_ < container_->approach.size())
    return container_->approach.at(pos_);

  if (pos_ < (container_->approach.size() + container_->process.size()))
    return container_->process.at(pos_ - container_->approach.size());

  if (pos_ < (container_->approach.size() + container_->process.size() + container_->departure.size()))
    return container_->departure.at(pos_ - container_->approach.size() - container_->process.size());

  throw std::runtime_error("Index is out of range!");
}

const tesseract_motion_planners::Waypoint::Ptr& ProcessSegmentDefinition::Iterator::operator*() const
{
  if (pos_ < container_->approach.size())
    return container_->approach.at(pos_);

  if (pos_ < (container_->approach.size() + container_->process.size()))
    return container_->process.at(pos_ - container_->approach.size());

  if (pos_ < (container_->approach.size() + container_->process.size() + container_->departure.size()))
    return container_->departure.at(pos_ - container_->approach.size() - container_->process.size());

  throw std::runtime_error("Index is out of range!");
}

const ProcessSegmentDefinition::Iterator& ProcessSegmentDefinition::Iterator::operator++()
{
  ++pos_;
  // although not strictly necessary for a range-based for loop
  // following the normal convention of returning a value from
  // operator++ is a good idea.
  return *this;
}

ProcessSegmentDefinition::Iterator ProcessSegmentDefinition::Iterator::operator++(int)
{
  ProcessSegmentDefinition::Iterator ret = *this;
  ++pos_;
  return ret;
}

const ProcessSegmentDefinition::Iterator& ProcessSegmentDefinition::Iterator::operator--()
{
  --pos_;
  // although not strictly necessary for a range-based for loop
  // following the normal convention of returning a value from
  // operator++ is a good idea.
  return *this;
}

ProcessSegmentDefinition::Iterator ProcessSegmentDefinition::Iterator::operator--(int)
{
  ProcessSegmentDefinition::Iterator ret = *this;
  --pos_;
  return ret;
}

ProcessSegmentDefinition::Iterator::difference_type ProcessSegmentDefinition::Iterator::
operator-(const Iterator& rhs) const
{
  return static_cast<ProcessSegmentDefinition::Iterator::difference_type>(pos_ - rhs.pos_);
}

ProcessSegmentDefinition::Iterator ProcessSegmentDefinition::Iterator::operator+(difference_type rhs) const
{
  return Iterator{ *container_, pos_ + static_cast<size_t>(rhs) };
}

ProcessSegmentDefinition::Iterator ProcessSegmentDefinition::Iterator::operator-(difference_type rhs) const
{
  return Iterator{ *container_, pos_ - static_cast<size_t>(rhs) };
}

bool ProcessSegmentDefinition::Iterator::operator==(const Iterator& rhs) const
{
  return ((pos_ == rhs.pos_) && (container_ == rhs.container_));
}

bool ProcessSegmentDefinition::Iterator::operator!=(const Iterator& rhs) const
{
  return ((pos_ != rhs.pos_) || (container_ != rhs.container_));
}

bool ProcessSegmentDefinition::Iterator::operator>(const Iterator& rhs) const
{
  return ((pos_ > rhs.pos_) && (container_ == rhs.container_));
}

bool ProcessSegmentDefinition::Iterator::operator<(const Iterator& rhs) const
{
  return ((pos_ < rhs.pos_) && (container_ == rhs.container_));
}

bool ProcessSegmentDefinition::Iterator::operator>=(const Iterator& rhs) const
{
  return ((pos_ >= rhs.pos_) && (container_ == rhs.container_));
}

bool ProcessSegmentDefinition::Iterator::operator<=(const Iterator& rhs) const
{
  return ((pos_ <= rhs.pos_) && (container_ == rhs.container_));
}

}  // namespace tesseract_process_planners
