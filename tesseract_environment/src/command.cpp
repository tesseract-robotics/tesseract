/**
 * @file add_link_command.cpp
 * @brief Used to add a link to the environment
 *
 * @author Levi Armstrong
 * @author Matthew Powelson
 * @date March 18, 2022
 * @version TODO
 * @bug No known bugs
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
#include <boost/serialization/access.hpp>
#include <boost/serialization/nvp.hpp>
#include <boost/serialization/split_free.hpp>
#include <memory>
#include <string>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract_environment/command.h>

namespace tesseract_environment
{
template <class Archive>
void save(Archive& ar, const CommandType& g, const unsigned int /*version*/)
{
  int value = static_cast<int>(g);
  ar &= BOOST_SERIALIZATION_NVP(value);
}

template <class Archive>
void load(Archive& ar, CommandType& g, const unsigned int /*version*/)
{
  int value = 0;
  ar &= BOOST_SERIALIZATION_NVP(value);
  g = static_cast<CommandType>(value);
}

template <class Archive>
void serialize(Archive& ar, CommandType& g, const unsigned int version)
{
  split_free(ar, g, version);
}

bool Command::operator==(const Command& rhs) const
{
  bool equal = true;
  equal &= type_ == rhs.type_;
  return equal;
}
bool Command::operator!=(const Command& rhs) const { return !operator==(rhs); }

template <class Archive>
void Command::serialize(Archive& ar, const unsigned int /*version*/)
{
  ar& BOOST_SERIALIZATION_NVP(type_);
}
}  // namespace tesseract_environment

#include <tesseract_common/serialization.h>
TESSERACT_SERIALIZE_ARCHIVES_INSTANTIATE(tesseract_environment::Command)
