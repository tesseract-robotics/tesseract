/**
 * @file name_id_testing.h
 * @brief Test-only access to NameId internals
 *
 * @author Roelof Oomen
 * @date July 10, 2026
 *
 * @copyright Copyright (c) 2026, Southwest Research Institute
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
#ifndef TESSERACT_COMMON_TEST_SUITE_NAME_ID_TESTING_H
#define TESSERACT_COMMON_TEST_SUITE_NAME_ID_TESTING_H

#include <utility>

#include <tesseract/common/types.h>

namespace tesseract::common
{
/** @brief Test-only access to NameId internals; NameId declares this a friend. */
struct NameIdTestAccess
{
  /** @brief Construct an id with an explicit value/name combination, e.g. a manufactured hash collision. */
  template <typename IdT>
  [[nodiscard]] static IdT create(NameIdValue value, const std::string& name)
  {
    IdT id;
    id.value_ = value;
    id.name_ = name;
    return id;
  }

  /**
   * @brief Two ids whose canonical pair order is the reverse of their alphabetical order.
   * @details OrderedIdPair canonicalizes by hash value, so a test that must tell canonical order apart from
   *          alphabetical order cannot use real hashes — those move with the boost version. The values here are
   *          injected so that OrderedIdPair(first, second).first() is the alphabetically later name.
   */
  template <typename IdT>
  [[nodiscard]] static std::pair<IdT, IdT> createReverseCanonical(const std::string& alphabetically_first,
                                                                  const std::string& alphabetically_second)
  {
    return { create<IdT>(2, alphabetically_first), create<IdT>(1, alphabetically_second) };
  }
};
}  // namespace tesseract::common

#endif  // TESSERACT_COMMON_TEST_SUITE_NAME_ID_TESTING_H
