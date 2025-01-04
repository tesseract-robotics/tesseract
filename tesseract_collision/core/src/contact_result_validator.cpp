/**
 * @file contact_result_validator.h
 * @brief Contact result validator
 *
 * @author Levi Armstrong
 * @date Jan 2, 2025
 * @version TODO
 * @bug No known bugs
 *
 * @copyright Copyright (c) 2025, Levi Armstrong
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

#include <tesseract_collision/core/contact_result_validator.h>

namespace tesseract_collision
{
template <class Archive>
void ContactResultValidator::serialize(Archive& /*ar*/, const unsigned int /*version*/)
{
}

}  // namespace tesseract_collision

#include <tesseract_common/serialization.h>
TESSERACT_SERIALIZE_ARCHIVES_INSTANTIATE(tesseract_collision::ContactResultValidator)
BOOST_CLASS_EXPORT_IMPLEMENT(tesseract_collision::ContactResultValidator)
