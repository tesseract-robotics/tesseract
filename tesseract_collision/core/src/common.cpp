/**
 * @file common.cpp
 * @brief This is a collection of common methods
 *
 * @author Levi Armstrong
 * @date Dec 18, 2017
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
#include <cstdio>
#include <console_bridge/console.h>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract_common/utils.h>
#include <tesseract_common/types.h>
#include <tesseract_collision/core/common.h>
#include <tesseract_collision/core/contact_result_validator.h>

namespace tesseract_collision
{
std::vector<ObjectPairKey>
getCollisionObjectPairs(const std::vector<std::string>& active_links,
                        const std::vector<std::string>& static_links,
                        const std::shared_ptr<const tesseract_common::ContactAllowedValidator>& validator)
{
  std::size_t num_pairs = active_links.size() * (active_links.size() - 1) / 2;
  num_pairs += (active_links.size() * static_links.size());

  std::vector<ObjectPairKey> clp;
  clp.reserve(num_pairs);

  // Create active to active pairs
  for (std::size_t i = 0; i < active_links.size() - 1; ++i)
  {
    const std::string& l1 = active_links[i];
    for (std::size_t j = i + 1; j < active_links.size(); ++j)
    {
      const std::string& l2 = active_links[j];
      if (validator == nullptr || (validator != nullptr && !(*validator)(l1, l2)))
        clp.push_back(tesseract_common::makeOrderedLinkPair(l1, l2));
    }
  }

  // Create active to static pairs
  for (const auto& l1 : active_links)
  {
    for (const auto& l2 : static_links)
    {
      if (validator == nullptr || (validator != nullptr && !(*validator)(l1, l2)))
        clp.push_back(tesseract_common::makeOrderedLinkPair(l1, l2));
    }
  }

  return clp;
}

bool isLinkActive(const std::vector<std::string>& active, const std::string& name)
{
  return active.empty() || (std::find(active.begin(), active.end(), name) != active.end());
}

bool isContactAllowed(const std::string& name1,
                      const std::string& name2,
                      const std::shared_ptr<const tesseract_common::ContactAllowedValidator>& validator,
                      bool verbose)
{
  // do not distance check geoms part of the same object / link / attached body
  if (name1 == name2)
    return true;

  if (validator != nullptr && (*validator)(name1, name2))
  {
    if (verbose)
    {
      CONSOLE_BRIDGE_logError(
          "Collision between '%s' and '%s' is allowed. No contacts are computed.", name1.c_str(), name2.c_str());
    }
    return true;
  }

  if (verbose)
  {
    CONSOLE_BRIDGE_logError("Actually checking collisions between %s and %s", name1.c_str(), name2.c_str());
  }

  return false;
}

ContactResult* processResult(ContactTestData& cdata,
                             ContactResult& contact,
                             const std::pair<std::string, std::string>& key,
                             bool found)
{
  if (cdata.req.is_valid && !(*cdata.req.is_valid)(contact))
    return nullptr;

  if ((cdata.req.calculate_distance || cdata.req.calculate_penetration) &&
      (contact.distance > cdata.collision_margin_data.getCollisionMargin(key.first, key.second)))
    return nullptr;

  if (!found)
  {
    if (cdata.req.type == ContactTestType::FIRST)
      cdata.done = true;

    return &(cdata.res->addContactResult(key, contact));
  }

  assert(cdata.req.type != ContactTestType::FIRST);
  if (cdata.req.type == ContactTestType::ALL)
    return &(cdata.res->addContactResult(key, contact));

  if (cdata.req.type == ContactTestType::CLOSEST)
  {
    const auto& cv = cdata.res->at(key);
    assert(!cv.empty());

    if (contact.distance < cv.front().distance)
      return &(cdata.res->setContactResult(key, contact));
  }

  //    else if (cdata.cdata.condition == DistanceRequestType::LIMITED)
  //    {
  //      assert(dr.size() < cdata.req->max_contacts_per_body);
  //      dr.emplace_back(contact);
  //      return &(dr.back());
  //    }

  return nullptr;
}

void scaleVertices(tesseract_common::VectorVector3d& vertices,
                   const Eigen::Vector3d& center,
                   const Eigen::Vector3d& scale)
{
  for (auto& v : vertices)
    v = scale.cwiseProduct(v - center) + center;
}

void scaleVertices(tesseract_common::VectorVector3d& vertices, const Eigen::Vector3d& scale)
{
  Eigen::Vector3d center(0, 0, 0);
  for (const auto& v : vertices)
    center = center + v;

  center = (1.0 / static_cast<double>(vertices.size())) * center;

  scaleVertices(vertices, center, scale);
}

}  // namespace tesseract_collision
