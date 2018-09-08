/**
 * @file contact_checker_common.h
 * @brief This is a collection of common methods
 *
 * @author Levi Armstrong
 * @date Dec 18, 2017
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
#ifndef TESSERACT_COLLISION_CONTACT_CHECKER_COMMON_H
#define TESSERACT_COLLISION_CONTACT_CHECKER_COMMON_H

#include <tesseract_core/basic_types.h>
#include <ros/console.h>

namespace tesseract
{
typedef std::pair<std::string, std::string> ObjectPairKey;

/**
 * @brief Get a key for two object to search the collision matrix
 * @param obj1 First collision object name
 * @param obj2 Second collision object name
 * @param pair A collision pair key. Will be updated if the obj's are valid.
 * @return True if the collision pair key was updated successfully.
 */
inline bool getObjectPairKey(const std::string& obj1, const std::string& obj2, ObjectPairKey &pair)
{
  if (obj1 < obj2)
  {
    pair = std::make_pair(obj1, obj2);
    return true;
  }
  // Avoid redundant entries like <link1,link2> and <link2,link1>
  else
    return false;
}

/**
 * @brief Determine if contact is allowed between two objects.
 * @param name1 The name of the first object
 * @param name2 The name of the second object
 * @param acm The contact allowed function
 * @param verbose If true print debug informaton
 * @return True if contact is allowed between the two object, otherwise false.
 */
inline bool
isContactAllowed(const std::string& name1, const std::string& name2, const IsContactAllowedFn acm, bool verbose = false)
{
  // do not distance check geoms part of the same object / link / attached body
  if (name1 == name2)
    return true;

  if (acm != nullptr && acm(name1, name2))
  {
    if (verbose)
    {
      ROS_DEBUG("Collision between '%s' and '%s' is allowed. No contacts are computed.", name1.c_str(), name2.c_str());
    }
    return true;
  }

  if (verbose)
  {
    ROS_DEBUG("Actually checking collisions between %s and %s", name1.c_str(), name2.c_str());
  }

  return false;
}

inline ContactResult* processResult(ContactDistanceData& cdata,
                                    ContactResult& contact,
                                    const std::pair<std::string, std::string>& key,
                                    bool found)
{
  if (!found)
  {
    ContactResultVector data;
    if (cdata.req->type == ContactRequestType::FIRST)
    {
      data.emplace_back(contact);
      cdata.done = true;
    }
    else
    {
      data.reserve(100);  // TODO: Need better way to initialize this
      data.emplace_back(contact);
    }

    return &(cdata.res->insert(std::make_pair(key, data)).first->second.back());
  }
  else
  {
    assert(cdata.req->type != ContactRequestType::FIRST);
    ContactResultVector& dr = cdata.res->at(key);
    if (cdata.req->type == ContactRequestType::ALL)
    {
      dr.emplace_back(contact);
      return &(dr.back());
    }
    else if (cdata.req->type == ContactRequestType::CLOSEST)
    {
      if (contact.distance < dr[0].distance)
      {
        dr[0] = contact;
        return &(dr[0]);
      }
    }
    //    else if (cdata.req->type == DistanceRequestType::LIMITED)
    //    {
    //      assert(dr.size() < cdata.req->max_contacts_per_body);
    //      dr.emplace_back(contact);
    //      return &(dr.back());
    //    }
  }

  return nullptr;
}
}

#endif  // TESSERACT_COLLISION_CONTACT_CHECKER_COMMON_H
