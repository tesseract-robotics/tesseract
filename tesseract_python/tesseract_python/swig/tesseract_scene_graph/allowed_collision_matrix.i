/**
 * @file allowed_collision_matrix.i
 * @brief SWIG interface file for tesseract_scene_graph/allowed_collision_matrix.h
 *
 * @author John Wason
 * @date December 10, 2019
 * @version TODO
 * @bug No known bugs
 *
 * @copyright Copyright (c) 2019, Wason Technology, LLC
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

%{
#include <tesseract_scene_graph/allowed_collision_matrix.h>
%}

%shared_ptr(tesseract_scene_graph::AllowedCollisionMatrix)

%inline %{
  struct AllowedCollisionEntry 
  {
    std::string link_name1;
    std::string link_name2;
    std::string reason;
  };
%}

%template(AllowedCollisionEntries) std::vector<AllowedCollisionEntry>;

namespace tesseract_scene_graph
{
class AllowedCollisionMatrix
{
public:
  
  using Ptr = std::shared_ptr<AllowedCollisionMatrix>;
  using ConstPtr = std::shared_ptr<const AllowedCollisionMatrix>;

  using AllowedCollisionEntries = std::unordered_map<LinkNamesPair, std::string, PairHash>;

  AllowedCollisionMatrix();
  virtual ~AllowedCollisionMatrix();

  virtual void addAllowedCollision(const std::string& link_name1,
                                   const std::string& link_name2,
                                   const std::string& reason);

%extend {
  
  std::vector<AllowedCollisionEntry> getAllAllowedCollisions()
  {
      std::vector<AllowedCollisionEntry> o;

      auto a = $self->getAllAllowedCollisions();
      for (const auto& a1 : a)
      {
        AllowedCollisionEntry o1;
        o1.link_name1 = a1.first.first;
        o1.link_name2 = a1.first.second;
        o1.reason = a1.second;
        o.push_back(o1);
      }
      return o;
  }  
}

  //const AllowedCollisionEntries getAllAllowedCollisions() const;

  virtual void removeAllowedCollision(const std::string& link_name1, const std::string& link_name2);

  virtual void removeAllowedCollision(const std::string& link_name);

  virtual bool isCollisionAllowed(const std::string& link_name1, const std::string& link_name2) const;

  void clearAllowedCollisions();
};

}  // namespace tesseract_scene_graph
