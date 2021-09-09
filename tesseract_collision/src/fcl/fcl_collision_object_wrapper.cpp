/**
 * @file fcl_collision_object_wrapper.cpp
 * @brief Collision Object Wrapper to modify AABB with contact distance threshold
 *
 * @author Levi Armstrong
 * @date April 14, 2020
 * @version TODO
 * @bug No known bugs
 *
 * @copyright Copyright (c) 2020, Southwest Research Institute
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

#include <tesseract_collision/fcl/fcl_collision_object_wrapper.h>

namespace tesseract_collision::tesseract_collision_fcl
{
FCLCollisionObjectWrapper::FCLCollisionObjectWrapper(const std::shared_ptr<fcl::CollisionGeometry<double>>& cgeom)
  : fcl::CollisionObject<double>(cgeom)
{
}

FCLCollisionObjectWrapper::FCLCollisionObjectWrapper(const std::shared_ptr<fcl::CollisionGeometry<double>>& cgeom,
                                                     const fcl::Transform3<double>& tf)
  : fcl::CollisionObject<double>(cgeom, tf)
{
}

FCLCollisionObjectWrapper::FCLCollisionObjectWrapper(const std::shared_ptr<fcl::CollisionGeometry<double>>& cgeom,
                                                     const fcl::Matrix3<double>& R,
                                                     const fcl::Vector3<double>& T)
  : fcl::CollisionObject<double>(cgeom, R, T)
{
}

void FCLCollisionObjectWrapper::setContactDistanceThreshold(double contact_distance)
{
  contact_distance_ = contact_distance;
  updateAABB();
}

double FCLCollisionObjectWrapper::getContactDistanceThreshold() const { return contact_distance_; }

void FCLCollisionObjectWrapper::updateAABB()
{
  if (t.linear().isIdentity())
  {
    aabb = translate(cgeom->aabb_local, t.translation());
    fcl::Vector3<double> delta = fcl::Vector3<double>::Constant(contact_distance_);
    aabb.min_ -= delta;
    aabb.max_ += delta;
  }
  else
  {
    fcl::Vector3<double> center = t * cgeom->aabb_center;
    fcl::Vector3<double> delta = fcl::Vector3<double>::Constant(cgeom->aabb_radius + contact_distance_);
    aabb.min_ = center - delta;
    aabb.max_ = center + delta;
  }
}

}  // namespace tesseract_collision::tesseract_collision_fcl
