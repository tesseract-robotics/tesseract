/**
 * @file continuous_contact_manager.i
 * @brief SWIG interface file for tesseract_collision/continuous_contact_manager.h
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
#include <tesseract_collision/core/continuous_contact_manager.h>
%}

%include "types.i"

%shared_ptr(tesseract_collision::ContinuousContactManager)

namespace tesseract_collision
{
class ContinuousContactManager
{
public:
  
  using Ptr = std::shared_ptr<ContinuousContactManager>;
  using ConstPtr = std::shared_ptr<const ContinuousContactManager>;

  virtual ~ContinuousContactManager();

  virtual std::shared_ptr<ContinuousContactManager> clone() const = 0;

  virtual bool addCollisionObject(const std::string& name,
                                  const int& mask_id,
                                  const CollisionShapesConst& shapes,
                                  const tesseract_common::VectorIsometry3d& shape_poses,
                                  bool enabled = true) = 0;

  virtual const CollisionShapesConst& getCollisionObjectGeometries(const std::string& name) const = 0;

  virtual const tesseract_common::VectorIsometry3d&
  getCollisionObjectGeometriesTransforms(const std::string& name) const = 0;

  virtual bool hasCollisionObject(const std::string& name) const = 0;

  
  virtual bool removeCollisionObject(const std::string& name) = 0;

  virtual bool enableCollisionObject(const std::string& name) = 0;

  virtual bool disableCollisionObject(const std::string& name) = 0;

  virtual void setCollisionObjectsTransform(const std::string& name, const Eigen::Isometry3d& pose) = 0;

  virtual void setCollisionObjectsTransform(const std::vector<std::string>& names,
                                            const tesseract_common::VectorIsometry3d& poses) = 0;

  virtual void setCollisionObjectsTransform(const tesseract_common::TransformMap& transforms) = 0;

  virtual void setActiveCollisionObjects(const std::vector<std::string>& names) = 0;

  virtual const std::vector<std::string> getActiveCollisionObjects() const = 0;

  virtual void setContactDistanceThreshold(double contact_distance) = 0;

  virtual double getContactDistanceThreshold() const = 0;

  // TODO: IsContactAllowedFn
  //virtual void setIsContactAllowedFn(IsContactAllowedFn fn) = 0;

  //virtual IsContactAllowedFn getIsContactAllowedFn() const = 0;
  
  //virtual void contactTest(ContactResultMap& collisions, const ContactTestType& type) = 0;
  %extend {
    tesseract_collision::ContactResultMap contactTest(const tesseract_collision::ContactTestType& type)
    {
        tesseract_collision::ContactResultMap contacts;
        $self->contactTest(contacts, type);
        return contacts;
    }
  }
};

}  // namespace tesseract_collision
