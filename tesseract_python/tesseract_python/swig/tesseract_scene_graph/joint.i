/**
 * @file joint.i
 * @brief SWIG interface file for tesseract_scene_graph/joint.h
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
#include <tesseract_scene_graph/joint.h>
%}

%shared_ptr(tesseract_scene_graph::JointDynamics)
%shared_ptr(tesseract_scene_graph::JointLimits)
%shared_ptr(tesseract_scene_graph::JointSafety)
%shared_ptr(tesseract_scene_graph::JointMimic)
%shared_ptr(tesseract_scene_graph::JointCalibration)
%shared_ptr(tesseract_scene_graph::Joint)
%shared_ptr(tesseract_scene_graph::Link)

namespace tesseract_scene_graph
{
class Link;

class JointDynamics
{
public:
  using Ptr = std::shared_ptr<JointDynamics>;
  using ConstPtr = std::shared_ptr<const JointDynamics>;

  JointDynamics() { this->clear(); }
  double damping;
  double friction;

  void clear();
};

class JointLimits
{
public:
  using Ptr = std::shared_ptr<JointLimits>;
  using ConstPtr = std::shared_ptr<const JointLimits>;

  JointLimits();
  double lower;
  double upper;
  double effort;
  double velocity;

  void clear();
};

class JointSafety
{
public:
  using Ptr = std::shared_ptr<JointSafety>;
  using ConstPtr = std::shared_ptr<const JointSafety>;

  JointSafety();

  double soft_upper_limit;
  double soft_lower_limit;
  double k_position;
  double k_velocity;

  void clear();
};

class JointCalibration
{
public:
  using Ptr = std::shared_ptr<JointCalibration>;
  using ConstPtr = std::shared_ptr<const JointCalibration>;

  JointCalibration();
  double reference_position;
  double rising, falling;

  void clear();
};

class JointMimic
{
public:
  using Ptr = std::shared_ptr<JointMimic>;
  using ConstPtr = std::shared_ptr<const JointMimic>;

  JointMimic() { this->clear(); }
  double offset;
  double multiplier;
  std::string joint_name;

  void clear();
};

enum class JointType
{
  UNKNOWN,
  REVOLUTE,
  CONTINUOUS,
  PRISMATIC,
  FLOATING,
  PLANAR,
  FIXED
};

class Joint
{
public:
  
  using Ptr = std::shared_ptr<Joint>;
  using ConstPtr = std::shared_ptr<const Joint>;

  Joint(std::string name) : name_(name) { this->clear(); }

  const std::string& getName() const { return name_; }

  JointType type;

  Eigen::Vector3d axis;

  std::string child_link_name;

  std::string parent_link_name;

  Eigen::Isometry3d parent_to_joint_origin_transform;

  JointDynamics::Ptr dynamics;

  JointLimits::Ptr limits;

  JointSafety::Ptr safety;

  JointCalibration::Ptr calibration;

  JointMimic::Ptr mimic;

  void clear();
};

}  // namespace tesseract_scene_graph
