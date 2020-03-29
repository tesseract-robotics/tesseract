/**
 * @file types.i
 * @brief SWIG interface file for tesseract_environment/core/types.h
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
#include <tesseract_environment/core/types.h>
%}

%shared_ptr(tesseract_environment::EnvState)
%shared_ptr(tesseract_environment::AdjacencyMapPair)
%shared_ptr(tesseract_environment::AdjacencyMap)

namespace tesseract_environment
{

struct EnvState
{
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  using Ptr = std::shared_ptr<EnvState>;
  using ConstPtr = std::shared_ptr<const EnvState>;

  EnvState() : transforms(link_transforms) {}
  virtual ~EnvState() = default;
  EnvState(const EnvState& other)
    : joints(other.joints)
    , link_transforms(other.link_transforms)
    , transforms(link_transforms)
    , joint_transforms(other.joint_transforms)
  {
  }

  EnvState& operator=(const EnvState& other)
  {
    joints = other.joints;
    link_transforms = other.link_transforms;
    transforms = link_transforms;
    joint_transforms = other.joint_transforms;
    return (*this);
  }

  EnvState(EnvState&& other) noexcept
    : joints(std::move(other.joints))
    , link_transforms(std::move(other.link_transforms))
    , transforms(link_transforms)
    , joint_transforms(std::move(other.joint_transforms))
  {
  }

  EnvState& operator=(EnvState&& other) noexcept
  {
    joints = std::move(other.joints);
    link_transforms = std::move(other.link_transforms);
    transforms = link_transforms;
    joint_transforms = std::move(other.joint_transforms);
    return (*this);
  }

  /**  @brief The joint values used for calculating the joint and link transforms */
  std::unordered_map<std::string, double> joints;

  /** @brief The link transforms in world coordinate system */
  tesseract_common::TransformMap link_transforms;

  /** @brief (DEPRECATED) The link transforms in world coordinate system */
  DEPRECATED("Use member variable link_transforms.") tesseract_common::TransformMap& transforms;

  /** @brief The joint transforms in world coordinate system */
  tesseract_common::TransformMap joint_transforms;
};

struct AdjacencyMapPair
{
  using Ptr = std::shared_ptr<AdjacencyMapPair>;
  using ConstPtr = std::shared_ptr<const AdjacencyMapPair>;

  std::string link_name;
  Eigen::Isometry3d transform;
};

class AdjacencyMap
{
public:
  using Ptr = std::shared_ptr<AdjacencyMap>;
  using ConstPtr = std::shared_ptr<const AdjacencyMap>;

  AdjacencyMap(const tesseract_scene_graph::SceneGraph::ConstPtr& scene_graph,
               const std::vector<std::string>& active_links,
               const tesseract_common::TransformMap& state);

  virtual ~AdjacencyMap();

  const std::vector<std::string>& getActiveLinkNames() const;

  AdjacencyMapPair::ConstPtr getLinkMapping(const std::string& link_name) const;
};

}  // namespace tesseract_environment
