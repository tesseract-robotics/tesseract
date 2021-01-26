/**
 * @file kdl_fwd_kin_chain_factory.h
 * @brief Tesseract KDL Forward kinematics chain factory.
 *
 * @author Levi Armstrong
 * @date April 15, 2018
 * @version TODO
 * @bug No known bugs
 *
 * @copyright Copyright (c) 2013, Southwest Research Institute
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
#ifndef TESSERACT_KINEMATICS_KDL_FWD_KIN_CHAIN_FACTORY_H
#define TESSERACT_KINEMATICS_KDL_FWD_KIN_CHAIN_FACTORY_H
#include <tesseract_kinematics/core/forward_kinematics_factory.h>
#include <tesseract_kinematics/kdl/kdl_fwd_kin_chain.h>

#ifdef SWIG
%shared_ptr(tesseract_kinematics::KDLFwdKinChainFactory)
#endif  // SWIG

namespace tesseract_kinematics
{
class KDLFwdKinChainFactory : public ForwardKinematicsFactory
{
public:
  KDLFwdKinChainFactory() : name_(KDLFwdKinChain().getSolverName()) {}

  const std::string& getName() const override { return name_; }

  ForwardKinematicsFactoryType getType() const override { return ForwardKinematicsFactoryType::CHAIN; }

  ForwardKinematics::Ptr create(tesseract_scene_graph::SceneGraph::ConstPtr scene_graph,
                                const std::string& base_link,
                                const std::string& tip_link,
                                const std::string& name) const override
  {
    auto kin = std::make_shared<KDLFwdKinChain>();
    if (!kin->init(scene_graph, base_link, tip_link, name))
      return nullptr;

    return kin;
  }

  ForwardKinematics::Ptr create(tesseract_scene_graph::SceneGraph::ConstPtr scene_graph,
                                const std::vector<std::pair<std::string, std::string>>& chains,  // NOLINT
                                const std::string& name) const override
  {
    auto kin = std::make_shared<KDLFwdKinChain>();
    if (!kin->init(scene_graph, chains, name))
      return nullptr;

    return kin;
  }

private:
  std::string name_;
};

}  // namespace tesseract_kinematics
#endif  // TESSERACT_KINEMATICS_KDL_FWD_KIN_CHAIN_FACTORY_H
