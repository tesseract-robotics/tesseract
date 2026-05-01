/**
 * @file rtp_factory.h
 * @brief Robot with Tool Positioner Inverse kinematics Factory.
 *
 * @copyright Copyright (c) 2026
 *
 * @par License
 * Software License Agreement (Apache License)
 * @par
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 * http://www.apache.org/licenses/LICENSE-2.0
 */
#ifndef TESSERACT_KINEMATICS_RTP_FACTORY_H
#define TESSERACT_KINEMATICS_RTP_FACTORY_H

#include <tesseract/kinematics/kinematics_plugin_factory.h>
#include <boost_plugin_loader/macros.h>

namespace tesseract::kinematics
{
class RTPInvKinFactory : public InvKinFactory
{
  std::unique_ptr<InverseKinematics> create(const std::string& solver_name,
                                            const tesseract::scene_graph::SceneGraph& scene_graph,
                                            const tesseract::scene_graph::SceneState& scene_state,
                                            const KinematicsPluginFactory& plugin_factory,
                                            const YAML::Node& config) const override final;
};

PLUGIN_ANCHOR_DECL(RTPInvKinFactoriesAnchor)

}  // namespace tesseract::kinematics

#endif  // TESSERACT_KINEMATICS_RTP_FACTORY_H
