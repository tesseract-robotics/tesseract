/**
 * @file abb_opw_fixture.h
 * @brief OPW/KDL solver factories for the ABB IRB2400 + tool positioner test fixture.
 *
 * @details Shared by rtp_kinematics_unit.cpp and the RTP-vs-KDL benchmark so both measure and
 * verify the same robot. If the OPW parameters below drift from the URDF the unit tests fail
 * loudly, but a benchmark carrying its own copy would silently report numbers for a different
 * manipulator - which is why these live in one place.
 *
 * Deliberately not folded into kinematics_test_utils.h: that header is included by test targets
 * which do not link tesseract::kinematics_opw, and pulling the OPW headers in would break their
 * builds. Include this only from targets linking both kinematics_opw and kinematics_kdl.
 *
 * @author Roelof Oomen
 * @date May 1, 2026
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
 * @par
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */
#ifndef TESSERACT_KINEMATICS_ABB_OPW_FIXTURE_H
#define TESSERACT_KINEMATICS_ABB_OPW_FIXTURE_H

#include <tesseract/common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <cmath>
#include <memory>
#include <string>
#include <opw_kinematics/opw_parameters.h>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract/kinematics/forward_kinematics.h>
#include <tesseract/kinematics/inverse_kinematics.h>
#include <tesseract/kinematics/kdl/kdl_fwd_kin_chain.h>
#include <tesseract/kinematics/opw/opw_inv_kin.h>
#include <tesseract/scene_graph/fwd.h>

namespace tesseract::kinematics::test_suite
{
/** @brief Base of the ABB IRB2400 manipulator chain. */
inline const std::string ABB_BASE_LINK = "base_link";  // NOLINT(cert-err58-cpp)
/** @brief Tip of the manipulator chain, and base of the tool positioner chain. */
inline const std::string ABB_MANIP_TIP_LINK = "tool0";  // NOLINT(cert-err58-cpp)
/** @brief Tip of the tool positioner chain - the frame RTP target poses are expressed for. */
inline const std::string ABB_TOOL_TIP_LINK = "tool_tip";  // NOLINT(cert-err58-cpp)

/** @brief OPW parameters for the ABB IRB2400 in support/urdf/abb_irb2400.urdf. */
inline opw_kinematics::Parameters<double> getOPWKinematicsParamABB()
{
  opw_kinematics::Parameters<double> opw_params;
  opw_params.a1 = (0.100);
  opw_params.a2 = (-0.135);
  opw_params.b = (0.000);
  opw_params.c1 = (0.615);
  opw_params.c2 = (0.705);
  opw_params.c3 = (0.755);
  opw_params.c4 = (0.085);
  opw_params.offsets[2] = -M_PI / 2.0;
  return opw_params;
}

/** @brief Closed-form OPW solver over the manipulator chain (ABB_BASE_LINK -> ABB_MANIP_TIP_LINK). */
inline InverseKinematics::UPtr makeOPWInvKinABB(const tesseract::scene_graph::SceneGraph& scene_graph)
{
  auto robot_fwd_kin = std::make_unique<KDLFwdKinChain>(scene_graph, ABB_BASE_LINK, ABB_MANIP_TIP_LINK);
  return std::make_unique<OPWInvKin>(getOPWKinematicsParamABB(),
                                     robot_fwd_kin->getBaseLinkName(),
                                     robot_fwd_kin->getTipLinkNames()[0],
                                     robot_fwd_kin->getJointNames());
}

/** @brief Forward kinematics over the tool positioner chain (ABB_MANIP_TIP_LINK -> ABB_TOOL_TIP_LINK). */
inline ForwardKinematics::UPtr makeToolFwdKinABB(const tesseract::scene_graph::SceneGraph& scene_graph)
{
  return std::make_unique<KDLFwdKinChain>(scene_graph, ABB_MANIP_TIP_LINK, ABB_TOOL_TIP_LINK);
}

}  // namespace tesseract::kinematics::test_suite

#endif  // TESSERACT_KINEMATICS_ABB_OPW_FIXTURE_H
