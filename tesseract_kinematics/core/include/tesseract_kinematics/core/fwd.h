/**
 * @file fwd.h
 * @brief Tesseract Kinematics Forward Declarations.
 *
 * @author Levi Armstrong
 * @date February 21, 2024
 * @version TODO
 * @bug No known bugs
 *
 * @copyright Copyright (c) 2024, Levi Armstrong
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
#ifndef TESSERACT_KINEMATICS_FWD_H
#define TESSERACT_KINEMATICS_FWD_H

namespace tesseract_kinematics
{
class ForwardKinematics;
class InverseKinematics;
class JointGroup;
struct KinGroupIKInput;
class KinematicGroup;
class InvKinFactory;
class FwdKinFactory;
class KinematicsPluginFactory;
class REPInvKinFactory;
class REPInvKin;
class ROPInvKinFactory;
class ROPInvKin;
struct URParameters;
struct ManipulabilityEllipsoid;
struct Manipulability;
}  // namespace tesseract_kinematics

#endif  // TESSERACT_KINEMATICS_FWD_H
