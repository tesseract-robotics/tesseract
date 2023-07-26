/*
 * Software License Agreement (Apache License)
 *
 * Copyright (c) 2023, Southwest Research Institute
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */
#ifndef TESSERACT_KINEMATICS_ABB_IRB2400_IKFAST_KINEMATICS_H
#define TESSERACT_KINEMATICS_ABB_IRB2400_IKFAST_KINEMATICS_H

#include <tesseract_kinematics/ikfast/ikfast_factory_boilerplate.h>
#include "abb_irb2400_ikfast_solver.hpp"

TESSERACT_ADD_INV_KIN_PLUGIN(tesseract_kinematics::IKFastInvKinFactory, AbbIRB2400Kinematics)

#endif  // TESSERACT_KINEMATICS_ABB_IRB2400_IKFAST_KINEMATICS_H
