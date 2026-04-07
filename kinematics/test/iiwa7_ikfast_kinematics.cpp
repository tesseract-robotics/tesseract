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
#include <tesseract/kinematics/ikfast/ikfast_factory_boilerplate.h>

#include <tesseract/common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include "iiwa7_ikfast_solver.hpp"
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract/common/schema_registration.h>
#include <tesseract/common/property_tree.h>

// NOLINTNEXTLINE(cppcoreguidelines-avoid-non-const-global-variables)
TESSERACT_ADD_INV_KIN_PLUGIN(tesseract::kinematics::IKFastInvKinFactory, iiwa7Kinematics)

static tesseract::common::PropertyTree ikFastInvKinSchema()
{
  return tesseract::common::PropertyTreeBuilder()
      .string("base_link")
      .required()
      .done()
      .string("tip_link")
      .required()
      .done()
      .integer("n_joints")
      .required()
      .minimum(1)
      .done()
      .build();
}

TESSERACT_SCHEMA_REGISTER(iiwa7Kinematics, ikFastInvKinSchema);
TESSERACT_SCHEMA_REGISTER_DERIVED_TYPE(tesseract::kinematics::InvKinFactory, iiwa7Kinematics);
