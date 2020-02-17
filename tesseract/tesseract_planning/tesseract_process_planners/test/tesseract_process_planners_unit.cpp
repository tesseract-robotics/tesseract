/**
 * @file tesseract_process_planners_unit.cpp
 * @brief tests for the process planner generators. Several of the
 * generators are instantiated.
 *
 * @author Levi Armstrong
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

#include <tesseract_common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <gtest/gtest.h>
#include <algorithm>
#include <memory>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract_process_planners/generators/axial_approach_generator.h>
#include <tesseract_process_planners/generators/axial_departure_generator.h>
#include <tesseract_process_planners/generators/linear_transition_generator.h>
#include <tesseract_process_planners/generators/passthrough_process_generator.h>
#include <tesseract_process_planners/generators/passthrough_transition_generator.h>
#include <tesseract_process_planners/process_definition.h>
#include <tesseract_process_planners/process_planner.h>

TEST(TesseractProcessPlannersUnit, Instantiation)  // NOLINT
{
  using namespace tesseract_process_planners;

  AxialApproachGenerator generator1(Eigen::Isometry3d::Identity(), 5);
  AxialDepartureGenerator generator2(Eigen::Isometry3d::Identity(), 5);
  LinearTransitionGenerator generator3(5);
  PassthroughProcessGenerator generator4;
  PassthroughTransitionGenerator generator5;
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);

  return RUN_ALL_TESTS();
}
