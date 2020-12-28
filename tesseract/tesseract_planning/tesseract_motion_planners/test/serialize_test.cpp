/**
 * @file utils_test.cpp
 * @brief
 *
 * @author Matthew Powelson
 * @date June 15, 2020
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
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract_motion_planners/trajopt/profile/trajopt_default_plan_profile.h>
#include <tesseract_motion_planners/trajopt/profile/trajopt_default_composite_profile.h>
#include <tesseract_motion_planners/trajopt/serialize.h>
#include <tesseract_motion_planners/trajopt/deserialize.h>

#include <tesseract_motion_planners/ompl/profile/ompl_default_plan_profile.h>
#include <tesseract_motion_planners/ompl/serialize.h>
#include <tesseract_motion_planners/ompl/deserialize.h>

#include <tesseract_motion_planners/descartes/profile/descartes_default_plan_profile.h>
#include <tesseract_motion_planners/descartes/serialize.h>
#include <tesseract_motion_planners/descartes/deserialize.h>

#include <tesseract_common/utils.h>

using namespace tesseract_planning;

TrajOptDefaultCompositeProfile getTrajOptCompositeProfile()
{
  TrajOptDefaultCompositeProfile composite_profile;

  CollisionCostConfig collision_cost_config;
  composite_profile.collision_cost_config = collision_cost_config;

  CollisionConstraintConfig collision_constraint_config;
  composite_profile.collision_constraint_config = collision_constraint_config;

  composite_profile.velocity_coeff = Eigen::VectorXd::Ones(6) * 10;
  composite_profile.acceleration_coeff = Eigen::VectorXd::Ones(6) * 10;
  composite_profile.jerk_coeff = Eigen::VectorXd::Ones(6) * 10;

  composite_profile.smooth_velocities = false;

  return composite_profile;
}

TrajOptDefaultPlanProfile getTrajOptPlanProfile()
{
  TrajOptDefaultPlanProfile plan_profile;

  plan_profile.cartesian_coeff = Eigen::VectorXd::Ones(6) * 10;
  plan_profile.joint_coeff = Eigen::VectorXd::Ones(6) * 9;

  plan_profile.term_type = trajopt::TermType::TT_COST;

  return plan_profile;
}

OMPLDefaultPlanProfile getOMPLPlanProfile()
{
  OMPLDefaultPlanProfile ompl_profile;

  ompl_profile.simplify = true;

  ompl_profile.planners.push_back(std::make_shared<const SBLConfigurator>());

  ompl_profile.planners.push_back(std::make_shared<const ESTConfigurator>());

  ompl_profile.planners.push_back(std::make_shared<const LBKPIECE1Configurator>());

  ompl_profile.planners.push_back(std::make_shared<const BKPIECE1Configurator>());

  ompl_profile.planners.push_back(std::make_shared<const KPIECE1Configurator>());

  ompl_profile.planners.push_back(std::make_shared<const BiTRRTConfigurator>());

  ompl_profile.planners.push_back(std::make_shared<const RRTConfigurator>());

  ompl_profile.planners.push_back(std::make_shared<const RRTConnectConfigurator>());

  ompl_profile.planners.push_back(std::make_shared<const RRTstarConfigurator>());

  ompl_profile.planners.push_back(std::make_shared<const TRRTConfigurator>());

  ompl_profile.planners.push_back(std::make_shared<const PRMConfigurator>());

  ompl_profile.planners.push_back(std::make_shared<const PRMstarConfigurator>());

  ompl_profile.planners.push_back(std::make_shared<const LazyPRMstarConfigurator>());

  ompl_profile.planners.push_back(std::make_shared<const SPARSConfigurator>());

  return ompl_profile;
}

DescartesDefaultPlanProfile<double> genDescartesPlanProfile()
{
  DescartesDefaultPlanProfile<double> descartes_profile;

  descartes_profile.enable_edge_collision = true;

  return descartes_profile;
}

TEST(TesseractMotionPlannersTrajoptSerializeUnit, SerializeTrajoptDefaultCompositeToXml)  // NOLINT
{
  // Write program to file
  TrajOptDefaultCompositeProfile comp_profile = getTrajOptCompositeProfile();
  EXPECT_TRUE(toXMLFile(comp_profile, tesseract_common::getTempPath() + "trajopt_default_composite_example_input.xml"));

  // Import file
  TrajOptDefaultCompositeProfile imported_comp_profile =
      trajOptCompositeFromXMLFile(tesseract_common::getTempPath() + "trajopt_default_composite_example_input.xml");

  // Re-write file and compare changed from default term
  EXPECT_TRUE(toXMLFile(imported_comp_profile,
                        tesseract_common::getTempPath() + "trajopt_default_composite_example_input2.xml"));
  EXPECT_TRUE(comp_profile.smooth_velocities == imported_comp_profile.smooth_velocities);
}

TEST(TesseractMotionPlannersTrajoptSerializeUnit, SerializeTrajoptDefaultPlanToXml)  // NOLINT
{
  // Write program to file
  TrajOptDefaultPlanProfile plan_profile = getTrajOptPlanProfile();
  EXPECT_TRUE(toXMLFile(plan_profile, tesseract_common::getTempPath() + "trajopt_default_plan_example_input.xml"));

  // Import file
  TrajOptDefaultPlanProfile imported_plan_profile = trajOptPlanFromXMLFile(tesseract_common::getTempPath() + "trajopt_"
                                                                                                             "default_"
                                                                                                             "plan_"
                                                                                                             "example_"
                                                                                                             "input."
                                                                                                             "xml");

  // Re-write file and compare changed from default term
  EXPECT_TRUE(
      toXMLFile(imported_plan_profile, tesseract_common::getTempPath() + "trajopt_default_plan_example_input2.xml"));
  EXPECT_TRUE(plan_profile.term_type == imported_plan_profile.term_type);
}

TEST(TesseractMotionPlannersOMPLSerializeUnit, SerializeOMPLDefaultPlanToXml)  // NOLINT
{
  // Write program to file
  OMPLDefaultPlanProfile plan_profile = getOMPLPlanProfile();
  EXPECT_TRUE(toXMLFile(plan_profile, tesseract_common::getTempPath() + "ompl_default_plan_example_input.xml"));

  // Import file
  OMPLDefaultPlanProfile imported_plan_profile = omplPlanFromXMLFile(tesseract_common::getTempPath() + "ompl_default_"
                                                                                                       "plan_example_"
                                                                                                       "input.xml");

  // Re-write file and compare changed from default term
  EXPECT_TRUE(
      toXMLFile(imported_plan_profile, tesseract_common::getTempPath() + "ompl_default_plan_example_input2.xml"));
  EXPECT_TRUE(plan_profile.simplify == imported_plan_profile.simplify);
}

TEST(TesseractMotionPlannersDescartesSerializeUnit, SerializeDescartesDefaultPlanToXml)  // NOLINT
{
  // Write program to file
  DescartesDefaultPlanProfile<double> plan_profile = genDescartesPlanProfile();
  EXPECT_TRUE(toXMLFile(plan_profile, tesseract_common::getTempPath() + "descartes_default_plan_example_input.xml"));

  // Import file
  DescartesDefaultPlanProfile<double> imported_plan_profile =
      descartesPlanFromXMLFile(tesseract_common::getTempPath() + "descartes_default_plan_example_input.xml");

  // Re-write file and compare changed from default term
  EXPECT_TRUE(
      toXMLFile(imported_plan_profile, tesseract_common::getTempPath() + "descartes_default_plan_example_input2.xml"));
  EXPECT_TRUE(plan_profile.enable_edge_collision == imported_plan_profile.enable_edge_collision);
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);

  return RUN_ALL_TESTS();
}
