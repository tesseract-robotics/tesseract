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
