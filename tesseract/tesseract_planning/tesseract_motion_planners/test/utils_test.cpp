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
#include <boost/filesystem.hpp>

TESSERACT_COMMON_IGNORE_WARNINGS_POP
#include <tesseract/tesseract.h>
#include <tesseract_motion_planners/core/utils.h>
#include <tesseract_command_language/plan_instruction.h>

using namespace tesseract;
using namespace tesseract_planning;

bool DEBUG = false;

std::string locateResource(const std::string& url)
{
  std::string mod_url = url;
  if (url.find("package://tesseract_support") == 0)
  {
    mod_url.erase(0, strlen("package://tesseract_support"));
    size_t pos = mod_url.find('/');
    if (pos == std::string::npos)
    {
      return std::string();
    }

    std::string package = mod_url.substr(0, pos);
    mod_url.erase(0, pos);
    std::string package_path = std::string(TESSERACT_SUPPORT_DIR);

    if (package_path.empty())
    {
      return std::string();
    }

    mod_url = package_path + mod_url;
  }

  return mod_url;
}

class TesseractPlanningUtilsUnit : public ::testing::Test
{
protected:
  Tesseract::Ptr tesseract_ptr_;

  void SetUp() override
  {
    tesseract_scene_graph::ResourceLocator::Ptr locator =
        std::make_shared<tesseract_scene_graph::SimpleResourceLocator>(locateResource);
    Tesseract::Ptr tesseract = std::make_shared<Tesseract>();
    boost::filesystem::path urdf_path(std::string(TESSERACT_SUPPORT_DIR) + "/urdf/lbr_iiwa_14_r820.urdf");
    boost::filesystem::path srdf_path(std::string(TESSERACT_SUPPORT_DIR) + "/urdf/lbr_iiwa_14_r820.srdf");
    EXPECT_TRUE(tesseract->init(urdf_path, srdf_path, locator));
    tesseract_ptr_ = tesseract;
  }
};

TEST_F(TesseractPlanningUtilsUnit, toCartesianWaypoint)  // NOLINT
{
  EXPECT_TRUE(true);
}

TEST_F(TesseractPlanningUtilsUnit, toJointWaypoint)  // NOLINT
{
  EXPECT_TRUE(true);
}

TEST_F(TesseractPlanningUtilsUnit, Flatten)  // NOLINT
{
  // Create a composite
  CompositeInstruction composite;
  composite.setDescription("Flatten Unit: Composite");
  std::size_t i_max = 4;
  std::size_t j_max = 3;
  std::size_t k_max = 2;

  for (std::size_t i = 0; i < i_max; i++)
  {
    CompositeInstruction sub_composite;
    sub_composite.setDescription("sub_composite_" + std::to_string(i));
    for (std::size_t j = 0; j < j_max; j++)
    {
      CompositeInstruction sub_sub_composite;
      sub_sub_composite.setDescription("sub_sub_composite_" + std::to_string(j));
      for (std::size_t k = 0; k < k_max; k++)
      {
        Waypoint wp = CartesianWaypoint(Eigen::Isometry3d::Identity());
        PlanInstruction instruction(wp, PlanInstructionType::LINEAR);
        instruction.setDescription("instruction_" + std::to_string(i) + "_" + std::to_string(j) + "_" +
                                   std::to_string(k));
        sub_sub_composite.push_back(instruction);
      }
      sub_composite.push_back(sub_sub_composite);
    }
    composite.push_back(sub_composite);
  }

  // Flatten(composite);
  {
    if (DEBUG)
      composite.print();

    // Flatten the composite
    std::vector<std::reference_wrapper<Instruction>> flattened = Flatten(composite);
    EXPECT_EQ(flattened.size(), i_max * j_max * k_max);

    // Now change something in the flattened composite
    int num_composites = 0;
    for (std::size_t i = 0; i < flattened.size(); i++)
    {
      if (isCompositeInstruction(flattened[i].get().getType()))
        num_composites++;
      flattened[i].get().setDescription("test_" + std::to_string(i));
    }
    EXPECT_EQ(num_composites, 0);

    if (DEBUG)
      composite.print();

    // Now make sure the original changed
    std::size_t cumulative = 0;
    for (std::size_t i = 0; i < i_max; i++)
    {
      for (std::size_t j = 0; j < j_max; j++)
      {
        for (std::size_t k = 0; k < k_max; k++)
        {
          EXPECT_EQ(
              "test_" + std::to_string(cumulative),
              composite[i].cast<CompositeInstruction>()->at(j).cast<CompositeInstruction>()->at(k).getDescription());
          cumulative++;
        }
      }
    }
  }

  // Flatten(composite, true);
  {
    if (DEBUG)
      composite.print();

    // Flatten the composite keeping the composite instructions
    std::vector<std::reference_wrapper<Instruction>> flattened = Flatten(composite, true);
    EXPECT_EQ(flattened.size(), i_max * j_max * k_max + 16);  // Add 16 for the composite instructions

    // Now change something in the flattened composite
    int num_composites = 0;
    for (std::size_t i = 0; i < flattened.size(); i++)
    {
      if (isCompositeInstruction(flattened[i].get().getType()))
        num_composites++;
      flattened[i].get().setDescription("test_" + std::to_string(i));
    }
    EXPECT_EQ(num_composites, 16);

    if (DEBUG)
    {
      std::cout << "----- Flattened -----" << std::endl;
      for (auto& i : flattened)
        i.get().print();
      std::cout << "----- Composite -----" << std::endl;
      composite.print();
    }

    // Now make sure the original changed
    std::size_t cumulative = 0;
    for (std::size_t i = 0; i < i_max; i++)
    {
      EXPECT_EQ("test_" + std::to_string(cumulative), composite[i].cast<CompositeInstruction>()->getDescription());
      cumulative++;
      for (std::size_t j = 0; j < j_max; j++)
      {
        EXPECT_EQ("test_" + std::to_string(cumulative),
                  composite[i].cast<CompositeInstruction>()->at(j).cast<CompositeInstruction>()->getDescription());
        cumulative++;
        for (std::size_t k = 0; k < k_max; k++)
        {
          EXPECT_EQ(
              "test_" + std::to_string(cumulative),
              composite[i].cast<CompositeInstruction>()->at(j).cast<CompositeInstruction>()->at(k).getDescription());
          cumulative++;
        }
      }
    }
  }
}

TEST_F(TesseractPlanningUtilsUnit, FlattenToPattern)  // NOLINT
{
  // Create a composite
  CompositeInstruction composite;
  composite.setDescription("FlattenToPattern Unit: Composite");
  CompositeInstruction pattern;
  std::size_t i_max = 4;
  std::size_t j_max = 3;
  std::size_t k_max = 2;

  // These loops create a nested set of composites. Note that the pattern does not have the last nesting
  for (std::size_t i = 0; i < i_max; i++)
  {
    CompositeInstruction sub_composite;
    sub_composite.setDescription("sub_composite_" + std::to_string(i));
    CompositeInstruction sub_pattern;
    sub_pattern.setDescription("sub_pattern_" + std::to_string(i));

    for (std::size_t j = 0; j < j_max; j++)
    {
      CompositeInstruction sub_sub_composite;
      sub_sub_composite.setDescription("sub_sub_composite_" + std::to_string(j));

      Waypoint wp = CartesianWaypoint(Eigen::Isometry3d::Identity());
      PlanInstruction pattern_instruction(wp, PlanInstructionType::LINEAR);
      pattern_instruction.setDescription("pattern_instruction_" + std::to_string(i) + "_" + std::to_string(j));
      sub_pattern.push_back(pattern_instruction);
      for (std::size_t k = 0; k < k_max; k++)
      {
        Waypoint wp = CartesianWaypoint(Eigen::Isometry3d::Identity());
        PlanInstruction instruction(wp, PlanInstructionType::LINEAR);
        instruction.setDescription("instruction_" + std::to_string(i) + "_" + std::to_string(j) + "_" +
                                   std::to_string(k));
        sub_sub_composite.push_back(instruction);
      }
      sub_composite.push_back(sub_sub_composite);
    }
    composite.push_back(sub_composite);
    pattern.push_back(sub_pattern);
  }

  // FlattenToPattern(composite, pattern)
  {
    if (DEBUG)
    {
      std::cout << "Composite: " << std::endl;
      composite.print();
      std::cout << "Pattern: " << std::endl;
      pattern.print();
    }
    // Flatten the composite
    std::vector<std::reference_wrapper<Instruction>> flattened = FlattenToPattern(composite, pattern);
    EXPECT_EQ(flattened.size(), i_max * j_max);

    // Now change something in the flattened composite
    int num_composites = 0;
    for (std::size_t i = 0; i < flattened.size(); i++)
    {
      if (isCompositeInstruction(flattened[i].get().getType()))
        num_composites++;
      flattened[i].get().setDescription("test_" + std::to_string(i));
    }
    EXPECT_EQ(num_composites, 12);

    if (DEBUG)
    {
      std::cout << "Flattened Composite: " << std::endl;
      for (const auto& i : flattened)
        i.get().print();
    }

    // Now make sure the original changed
    std::size_t cumulative = 0;
    for (std::size_t i = 0; i < i_max; i++)
    {
      for (std::size_t j = 0; j < j_max; j++)
      {
        EXPECT_EQ("test_" + std::to_string(cumulative),
                  composite[i].cast<CompositeInstruction>()->at(j).getDescription());
        cumulative++;
      }
    }
  }

  // FlattenToPattern(composte, pattern, true)
  {
    if (DEBUG)
    {
      std::cout << "Composite after changing description: " << std::endl;
      composite.print();
    }

    if (DEBUG)
    {
      std::cout << "Composite: " << std::endl;
      composite.print();
      std::cout << "Pattern: " << std::endl;
      pattern.print();
    }
    // Flatten the composite
    std::vector<std::reference_wrapper<Instruction>> flattened = FlattenToPattern(composite, pattern, true);
    EXPECT_EQ(flattened.size(),
              i_max * j_max + 4);  // Add 4 for the extra composite instructions that would be flattened

    // Now change something in the flattened composite
    int num_composites = 0;
    for (std::size_t i = 0; i < flattened.size(); i++)
    {
      if (isCompositeInstruction(flattened[i].get().getType()))
        num_composites++;
      flattened[i].get().setDescription("test_" + std::to_string(i));
    }
    EXPECT_EQ(num_composites, 16);

    if (DEBUG)
    {
      std::cout << "Flattened Composite: " << std::endl;
      for (const auto& i : flattened)
        i.get().print();
    }

    // Now make sure the original changed
    std::size_t cumulative = 0;
    for (std::size_t i = 0; i < i_max; i++)
    {
      EXPECT_EQ("test_" + std::to_string(cumulative), composite[i].cast<CompositeInstruction>()->getDescription());
      cumulative++;
      for (std::size_t j = 0; j < j_max; j++)
      {
        EXPECT_EQ("test_" + std::to_string(cumulative),
                  composite[i].cast<CompositeInstruction>()->at(j).getDescription());
        cumulative++;
      }
    }

    if (DEBUG)
    {
      std::cout << "Composite after changing description: " << std::endl;
      composite.print();
    }
  }
}

TEST_F(TesseractPlanningUtilsUnit, GenerateSeed)  // NOLINT
{
  EXPECT_TRUE(true);
}
int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);

  return RUN_ALL_TESTS();
}
