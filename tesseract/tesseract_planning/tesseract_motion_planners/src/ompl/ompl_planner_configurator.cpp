/**
 * @file ompl_planner_configurator.cpp
 * @brief Tesseract OMPL planner configurators
 *
 * @author Levi Armstrong
 * @date February 1, 2020
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
#include <ompl/geometric/planners/sbl/SBL.h>
#include <ompl/geometric/planners/est/EST.h>
#include <ompl/geometric/planners/kpiece/LBKPIECE1.h>
#include <ompl/geometric/planners/kpiece/BKPIECE1.h>
#include <ompl/geometric/planners/kpiece/KPIECE1.h>
#include <ompl/geometric/planners/rrt/BiTRRT.h>
#include <ompl/geometric/planners/rrt/RRT.h>
#include <ompl/geometric/planners/rrt/RRTConnect.h>
#include <ompl/geometric/planners/rrt/RRTstar.h>
#include <ompl/geometric/planners/rrt/TRRT.h>
#include <ompl/geometric/planners/prm/PRM.h>
#include <ompl/geometric/planners/prm/PRMstar.h>
#include <ompl/geometric/planners/prm/LazyPRMstar.h>
#include <ompl/geometric/planners/prm/SPARS.h>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract_motion_planners/ompl/ompl_planner_configurator.h>
#include <tesseract_common/utils.h>

namespace tesseract_planning
{
SBLConfigurator::SBLConfigurator(const tinyxml2::XMLElement& xml_element)
{
  const tinyxml2::XMLElement* sbl_element = xml_element.FirstChildElement("SBL");
  const tinyxml2::XMLElement* range_element = sbl_element->FirstChildElement("Range");

  tinyxml2::XMLError status;

  if (range_element)
  {
    std::string range_string;
    status = tesseract_common::QueryStringText(range_element, range_string);
    if (status != tinyxml2::XML_NO_ATTRIBUTE && status != tinyxml2::XML_SUCCESS)
      throw std::runtime_error("OMPLConfigurator: SBL: Error parsing Range string");

    if (!tesseract_common::isNumeric(range_string))
      throw std::runtime_error("OMPLConfigurator: SBL: Range is not a numeric values.");

    tesseract_common::toNumeric<double>(range_string, range);
  }
}

ompl::base::PlannerPtr SBLConfigurator::create(ompl::base::SpaceInformationPtr si) const
{
  auto planner = std::make_shared<ompl::geometric::SBL>(si);
  planner->setRange(range);
  return planner;
}

OMPLPlannerType SBLConfigurator::getType() const { return OMPLPlannerType::SBL; }

tinyxml2::XMLElement* SBLConfigurator::toXML(tinyxml2::XMLDocument& doc) const
{
  tinyxml2::XMLElement* ompl_xml = doc.NewElement("SBL");

  tinyxml2::XMLElement* range_xml = doc.NewElement("Range");
  range_xml->SetText(range);
  ompl_xml->InsertEndChild(range_xml);

  return ompl_xml;
}

ESTConfigurator::ESTConfigurator(const tinyxml2::XMLElement& xml_element)
{
  const tinyxml2::XMLElement* est_element = xml_element.FirstChildElement("EST");
  const tinyxml2::XMLElement* range_element = est_element->FirstChildElement("Range");
  const tinyxml2::XMLElement* goal_bias_element = est_element->FirstChildElement("GoalBias");

  tinyxml2::XMLError status;

  if (range_element)
  {
    std::string range_string;
    status = tesseract_common::QueryStringText(range_element, range_string);
    if (status != tinyxml2::XML_NO_ATTRIBUTE && status != tinyxml2::XML_SUCCESS)
      throw std::runtime_error("OMPLConfigurator: EST: Error parsing Range string");

    if (!tesseract_common::isNumeric(range_string))
      throw std::runtime_error("OMPLConfigurator: EST: Range is not a numeric values.");

    tesseract_common::toNumeric<double>(range_string, range);
  }

  if (goal_bias_element)
  {
    std::string goal_bias_string;
    status = tesseract_common::QueryStringText(goal_bias_element, goal_bias_string);
    if (status != tinyxml2::XML_NO_ATTRIBUTE && status != tinyxml2::XML_SUCCESS)
      throw std::runtime_error("OMPLConfigurator: EST: Error parsing GoalBias string");

    if (!tesseract_common::isNumeric(goal_bias_string))
      throw std::runtime_error("OMPLConfigurator: EST: GoalBias is not a numeric values.");

    tesseract_common::toNumeric<double>(goal_bias_string, goal_bias);
  }
}

ompl::base::PlannerPtr ESTConfigurator::create(ompl::base::SpaceInformationPtr si) const
{
  auto planner = std::make_shared<ompl::geometric::EST>(si);
  planner->setRange(range);
  planner->setGoalBias(goal_bias);
  return planner;
}

OMPLPlannerType ESTConfigurator::getType() const { return OMPLPlannerType::EST; }

tinyxml2::XMLElement* ESTConfigurator::toXML(tinyxml2::XMLDocument& doc) const
{
  tinyxml2::XMLElement* ompl_xml = doc.NewElement("EST");

  tinyxml2::XMLElement* range_xml = doc.NewElement("Range");
  range_xml->SetText(range);
  ompl_xml->InsertEndChild(range_xml);

  tinyxml2::XMLElement* goal_bias_xml = doc.NewElement("GoalBias");
  goal_bias_xml->SetText(goal_bias);
  ompl_xml->InsertEndChild(goal_bias_xml);

  return ompl_xml;
}

LBKPIECE1Configurator::LBKPIECE1Configurator(const tinyxml2::XMLElement& xml_element)
{
  const tinyxml2::XMLElement* lbkpiece1_element = xml_element.FirstChildElement("LBKPIECE1");
  const tinyxml2::XMLElement* range_element = lbkpiece1_element->FirstChildElement("Range");
  const tinyxml2::XMLElement* border_fraction_element = lbkpiece1_element->FirstChildElement("BorderFraction");
  const tinyxml2::XMLElement* min_valid_path_fraction_element = lbkpiece1_element->FirstChildElement("MinValidPathFract"
                                                                                                     "ion");

  tinyxml2::XMLError status;

  if (range_element)
  {
    std::string range_string;
    status = tesseract_common::QueryStringText(range_element, range_string);
    if (status != tinyxml2::XML_NO_ATTRIBUTE && status != tinyxml2::XML_SUCCESS)
      throw std::runtime_error("OMPLConfigurator: LBKPIECE1: Error parsing Range string");

    if (!tesseract_common::isNumeric(range_string))
      throw std::runtime_error("OMPLConfigurator: LBKPIECE1: Range is not a numeric values.");

    tesseract_common::toNumeric<double>(range_string, range);
  }

  if (border_fraction_element)
  {
    std::string border_fraction_string;
    status = tesseract_common::QueryStringText(border_fraction_element, border_fraction_string);
    if (status != tinyxml2::XML_NO_ATTRIBUTE && status != tinyxml2::XML_SUCCESS)
      throw std::runtime_error("OMPLConfigurator: LBKPIECE1: Error parsing BorderFraction string");

    if (!tesseract_common::isNumeric(border_fraction_string))
      throw std::runtime_error("OMPLConfigurator: LBKPIECE1: BorderFraction is not a numeric values.");

    tesseract_common::toNumeric<double>(border_fraction_string, border_fraction);
  }

  if (min_valid_path_fraction_element)
  {
    std::string min_valid_path_fraction_string;
    status = tesseract_common::QueryStringText(min_valid_path_fraction_element, min_valid_path_fraction_string);
    if (status != tinyxml2::XML_NO_ATTRIBUTE && status != tinyxml2::XML_SUCCESS)
      throw std::runtime_error("OMPLConfigurator: LBKPIECE1: Error parsing MinValidPathFraction string");

    if (!tesseract_common::isNumeric(min_valid_path_fraction_string))
      throw std::runtime_error("OMPLConfigurator: LBKPIECE1: MinValidPathFraction is not a numeric values.");

    tesseract_common::toNumeric<double>(min_valid_path_fraction_string, min_valid_path_fraction);
  }
}

ompl::base::PlannerPtr LBKPIECE1Configurator::create(ompl::base::SpaceInformationPtr si) const
{
  auto planner = std::make_shared<ompl::geometric::LBKPIECE1>(si);
  planner->setRange(range);
  planner->setBorderFraction(border_fraction);
  planner->setMinValidPathFraction(min_valid_path_fraction);
  return planner;
}

OMPLPlannerType LBKPIECE1Configurator::getType() const { return OMPLPlannerType::LBKPIECE1; }

tinyxml2::XMLElement* LBKPIECE1Configurator::toXML(tinyxml2::XMLDocument& doc) const
{
  tinyxml2::XMLElement* ompl_xml = doc.NewElement("LBKPIECE1");

  tinyxml2::XMLElement* range_xml = doc.NewElement("Range");
  range_xml->SetText(range);
  ompl_xml->InsertEndChild(range_xml);

  tinyxml2::XMLElement* border_fraction_xml = doc.NewElement("BorderFraction");
  border_fraction_xml->SetText(border_fraction);
  ompl_xml->InsertEndChild(border_fraction_xml);

  tinyxml2::XMLElement* min_valid_path_fraction_xml = doc.NewElement("MinValidPathFraction");
  min_valid_path_fraction_xml->SetText(min_valid_path_fraction);
  ompl_xml->InsertEndChild(min_valid_path_fraction_xml);

  return ompl_xml;
}

BKPIECE1Configurator::BKPIECE1Configurator(const tinyxml2::XMLElement& xml_element)
{
  const tinyxml2::XMLElement* bkpiece1_element = xml_element.FirstChildElement("BKPIECE1");
  const tinyxml2::XMLElement* range_element = bkpiece1_element->FirstChildElement("Range");
  const tinyxml2::XMLElement* border_fraction_element = bkpiece1_element->FirstChildElement("BorderFraction");
  const tinyxml2::XMLElement* failed_expansion_score_factor_element = bkpiece1_element->FirstChildElement("FailedExpans"
                                                                                                          "ionScoreFact"
                                                                                                          "or");
  const tinyxml2::XMLElement* min_valid_path_fraction_element = bkpiece1_element->FirstChildElement("MinValidPathFracti"
                                                                                                    "on");

  tinyxml2::XMLError status;

  if (range_element)
  {
    std::string range_string;
    status = tesseract_common::QueryStringText(range_element, range_string);
    if (status != tinyxml2::XML_NO_ATTRIBUTE && status != tinyxml2::XML_SUCCESS)
      throw std::runtime_error("OMPLConfigurator: BKPIECE1: Error parsing Range string");

    if (!tesseract_common::isNumeric(range_string))
      throw std::runtime_error("OMPLConfigurator: BKPIECE1: Range is not a numeric values.");

    tesseract_common::toNumeric<double>(range_string, range);
  }

  if (border_fraction_element)
  {
    std::string border_fraction_string;
    status = tesseract_common::QueryStringText(border_fraction_element, border_fraction_string);
    if (status != tinyxml2::XML_NO_ATTRIBUTE && status != tinyxml2::XML_SUCCESS)
      throw std::runtime_error("OMPLConfigurator: BKPIECE1: Error parsing BorderFraction string");

    if (!tesseract_common::isNumeric(border_fraction_string))
      throw std::runtime_error("OMPLConfigurator: BKPIECE1: BorderFraction is not a numeric values.");

    tesseract_common::toNumeric<double>(border_fraction_string, border_fraction);
  }

  if (failed_expansion_score_factor_element)
  {
    std::string failed_expansion_score_factor_string;
    status =
        tesseract_common::QueryStringText(failed_expansion_score_factor_element, failed_expansion_score_factor_string);
    if (status != tinyxml2::XML_NO_ATTRIBUTE && status != tinyxml2::XML_SUCCESS)
      throw std::runtime_error("OMPLConfigurator: BKPIECE1: Error parsing FailedExpansionScoreFactor string");

    if (!tesseract_common::isNumeric(failed_expansion_score_factor_string))
      throw std::runtime_error("OMPLConfigurator: BKPIECE1: FailedExpansionScoreFactor is not a numeric values.");

    tesseract_common::toNumeric<double>(failed_expansion_score_factor_string, failed_expansion_score_factor);
  }

  if (min_valid_path_fraction_element)
  {
    std::string min_valid_path_fraction_string;
    status = tesseract_common::QueryStringText(min_valid_path_fraction_element, min_valid_path_fraction_string);
    if (status != tinyxml2::XML_NO_ATTRIBUTE && status != tinyxml2::XML_SUCCESS)
      throw std::runtime_error("OMPLConfigurator: BKPIECE1: Error parsing MinValidPathFraction string");

    if (!tesseract_common::isNumeric(min_valid_path_fraction_string))
      throw std::runtime_error("OMPLConfigurator: BKPIECE1: MinValidPathFraction is not a numeric values.");

    tesseract_common::toNumeric<double>(min_valid_path_fraction_string, min_valid_path_fraction);
  }
}

ompl::base::PlannerPtr BKPIECE1Configurator::create(ompl::base::SpaceInformationPtr si) const
{
  auto planner = std::make_shared<ompl::geometric::BKPIECE1>(si);
  planner->setRange(range);
  planner->setBorderFraction(border_fraction);
  planner->setFailedExpansionCellScoreFactor(failed_expansion_score_factor);
  planner->setMinValidPathFraction(min_valid_path_fraction);
  return planner;
}

OMPLPlannerType BKPIECE1Configurator::getType() const { return OMPLPlannerType::BKPIECE1; }

tinyxml2::XMLElement* BKPIECE1Configurator::toXML(tinyxml2::XMLDocument& doc) const
{
  tinyxml2::XMLElement* ompl_xml = doc.NewElement("BKPIECE1");

  tinyxml2::XMLElement* range_xml = doc.NewElement("Range");
  range_xml->SetText(range);
  ompl_xml->InsertEndChild(range_xml);

  tinyxml2::XMLElement* border_fraction_xml = doc.NewElement("BorderFraction");
  border_fraction_xml->SetText(border_fraction);
  ompl_xml->InsertEndChild(border_fraction_xml);

  tinyxml2::XMLElement* failed_expansion_score_factor_xml = doc.NewElement("FailedExpansionScoreFactor");
  failed_expansion_score_factor_xml->SetText(failed_expansion_score_factor);
  ompl_xml->InsertEndChild(failed_expansion_score_factor_xml);

  tinyxml2::XMLElement* min_valid_path_fraction_xml = doc.NewElement("MinValidPathFraction");
  min_valid_path_fraction_xml->SetText(min_valid_path_fraction);
  ompl_xml->InsertEndChild(min_valid_path_fraction_xml);

  return ompl_xml;
}

KPIECE1Configurator::KPIECE1Configurator(const tinyxml2::XMLElement& xml_element)
{
  const tinyxml2::XMLElement* kpiece1_element = xml_element.FirstChildElement("KPIECE1");
  const tinyxml2::XMLElement* range_element = kpiece1_element->FirstChildElement("Range");
  const tinyxml2::XMLElement* goal_bias_element = kpiece1_element->FirstChildElement("GoalBias");
  const tinyxml2::XMLElement* border_fraction_element = kpiece1_element->FirstChildElement("BorderFraction");
  const tinyxml2::XMLElement* failed_expansion_score_factor_element = kpiece1_element->FirstChildElement("FailedExpansi"
                                                                                                         "onScoreFacto"
                                                                                                         "r");
  const tinyxml2::XMLElement* min_valid_path_fraction_element = kpiece1_element->FirstChildElement("MinValidPathFractio"
                                                                                                   "n");

  tinyxml2::XMLError status;

  if (range_element)
  {
    std::string range_string;
    status = tesseract_common::QueryStringText(range_element, range_string);
    if (status != tinyxml2::XML_NO_ATTRIBUTE && status != tinyxml2::XML_SUCCESS)
      throw std::runtime_error("OMPLConfigurator: KPIECE1Configurator: Error parsing Range string");

    if (!tesseract_common::isNumeric(range_string))
      throw std::runtime_error("OMPLConfigurator: KPIECE1Configurator: Range is not a numeric values.");

    tesseract_common::toNumeric<double>(range_string, range);
  }

  if (goal_bias_element)
  {
    std::string goal_bias_string;
    status = tesseract_common::QueryStringText(goal_bias_element, goal_bias_string);
    if (status != tinyxml2::XML_NO_ATTRIBUTE && status != tinyxml2::XML_SUCCESS)
      throw std::runtime_error("OMPLConfigurator: KPIECE1: Error parsing GoalBias string");

    if (!tesseract_common::isNumeric(goal_bias_string))
      throw std::runtime_error("OMPLConfigurator: KPIECE1: GoalBias is not a numeric values.");

    tesseract_common::toNumeric<double>(goal_bias_string, goal_bias);
  }

  if (border_fraction_element)
  {
    std::string border_fraction_string;
    status = tesseract_common::QueryStringText(border_fraction_element, border_fraction_string);
    if (status != tinyxml2::XML_NO_ATTRIBUTE && status != tinyxml2::XML_SUCCESS)
      throw std::runtime_error("OMPLConfigurator: KPIECE1: Error parsing BorderFraction string");

    if (!tesseract_common::isNumeric(border_fraction_string))
      throw std::runtime_error("OMPLConfigurator: KPIECE1: BorderFraction is not a numeric values.");

    tesseract_common::toNumeric<double>(border_fraction_string, border_fraction);
  }

  if (failed_expansion_score_factor_element)
  {
    std::string failed_expansion_score_factor_string;
    status =
        tesseract_common::QueryStringText(failed_expansion_score_factor_element, failed_expansion_score_factor_string);
    if (status != tinyxml2::XML_NO_ATTRIBUTE && status != tinyxml2::XML_SUCCESS)
      throw std::runtime_error("OMPLConfigurator: KPIECE1: Error parsing FailedExpansionScoreFactor string");

    if (!tesseract_common::isNumeric(failed_expansion_score_factor_string))
      throw std::runtime_error("OMPLConfigurator: KPIECE1: FailedExpansionScoreFactor is not a numeric values.");

    tesseract_common::toNumeric<double>(failed_expansion_score_factor_string, failed_expansion_score_factor);
  }

  if (min_valid_path_fraction_element)
  {
    std::string min_valid_path_fraction_string;
    status = tesseract_common::QueryStringText(min_valid_path_fraction_element, min_valid_path_fraction_string);
    if (status != tinyxml2::XML_NO_ATTRIBUTE && status != tinyxml2::XML_SUCCESS)
      throw std::runtime_error("OMPLConfigurator: KPIECE1: Error parsing MinValidPathFraction string");

    if (!tesseract_common::isNumeric(min_valid_path_fraction_string))
      throw std::runtime_error("OMPLConfigurator: KPIECE1: MinValidPathFraction is not a numeric values.");

    tesseract_common::toNumeric<double>(min_valid_path_fraction_string, min_valid_path_fraction);
  }
}

ompl::base::PlannerPtr KPIECE1Configurator::create(ompl::base::SpaceInformationPtr si) const
{
  auto planner = std::make_shared<ompl::geometric::KPIECE1>(si);
  planner->setRange(range);
  planner->setGoalBias(goal_bias);
  planner->setBorderFraction(border_fraction);
  planner->setFailedExpansionCellScoreFactor(failed_expansion_score_factor);
  planner->setMinValidPathFraction(min_valid_path_fraction);
  return planner;
}

OMPLPlannerType KPIECE1Configurator::getType() const { return OMPLPlannerType::KPIECE1; }

tinyxml2::XMLElement* KPIECE1Configurator::toXML(tinyxml2::XMLDocument& doc) const
{
  tinyxml2::XMLElement* ompl_xml = doc.NewElement("KPIECE1");

  tinyxml2::XMLElement* range_xml = doc.NewElement("Range");
  range_xml->SetText(range);
  ompl_xml->InsertEndChild(range_xml);

  tinyxml2::XMLElement* goal_bias_xml = doc.NewElement("GoalBias");
  goal_bias_xml->SetText(goal_bias);
  ompl_xml->InsertEndChild(goal_bias_xml);

  tinyxml2::XMLElement* border_fraction_xml = doc.NewElement("BorderFraction");
  border_fraction_xml->SetText(border_fraction);
  ompl_xml->InsertEndChild(border_fraction_xml);

  tinyxml2::XMLElement* failed_expansion_score_factor_xml = doc.NewElement("FailedExpansionScoreFactor");
  failed_expansion_score_factor_xml->SetText(failed_expansion_score_factor);
  ompl_xml->InsertEndChild(failed_expansion_score_factor_xml);

  tinyxml2::XMLElement* min_valid_path_fraction_xml = doc.NewElement("MinValidPathFraction");
  min_valid_path_fraction_xml->SetText(min_valid_path_fraction);
  ompl_xml->InsertEndChild(min_valid_path_fraction_xml);

  return ompl_xml;
}

BiTRRTConfigurator::BiTRRTConfigurator(const tinyxml2::XMLElement& xml_element)
{
  const tinyxml2::XMLElement* bitrrt_element = xml_element.FirstChildElement("BiTRRT");
  const tinyxml2::XMLElement* range_element = bitrrt_element->FirstChildElement("Range");
  const tinyxml2::XMLElement* temp_change_factor_element = bitrrt_element->FirstChildElement("TempChangeFactor");
  const tinyxml2::XMLElement* cost_threshold_element = bitrrt_element->FirstChildElement("CostThreshold");
  const tinyxml2::XMLElement* init_temperature_element = bitrrt_element->FirstChildElement("InitTemperature");
  const tinyxml2::XMLElement* frontier_threshold_element = bitrrt_element->FirstChildElement("FrontierThreshold");
  const tinyxml2::XMLElement* frontier_node_ratio_element = bitrrt_element->FirstChildElement("FrontierNodeRatio");

  tinyxml2::XMLError status;

  if (range_element)
  {
    std::string range_string;
    status = tesseract_common::QueryStringText(range_element, range_string);
    if (status != tinyxml2::XML_NO_ATTRIBUTE && status != tinyxml2::XML_SUCCESS)
      throw std::runtime_error("OMPLConfigurator: BiTRRT: Error parsing Range string");

    if (!tesseract_common::isNumeric(range_string))
      throw std::runtime_error("OMPLConfigurator: BiTRRT: Range is not a numeric values.");

    tesseract_common::toNumeric<double>(range_string, range);
  }

  if (temp_change_factor_element)
  {
    std::string temp_change_factor_string;
    status = tesseract_common::QueryStringText(temp_change_factor_element, temp_change_factor_string);
    if (status != tinyxml2::XML_NO_ATTRIBUTE && status != tinyxml2::XML_SUCCESS)
      throw std::runtime_error("OMPLConfigurator: BiTRRT: Error parsing TempChangeFactor string");

    if (!tesseract_common::isNumeric(temp_change_factor_string))
      throw std::runtime_error("OMPLConfigurator: BiTRRT: TempChangeFactor is not a numeric values.");

    tesseract_common::toNumeric<double>(temp_change_factor_string, temp_change_factor);
  }

  if (cost_threshold_element)
  {
    std::string cost_threshold_string;
    status = tesseract_common::QueryStringText(cost_threshold_element, cost_threshold_string);
    if (status != tinyxml2::XML_NO_ATTRIBUTE && status != tinyxml2::XML_SUCCESS)
      throw std::runtime_error("OMPLConfigurator: BiTRRT: Error parsing CostThreshold string");

    if (!tesseract_common::isNumeric(cost_threshold_string))
    {
      if (cost_threshold_string != "inf")
        throw std::runtime_error("OMPLConfigurator: BiTRRT: CostThreshold is not a numeric values.");
    }
    else
      tesseract_common::toNumeric<double>(cost_threshold_string, cost_threshold);
  }

  if (init_temperature_element)
  {
    std::string init_temperature_string;
    status = tesseract_common::QueryStringText(init_temperature_element, init_temperature_string);
    if (status != tinyxml2::XML_NO_ATTRIBUTE && status != tinyxml2::XML_SUCCESS)
      throw std::runtime_error("OMPLConfigurator: BiTRRT: Error parsing InitTemperature string");

    if (!tesseract_common::isNumeric(init_temperature_string))
      throw std::runtime_error("OMPLConfigurator: BiTRRT: InitTemperature is not a numeric values.");

    tesseract_common::toNumeric<double>(init_temperature_string, init_temperature);
  }

  if (frontier_threshold_element)
  {
    std::string frontier_threshold_string;
    status = tesseract_common::QueryStringText(frontier_threshold_element, frontier_threshold_string);
    if (status != tinyxml2::XML_NO_ATTRIBUTE && status != tinyxml2::XML_SUCCESS)
      throw std::runtime_error("OMPLConfigurator: BiTRRT: Error parsing FrontierThreshold string");

    if (!tesseract_common::isNumeric(frontier_threshold_string))
      throw std::runtime_error("OMPLConfigurator: BiTRRT: FrontierThreshold is not a numeric values.");

    tesseract_common::toNumeric<double>(frontier_threshold_string, frontier_threshold);
  }

  if (frontier_node_ratio_element)
  {
    std::string frontier_node_ratio_string;
    status = tesseract_common::QueryStringText(frontier_node_ratio_element, frontier_node_ratio_string);
    if (status != tinyxml2::XML_NO_ATTRIBUTE && status != tinyxml2::XML_SUCCESS)
      throw std::runtime_error("OMPLConfigurator: BiTRRT: Error FrontierNodeRatio GoalBias string");

    if (!tesseract_common::isNumeric(frontier_node_ratio_string))
      throw std::runtime_error("OMPLConfigurator: BiTRRT: FrontierNodeRatio is not a numeric values.");

    tesseract_common::toNumeric<double>(frontier_node_ratio_string, frontier_node_ratio);
  }
}

ompl::base::PlannerPtr BiTRRTConfigurator::create(ompl::base::SpaceInformationPtr si) const
{
  auto planner = std::make_shared<ompl::geometric::BiTRRT>(si);
  planner->setRange(range);
  planner->setTempChangeFactor(temp_change_factor);
  planner->setCostThreshold(cost_threshold);
  planner->setInitTemperature(init_temperature);
  planner->setFrontierThreshold(frontier_threshold);
  planner->setFrontierNodeRatio(frontier_node_ratio);
  return planner;
}

OMPLPlannerType BiTRRTConfigurator::getType() const { return OMPLPlannerType::BiTRRT; }

tinyxml2::XMLElement* BiTRRTConfigurator::toXML(tinyxml2::XMLDocument& doc) const
{
  tinyxml2::XMLElement* ompl_xml = doc.NewElement("BiTRRT");

  tinyxml2::XMLElement* range_xml = doc.NewElement("Range");
  range_xml->SetText(range);
  ompl_xml->InsertEndChild(range_xml);

  tinyxml2::XMLElement* temp_change_factor_xml = doc.NewElement("TempChangeFactor");
  temp_change_factor_xml->SetText(temp_change_factor);
  ompl_xml->InsertEndChild(temp_change_factor_xml);

  tinyxml2::XMLElement* cost_threshold_xml = doc.NewElement("CostThreshold");
  cost_threshold_xml->SetText(cost_threshold);
  ompl_xml->InsertEndChild(cost_threshold_xml);

  tinyxml2::XMLElement* init_temperature_xml = doc.NewElement("InitTemperature");
  init_temperature_xml->SetText(init_temperature);
  ompl_xml->InsertEndChild(init_temperature_xml);

  tinyxml2::XMLElement* frontier_threshold_xml = doc.NewElement("FrontierThreshold");
  frontier_threshold_xml->SetText(frontier_threshold);
  ompl_xml->InsertEndChild(frontier_threshold_xml);

  tinyxml2::XMLElement* frontier_node_ratio_xml = doc.NewElement("FrontierNodeRatio");
  frontier_node_ratio_xml->SetText(frontier_node_ratio);
  ompl_xml->InsertEndChild(frontier_node_ratio_xml);

  return ompl_xml;
}

RRTConfigurator::RRTConfigurator(const tinyxml2::XMLElement& xml_element)
{
  const tinyxml2::XMLElement* rrt_element = xml_element.FirstChildElement("RRT");
  const tinyxml2::XMLElement* range_element = rrt_element->FirstChildElement("Range");
  const tinyxml2::XMLElement* goal_bias_element = rrt_element->FirstChildElement("GoalBias");

  tinyxml2::XMLError status;

  if (range_element)
  {
    std::string range_string;
    status = tesseract_common::QueryStringText(range_element, range_string);
    if (status != tinyxml2::XML_NO_ATTRIBUTE && status != tinyxml2::XML_SUCCESS)
      throw std::runtime_error("OMPLConfigurator: RRT: Error parsing Range string");

    if (!tesseract_common::isNumeric(range_string))
      throw std::runtime_error("OMPLConfigurator: RRT: Range is not a numeric values.");

    tesseract_common::toNumeric<double>(range_string, range);
  }

  if (goal_bias_element)
  {
    std::string goal_bias_string;
    status = tesseract_common::QueryStringText(goal_bias_element, goal_bias_string);
    if (status != tinyxml2::XML_NO_ATTRIBUTE && status != tinyxml2::XML_SUCCESS)
      throw std::runtime_error("OMPLConfigurator: RRT: Error parsing GoalBias string");

    if (!tesseract_common::isNumeric(goal_bias_string))
      throw std::runtime_error("OMPLConfigurator: RRT: GoalBias is not a numeric values.");

    tesseract_common::toNumeric<double>(goal_bias_string, goal_bias);
  }
}

ompl::base::PlannerPtr RRTConfigurator::create(ompl::base::SpaceInformationPtr si) const
{
  auto planner = std::make_shared<ompl::geometric::RRT>(si);
  planner->setRange(range);
  planner->setGoalBias(goal_bias);
  return planner;
}

OMPLPlannerType RRTConfigurator::getType() const { return OMPLPlannerType::RRT; }

tinyxml2::XMLElement* RRTConfigurator::toXML(tinyxml2::XMLDocument& doc) const
{
  tinyxml2::XMLElement* ompl_xml = doc.NewElement("RRT");

  tinyxml2::XMLElement* range_xml = doc.NewElement("Range");
  range_xml->SetText(range);
  ompl_xml->InsertEndChild(range_xml);

  tinyxml2::XMLElement* goal_bias_xml = doc.NewElement("GoalBias");
  goal_bias_xml->SetText(goal_bias);
  ompl_xml->InsertEndChild(goal_bias_xml);

  return ompl_xml;
}

RRTConnectConfigurator::RRTConnectConfigurator(const tinyxml2::XMLElement& xml_element)
{
  const tinyxml2::XMLElement* rrt_connect_element = xml_element.FirstChildElement("RRTConnect");
  const tinyxml2::XMLElement* range_element = rrt_connect_element->FirstChildElement("Range");

  tinyxml2::XMLError status;

  if (range_element)
  {
    std::string range_string;
    status = tesseract_common::QueryStringText(range_element, range_string);
    if (status != tinyxml2::XML_NO_ATTRIBUTE && status != tinyxml2::XML_SUCCESS)
      throw std::runtime_error("OMPLConfigurator: RRTConnect: Error parsing Range string");

    if (!tesseract_common::isNumeric(range_string))
      throw std::runtime_error("OMPLConfigurator: RRTConnect: Range is not a numeric values.");

    tesseract_common::toNumeric<double>(range_string, range);
  }
}

ompl::base::PlannerPtr RRTConnectConfigurator::create(ompl::base::SpaceInformationPtr si) const
{
  auto planner = std::make_shared<ompl::geometric::RRTConnect>(si);
  planner->setRange(range);
  return planner;
}

OMPLPlannerType RRTConnectConfigurator::getType() const { return OMPLPlannerType::RRTConnect; }

tinyxml2::XMLElement* RRTConnectConfigurator::toXML(tinyxml2::XMLDocument& doc) const
{
  tinyxml2::XMLElement* ompl_xml = doc.NewElement("RRTConnect");

  tinyxml2::XMLElement* range_xml = doc.NewElement("Range");
  range_xml->SetText(range);
  ompl_xml->InsertEndChild(range_xml);

  return ompl_xml;
}

RRTstarConfigurator::RRTstarConfigurator(const tinyxml2::XMLElement& xml_element)
{
  const tinyxml2::XMLElement* rrt_star_element = xml_element.FirstChildElement("RRTstar");
  const tinyxml2::XMLElement* range_element = rrt_star_element->FirstChildElement("Range");
  const tinyxml2::XMLElement* goal_bias_element = rrt_star_element->FirstChildElement("GoalBias");
  const tinyxml2::XMLElement* delay_collision_checking_element = rrt_star_element->FirstChildElement("DelayCollisionChe"
                                                                                                     "cking");

  tinyxml2::XMLError status;

  if (range_element)
  {
    std::string range_string;
    status = tesseract_common::QueryStringText(range_element, range_string);
    if (status != tinyxml2::XML_NO_ATTRIBUTE && status != tinyxml2::XML_SUCCESS)
      throw std::runtime_error("OMPLConfigurator: RRTstar: Error parsing Range string");

    if (!tesseract_common::isNumeric(range_string))
      throw std::runtime_error("OMPLConfigurator: RRTstar: Range is not a numeric values.");

    tesseract_common::toNumeric<double>(range_string, range);
  }

  if (goal_bias_element)
  {
    std::string goal_bias_string;
    status = tesseract_common::QueryStringText(goal_bias_element, goal_bias_string);
    if (status != tinyxml2::XML_NO_ATTRIBUTE && status != tinyxml2::XML_SUCCESS)
      throw std::runtime_error("OMPLConfigurator: RRTstar: Error parsing GoalBias string");

    if (!tesseract_common::isNumeric(goal_bias_string))
      throw std::runtime_error("OMPLConfigurator: RRTstar: GoalBias is not a numeric values.");

    tesseract_common::toNumeric<double>(goal_bias_string, goal_bias);
  }

  if (delay_collision_checking_element)
  {
    std::string delay_collision_checking_string;
    status = delay_collision_checking_element->QueryBoolText(&delay_collision_checking);
    if (status != tinyxml2::XML_NO_ATTRIBUTE && status != tinyxml2::XML_SUCCESS)
      throw std::runtime_error("OMPLConfigurator: RRTstar: Error parsing DelayCollisionChecking string");
  }
}

ompl::base::PlannerPtr RRTstarConfigurator::create(ompl::base::SpaceInformationPtr si) const
{
  auto planner = std::make_shared<ompl::geometric::RRTstar>(si);
  planner->setRange(range);
  planner->setGoalBias(goal_bias);
  planner->setDelayCC(delay_collision_checking);
  return planner;
}

OMPLPlannerType RRTstarConfigurator::getType() const { return OMPLPlannerType::RRTstar; }

tinyxml2::XMLElement* RRTstarConfigurator::toXML(tinyxml2::XMLDocument& doc) const
{
  tinyxml2::XMLElement* ompl_xml = doc.NewElement("RRTstar");

  tinyxml2::XMLElement* range_xml = doc.NewElement("Range");
  range_xml->SetText(range);
  ompl_xml->InsertEndChild(range_xml);

  tinyxml2::XMLElement* goal_bias_xml = doc.NewElement("GoalBias");
  goal_bias_xml->SetText(goal_bias);
  ompl_xml->InsertEndChild(goal_bias_xml);

  tinyxml2::XMLElement* delay_collision_checking_xml = doc.NewElement("DelayCollisionChecking");
  delay_collision_checking_xml->SetText(delay_collision_checking);
  ompl_xml->InsertEndChild(delay_collision_checking_xml);

  return ompl_xml;
}

TRRTConfigurator::TRRTConfigurator(const tinyxml2::XMLElement& xml_element)
{
  const tinyxml2::XMLElement* trrt_element = xml_element.FirstChildElement("TRRT");
  const tinyxml2::XMLElement* range_element = trrt_element->FirstChildElement("Range");
  const tinyxml2::XMLElement* goal_bias_element = trrt_element->FirstChildElement("GoalBias");
  const tinyxml2::XMLElement* temp_change_factor_element = trrt_element->FirstChildElement("TempChangeFactor");
  const tinyxml2::XMLElement* init_temperature_element = trrt_element->FirstChildElement("InitTemp");
  const tinyxml2::XMLElement* frontier_threshold_element = trrt_element->FirstChildElement("FrontierThreshold");
  const tinyxml2::XMLElement* frontier_node_ratio_element = trrt_element->FirstChildElement("FrontierNodeRatio");

  tinyxml2::XMLError status;

  if (range_element)
  {
    std::string range_string;
    status = tesseract_common::QueryStringText(range_element, range_string);
    if (status != tinyxml2::XML_NO_ATTRIBUTE && status != tinyxml2::XML_SUCCESS)
      throw std::runtime_error("OMPLConfigurator: TRRT: Error parsing Range string");

    if (!tesseract_common::isNumeric(range_string))
      throw std::runtime_error("OMPLConfigurator: TRRT: Range is not a numeric values.");

    tesseract_common::toNumeric<double>(range_string, range);
  }

  if (goal_bias_element)
  {
    std::string goal_bias_string;
    status = tesseract_common::QueryStringText(goal_bias_element, goal_bias_string);
    if (status != tinyxml2::XML_NO_ATTRIBUTE && status != tinyxml2::XML_SUCCESS)
      throw std::runtime_error("OMPLConfigurator: TRRT: Error parsing GoalBias string");

    if (!tesseract_common::isNumeric(goal_bias_string))
      throw std::runtime_error("OMPLConfigurator: TRRT: GoalBias is not a numeric values.");

    tesseract_common::toNumeric<double>(goal_bias_string, goal_bias);
  }

  if (temp_change_factor_element)
  {
    std::string temp_change_factor_string;
    status = tesseract_common::QueryStringText(temp_change_factor_element, temp_change_factor_string);
    if (status != tinyxml2::XML_NO_ATTRIBUTE && status != tinyxml2::XML_SUCCESS)
      throw std::runtime_error("OMPLConfigurator: TRRT: Error parsing TempChangeFactor string");

    if (!tesseract_common::isNumeric(temp_change_factor_string))
      throw std::runtime_error("OMPLConfigurator: TRRT: TempChangeFactor is not a numeric values.");

    tesseract_common::toNumeric<double>(temp_change_factor_string, temp_change_factor);
  }

  if (init_temperature_element)
  {
    std::string init_temperature_string;
    status = tesseract_common::QueryStringText(init_temperature_element, init_temperature_string);
    if (status != tinyxml2::XML_NO_ATTRIBUTE && status != tinyxml2::XML_SUCCESS)
      throw std::runtime_error("OMPLConfigurator: TRRT: Error parsing InitTemp string");

    if (!tesseract_common::isNumeric(init_temperature_string))
      throw std::runtime_error("OMPLConfigurator: TRRT: InitTemp is not a numeric values.");

    tesseract_common::toNumeric<double>(init_temperature_string, init_temperature);
  }

  if (frontier_threshold_element)
  {
    std::string frontier_threshold_string;
    status = tesseract_common::QueryStringText(frontier_threshold_element, frontier_threshold_string);
    if (status != tinyxml2::XML_NO_ATTRIBUTE && status != tinyxml2::XML_SUCCESS)
      throw std::runtime_error("OMPLConfigurator: TRRT: Error parsing FrontierThreshold string");

    if (!tesseract_common::isNumeric(frontier_threshold_string))
      throw std::runtime_error("OMPLConfigurator: TRRT: FrontierThreshold is not a numeric values.");

    tesseract_common::toNumeric<double>(frontier_threshold_string, frontier_threshold);
  }

  if (frontier_node_ratio_element)
  {
    std::string frontier_node_ratio_string;
    status = tesseract_common::QueryStringText(frontier_node_ratio_element, frontier_node_ratio_string);
    if (status != tinyxml2::XML_NO_ATTRIBUTE && status != tinyxml2::XML_SUCCESS)
      throw std::runtime_error("OMPLConfigurator: TRRT: Error parsing FrontierNodeRatio string");

    if (!tesseract_common::isNumeric(frontier_node_ratio_string))
      throw std::runtime_error("OMPLConfigurator: TRRT: FrontierNodeRatio is not a numeric values.");

    tesseract_common::toNumeric<double>(frontier_node_ratio_string, frontier_node_ratio);
  }
}

ompl::base::PlannerPtr TRRTConfigurator::create(ompl::base::SpaceInformationPtr si) const
{
  auto planner = std::make_shared<ompl::geometric::TRRT>(si);
  planner->setRange(range);
  planner->setGoalBias(goal_bias);
  planner->setTempChangeFactor(temp_change_factor);
  planner->setInitTemperature(init_temperature);
  planner->setFrontierThreshold(frontier_threshold);
  planner->setFrontierNodeRatio(frontier_node_ratio);
  return planner;
}

OMPLPlannerType TRRTConfigurator::getType() const { return OMPLPlannerType::TRRT; }

tinyxml2::XMLElement* TRRTConfigurator::toXML(tinyxml2::XMLDocument& doc) const
{
  tinyxml2::XMLElement* ompl_xml = doc.NewElement("TRRT");

  tinyxml2::XMLElement* range_xml = doc.NewElement("Range");
  range_xml->SetText(range);
  ompl_xml->InsertEndChild(range_xml);

  tinyxml2::XMLElement* goal_bias_xml = doc.NewElement("GoalBias");
  goal_bias_xml->SetText(goal_bias);
  ompl_xml->InsertEndChild(goal_bias_xml);

  tinyxml2::XMLElement* temp_change_factor_xml = doc.NewElement("TempChangeFactor");
  temp_change_factor_xml->SetText(temp_change_factor);
  ompl_xml->InsertEndChild(temp_change_factor_xml);

  tinyxml2::XMLElement* init_temperature_xml = doc.NewElement("InitTemp");
  init_temperature_xml->SetText(init_temperature);
  ompl_xml->InsertEndChild(init_temperature_xml);

  tinyxml2::XMLElement* frontier_threshold_xml = doc.NewElement("FrontierThreshold");
  frontier_threshold_xml->SetText(frontier_threshold);
  ompl_xml->InsertEndChild(frontier_threshold_xml);

  tinyxml2::XMLElement* frontier_node_ratio_xml = doc.NewElement("FrontierNodeRatio");
  frontier_node_ratio_xml->SetText(frontier_node_ratio);
  ompl_xml->InsertEndChild(frontier_node_ratio_xml);

  return ompl_xml;
}

PRMConfigurator::PRMConfigurator(const tinyxml2::XMLElement& xml_element)
{
  const tinyxml2::XMLElement* prm_element = xml_element.FirstChildElement("PRM");
  const tinyxml2::XMLElement* max_nearest_neighbors_element = prm_element->FirstChildElement("MaxNearestNeighbors");

  tinyxml2::XMLError status;

  if (max_nearest_neighbors_element)
  {
    std::string max_nearest_neighbors_string;
    status = tesseract_common::QueryStringText(max_nearest_neighbors_element, max_nearest_neighbors_string);
    if (status != tinyxml2::XML_NO_ATTRIBUTE && status != tinyxml2::XML_SUCCESS)
      throw std::runtime_error("OMPLConfigurator: PRM: Error parsing MaxNearestNeighbors string");

    if (!tesseract_common::isNumeric(max_nearest_neighbors_string))
      throw std::runtime_error("OMPLConfigurator: PRM: MaxNearestNeighbors is not a numeric values.");

    tesseract_common::toNumeric<int>(max_nearest_neighbors_string, max_nearest_neighbors);
  }
}

ompl::base::PlannerPtr PRMConfigurator::create(ompl::base::SpaceInformationPtr si) const
{
  auto planner = std::make_shared<ompl::geometric::PRM>(si);
  planner->setMaxNearestNeighbors(static_cast<unsigned>(max_nearest_neighbors));
  return planner;
}

OMPLPlannerType PRMConfigurator::getType() const { return OMPLPlannerType::PRM; }

tinyxml2::XMLElement* PRMConfigurator::toXML(tinyxml2::XMLDocument& doc) const
{
  tinyxml2::XMLElement* ompl_xml = doc.NewElement("PRM");

  tinyxml2::XMLElement* max_nearest_neighbors_xml = doc.NewElement("MaxNearestNeighbors");
  max_nearest_neighbors_xml->SetText(max_nearest_neighbors);
  ompl_xml->InsertEndChild(max_nearest_neighbors_xml);

  return ompl_xml;
}

PRMstarConfigurator::PRMstarConfigurator(const tinyxml2::XMLElement&) {}

ompl::base::PlannerPtr PRMstarConfigurator::create(ompl::base::SpaceInformationPtr si) const
{
  return std::make_shared<ompl::geometric::PRMstar>(si);
}

OMPLPlannerType PRMstarConfigurator::getType() const { return OMPLPlannerType::PRMstar; }

tinyxml2::XMLElement* PRMstarConfigurator::toXML(tinyxml2::XMLDocument& doc) const
{
  tinyxml2::XMLElement* ompl_xml = doc.NewElement("PRMstar");

  return ompl_xml;
}

LazyPRMstarConfigurator::LazyPRMstarConfigurator(const tinyxml2::XMLElement&) {}

ompl::base::PlannerPtr LazyPRMstarConfigurator::create(ompl::base::SpaceInformationPtr si) const
{
  return std::make_shared<ompl::geometric::LazyPRMstar>(si);
}

OMPLPlannerType LazyPRMstarConfigurator::getType() const { return OMPLPlannerType::LazyPRMstar; }

tinyxml2::XMLElement* LazyPRMstarConfigurator::toXML(tinyxml2::XMLDocument& doc) const
{
  tinyxml2::XMLElement* ompl_xml = doc.NewElement("LazyPRMstar");

  return ompl_xml;
}

SPARSConfigurator::SPARSConfigurator(const tinyxml2::XMLElement& xml_element)
{
  const tinyxml2::XMLElement* spars_element = xml_element.FirstChildElement("SPARS");
  const tinyxml2::XMLElement* max_failures_element = spars_element->FirstChildElement("MaxFailures");
  const tinyxml2::XMLElement* dense_delta_fraction_element = spars_element->FirstChildElement("DenseDataFraction");
  const tinyxml2::XMLElement* sparse_delta_fraction_element = spars_element->FirstChildElement("SparseDeltaFraction");
  const tinyxml2::XMLElement* stretch_factor_element = spars_element->FirstChildElement("StretchFactor");

  tinyxml2::XMLError status;

  if (max_failures_element)
  {
    std::string max_failures_string;
    status = tesseract_common::QueryStringText(max_failures_element, max_failures_string);
    if (status != tinyxml2::XML_NO_ATTRIBUTE && status != tinyxml2::XML_SUCCESS)
      throw std::runtime_error("OMPLConfigurator: SPARS: Error parsing MaxFailures string");

    if (!tesseract_common::isNumeric(max_failures_string))
      throw std::runtime_error("OMPLConfigurator: SPARS: MaxFailures is not a numeric values.");

    tesseract_common::toNumeric<int>(max_failures_string, max_failures);
  }

  if (dense_delta_fraction_element)
  {
    std::string dense_delta_fraction_string;
    status = tesseract_common::QueryStringText(dense_delta_fraction_element, dense_delta_fraction_string);
    if (status != tinyxml2::XML_NO_ATTRIBUTE && status != tinyxml2::XML_SUCCESS)
      throw std::runtime_error("OMPLConfigurator: SPARS: Error parsing DenseDataFraction string");

    if (!tesseract_common::isNumeric(dense_delta_fraction_string))
      throw std::runtime_error("OMPLConfigurator: SPARS: DenseDataFraction is not a numeric values.");

    tesseract_common::toNumeric<double>(dense_delta_fraction_string, dense_delta_fraction);
  }

  if (sparse_delta_fraction_element)
  {
    std::string sparse_delta_fraction_string;
    status = tesseract_common::QueryStringText(sparse_delta_fraction_element, sparse_delta_fraction_string);
    if (status != tinyxml2::XML_NO_ATTRIBUTE && status != tinyxml2::XML_SUCCESS)
      throw std::runtime_error("OMPLConfigurator: SPARS: Error parsing SparseDeltaFraction string");

    if (!tesseract_common::isNumeric(sparse_delta_fraction_string))
      throw std::runtime_error("OMPLConfigurator: SPARS: SparseDeltaFraction is not a numeric values.");

    tesseract_common::toNumeric<double>(sparse_delta_fraction_string, sparse_delta_fraction);
  }

  if (stretch_factor_element)
  {
    std::string stretch_factor_string;
    status = tesseract_common::QueryStringText(stretch_factor_element, stretch_factor_string);
    if (status != tinyxml2::XML_NO_ATTRIBUTE && status != tinyxml2::XML_SUCCESS)
      throw std::runtime_error("OMPLConfigurator: SPARS: Error parsing StretchFactor string");

    if (!tesseract_common::isNumeric(stretch_factor_string))
      throw std::runtime_error("OMPLConfigurator: SPARS: StretchFactor is not a numeric values.");

    tesseract_common::toNumeric<double>(stretch_factor_string, stretch_factor);
  }
}

ompl::base::PlannerPtr SPARSConfigurator::create(ompl::base::SpaceInformationPtr si) const
{
  auto planner = std::make_shared<ompl::geometric::SPARS>(si);
  planner->setMaxFailures(static_cast<unsigned>(max_failures));
  planner->setDenseDeltaFraction(dense_delta_fraction);
  planner->setSparseDeltaFraction(sparse_delta_fraction);
  planner->setStretchFactor(stretch_factor);
  return planner;
}

OMPLPlannerType SPARSConfigurator::getType() const { return OMPLPlannerType::SPARS; }

tinyxml2::XMLElement* SPARSConfigurator::toXML(tinyxml2::XMLDocument& doc) const
{
  tinyxml2::XMLElement* ompl_xml = doc.NewElement("SPARS");

  tinyxml2::XMLElement* max_failures_xml = doc.NewElement("MaxFailures");
  max_failures_xml->SetText(max_failures);
  ompl_xml->InsertEndChild(max_failures_xml);

  tinyxml2::XMLElement* dense_delta_fraction_xml = doc.NewElement("DenseDataFraction");
  dense_delta_fraction_xml->SetText(dense_delta_fraction);
  ompl_xml->InsertEndChild(dense_delta_fraction_xml);

  tinyxml2::XMLElement* sparse_delta_fraction_xml = doc.NewElement("SparseDeltaFraction");
  sparse_delta_fraction_xml->SetText(sparse_delta_fraction);
  ompl_xml->InsertEndChild(sparse_delta_fraction_xml);

  tinyxml2::XMLElement* stretch_factor_xml = doc.NewElement("StretchFactor");
  stretch_factor_xml->SetText(stretch_factor);
  ompl_xml->InsertEndChild(stretch_factor_xml);

  return ompl_xml;
}
}  // namespace tesseract_planning
