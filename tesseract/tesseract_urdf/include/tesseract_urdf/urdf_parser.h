/**
 * @file urdf_parser.h
 * @brief A urdf parser for tesseract
 *
 * @author Levi Armstrong
 * @date September 1, 2019
 * @version TODO
 * @bug No known bugs
 *
 * @copyright Copyright (c) 2019, Southwest Research Institute
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
#ifndef TESSERACT_URDF_URDF_PARSER_H
#define TESSERACT_URDF_URDF_PARSER_H

#include <tesseract_common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <fstream>
#include <tesseract_common/status_code.h>
#include <tinyxml2.h>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract_scene_graph/graph.h>
#include <tesseract_scene_graph/utils.h>
#include <tesseract_scene_graph/resource_locator.h>
#include <tesseract_urdf/joint.h>
#include <tesseract_urdf/link.h>
#include <tesseract_urdf/utils.h>
#include <tesseract_urdf/material.h>

namespace tesseract_urdf
{
class URDFStatusCategory : public tesseract_common::StatusCategory
{
public:
  URDFStatusCategory(std::string desc = "") : name_("URDFStatusCategory"), desc_(std::move(desc)) {}
  const std::string& name() const noexcept override { return name_; }
  std::string message(int code) const override
  {
    switch (code)
    {
      case Success:
        return "Successfully parsed urdf: '" + desc_ + "'!";
      case ErrorOpeningFile:
        return "Failed to open urdf file: '" + desc_ + "'!";
      case ErrorAttributeName:
        return "Missing or failed parsing urdf attribute 'name' for urdf '" + desc_ + "'!";
      case ErrorParsingRobotElement:
        return "Error parsing urdf 'robot' element for urdf '" + desc_ + "'!";
      case ErrorParsingAvailableMaterialElement:
        return "Error parsing urdf global 'material' element for urdf '" + desc_ + "'!";
      case ErrorParsingLinkElement:
        return "Error parsing urdf 'link' element for urdf '" + desc_ + "'!";
      case ErrorLocatingGlobalMaterial:
        return "Error parsing urdf, unable to locate global material for urdf '" + desc_ + "'!";
      case ErrorLinkNamesNotUnique:
        return "Error parsing urdf, link names are not unique for urdf '" + desc_ + "'!";
      case ErrorAddingLinkToSceneGraph:
        return "Error parsing urdf, failed to add link to scene graph for urdf '" + desc_ + "'!";
      case ErrorNoLinks:
        return "Error parsing urdf, no links were found for urdf '" + desc_ + "'!";
      case ErrorParsingJointElement:
        return "Error parsing urdf 'joint' element for urdf '" + desc_ + "'!";
      case ErrorJointNamesNotUnique:
        return "Error parsing urdf, joint names are not unique for urdf '" + desc_ + "'!";
      case ErrorAddingJointToSceneGraph:
        return "Error parsing urdf, failed to add joint to scene graph for urdf '" + desc_ + "'!";
      case ErrorNoJoints:
        return "Error parsing urdf, no joints were found for urdf '" + desc_ + "'!";
      case ErrorIsNotTree:
        return "Error parsing urdf, is not a tree structure for urdf '" + desc_ + "'!";
      case ErrorIsAcyclic:
        return "Error parsing urdf, is acyclic for urdf '" + desc_ + "'!";
      case ErrorAttributeVersion:
        return "Failed parsing urdf attribute 'version' for urdf '" + desc_ + "'!";
      default:
        return "Invalid error code for " + name_ + " for urdf '" + desc_ + "'!";
    }
  }

  enum
  {
    Success = 0,
    ErrorOpeningFile = -1,
    ErrorParsingRobotElement = -2,
    ErrorAttributeName = -3,
    ErrorParsingAvailableMaterialElement = -4,
    ErrorParsingLinkElement = -5,
    ErrorLocatingGlobalMaterial = -6,
    ErrorLinkNamesNotUnique = -7,
    ErrorAddingLinkToSceneGraph = -8,
    ErrorNoLinks = -9,
    ErrorParsingJointElement = -10,
    ErrorJointNamesNotUnique = -11,
    ErrorAddingJointToSceneGraph = -12,
    ErrorNoJoints = -13,
    ErrorIsNotTree = -14,
    ErrorIsAcyclic = -15,
    ErrorAttributeVersion = -16
  };

private:
  std::string name_;
  std::string desc_;
};

/**
 * @brief Parse a URDF string into a Tesseract Scene Graph
 * @param scene_graph Tesseract Scene Graph
 * @param urdf_xml_string URDF xml string
 * @param locator The resource locator function
 * @return Status Code
 */
tesseract_common::StatusCode::Ptr parseURDFString(tesseract_scene_graph::SceneGraph::Ptr& scene_graph,
                                                  const std::string& urdf_xml_string,
                                                  const tesseract_scene_graph::ResourceLocator::Ptr& locator);

/**
 * @brief Parse a URDF file into a Tesseract Scene Graph
 * @param scene_graph Tesseract Scene Graph
 * @param path URDF file path
 * @param locator The resource locator function
 * @return Status Code
 */
tesseract_common::StatusCode::Ptr parseURDFFile(tesseract_scene_graph::SceneGraph::Ptr& scene_graph,
                                                const std::string& path,
                                                const tesseract_scene_graph::ResourceLocator::Ptr& locator);

/**
 * @brief Parse a URDF string into a Tesseract Scene Graph
 * @param urdf_xml_string URDF xml string
 * @param locator The resource locator function
 * @return Tesseract Scene Graph, nullptr if failed to parse URDF
 */
tesseract_scene_graph::SceneGraph::Ptr parseURDFString(const std::string& urdf_xml_string,
                                                       const tesseract_scene_graph::ResourceLocator::Ptr& locator);

/**
 * @brief Parse a URDF file into a Tesseract Scene Graph
 * @param URDF file path
 * @param The resource locator function
 * @return Tesseract Scene Graph, nullptr if failed to parse URDF
 */
tesseract_scene_graph::SceneGraph::Ptr parseURDFFile(const std::string& path,
                                                     const tesseract_scene_graph::ResourceLocator::Ptr& locator);

}  // namespace tesseract_urdf

#endif
