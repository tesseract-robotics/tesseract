#ifndef TESSERACT_URDF_COMMON_UNIT_H
#define TESSERACT_URDF_COMMON_UNIT_H

#include <tesseract_common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <gtest/gtest.h>
#include <vector>
#include <tinyxml2.h>
#include <console_bridge/console.h>
#include <tesseract_scene_graph/utils.h>
#include <tesseract_scene_graph/resource_locator.h>
#include <tesseract_urdf/urdf_parser.h>
#include <tesseract_common/utils.h>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

inline std::string locateResource(const std::string& url)
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

template <typename ElementType>
bool runTest(ElementType& type,
             std::function<ElementType(const tinyxml2::XMLElement*, const int)> func,
             const std::string& xml_string,
             const std::string& element_name,
             int version)
{
  tinyxml2::XMLDocument xml_doc;
  EXPECT_TRUE(xml_doc.Parse(xml_string.c_str()) == tinyxml2::XML_SUCCESS);

  tinyxml2::XMLElement* element = xml_doc.FirstChildElement(element_name.c_str());
  EXPECT_TRUE(element != nullptr);

  try
  {
    type = func(element, version);
  }
  catch (const std::exception& e)
  {
    tesseract_common::printNestedException(e);
    return false;
  }

  return true;
}

template <typename ElementType>
bool runTest(
    ElementType& type,
    std::function<
        ElementType(const tinyxml2::XMLElement*, const tesseract_scene_graph::ResourceLocator::Ptr&, bool, const int)>
        func,
    const std::string& xml_string,
    const std::string& element_name,
    const tesseract_scene_graph::ResourceLocator::Ptr& locator,
    int version,
    bool visual)
{
  tinyxml2::XMLDocument xml_doc;
  EXPECT_TRUE(xml_doc.Parse(xml_string.c_str()) == tinyxml2::XML_SUCCESS);

  tinyxml2::XMLElement* element = xml_doc.FirstChildElement(element_name.c_str());
  EXPECT_TRUE(element != nullptr);

  try
  {
    type = func(element, locator, visual, version);
  }
  catch (const std::exception& e)
  {
    tesseract_common::printNestedException(e);
    return false;
  }

  return true;
}

template <typename ElementType>
bool runTest(
    ElementType& type,
    std::function<
        ElementType(const tinyxml2::XMLElement*, const tesseract_scene_graph::ResourceLocator::Ptr&, const int)> func,
    const std::string& xml_string,
    const std::string& element_name,
    const tesseract_scene_graph::ResourceLocator::Ptr& locator,
    int version)
{
  tinyxml2::XMLDocument xml_doc;
  EXPECT_TRUE(xml_doc.Parse(xml_string.c_str()) == tinyxml2::XML_SUCCESS);

  tinyxml2::XMLElement* element = xml_doc.FirstChildElement(element_name.c_str());
  EXPECT_TRUE(element != nullptr);

  try
  {
    type = func(element, locator, version);
  }
  catch (const std::exception& e)
  {
    tesseract_common::printNestedException(e);
    return false;
  }

  return true;
}

template <typename ElementType>
bool runTest(ElementType& type,
             std::function<ElementType(const tinyxml2::XMLElement*,
                                       const tesseract_scene_graph::ResourceLocator::Ptr&,
                                       std::unordered_map<std::string, tesseract_scene_graph::Material::Ptr>&,
                                       const int)> func,
             const std::string& xml_string,
             const std::string& element_name,
             const tesseract_scene_graph::ResourceLocator::Ptr& locator,
             std::unordered_map<std::string, tesseract_scene_graph::Material::Ptr>& available_materials,
             int version)
{
  tinyxml2::XMLDocument xml_doc;
  EXPECT_TRUE(xml_doc.Parse(xml_string.c_str()) == tinyxml2::XML_SUCCESS);

  tinyxml2::XMLElement* element = xml_doc.FirstChildElement(element_name.c_str());
  EXPECT_TRUE(element != nullptr);

  try
  {
    type = func(element, locator, available_materials, version);
  }
  catch (const std::exception& e)
  {
    tesseract_common::printNestedException(e);
    return false;
  }

  return true;
}

template <typename ElementType>
bool runTest(ElementType& type,
             std::function<ElementType(const tinyxml2::XMLElement*,
                                       std::unordered_map<std::string, tesseract_scene_graph::Material::Ptr>&,
                                       bool,
                                       const int)> func,
             const std::string& xml_string,
             const std::string& element_name,
             std::unordered_map<std::string, tesseract_scene_graph::Material::Ptr>& available_materials,
             int version,
             const bool visual)
{
  tinyxml2::XMLDocument xml_doc;
  EXPECT_TRUE(xml_doc.Parse(xml_string.c_str()) == tinyxml2::XML_SUCCESS);

  tinyxml2::XMLElement* element = xml_doc.FirstChildElement(element_name.c_str());
  EXPECT_TRUE(element != nullptr);

  try
  {
    type = func(element, available_materials, visual, version);
  }
  catch (const std::exception& e)
  {
    tesseract_common::printNestedException(e);
    return false;
  }

  return true;
}

#endif  // TESSERACT_URDF_COMMON_UNIT_H
