#ifndef TESSERACT_URDF_COMMON_UNIT_H
#define TESSERACT_URDF_COMMON_UNIT_H

#include <tesseract_common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <vector>
#include <tinyxml2.h>
#include <console_bridge/console.h>
#include <tesseract_common/status_code.h>
#include <tesseract_scene_graph/utils.h>
#include <tesseract_scene_graph/resource_locator.h>
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
tesseract_common::StatusCode::Ptr
runTest(ElementType& type, const std::string& xml_string, const std::string& element_name, const int version)
{
  tinyxml2::XMLDocument xml_doc;
  EXPECT_TRUE(xml_doc.Parse(xml_string.c_str()) == tinyxml2::XML_SUCCESS);

  tinyxml2::XMLElement* element = xml_doc.FirstChildElement(element_name.c_str());
  EXPECT_TRUE(element != nullptr);

  auto status = tesseract_urdf::parse(type, element, version);
  if (!(*status))
  {
    CONSOLE_BRIDGE_logError(status->message().c_str());
  }

  return status;
}

template <typename ElementType>
tesseract_common::StatusCode::Ptr runTest(ElementType& type,
                                          const std::string& xml_string,
                                          const std::string& element_name,
                                          const tesseract_scene_graph::ResourceLocator::Ptr& locator,
                                          const int version,
                                          bool visual)
{
  tinyxml2::XMLDocument xml_doc;
  EXPECT_TRUE(xml_doc.Parse(xml_string.c_str()) == tinyxml2::XML_SUCCESS);

  tinyxml2::XMLElement* element = xml_doc.FirstChildElement(element_name.c_str());
  EXPECT_TRUE(element != nullptr);

  auto status = tesseract_urdf::parse(type, element, locator, visual, version);
  if (!(*status))
  {
    CONSOLE_BRIDGE_logError(status->message().c_str());
  }

  return status;
}

template <typename ElementType>
tesseract_common::StatusCode::Ptr runTest(ElementType& type,
                                          const std::string& xml_string,
                                          const std::string& element_name,
                                          const tesseract_scene_graph::ResourceLocator::Ptr& locator,
                                          const int version)
{
  tinyxml2::XMLDocument xml_doc;
  EXPECT_TRUE(xml_doc.Parse(xml_string.c_str()) == tinyxml2::XML_SUCCESS);

  tinyxml2::XMLElement* element = xml_doc.FirstChildElement(element_name.c_str());
  EXPECT_TRUE(element != nullptr);

  auto status = tesseract_urdf::parse(type, element, locator, version);
  if (!(*status))
  {
    CONSOLE_BRIDGE_logError(status->message().c_str());
  }

  return status;
}

template <typename ElementType>
tesseract_common::StatusCode::Ptr
runTest(ElementType& type,
        const std::string& xml_string,
        const std::string& element_name,
        const tesseract_scene_graph::ResourceLocator::Ptr& locator,
        std::unordered_map<std::string, tesseract_scene_graph::Material::Ptr>& available_materials,
        const int version)
{
  tinyxml2::XMLDocument xml_doc;
  EXPECT_TRUE(xml_doc.Parse(xml_string.c_str()) == tinyxml2::XML_SUCCESS);

  tinyxml2::XMLElement* element = xml_doc.FirstChildElement(element_name.c_str());
  EXPECT_TRUE(element != nullptr);

  auto status = tesseract_urdf::parse(type, element, locator, available_materials, version);
  if (!(*status))
  {
    CONSOLE_BRIDGE_logError(status->message().c_str());
  }

  return status;
}

template <typename ElementType>
tesseract_common::StatusCode::Ptr
runTest(ElementType& type,
        const std::string& xml_string,
        const std::string& element_name,
        std::unordered_map<std::string, tesseract_scene_graph::Material::Ptr>& available_materials,
        const int version,
        const bool visual)
{
  tinyxml2::XMLDocument xml_doc;
  EXPECT_TRUE(xml_doc.Parse(xml_string.c_str()) == tinyxml2::XML_SUCCESS);

  tinyxml2::XMLElement* element = xml_doc.FirstChildElement(element_name.c_str());
  EXPECT_TRUE(element != nullptr);

  auto status = tesseract_urdf::parse(type, element, available_materials, visual, version);
  if (!(*status))
  {
    CONSOLE_BRIDGE_logError(status->message().c_str());
  }

  return status;
}

#endif  // TESSERACT_URDF_COMMON_UNIT_H
