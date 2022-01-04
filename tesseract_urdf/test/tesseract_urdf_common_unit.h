#ifndef TESSERACT_URDF_COMMON_UNIT_H
#define TESSERACT_URDF_COMMON_UNIT_H

#include <tesseract_common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <fstream>
#include <iostream>
#include <vector>

#include <gtest/gtest.h>
#include <console_bridge/console.h>
#include <tesseract_common/resource_locator.h>
#include <tesseract_common/utils.h>
#include <tesseract_urdf/urdf_parser.h>
#include <tinyxml2.h>
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
    std::function<ElementType(const tinyxml2::XMLElement*, const tesseract_common::ResourceLocator&, bool, const int)>
        func,
    const std::string& xml_string,
    const std::string& element_name,
    const tesseract_common::ResourceLocator& locator,
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
    std::function<ElementType(const tinyxml2::XMLElement*, const tesseract_common::ResourceLocator&, const int)> func,
    const std::string& xml_string,
    const std::string& element_name,
    const tesseract_common::ResourceLocator& locator,
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
                                       const tesseract_common::ResourceLocator&,
                                       std::unordered_map<std::string, tesseract_scene_graph::Material::Ptr>&,
                                       const int)> func,
             const std::string& xml_string,
             const std::string& element_name,
             const tesseract_common::ResourceLocator& locator,
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

/**
 * @brief stripNewline - For some reason, tinyxml2 printers add a trailing newline.  This function removes it.
 * @param in - input string
 * @return out - string equal to input string with trailing newlines removed.
 */
inline std::string stripNewline(const std::string& in)
{
  std::string out;
  if (!in.empty() && in.back() == '\n')
    out = in.substr(0, in.size() - 1);
  else
    out = in;
  return out;
}

inline std::string toString(const tinyxml2::XMLElement* element)
{
  std::string ret;
  if (element != nullptr)
  {
    tinyxml2::XMLPrinter printer;
    element->Accept(&printer);
    std::stringstream ss;
    ss << printer.CStr();
    ret = stripNewline(ss.str());
  }
  return ret;
}

/**
 * @brief writeTest - test write functions
 * @param type - object of type being tested
 * @param func - function to write object to URDF-XML
 * @param text - xml text generated
 * @return 0 if success, 1 if exception thrown, 2 if nullptr generated
 */
template <typename TessType>
int writeTest(TessType& type,
              std::function<tinyxml2::XMLElement*(const TessType&, tinyxml2::XMLDocument&)> func,
              std::string& text)
{
  tinyxml2::XMLDocument doc;
  int status = 0;
  try
  {
    tinyxml2::XMLElement* element = func(type, doc);
    text = toString(element);
    if (element != nullptr)
      status = 0;
    else
      status = 2;
  }
  catch (...)
  {
    text = "";
    status = 1;
  }
  return status;
}

/**
 * @brief writeTest - test write functions for links
 * @param type - object of type being tested
 * @param func - function to write object to URDF-XML
 * @param text - xml text generated
 * @param directory - directory to save files
 * @return 0 if success, 1 if exception thrown, 2 if nullptr generated
 */
template <typename TessType>
int writeTest(TessType& type,
              std::function<tinyxml2::XMLElement*(const TessType&, tinyxml2::XMLDocument&, const std::string&)> func,
              std::string& text,
              const std::string& directory)
{
  tinyxml2::XMLDocument doc;
  int status = 0;
  try
  {
    tinyxml2::XMLElement* element = func(type, doc, directory);
    text = toString(element);
    if (element != nullptr)
      status = 0;
    else
      status = 2;
  }
  catch (...)
  {
    text = "";
    status = 1;
  }
  return status;
}

/**
 * @brief writeTest - test write functions for meshes
 * @param type - object of type being tested
 * @param func - function to write object to URDF-XML
 * @param text - xml text generated
 * @param directory - directory to save files
 * @param filename - name of link to which geometry is attached
 * @return 0 if success, 1 if exception thrown, 2 if nullptr generated
 */
template <typename TessType>
int writeTest(
    TessType& type,
    std::function<
        tinyxml2::XMLElement*(const TessType&, tinyxml2::XMLDocument&, const std::string&, const std::string&)> func,
    std::string& text,
    const std::string& directory,
    const std::string& filename)
{
  tinyxml2::XMLDocument doc;
  int status = 0;
  try
  {
    tinyxml2::XMLElement* element = func(type, doc, directory, filename);
    text = toString(element);
    if (element != nullptr)
      status = 0;
    else
      status = 2;
  }
  catch (...)
  {
    text = "";
    status = 1;
  }
  return status;
}

/**
 * @brief writeTest - test write functions for collision and visual geometries
 * @param type - object of type being tested
 * @param func - function to write object to URDF-XML
 * @param text - xml text generated
 * @param directory - directory to save files
 * @param link_name - name of link to which geometry is attached
 * @param id - index of this geometry in list
 * @return 0 if success, 1 if exception thrown, 2 if nullptr generated
 */
template <typename TessType>
int writeTest(TessType& type,
              std::function<tinyxml2::XMLElement*(const TessType&,
                                                  tinyxml2::XMLDocument&,
                                                  const std::string&,
                                                  const std::string&,
                                                  const int)> func,
              std::string& text,
              const std::string& directory,
              const std::string& link_name,
              const int id)
{
  tinyxml2::XMLDocument doc;
  int status = 0;
  try
  {
    tinyxml2::XMLElement* element = func(type, doc, directory, link_name, id);
    text = toString(element);
    if (element != nullptr)
      status = 0;
    else
      status = 2;
  }
  catch (...)
  {
    text = "";
    status = 1;
  }
  return status;
}

inline void writeTextToFile(const std::string& path, const std::string& text)
{
  std::ofstream file;
  file.open(path);
  file << text;
  file.close();
}

#endif  // TESSERACT_URDF_COMMON_UNIT_H
