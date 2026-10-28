/**
 * @file signed_distance_field.cpp
 * @brief Parse signed distance field from xml string
 *
 * @author Joel Kang
 * @date June 26, 2026
 *
 * @copyright Copyright (c) 2026, Tesseract
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

#include <tesseract/common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <algorithm>
#include <cctype>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <locale>
#include <sstream>
#include <stdexcept>
#include <vector>

#include <boost/algorithm/string/classification.hpp>
#include <boost/algorithm/string/split.hpp>
#include <Eigen/Geometry>
#include <tesseract/common/utils.h>
#include <tinyxml2.h>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract/geometry/impl/signed_distance_field.h>
#include <tesseract/geometry/impl/signed_distance_field_utils.h>
#include <tesseract/common/resource_locator.h>
#include <tesseract/urdf/signed_distance_field.h>
#include <tesseract/urdf/utils.h>

namespace tesseract::urdf
{
namespace
{
bool hasExtension(const std::string& filename, const std::string& extension)
{
  std::string actual = std::filesystem::path(filename).extension().string();
  std::transform(actual.begin(), actual.end(), actual.begin(), [](unsigned char value) {
    return static_cast<char>(std::tolower(value));
  });
  return actual == extension;
}

/** @brief Parse @p element's optional `band_clamp` attribute into @p band_clamp; false when absent. */
bool parseBandClamp(const tinyxml2::XMLElement* element, double& band_clamp)
{
  std::string clamp_string;
  if (tesseract::common::QueryStringAttribute(element, "band_clamp", clamp_string) != tinyxml2::XML_SUCCESS)
    return false;
  if (!tesseract::common::isNumeric(clamp_string) || !tesseract::common::toNumeric<double>(clamp_string, band_clamp))
    std::throw_with_nested(std::runtime_error("SignedDistanceField: Failed parsing attribute 'band_clamp'!"));
  return true;
}

/** @brief @p value with full precision in the classic locale, for an XML attribute. */
std::string toAttribute(double value)
{
  std::stringstream text;
  text.imbue(std::locale::classic());
  text.precision(17);
  text << value;
  return text.str();
}
}  // namespace

tesseract::geometry::SignedDistanceField::Ptr
parseSignedDistanceField(const tinyxml2::XMLElement* xml_element, const tesseract::common::ResourceLocator& locator)
{
  std::string filename;
  if (tesseract::common::QueryStringAttribute(xml_element, "filename", filename) != tinyxml2::XML_SUCCESS)
    std::throw_with_nested(std::runtime_error("SignedDistanceField: Missing or failed parsing attribute 'filename'!"));

  std::string scale_string;
  Eigen::Vector3d scale(1, 1, 1);
  if (tesseract::common::QueryStringAttribute(xml_element, "scale", scale_string) == tinyxml2::XML_SUCCESS)
  {
    std::vector<std::string> tokens;
    boost::split(tokens, scale_string, boost::is_any_of(" "), boost::token_compress_on);
    if (tokens.size() != 3 || !tesseract::common::isNumeric(tokens))
      std::throw_with_nested(std::runtime_error("SignedDistanceField: Failed parsing attribute 'scale'!"));

    double sx{ 0 }, sy{ 0 }, sz{ 0 };
    // No need to check return values because the tokens are verified above
    tesseract::common::toNumeric<double>(tokens[0], sx);
    tesseract::common::toNumeric<double>(tokens[1], sy);
    tesseract::common::toNumeric<double>(tokens[2], sz);

    if (!(sx > 0))
      std::throw_with_nested(std::runtime_error("SignedDistanceField: Scale x is not greater than zero!"));

    if (!(sy > 0))
      std::throw_with_nested(std::runtime_error("SignedDistanceField: Scale y is not greater than zero!"));

    if (!(sz > 0))
      std::throw_with_nested(std::runtime_error("SignedDistanceField: Scale z is not greater than zero!"));

    scale = Eigen::Vector3d(sx, sy, sz);
  }

  // An .nvdb file is read sparse, kept as stored, when the element names its grid or band clamp:
  // `grid` picks one grid of a file holding several, `band_clamp` is that grid's clamp (default 0,
  // the values never saturate). Without either the file is read as one dense grid.
  std::string grid_name;
  const bool has_grid =
      tesseract::common::QueryStringAttribute(xml_element, "grid", grid_name) == tinyxml2::XML_SUCCESS;
  double band_clamp{ 0 };
  const bool has_clamp = parseBandClamp(xml_element, band_clamp);
  const bool sparse = has_grid || has_clamp;

  // `level` children, finest first, stack grids of the file as levels of one surface (see the
  // multi-level SignedDistanceField constructor): each names its grid by `grid` or `index` and gives
  // its `band_clamp`.
  struct Level
  {
    std::string grid;
    std::size_t index{ 0 };
    bool by_index{ false };
    double band_clamp{ 0 };
  };
  std::vector<Level> level_specs;
  for (const tinyxml2::XMLElement* child = xml_element->FirstChildElement("level"); child != nullptr;
       child = child->NextSiblingElement("level"))
  {
    Level level;
    const bool named = tesseract::common::QueryStringAttribute(child, "grid", level.grid) == tinyxml2::XML_SUCCESS;
    std::string index_string;
    level.by_index = tesseract::common::QueryStringAttribute(child, "index", index_string) == tinyxml2::XML_SUCCESS;
    if (named == level.by_index)
      std::throw_with_nested(std::runtime_error("SignedDistanceField: a level needs exactly one of 'grid' and "
                                                "'index'!"));
    int index{ -1 };
    if (level.by_index && (!tesseract::common::toNumeric<int>(index_string, index) || index < 0))
      std::throw_with_nested(std::runtime_error("SignedDistanceField: Failed parsing level attribute 'index'!"));
    level.index = static_cast<std::size_t>(index);
    parseBandClamp(child, level.band_clamp);
    level_specs.push_back(level);
  }
  if (!level_specs.empty() && sparse)
    std::throw_with_nested(std::runtime_error("SignedDistanceField: 'grid' and 'band_clamp' belong on the levels of a "
                                              "multi-level field!"));

  tesseract::common::Resource::Ptr resource = locator.locateResource(filename);
  if (resource == nullptr)
    std::throw_with_nested(std::runtime_error("SignedDistanceField: Missing resource '" + filename + "'!"));

  const std::vector<std::uint8_t> data = resource->getResourceContents();
  if (data.empty())
    std::throw_with_nested(std::runtime_error("SignedDistanceField: Error importing from '" + filename + "'!"));

  tesseract::geometry::SignedDistanceField::Ptr geom;
  try
  {
    if ((sparse || !level_specs.empty()) && !hasExtension(filename, ".nvdb"))
      throw std::runtime_error("'grid', 'band_clamp' and levels need an .nvdb file");
    if (!level_specs.empty())
    {
      std::vector<tesseract::geometry::SignedDistanceField::ConstPtr> levels;
      levels.reserve(level_specs.size());
      for (const Level& level : level_specs)
        levels.push_back(
            level.by_index ?
                tesseract::geometry::readSparseSignedDistanceFieldNVDB(data, level.band_clamp, scale, level.index) :
                tesseract::geometry::readSparseSignedDistanceFieldNVDB(data, level.band_clamp, scale, level.grid));
      geom = std::make_shared<tesseract::geometry::SignedDistanceField>(std::move(levels));
    }
    else if (hasExtension(filename, ".vdb"))
      geom = tesseract::geometry::readSignedDistanceFieldVDB(data, scale);
    else if (hasExtension(filename, ".nvdb") && sparse)
      geom = tesseract::geometry::readSparseSignedDistanceFieldNVDB(data, band_clamp, scale, grid_name);
    else if (hasExtension(filename, ".nvdb"))
      geom = tesseract::geometry::readSignedDistanceFieldNVDB(data, scale);
    else
      throw std::runtime_error("unsupported file extension");
  }
  catch (...)
  {
    std::throw_with_nested(
        std::runtime_error("SignedDistanceField: Failed parsing field data from '" + filename + "'!"));
  }

  return geom;
}

tinyxml2::XMLElement* writeSignedDistanceField(const tesseract::geometry::SignedDistanceField::ConstPtr& sdf,
                                               tinyxml2::XMLDocument& doc,
                                               const std::string& package_path,
                                               const std::string& filename)
{
  if (sdf == nullptr)
    std::throw_with_nested(std::runtime_error("Signed distance field is nullptr and cannot be converted to XML"));
  tinyxml2::XMLElement* xml_element = doc.NewElement(SIGNED_DISTANCE_FIELD_ELEMENT_NAME.data());
  Eigen::IOFormat eigen_format(Eigen::StreamPrecision, Eigen::DontAlignCols, " ", " ");

  std::string filepath = trailingSlash(package_path) + noLeadingSlash(filename);
  std::vector<std::uint8_t> data;
  if (hasExtension(filename, ".vdb"))
    data = tesseract::geometry::writeSignedDistanceFieldVDB(*sdf);
  else if (hasExtension(filename, ".nvdb"))
    data = tesseract::geometry::writeSignedDistanceFieldNVDB(*sdf);
  else
    throw std::runtime_error("SignedDistanceField: unsupported file extension for '" + filename + "'");
  std::ofstream out(filepath, std::ios::out | std::ios::binary | std::ios::trunc);
  if (!out.is_open())
    std::throw_with_nested(std::runtime_error("Could not open signed distance field file `" + filepath + "`!"));
  // NOLINTNEXTLINE(cppcoreguidelines-pro-type-reinterpret-cast) byte blob -> char buffer for binary write
  out.write(reinterpret_cast<const char*>(data.data()), static_cast<std::streamsize>(data.size()));
  if (out.fail())
    std::throw_with_nested(std::runtime_error("Could not write signed distance field to file `" + filepath + "`!"));
  out.close();

  xml_element->SetAttribute("filename", makeURDFFilePath(package_path, filename).c_str());

  if (!sdf->getScale().isOnes())
  {
    std::stringstream scale_string;
    scale_string << sdf->getScale().format(eigen_format);
    xml_element->SetAttribute("scale", scale_string.str().c_str());
  }

  // A multi-level field's file holds its levels' grids in order, which the levels name by index (their
  // own names need not differ). A single grid's file holds just it, so the clamp alone marks it sparse.
  const auto& levels = sdf->getLevels();
  for (std::size_t i = 0; i < levels.size(); ++i)
  {
    tinyxml2::XMLElement* level = doc.NewElement("level");
    level->SetAttribute("index", static_cast<unsigned>(i));
    level->SetAttribute("band_clamp", toAttribute(levels[i]->getBandClamp()).c_str());
    xml_element->InsertEndChild(level);
  }
  if (sdf->isSparse() && levels.empty())
    xml_element->SetAttribute("band_clamp", toAttribute(sdf->getBandClamp()).c_str());

  return xml_element;
}

}  // namespace tesseract::urdf
