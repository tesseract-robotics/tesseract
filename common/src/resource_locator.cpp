/**
 * @file resource_locator.cpp
 * @brief Resource locator functions
 *
 * @author John Wason
 * @date October 25, 2019
 *
 * @copyright Copyright (c) 2019, Wason Technology, LLC
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
#include <fstream>
#include <console_bridge/console.h>
#include <iostream>
#include <boost/algorithm/string/classification.hpp>
#include <boost/algorithm/string/split.hpp>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract/common/resource_locator.h>
#include <tesseract/common/types.h>
#include <tesseract/common/utils.h>

namespace tesseract::common
{
bool isRelativePath(const std::string& url)
{
  std::filesystem::path path(url);
  return (url.find("file:///") != 0 && url.find("package://") != 0 && path.is_relative());
}

bool ResourceLocator::operator==(const ResourceLocator& /*rhs*/) const { return true; }
bool ResourceLocator::operator!=(const ResourceLocator& /*rhs*/) const { return false; }

GeneralResourceLocator::GeneralResourceLocator(const std::vector<std::string>& environment_variables)
{
  for (const auto& env_variable : environment_variables)
  {
    loadEnvironmentVariable(env_variable);
  }
}

GeneralResourceLocator::GeneralResourceLocator(const std::vector<std::filesystem::path>& paths,
                                               const std::vector<std::string>& environment_variables)
{
  for (const auto& path : paths)
  {
    addPath(path);
  }

  for (const auto& env_variable : environment_variables)
  {
    loadEnvironmentVariable(env_variable);
  }
}

bool GeneralResourceLocator::loadEnvironmentVariable(const std::string& environment_variable)
{
  char* ros_package_paths = std::getenv(environment_variable.c_str());
  if (ros_package_paths != nullptr)
  {
    std::vector<std::string> tokens;
#ifndef _WIN32
    boost::split(tokens, ros_package_paths, boost::is_any_of(":"), boost::token_compress_on);
#else
    boost::split(tokens, ros_package_paths, boost::is_any_of(";"), boost::token_compress_on);
#endif
    for (const auto& token : tokens)
      processToken(token);

    return true;
  }
  return false;
}

bool GeneralResourceLocator::addPath(const std::filesystem::path& path)
{
  if (std::filesystem::is_directory(path) && std::filesystem::exists(path))
  {
    processToken(path.string());
    return true;
  }

  CONSOLE_BRIDGE_logError("Package Path does not exist: %s", path.string().c_str());
  return false;
}

void GeneralResourceLocator::processToken(const std::string& token)
{
  std::filesystem::path d(token);
  if (std::filesystem::is_directory(d) && std::filesystem::exists(d))
  {
    // Check current directory
    std::filesystem::path check = d;
    check.append("package.xml");
    if (std::filesystem::exists(check))
    {
      std::string dir_name = d.filename().string();
      if (package_paths_.find(dir_name) == package_paths_.end())
        package_paths_[dir_name] = d.string();
    }

    // Check all subdirectories
    std::filesystem::recursive_directory_iterator dir(d), end;
    while (dir != end)
    {
      std::filesystem::path check = dir->path();
      check.append("package.xml");
      if (std::filesystem::exists(check))
      {
        std::string dir_name = dir->path().filename().string();
        if (package_paths_.find(dir_name) == package_paths_.end())
          package_paths_[dir_name] = dir->path().string();

        dir.disable_recursion_pending();  // don't recurse into this directory.
      }

      ++dir;
    }
  }
  else
  {
    CONSOLE_BRIDGE_logError("Package Path does not exist: %s", token.c_str());
  }
}

std::size_t findSeparator(const std::string& str)
{
  const size_t pos_slash = str.find('/');
  const size_t pos_backslash = str.find('\\');

  if (pos_slash != std::string::npos && pos_backslash != std::string::npos)
    return std::min(pos_slash, pos_backslash);

  if (pos_slash != std::string::npos)
    return pos_slash;

  if (pos_backslash != std::string::npos)
    return pos_backslash;

  return std::string::npos;
}

std::shared_ptr<Resource> GeneralResourceLocator::locateResource(const std::string& url) const
{
  std::string mod_url = url;
  if (url.find("file:///") == 0)
  {
    mod_url.erase(0, strlen("file://"));
    const size_t pos = findSeparator(mod_url);
    if (pos == std::string::npos)
      return nullptr;
  }
  else if (url.find("package://") == 0)
  {
    mod_url.erase(0, strlen("package://"));
    const size_t pos = findSeparator(mod_url);
    if (pos == std::string::npos)
      return nullptr;

    std::string package = mod_url.substr(0, pos);
    mod_url.erase(0, pos);

    auto find_package = package_paths_.find(package);
    if (find_package != package_paths_.end())
    {
      mod_url = find_package->second + mod_url;
    }
    else
    {
      CONSOLE_BRIDGE_logError("Failed to find package resource %s for %s", package.c_str(), url.c_str());
      return nullptr;
    }
  }

  if (!std::filesystem::path(mod_url).is_absolute())
  {
    CONSOLE_BRIDGE_logWarn("Resource not handled: %s", mod_url.c_str());
    return nullptr;
  }

  return std::make_shared<SimpleLocatedResource>(url, mod_url, std::make_shared<GeneralResourceLocator>(*this));
}

bool GeneralResourceLocator::operator==(const GeneralResourceLocator& rhs) const
{
  return tesseract::common::isIdenticalMap<std::unordered_map<std::string, std::string>, std::string>(
      package_paths_, rhs.package_paths_);
}
bool GeneralResourceLocator::operator!=(const GeneralResourceLocator& rhs) const { return !operator==(rhs); }

bool Resource::operator==(const Resource& /*rhs*/) const { return true; }
bool Resource::operator!=(const Resource& /*rhs*/) const { return false; }

SimpleLocatedResource::SimpleLocatedResource(std::string url, std::string filename, ResourceLocator::ConstPtr parent)
  : url_(std::move(url)), filename_(std::move(filename)), parent_(std::move(parent))
{
}

bool SimpleLocatedResource::isFile() const { return true; }

std::string SimpleLocatedResource::getUrl() const { return url_; }

std::string SimpleLocatedResource::getFilePath() const { return filename_; }

std::vector<uint8_t> SimpleLocatedResource::getResourceContents() const
{
  // https://codereview.stackexchange.com/questions/22901/reading-all-bytes-from-a-file

  std::ifstream ifs(filename_, std::ios::binary | std::ios::ate);
  if (ifs.fail())
  {
    CONSOLE_BRIDGE_logError("Could not read all bytes from file: %s", filename_.c_str());
    return {};
  }
  std::ifstream::pos_type pos = ifs.tellg();

  std::vector<uint8_t> file_contents(static_cast<size_t>(pos));

  ifs.seekg(0, std::ios::beg);
  ifs.read(reinterpret_cast<std::ifstream::char_type*>(&file_contents[0]), pos);  // NOLINT

  return file_contents;
}

std::shared_ptr<std::istream> SimpleLocatedResource::getResourceContentStream() const
{
  std::shared_ptr<std::ifstream> ifs = std::make_shared<std::ifstream>(filename_, std::ios::binary);
  if (ifs->fail())
  {
    CONSOLE_BRIDGE_logError("Could not get resource: %s", filename_.c_str());
    return nullptr;
  }
  return ifs;
}

tesseract::common::Resource::Ptr SimpleLocatedResource::locateResource(const std::string& url) const
{
  if (parent_ == nullptr || url.empty())
    return nullptr;

  if (isRelativePath(url))
  {
    // Find the last occurrences of both separators
    std::size_t last_slash = url_.find_last_of('/');
    std::size_t last_backslash = url_.find_last_of('\\');
    std::size_t last_separator{ 0 };
    if (last_slash != std::string::npos && last_backslash != std::string::npos)
      last_separator = std::max(last_slash, last_backslash);
    else if (last_slash != std::string::npos)
      last_separator = last_slash;
    else if (last_backslash != std::string::npos)
      last_separator = last_backslash;
    else
      return nullptr;

    std::filesystem::path path(url);
    std::string url_base_path = url_.substr(0, last_separator);
    std::string new_url =
        url_base_path + std::string(1, std::filesystem::path::preferred_separator) + path.filename().string();
    CONSOLE_BRIDGE_logDebug("new_url: %s", new_url.c_str());
    return parent_->locateResource(new_url);
  }

  return parent_->locateResource(url);
}

bool SimpleLocatedResource::operator==(const SimpleLocatedResource& rhs) const
{
  bool equal = true;
  equal &= Resource::operator==(rhs);
  equal &= url_ == rhs.url_;
  equal &= filename_ == rhs.filename_;
  equal &= tesseract::common::pointersEqual(parent_, rhs.parent_);
  return equal;
}

bool SimpleLocatedResource::operator!=(const SimpleLocatedResource& rhs) const { return !operator==(rhs); }

BytesResource::BytesResource(std::string url, std::vector<uint8_t> bytes, ResourceLocator::ConstPtr parent)
  : url_(std::move(url)), bytes_(std::move(bytes)), parent_(std::move(parent))
{
}

BytesResource::BytesResource(std::string url, const uint8_t* bytes, size_t bytes_len, ResourceLocator::ConstPtr parent)
  : url_(std::move(url))
  , bytes_(std::vector<uint8_t>(bytes, bytes + bytes_len))  // NOLINT
  , parent_(std::move(parent))
{
}

bool BytesResource::isFile() const { return false; }
std::string BytesResource::getUrl() const { return url_; }
std::string BytesResource::getFilePath() const { return ""; }
std::vector<uint8_t> BytesResource::getResourceContents() const { return bytes_; }
std::shared_ptr<std::istream> BytesResource::getResourceContentStream() const
{
  std::shared_ptr<std::stringstream> o = std::make_shared<std::stringstream>();
  o->write((const char*)&bytes_.at(0), static_cast<std::streamsize>(bytes_.size()));  // NOLINT
  o->seekg(0, std::stringstream::beg);
  return o;
}

Resource::Ptr BytesResource::locateResource(const std::string& url) const
{
  if (parent_ == nullptr || url.empty())
    return nullptr;

  tesseract::common::Resource::Ptr resource = parent_->locateResource(url);
  if (resource != nullptr)
    return resource;

  if (!isRelativePath(url))
    return nullptr;

  auto last_slash = url_.find_last_of('/');
  if (last_slash == std::string::npos)
    return nullptr;

  std::filesystem::path path(url);
  std::string url_base_path = url_.substr(0, last_slash);
  std::string new_url = url_base_path + "/" + path.filename().string();
  return parent_->locateResource(new_url);
}

bool BytesResource::operator==(const BytesResource& rhs) const
{
  bool equal = true;
  equal &= Resource::operator==(rhs);
  equal &= url_ == rhs.url_;
  equal &= bytes_ == rhs.bytes_;
  equal &= tesseract::common::pointersEqual(parent_, rhs.parent_);
  return equal;
}

bool BytesResource::operator!=(const BytesResource& rhs) const { return !operator==(rhs); }

}  // namespace tesseract::common
