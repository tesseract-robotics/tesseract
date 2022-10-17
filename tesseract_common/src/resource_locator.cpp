/**
 * @file resource_locator.cpp
 * @brief Resource locator functions
 *
 * @author John Wason
 * @date October 25, 2019
 * @version TODO
 * @bug No known bugs
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

#include <tesseract_common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <fstream>
#include <console_bridge/console.h>
#include <cassert>
#include <iostream>
#include <mutex>
#include <boost/serialization/nvp.hpp>
#include <boost/serialization/shared_ptr.hpp>
#include <boost/serialization/vector.hpp>
#include <boost/algorithm/string/classification.hpp>
#include <boost/algorithm/string/split.hpp>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract_common/resource_locator.h>
#include <tesseract_common/types.h>
#include <tesseract_common/utils.h>

namespace tesseract_common
{
bool ResourceLocator::operator==(const ResourceLocator& /*rhs*/) const { return true; }
bool ResourceLocator::operator!=(const ResourceLocator& /*rhs*/) const { return false; }

template <class Archive>
void ResourceLocator::serialize(Archive& /*ar*/, const unsigned int /*version*/)
{
}

GeneralResourceLocator::GeneralResourceLocator()
{
  // This was added to allow user defined resource path
  // When using this within a snap you can map host ros package paths to this environment variable
  char* tesseract_resource_paths = std::getenv("TESSERACT_RESOURCE_PATH");
  if (tesseract_resource_paths != nullptr)
  {
    std::vector<std::string> tokens;
#ifndef _WIN32
    boost::split(tokens, tesseract_resource_paths, boost::is_any_of(":"), boost::token_compress_on);
#else
    boost::split(tokens, tesseract_resource_paths, boost::is_any_of(";"), boost::token_compress_on);
#endif
    for (const auto& token : tokens)
    {
      tesseract_common::fs::path d(token);
      if (tesseract_common::fs::is_directory(d) && tesseract_common::fs::exists(d))
      {
        std::string dir_name = d.filename().string();
        if (package_paths_.find(dir_name) == package_paths_.end())
          package_paths_[dir_name] = token;
      }
      else
      {
        CONSOLE_BRIDGE_logWarn("Package Path does not exist: %s", token.c_str());
      }
    }
  }

  char* ros_package_paths = std::getenv("ROS_PACKAGE_PATH");
  if (ros_package_paths != nullptr)
  {
    std::vector<std::string> tokens;
#ifndef _WIN32
    boost::split(tokens, ros_package_paths, boost::is_any_of(":"), boost::token_compress_on);
#else
    boost::split(tokens, ros_package_paths, boost::is_any_of(";"), boost::token_compress_on);
#endif
    for (const auto& token : tokens)
    {
      tesseract_common::fs::path d(token);
      if (tesseract_common::fs::is_directory(d) && tesseract_common::fs::exists(d))
      {
        std::string dir_name = d.filename().string();
        if (package_paths_.find(dir_name) == package_paths_.end())
          package_paths_[dir_name] = token;
      }
      else
      {
        CONSOLE_BRIDGE_logError("Package Path does not exist: &s", token.c_str());
      }
    }
  }
}

std::shared_ptr<Resource> GeneralResourceLocator::locateResource(const std::string& url) const
{
  std::string mod_url = url;
  if (url.find("file:///") == 0)
  {
    mod_url.erase(0, strlen("file://"));
    size_t pos = mod_url.find('/');
    if (pos == std::string::npos)
      return nullptr;
  }
  else if (url.find("package://") == 0)
  {
    mod_url.erase(0, strlen("package://"));
    size_t pos = mod_url.find('/');
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

  if (!tesseract_common::fs::path(mod_url).is_complete())
  {
    CONSOLE_BRIDGE_logWarn("Resource not handled: %s", mod_url.c_str());
    return nullptr;
  }

  return std::make_shared<SimpleLocatedResource>(url, mod_url, std::make_shared<GeneralResourceLocator>(*this));
}

bool GeneralResourceLocator::operator==(const GeneralResourceLocator& /*rhs*/) const { return true; }
bool GeneralResourceLocator::operator!=(const GeneralResourceLocator& /*rhs*/) const { return false; }

template <class Archive>
void GeneralResourceLocator::serialize(Archive& ar, const unsigned int /*version*/)
{
  ar& BOOST_SERIALIZATION_BASE_OBJECT_NVP(ResourceLocator);
}

bool Resource::operator==(const Resource& /*rhs*/) const { return true; }
bool Resource::operator!=(const Resource& /*rhs*/) const { return false; }

template <class Archive>
void Resource::serialize(Archive& /*ar*/, const unsigned int /*version*/)
{
}

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

tesseract_common::Resource::Ptr SimpleLocatedResource::locateResource(const std::string& url) const
{
  if (parent_ == nullptr || url.empty())
    return nullptr;

  tesseract_common::Resource::Ptr resource = parent_->locateResource(url);
  if (resource != nullptr)
    return resource;

  tesseract_common::fs::path path(url);
  if (!path.is_relative())
    return nullptr;

  auto last_slash = url_.find_last_of('/');
  if (last_slash == std::string::npos)
    return nullptr;

  std::string url_base_path = url_.substr(0, last_slash);
  std::string new_url = url_base_path + "/" + path.filename().string();
  return parent_->locateResource(new_url);
}

bool SimpleLocatedResource::operator==(const SimpleLocatedResource& rhs) const
{
  bool equal = true;
  equal &= Resource::operator==(rhs);
  equal &= url_ == rhs.url_;
  equal &= filename_ == rhs.filename_;
  equal &= tesseract_common::pointersEqual(parent_, rhs.parent_);
  return equal;
}

bool SimpleLocatedResource::operator!=(const SimpleLocatedResource& rhs) const { return !operator==(rhs); }

template <class Archive>
void SimpleLocatedResource::serialize(Archive& ar, const unsigned int /*version*/)
{
  ar& BOOST_SERIALIZATION_BASE_OBJECT_NVP(Resource);
  ar& BOOST_SERIALIZATION_NVP(url_);
  ar& BOOST_SERIALIZATION_NVP(filename_);
  ar& BOOST_SERIALIZATION_NVP(parent_);
}

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
  o->seekg(0, o->beg);
  return o;
}

Resource::Ptr BytesResource::locateResource(const std::string& url) const
{
  if (parent_ == nullptr || url.empty())
    return nullptr;

  tesseract_common::Resource::Ptr resource = parent_->locateResource(url);
  if (resource != nullptr)
    return resource;

  tesseract_common::fs::path path(url);
  if (!path.is_relative())
    return nullptr;

  auto last_slash = url_.find_last_of('/');
  if (last_slash == std::string::npos)
    return nullptr;

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
  equal &= tesseract_common::pointersEqual(parent_, rhs.parent_);
  return equal;
}

bool BytesResource::operator!=(const BytesResource& rhs) const { return !operator==(rhs); }

template <class Archive>
void BytesResource::serialize(Archive& ar, const unsigned int /*version*/)
{
  ar& BOOST_SERIALIZATION_BASE_OBJECT_NVP(Resource);
  ar& BOOST_SERIALIZATION_NVP(url_);
  ar& BOOST_SERIALIZATION_NVP(bytes_);
  ar& BOOST_SERIALIZATION_NVP(parent_);
}

}  // namespace tesseract_common

#include <tesseract_common/serialization.h>
TESSERACT_SERIALIZE_ARCHIVES_INSTANTIATE(tesseract_common::ResourceLocator)
TESSERACT_SERIALIZE_ARCHIVES_INSTANTIATE(tesseract_common::GeneralResourceLocator)
TESSERACT_SERIALIZE_ARCHIVES_INSTANTIATE(tesseract_common::Resource)
TESSERACT_SERIALIZE_ARCHIVES_INSTANTIATE(tesseract_common::SimpleLocatedResource)
TESSERACT_SERIALIZE_ARCHIVES_INSTANTIATE(tesseract_common::BytesResource)
BOOST_CLASS_EXPORT_IMPLEMENT(tesseract_common::GeneralResourceLocator)
BOOST_CLASS_EXPORT_IMPLEMENT(tesseract_common::SimpleLocatedResource)
BOOST_CLASS_EXPORT_IMPLEMENT(tesseract_common::BytesResource)
