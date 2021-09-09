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
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract_common/resource_locator.h>
#include <tesseract_common/types.h>

namespace tesseract_common
{
SimpleResourceLocator::SimpleResourceLocator(SimpleResourceLocatorFn locator_function)
  : locator_function_(std::move(locator_function))
{
  assert(locator_function_);
}

tesseract_common::Resource::Ptr SimpleResourceLocator::locateResource(const std::string& url) const
{
  std::string filename = locator_function_(url);
  if (!tesseract_common::fs::path(filename).is_complete())
    return nullptr;
  return std::make_shared<SimpleLocatedResource>(url, filename, std::make_shared<SimpleResourceLocator>(*this));
}

SimpleLocatedResource::SimpleLocatedResource(const std::string& url,
                                             const std::string& filename,
                                             const SimpleResourceLocator::ConstPtr& parent)
{
  url_ = url;
  filename_ = filename;
  parent_ = parent;
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
    return std::vector<uint8_t>();
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
  if (parent_ == nullptr)
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

BytesResource::BytesResource(std::string url, std::vector<uint8_t> bytes)
{
  url_ = std::move(url);
  bytes_ = std::move(bytes);
}

BytesResource::BytesResource(std::string url, const uint8_t* bytes, size_t bytes_len)
{
  url_ = std::move(url);
  bytes_ = std::vector<uint8_t>(bytes, bytes + bytes_len);  // NOLINT
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

Resource::Ptr BytesResource::locateResource(const std::string& /*url*/) const { return nullptr; }

}  // namespace tesseract_common
