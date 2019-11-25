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
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract_scene_graph/resource_locator.h>

namespace tesseract_scene_graph
{
SimpleResourceLocator::SimpleResourceLocator(SimpleResourceLocator::ResourceLocatorFn locator_function)
  : locator_function_(std::move(locator_function))
{
  assert(locator_function_);
}

tesseract_common::Resource::Ptr SimpleResourceLocator::locateResource(const std::string& url)
{
  std::string filename = locator_function_(url);
  if (filename.empty())
    return nullptr;
  return std::make_shared<SimpleLocatedResource>(url, filename);
}

SimpleLocatedResource::SimpleLocatedResource(const std::string& url, const std::string& filename)
{
  url_ = url;
  filename_ = filename;
}

bool SimpleLocatedResource::isFile() { return true; }

std::string SimpleLocatedResource::getUrl() { return url_; }

std::string SimpleLocatedResource::getFilePath() { return filename_; }

std::vector<uint8_t> SimpleLocatedResource::getResourceContents()
{
  // https://codereview.stackexchange.com/questions/22901/reading-all-bytes-from-a-file

  std::ifstream ifs(filename_, std::ios::binary | std::ios::ate);
  if (!ifs)
  {
    return std::vector<uint8_t>();
  }
  std::ifstream::pos_type pos = ifs.tellg();

  std::vector<uint8_t> file_contents(static_cast<size_t>(pos));

  ifs.seekg(0, std::ios::beg);
  ifs.read(reinterpret_cast<std::ifstream::char_type*>(&file_contents[0]), pos);

  return file_contents;
}

std::shared_ptr<std::istream> SimpleLocatedResource::getResourceContentStream()
{
  std::shared_ptr<std::ifstream> f = std::make_shared<std::ifstream>(filename_, std::ios::binary);
  return f;
}
}  // namespace tesseract_scene_graph
