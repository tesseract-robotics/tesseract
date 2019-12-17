/**
 * @file resource_locator.h
 * @brief Locate and retrieve resource data
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

#ifndef TESSERACT_SCENE_GRAPH_RESOURCE_LOCATOR_H
#define TESSERACT_SCENE_GRAPH_RESOURCE_LOCATOR_H

#include <tesseract_common/resource.h>
#include <tesseract_scene_graph/allowed_collision_matrix.h>
#include <tesseract_scene_graph/graph.h>
#include <tesseract_scene_graph/parser/srdf_parser.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <functional>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

namespace tesseract_scene_graph
{
/**
 * @brief Abstract class for resource loaders
 *
 */
class ResourceLocator
{
public:
  using Ptr = std::shared_ptr<ResourceLocator>;
  using ConstPtr = std::shared_ptr<const ResourceLocator>;

  ResourceLocator() = default;
  virtual ~ResourceLocator() = default;
  ResourceLocator(const ResourceLocator&) = default;
  ResourceLocator& operator=(const ResourceLocator&) = default;
  ResourceLocator(ResourceLocator&&) = default;
  ResourceLocator& operator=(ResourceLocator&&) = default;

  /**
   * @brief Locate a resource based on a URL
   *
   * @param url The URL of the resource
   * @return A shared pointer to a Resource instance, or nullptr if not found
   */
  virtual tesseract_common::Resource::Ptr locateResource(const std::string& url) = 0;
};

/**
 * @brief Resource locator implementation using a provided function to locate file resources
 *
 */
class SimpleResourceLocator : public ResourceLocator
{
public:
  using Ptr = std::shared_ptr<SimpleResourceLocator>;
  using ConstPtr = std::shared_ptr<const SimpleResourceLocator>;

  using ResourceLocatorFn = std::function<std::string(const std::string&)>;

  /**
   * @brief Construct a new Simple Resource Locator object
   *
   * @param locator_function Function to use to resolve resource file paths from URLs
   */
  SimpleResourceLocator(ResourceLocatorFn locator_function);

  tesseract_common::Resource::Ptr locateResource(const std::string& url) override;

protected:
  ResourceLocatorFn locator_function_;
};

/**
 * @brief Resource implementation for a local file
 *
 */
class SimpleLocatedResource : public tesseract_common::Resource
{
public:
  using Ptr = std::shared_ptr<SimpleLocatedResource>;
  using ConstPtr = std::shared_ptr<const SimpleLocatedResource>;

  SimpleLocatedResource(const std::string& url, const std::string& filename);

  bool isFile() override;

  std::string getUrl() override;

  std::string getFilePath() override;

  std::vector<uint8_t> getResourceContents() override;

  std::shared_ptr<std::istream> getResourceContentStream() override;

protected:
  std::string url_;
  std::string filename_;
};

}  // namespace tesseract_scene_graph

#endif  // TESSERACT_SCENE_GRAPH_RESOURCE_LOCATOR_H
