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
#ifndef TESSERACT_COMMON_RESOURCE_LOCATOR_H
#define TESSERACT_COMMON_RESOURCE_LOCATOR_H

#include <tesseract_common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <functional>
#include <memory>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#ifdef SWIG
%shared_ptr(tesseract_common::Resource)
%template(vector_uint8) std::vector<uint8_t>;
%pybuffer_binary(const uint8_t* bytes, size_t bytes_len);
%shared_ptr(tesseract_common::BytesResource)

%feature("director") tesseract_common::ResourceLocator;
%shared_ptr(tesseract_common::ResourceLocator)
%shared_ptr(tesseract_common::SimpleResourceLocator)
%shared_ptr(tesseract_common::SimpleLocatedResource)
#endif  // SWIG

namespace tesseract_common
{
// Forward declare
class Resource;

/** @brief Abstract class for resource loaders */
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
  virtual std::shared_ptr<Resource> locateResource(const std::string& url) const = 0;
};

/**  @brief Represents resource data available from a file or url */
class Resource : public ResourceLocator
{
public:
  using Ptr = std::shared_ptr<Resource>;
  using ConstPtr = std::shared_ptr<const Resource>;

  Resource() = default;
  ~Resource() override = default;
  Resource(const Resource&) = default;
  Resource& operator=(const Resource&) = default;
  Resource(Resource&&) = default;
  Resource& operator=(Resource&&) = default;

  /**
   * @brief Returns true if the located resource is a local file
   *
   * @return true if the resource is a local file, otherwise false
   */
  virtual bool isFile() const = 0;

  /**
   * @brief Get the original URL used to locate the file
   *
   * @return The URL of the resource
   */
  virtual std::string getUrl() const = 0;

  /**
   * @brief Get the file path of the resource. Only valid if isFile() is true.
   *
   * @return The file path to the resource
   */
  virtual std::string getFilePath() const = 0;

  /**
   * @brief Get the resource as bytes. This function may block
   *
   * @return Resource bytes as a uint8_t vector
   */
  virtual std::vector<uint8_t> getResourceContents() const = 0;

  /**
   * @brief Get the resource as a std::istream. This function and the returned stream may block
   *
   * @return A std::istream shared pointer for the resource data
   */
  virtual std::shared_ptr<std::istream> getResourceContentStream() const = 0;
};

using SimpleResourceLocatorFn = std::function<std::string(const std::string&)>;

/** @brief Resource locator implementation using a provided function to locate file resources */
class SimpleResourceLocator : public ResourceLocator
{
public:
  using Ptr = std::shared_ptr<SimpleResourceLocator>;
  using ConstPtr = std::shared_ptr<const SimpleResourceLocator>;

  /**
   * @brief Construct a new Simple Resource Locator object
   *
   * @param locator_function Function to use to resolve resource file paths from URLs
   */
  SimpleResourceLocator(SimpleResourceLocatorFn locator_function);
  ~SimpleResourceLocator() override = default;
  SimpleResourceLocator(const SimpleResourceLocator&) = default;
  SimpleResourceLocator& operator=(const SimpleResourceLocator&) = default;
  SimpleResourceLocator(SimpleResourceLocator&&) = default;
  SimpleResourceLocator& operator=(SimpleResourceLocator&&) = default;

  tesseract_common::Resource::Ptr locateResource(const std::string& url) const override final;

private:
  SimpleResourceLocatorFn locator_function_;
};

/** @brief Resource implementation for a local file */
class SimpleLocatedResource : public tesseract_common::Resource
{
public:
  using Ptr = std::shared_ptr<SimpleLocatedResource>;
  using ConstPtr = std::shared_ptr<const SimpleLocatedResource>;

  SimpleLocatedResource(const std::string& url,
                        const std::string& filename,
                        const SimpleResourceLocator::ConstPtr& parent = nullptr);
  ~SimpleLocatedResource() override = default;
  SimpleLocatedResource(const SimpleLocatedResource&) = default;
  SimpleLocatedResource& operator=(const SimpleLocatedResource&) = default;
  SimpleLocatedResource(SimpleLocatedResource&&) = default;
  SimpleLocatedResource& operator=(SimpleLocatedResource&&) = default;

  bool isFile() const override final;

  std::string getUrl() const override final;

  std::string getFilePath() const override final;

  std::vector<uint8_t> getResourceContents() const override final;

  std::shared_ptr<std::istream> getResourceContentStream() const override final;

  Resource::Ptr locateResource(const std::string& url) const override final;

private:
  std::string url_;
  std::string filename_;
  SimpleResourceLocator::ConstPtr parent_;
};

class BytesResource : public tesseract_common::Resource
{
public:
  BytesResource(std::string url, std::vector<uint8_t> bytes);
  BytesResource(std::string url, const uint8_t* bytes, size_t bytes_len);
  ~BytesResource() override = default;
  BytesResource(const BytesResource&) = default;
  BytesResource& operator=(const BytesResource&) = default;
  BytesResource(BytesResource&&) = default;
  BytesResource& operator=(BytesResource&&) = default;

  bool isFile() const override final;
  std::string getUrl() const override final;
  std::string getFilePath() const override final;
  std::vector<uint8_t> getResourceContents() const override final;
  std::shared_ptr<std::istream> getResourceContentStream() const override final;
  Resource::Ptr locateResource(const std::string& url) const override final;

private:
  std::string url_;
  std::vector<uint8_t> bytes_;
};

}  // namespace tesseract_common

#endif  // TESSERACT_COMMON_RESOURCE_LOCATOR_H
