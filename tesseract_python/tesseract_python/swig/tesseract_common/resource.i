/**
 * @file resource.i
 * @brief SWIG interface file for tesseract_common/resource.h
 *
 * @author John Wason
 * @date December 10, 2019
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

%{
#include <tesseract_common/resource.h>
%}

%include <pybuffer.i>

%shared_ptr(tesseract_common::Resource)

%template(vector_uint8) std::vector<uint8_t>;

%pybuffer_binary(const uint8_t* bytes, size_t bytes_len);

%shared_ptr(tesseract_common::BytesResource)

namespace tesseract_common
{
class Resource
{
public:
  using Ptr = std::shared_ptr<Resource>;
  using ConstPtr = std::shared_ptr<const Resource>;

  virtual ~Resource() = default;

  virtual bool isFile() = 0;
  virtual std::string getUrl() = 0;
  virtual std::string getFilePath() = 0;
  //TODO: Typemap to bytes instead of vector
  virtual std::vector<uint8_t> getResourceContents() = 0;  
};

class BytesResource : public tesseract_common::Resource
{
public:
  BytesResource(const std::string& url, const uint8_t* bytes, size_t bytes_len);
  virtual bool isFile() override;
  virtual std::string getUrl() override;
  virtual std::string getFilePath() override;
  virtual std::vector<uint8_t> getResourceContents() override;
  virtual std::shared_ptr<std::istream> getResourceContentStream() override;
};

}  // namespace tesseract_common