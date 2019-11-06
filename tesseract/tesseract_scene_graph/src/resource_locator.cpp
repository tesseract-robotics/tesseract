/**
 * @file graph.cpp
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
    {
        assert(locator_function);
        locator_function_ = locator_function;
    }

    tesseract_common::Resource::Ptr SimpleResourceLocator::LocateResource(const std::string& url)
    {
        std::string filename = locator_function_(url);
        return std::make_shared<SimpleLocatedResource>(url,filename);
    }

    SimpleLocatedResource::SimpleLocatedResource(const std::string& url, const std::string& filename)
    {
        url_ = url;
        filename_ = filename;
    }

    bool SimpleLocatedResource::IsFile()
    {
        return true;
    }

    std::string SimpleLocatedResource::GetUrl()
    {
        return url_;
    }

    std::string SimpleLocatedResource::GetFilePath()
    {
        return filename_;
    }

    std::vector<uint8_t> SimpleLocatedResource::GetResourceContents()
    {
        // https://stackoverflow.com/questions/4761529/efficient-way-of-reading-a-file-into-an-stdvectorchar
        std::basic_ifstream<uint8_t> f(filename_, std::ios::binary);
        std::vector<uint8_t> file_contents((std::istreambuf_iterator<uint8_t>(f)),
                               std::istreambuf_iterator<uint8_t>());
        return file_contents;
    }

    std::shared_ptr<std::istream> SimpleLocatedResource::GetResourceContentStream()
    {
        std::shared_ptr<std::ifstream> f = std::make_shared<std::ifstream>(filename_, std::ios::binary);
        return f;
    }
}