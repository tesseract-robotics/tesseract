/**
 * @file boost_filesystem_path.i
 * @brief Implement very simple wrapper for boost::filesystem::path
 *
 * @author John Wason
 * @date December 18, 2020
 * @version TODO
 * @bug No known bugs
 *
 * @copyright Copyright (c) 2020, Wason Technology, LLC
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
#include <boost/filesystem/path.hpp>
%}

namespace boost
{
namespace filesystem
{

%rename(FilesystemPath) path;
class path
{
public:

path(const std::string& s);
std::string string();

%pythoncode %{
def __str__(self):
    return self.string()
%}
};
}
}