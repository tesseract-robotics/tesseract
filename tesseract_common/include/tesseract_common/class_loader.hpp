/**
 * @file class_loader.hpp
 * @brief Class Loader to be used throughout Tesseract for loading plugins
 *
 * @author Levi Armstrong
 * @date March 25, 2021
 * @version TODO
 * @bug No known bugs
 *
 * @copyright Copyright (c) 2021, Southwest Research Institute
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
#ifndef TESSERACT_COMMON_CLASS_LOADER_HPP
#define TESSERACT_COMMON_CLASS_LOADER_HPP

#include <tesseract_common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <boost/config.hpp>
#include <boost/dll/import.hpp>
#include <boost/dll/alias.hpp>
#include <boost/dll/import_class.hpp>
#include <boost/dll/shared_library.hpp>
#include <console_bridge/console.h>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract_common/class_loader.h>

namespace tesseract_common
{
template <class ClassBase>
std::shared_ptr<ClassBase> ClassLoader::createSharedInstance(const std::string& symbol_name,
                                                             const std::string& library_name,
                                                             const std::string& library_directory)
{
  boost::system::error_code ec;
  boost::dll::shared_library lib;
  if (library_directory.empty())
  {
    boost::filesystem::path sl(library_name);
    boost::dll::load_mode::type mode =
        boost::dll::load_mode::append_decorations | boost::dll::load_mode::search_system_folders;
    lib = boost::dll::shared_library(sl, ec, mode);
  }
  else
  {
    boost::filesystem::path sl = boost::filesystem::path(library_directory) / library_name;
    lib = boost::dll::shared_library(sl, ec, boost::dll::load_mode::append_decorations);
  }

  // Check if it failed to find or load library
  if (ec)
    throw std::runtime_error("Failed to find or load library: " + decorate(library_name, library_directory) +
                             " with error: " + ec.message());

  // Check if library has symbol
  if (!lib.has(symbol_name))
    throw std::runtime_error("Failed to find symbol '" + symbol_name +
                             "' in library: " + decorate(library_name, library_directory));

#if BOOST_VERSION >= 107600
  boost::shared_ptr<ClassBase> plugin = boost::dll::import_symbol<ClassBase>(lib, symbol_name);
#else
  boost::shared_ptr<ClassBase> plugin = boost::dll::import<ClassBase>(lib, symbol_name);
#endif
  return std::shared_ptr<ClassBase>(plugin.get(), [plugin](ClassBase*) mutable { plugin.reset(); });
}

bool ClassLoader::isClassAvailable(const std::string& symbol_name,
                                   const std::string& library_name,
                                   const std::string& library_directory)
{
  boost::system::error_code ec;
  boost::dll::shared_library lib;
  if (library_directory.empty())
  {
    boost::filesystem::path sl(library_name);
    boost::dll::load_mode::type mode =
        boost::dll::load_mode::append_decorations | boost::dll::load_mode::search_system_folders;
    lib = boost::dll::shared_library(sl, ec, mode);
  }
  else
  {
    boost::filesystem::path sl = boost::filesystem::path(library_directory) / library_name;
    lib = boost::dll::shared_library(sl, ec, boost::dll::load_mode::append_decorations);
  }

  // Check if it failed to find or load library
  if (ec)
  {
    CONSOLE_BRIDGE_logDebug("Failed to find or load library: %s with error: %s",
                            decorate(library_name, library_directory).c_str(),
                            ec.message().c_str());
    return false;
  }

  return lib.has(symbol_name);
}

std::string ClassLoader::decorate(const std::string& library_name, const std::string& library_directory)
{
  boost::filesystem::path sl;
  if (library_directory.empty())
    sl = boost::filesystem::path(library_name);
  else
    sl = boost::filesystem::path(library_directory) / library_name;

  boost::filesystem::path actual_path =
      (std::strncmp(sl.filename().string().c_str(), "lib", 3) != 0 ?
           boost::filesystem::path((sl.has_parent_path() ? sl.parent_path() / L"lib" : L"lib").native() +
                                   sl.filename().native()) :
           sl);

  actual_path += boost::dll::shared_library::suffix();
  return actual_path.string();
}

}  // namespace tesseract_common

#endif  // TESSERACT_COMMON_CLASS_LOADER_HPP
