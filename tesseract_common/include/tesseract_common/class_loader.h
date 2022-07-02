/**
 * @file class_loader.h
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
#ifndef TESSERACT_COMMON_CLASS_LOADER_H
#define TESSERACT_COMMON_CLASS_LOADER_H

#include <tesseract_common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <memory>
#include <string>
#include <vector>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

namespace tesseract_common
{
/**
 * @brief This is a wrapper around Boost DLL for loading plugins within Tesseract
 * @details The library_name should not include the prefix 'lib' or suffix '.so'. It will add the correct prefix and
 * suffix based on the OS.
 *
 * The plugin must be exported using the macro TESSERACT_ADD_PLUGIN.
 * In the example below, the first parameter is the derived object and the second is the assigned symbol name which is
 * used for looding Example: TESSERACT_ADD_PLUGIN(my_namespace::MyPlugin, plugin)
 *
 *   auto p = ClassLoader::createSharedInstance<my_namespace::MyPluginBase>("my_plugin", "plugin");
 */
struct ClassLoader
{
  /**
   * @brief Create a shared instance for the provided symbol_name loaded from the library_name searching system folders
   * for library
   * @details The symbol name is the alias provide when calling TESSERACT_ADD_PLUGIN
   * @param symbol_name The symbol to create a shared instance of
   * @param library_name The library name to load which does not include the prefix 'lib' or suffix '.so'
   * @param library_directory The library directory, if empty it will enable search system directories
   * @return A shared pointer of the object with the symbol name located in library_name_
   */
  template <class ClassBase>
  static std::shared_ptr<ClassBase> createSharedInstance(const std::string& symbol_name,
                                                         const std::string& library_name,
                                                         const std::string& library_directory = "");

  /**
   * @brief Check if the symbol is available in the library_name searching system folders for library
   * @details The symbol name is the alias provide when calling TESSERACT_ADD_PLUGIN
   * @param symbol_name The symbol to create a shared instance of
   * @param library_name The library name to load which does not include the prefix 'lib' or suffix '.so'
   * @param library_directory The library directory, if empty it will enable search system directories
   * @return True if the symbol exists, otherwise false
   */
  static inline bool isClassAvailable(const std::string& symbol_name,
                                      const std::string& library_name,
                                      const std::string& library_directory = "");

  /**
   * @brief Get a list of available symbols under the provided section
   * @param section The section to search for available symbols
   * @param library_name The library name to load which does not include the prefix 'lib' or suffix '.so'
   * @param library_directory The library directory, if empty it will enable search system directories
   * @return A list of symbols if they exist.
   */
  static inline std::vector<std::string> getAvailableSymbols(const std::string& section,
                                                             const std::string& library_name,
                                                             const std::string& library_directory = "");

  /**
   * @brief Get a list of available sections
   * @param library_name The library name to load which does not include the prefix 'lib' or suffix '.so'
   * @param library_directory The library directory, if empty it will enable search system directories
   * @return A list of sections if they exist.
   */
  static inline std::vector<std::string> getAvailableSections(const std::string& library_name,
                                                              const std::string& library_directory = "",
                                                              bool include_hidden = false);

  /**
   * @brief Give library name without prefix and suffix it will return the library name with the prefix and suffix
   *
   * * For instance, for a library_name like "boost" it returns :
   * - path/to/libboost.so on posix platforms
   * - path/to/libboost.dylib on OSX
   * - path/to/boost.dll on Windows
   *
   * @todo When support is dropped for 18.04 switch to using boost::dll::shared_library::decorate
   * @param library_name The library name without prefix and suffix
   * @param library_directory The library directory, if empty it just returns the decorated library name
   * @return The library name or path with prefix and suffix
   */
  static inline std::string decorate(const std::string& library_name, const std::string& library_directory = "");
};
}  // namespace tesseract_common

// clang-format off
#define TESSERACT_ADD_PLUGIN_SECTIONED(DERIVED_CLASS, ALIAS, SECTION)                                                  \
  extern "C" BOOST_SYMBOL_EXPORT DERIVED_CLASS ALIAS;                                                                  \
  BOOST_DLL_SECTION(SECTION, read) BOOST_DLL_SELECTANY                                                                 \
  DERIVED_CLASS ALIAS;

#define TESSERACT_ADD_PLUGIN(DERIVED_CLASS, ALIAS)                                                                     \
  TESSERACT_ADD_PLUGIN_SECTIONED(DERIVED_CLASS, ALIAS, boostdll)

#define TESSERACT_PLUGIN_ANCHOR_DECL(ANCHOR_NAME) \
  const void* ANCHOR_NAME(); // NOLINT

#define TESSERACT_PLUGIN_ANCHOR_IMPL(ANCHOR_NAME) \
  const void* ANCHOR_NAME() { return (const void*)(&ANCHOR_NAME); } // NOLINT

// clang-format on

#include <tesseract_common/class_loader.hpp>

#endif  // TESSERACT_COMMON_CLASS_LOADER_H
