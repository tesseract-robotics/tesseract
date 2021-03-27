/**
 * @file plugin_loader.h
 * @brief Plugin loader class to be used throughout Tesseract for loading plugins
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
#ifndef TESSERACT_COMMON_PLUGIN_LOADER_H
#define TESSERACT_COMMON_PLUGIN_LOADER_H

#include <tesseract_common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <boost/dll/import.hpp>
#include <boost/dll/alias.hpp>
#include <boost/dll/import_class.hpp>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

namespace tesseract_common
{
/**
 * @brief This is a wrapper around Boost DLL for loading plugins within Tesseract
 * @details The library_name should not include the prefix 'lib' or postfix '.so'. It will add the correct prefix and
 * postfix based on the OS.
 *
 * The plugin must be exported using the macro TESSERACT_EXPORT_PLUGIN.
 * In the example below, the first parameter is the derived object and the second is the assinged symbol name which is
 * used for looding Example: TESSERACT_EXPORT_PLUGIN(my_namespace::MyPlugin, plugin)
 *
 *   auto p = PluginLoader::createSharedInstance<my_namespace::MyPluginBase>("plugin");
 */
class PluginLoader
{
public:
  /**
   * @brief A PluginLoader which will only look in the provided directory for the library
   * @details This will not search system folders for the library
   * @param library_directory The directory it should search for the library
   * @param library_name The library name to load which does not include the prefix 'lib' or postfix '.so'
   */
  PluginLoader(const std::string& library_directory, const std::string& library_name)
    : library_directory_(library_directory), library_name_(library_name)
  {
  }

  /**
   * @brief A PluginLoader will search system folders for the library
   * @param library_name The library name to load which does not include the prefix 'lib' or postfix '.so'
   */
  PluginLoader(const std::string& library_name) : library_name_(library_name) {}

  /**
   * @brief Get library directory. If empty then it will search system for the library name provided
   * @return The library directory to search for library
   */
  const std::string& getLibraryDirectory() const { return library_directory_; }

  /**
   * @brief Get the library name
   * @return The library name
   */
  const std::string& getLibraryName() const { return library_name_; }

  /**
   * @brief Create a shared instance for the provided symbol_name
   * @details The symbol name is the alias provide when calling TESSERACT_EXPORT_PLUGIN
   * @param symbol_name The symbol to create a shared instance of
   * @return A shared pointer of the object with the symbol name located in library_name_
   */
  template <class Base>
  std::shared_ptr<Base> createSharedInstance(const std::string& symbol_name)
  {
    if (library_directory_.empty())
      return createSharedInstance<Base>(library_name_, symbol_name);

    return createSharedInstance<Base>(library_directory_, library_name_, symbol_name);
  }

  /**
   * @brief Check if the symbol is available in the library_name_
   * @details The symbol name is the alias provide when calling TESSERACT_EXPORT_PLUGIN
   * @param symbol_name The symbol to create a shared instance of
   * @return True if the symbol exists, otherwise false
   */
  template <class Base>
  bool isClassAvailable(const std::string& symbol_name)
  {
    if (library_directory_.empty())
      return isClassAvailable<Base>(library_name_, symbol_name);

    return isClassAvailable<Base>(library_directory_, library_name_, symbol_name);
  }

  // Static methods below

  /**
   * @brief Create a shared instance for the provided symbol_name loaded from the library_name searching system folders
   * for library
   * @details The symbol name is the alias provide when calling TESSERACT_EXPORT_PLUGIN
   * @param library_name The library name to load which does not include the prefix 'lib' or postfix '.so'
   * @param symbol_name The symbol to create a shared instance of
   * @return A shared pointer of the object with the symbol name located in library_name_
   */
  template <class Base>
  static std::shared_ptr<Base> createSharedInstance(const std::string& library_name, const std::string& symbol_name)
  {
    boost::shared_ptr<Base> plugin = boost::dll::import<Base>(library_name,
                                                              symbol_name,
                                                              boost::dll::load_mode::append_decorations |
                                                                  boost::dll::load_mode::search_system_folders);
    return std::shared_ptr<Base>(plugin.get(), [plugin](Base*) mutable { plugin.reset(); });
  }

  /**
   * @brief Create a shared instance for the provided symbol_name loaded from the library_name located in
   * library_directory.
   * @details The symbol name is the alias provide when calling TESSERACT_EXPORT_PLUGIN
   * @param library_name The library name to load which does not include the prefix 'lib' or postfix '.so'
   * @param symbol_name The symbol to create a shared instance of
   * @return A shared pointer of the object with the symbol name located in library_name_
   */
  template <class Base>
  static std::shared_ptr<Base> createSharedInstance(const std::string& library_directory,
                                                    const std::string& library_name,
                                                    const std::string& symbol_name)
  {
    boost::dll::fs::path lib_dir(library_directory);
    boost::shared_ptr<Base> plugin =
        boost::dll::import<Base>(lib_dir / library_name, symbol_name, boost::dll::load_mode::append_decorations);
    return std::shared_ptr<Base>(plugin.get(), [plugin](Base*) mutable { plugin.reset(); });
  }

  /**
   * @brief Check if the symbol is available in the library_name searching system folders for library
   * @details The symbol name is the alias provide when calling TESSERACT_EXPORT_PLUGIN
   * @param symbol_name The symbol to create a shared instance of
   * @return True if the symbol exists, otherwise false
   */
  template <class Base>
  static bool isClassAvailable(const std::string& library_name, const std::string& symbol_name)
  {
    boost::dll::shared_library lib(
        library_name, boost::dll::load_mode::append_decorations | boost::dll::load_mode::search_system_folders);
    return lib.has(symbol_name);
  }

  /**
   * @brief Check if the symbol is available in the library_name searching library_directory for library
   * @details The symbol name is the alias provide when calling TESSERACT_EXPORT_PLUGIN
   * @param symbol_name The symbol to create a shared instance of
   * @return True if the symbol exists, otherwise false
   */
  template <class Base>
  static bool isClassAvailable(const std::string& library_directory,
                               const std::string& library_name,
                               const std::string& symbol_name)
  {
    boost::dll::fs::path lib_dir(library_directory);
    boost::dll::shared_library lib(lib_dir / library_name, boost::dll::load_mode::append_decorations);
    return lib.has(symbol_name);
  }

protected:
  std::string library_directory_;
  std::string library_name_;
};
}  // namespace tesseract_common

#define TESSERACT_EXPORT_PLUGIN(DERIVED_CLASS, ALIAS)                                                                  \
  extern "C" BOOST_SYMBOL_EXPORT DERIVED_CLASS ALIAS;                                                                  \
  DERIVED_CLASS ALIAS;

#endif  // TESSERACT_COMMON_PLUGIN_LOADER_H
