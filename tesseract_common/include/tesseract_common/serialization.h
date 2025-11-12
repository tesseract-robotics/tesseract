/**
 * @file serialization.h
 * @brief Boost serialization macros and helpers
 *
 * @author Levi Armstrong
 * @author Matthew Powelson
 * @date March 16, 2022
 * @version TODO
 * @bug No known bugs
 *
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
#ifndef TESSERACT_COMMON_SERIALIZATION_H
#define TESSERACT_COMMON_SERIALIZATION_H

#include <tesseract_common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <fstream>
#include <sstream>
#include <cereal/archives/binary.hpp>
#include <cereal/archives/xml.hpp>
#include <cereal/archives/json.hpp>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <filesystem>
#include <tesseract_common/serialization_extensions.h>

namespace tesseract_common
{
struct Serialization
{
  template <typename SerializableType>
  static std::string toArchiveStringXML(const SerializableType& archive_type, const std::string& name = "")
  {
    std::stringstream ss;
    {  // Must be scoped because all data is not written until the oost::archive::xml_oarchive goes out of scope
      cereal::XMLOutputArchive oa(ss);

      // Boost uses the same function for serialization and deserialization so it requires a non-const reference
      // Because we are only serializing here it is safe to cast away const
      if (name.empty())
        oa << cereal::make_nvp<SerializableType>("archive_type",
                                                 const_cast<SerializableType&>(archive_type));  // NOLINT
      else
        oa << cereal::make_nvp<SerializableType>(name.c_str(), const_cast<SerializableType&>(archive_type));  // NOLINT
    }

    return ss.str();
  }

  template <typename SerializableType>
  static bool toArchiveFileXML(const SerializableType& archive_type,
                               const std::string& file_path,
                               const std::string& name = "")
  {
    std::filesystem::path fp(file_path);
    if (!fp.has_extension())
      fp = std::filesystem::path(file_path + serialization::xml::extension<SerializableType>::value);

    std::ofstream os(fp.string());
    {  // Must be scoped because all data is not written until the oost::archive::xml_oarchive goes out of scope
      cereal::XMLOutputArchive oa(os);
      // Boost uses the same function for serialization and deserialization so it requires a non-const reference
      // Because we are only serializing here it is safe to cast away const
      if (name.empty())
        oa << cereal::make_nvp("archive_type", const_cast<SerializableType&>(archive_type));  // NOLINT
      else
        oa << cereal::make_nvp(name.c_str(), const_cast<SerializableType&>(archive_type));  // NOLINT
    }

    return true;
  }

  template <typename SerializableType>
  static std::string toArchiveStringJSON(const SerializableType& archive_type, const std::string& name = "")
  {
    std::stringstream ss;
    {  // Must be scoped because all data is not written until the oost::archive::xml_oarchive goes out of scope
      cereal::JSONOutputArchive oa(ss);

      // Boost uses the same function for serialization and deserialization so it requires a non-const reference
      // Because we are only serializing here it is safe to cast away const
      if (name.empty())
        oa << cereal::make_nvp("archive_type", const_cast<SerializableType&>(archive_type));  // NOLINT
      else
        oa << cereal::make_nvp(name.c_str(), const_cast<SerializableType&>(archive_type));  // NOLINT
    }

    return ss.str();
  }

  template <typename SerializableType>
  static bool toArchiveFileJSON(const SerializableType& archive_type,
                                const std::string& file_path,
                                const std::string& name = "")
  {
    std::filesystem::path fp(file_path);
    if (!fp.has_extension())
      fp = std::filesystem::path(file_path + serialization::json::extension<SerializableType>::value);

    std::ofstream os(fp.string());
    {  // Must be scoped because all data is not written until the oost::archive::xml_oarchive goes out of scope
      cereal::JSONOutputArchive oa(os);
      // Boost uses the same function for serialization and deserialization so it requires a non-const reference
      // Because we are only serializing here it is safe to cast away const
      if (name.empty())
        oa << cereal::make_nvp("archive_type", const_cast<SerializableType&>(archive_type));  // NOLINT
      else
        oa << cereal::make_nvp(name.c_str(), const_cast<SerializableType&>(archive_type));  // NOLINT
    }

    return true;
  }

  template <typename SerializableType>
  static bool toArchiveFileBinary(const SerializableType& archive_type,
                                  const std::string& file_path,
                                  const std::string& name = "")
  {
    std::filesystem::path fp(file_path);
    if (!fp.has_extension())
      fp = std::filesystem::path(file_path + serialization::binary::extension<SerializableType>::value);

    std::ofstream os(fp.string(), std::ios_base::binary);
    {  // Must be scoped because all data is not written until the oost::archive::xml_oarchive goes out of scope
      cereal::BinaryOutputArchive oa(os);
      // Boost uses the same function for serialization and deserialization so it requires a non-const reference
      // Because we are only serializing here it is safe to cast away const
      if (name.empty())
        oa << cereal::make_nvp("archive_type", const_cast<SerializableType&>(archive_type));  // NOLINT
      else
        oa << cereal::make_nvp(name.c_str(), const_cast<SerializableType&>(archive_type));  // NOLINT
    }

    return true;
  }

  template <typename SerializableType>
  static bool toArchiveFile(const SerializableType& archive_type,
                            const std::string& file_path,
                            const std::string& name = "")
  {
    std::filesystem::path fp(file_path);
    if (fp.extension() == serialization::binary::extension<SerializableType>::value)
      return toArchiveFileBinary<SerializableType>(archive_type, file_path, name);

    if (fp.extension() == serialization::xml::extension<SerializableType>::value)
      return toArchiveFileXML<SerializableType>(archive_type, file_path, name);

    return toArchiveFileJSON<SerializableType>(archive_type, file_path, name);
  }

  template <typename SerializableType>
  static std::vector<std::uint8_t> toArchiveBinaryData(const SerializableType& archive_type,
                                                       const std::string& name = "")
  {
    std::stringstream ss;
    {  // Must be scoped because all data is not written until the oost::archive::xml_oarchive goes out of scope
      cereal::BinaryOutputArchive oa(ss);

      // Boost uses the same function for serialization and deserialization so it requires a non-const reference
      // Because we are only serializing here it is safe to cast away const
      if (name.empty())
        oa << cereal::make_nvp("archive_type", const_cast<SerializableType&>(archive_type));  // NOLINT
      else
        oa << cereal::make_nvp(name.c_str(), const_cast<SerializableType&>(archive_type));  // NOLINT
    }

    std::string data = ss.str();
    return { data.begin(), data.end() };
  }

  template <typename SerializableType>
  static SerializableType fromArchiveStringXML(const std::string& archive_xml, const std::string& name = "")
  {
    SerializableType archive_type;

    {  // Must be scoped because all data is not written until the oost::archive::xml_oarchive goes out of scope
      std::stringstream ss(archive_xml);
      cereal::XMLInputArchive ia(ss);

      if (name.empty())
        ia >> cereal::make_nvp("archive_type", archive_type);  // NOLINT
      else
        ia >> cereal::make_nvp(name.c_str(), archive_type);  // NOLINT
    }

    return archive_type;
  }

  template <typename SerializableType>
  static SerializableType fromArchiveFileXML(const std::string& file_path, const std::string& name = "")
  {
    SerializableType archive_type;

    {  // Must be scoped because all data is not written until the oost::archive::xml_oarchive goes out of scope
      std::ifstream ifs(file_path);
      assert(ifs.good());
      cereal::XMLInputArchive ia(ifs);

      if (name.empty())
        ia >> cereal::make_nvp("archive_type", archive_type);  // NOLINT
      else
        ia >> cereal::make_nvp(name.c_str(), archive_type);  // NOLINT
    }

    return archive_type;
  }

  template <typename SerializableType>
  static SerializableType fromArchiveStringJSON(const std::string& archive_xml, const std::string& name = "")
  {
    SerializableType archive_type;

    {  // Must be scoped because all data is not written until the oost::archive::xml_oarchive goes out of scope
      std::stringstream ss(archive_xml);
      cereal::JSONInputArchive ia(ss);

      if (name.empty())
        ia >> cereal::make_nvp("archive_type", archive_type);  // NOLINT
      else
        ia >> cereal::make_nvp(name.c_str(), archive_type);  // NOLINT
    }

    return archive_type;
  }

  template <typename SerializableType>
  static SerializableType fromArchiveFileJSON(const std::string& file_path, const std::string& name = "")
  {
    SerializableType archive_type;

    {  // Must be scoped because all data is not written until the oost::archive::xml_oarchive goes out of scope
      std::ifstream ifs(file_path);
      assert(ifs.good());
      cereal::JSONInputArchive ia(ifs);

      if (name.empty())
        ia >> cereal::make_nvp("archive_type", archive_type);  // NOLINT
      else
        ia >> cereal::make_nvp(name.c_str(), archive_type);  // NOLINT
    }

    return archive_type;
  }

  template <typename SerializableType>
  static SerializableType fromArchiveFileBinary(const std::string& file_path, const std::string& name = "")
  {
    SerializableType archive_type;

    {  // Must be scoped because all data is not written until the oost::archive::xml_oarchive goes out of scope
      std::ifstream ifs(file_path, std::ios_base::binary);
      assert(ifs.good());
      cereal::BinaryInputArchive ia(ifs);

      if (name.empty())
        ia >> cereal::make_nvp("archive_type", archive_type);  // NOLINT
      else
        ia >> cereal::make_nvp(name.c_str(), archive_type);  // NOLINT
    }

    return archive_type;
  }

  template <typename SerializableType>
  static SerializableType fromArchiveFile(const std::string& file_path, const std::string& name = "")
  {
    std::filesystem::path fp(file_path);
    if (fp.extension() == serialization::binary::extension<SerializableType>::value)
      return fromArchiveFileBinary<SerializableType>(file_path, name);

    if (fp.extension() == serialization::xml::extension<SerializableType>::value)
      return fromArchiveFileXML<SerializableType>(file_path, name);

    return fromArchiveFileJSON<SerializableType>(file_path, name);
  }

  template <typename SerializableType>
  static SerializableType fromArchiveBinaryData(const std::vector<std::uint8_t>& archive_binary,
                                                const std::string& name = "")
  {
    SerializableType archive_type;

    {  // Must be scoped because all data is not written until the oost::archive::xml_oarchive goes out of scope
      std::stringstream ss;
      std::copy(archive_binary.begin(), archive_binary.end(), std::ostreambuf_iterator<char>(ss));
      cereal::BinaryInputArchive ia(ss);

      if (name.empty())
        ia >> cereal::make_nvp("archive_type", archive_type);  // NOLINT
      else
        ia >> cereal::make_nvp(name.c_str(), archive_type);  // NOLINT
    }

    return archive_type;
  }
};
}  // namespace tesseract_common
#endif  // TESSERACT_COMMON_SERIALIZATION_H
