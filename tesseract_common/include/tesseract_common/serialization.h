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
#include <variant>
#include <Eigen/Dense>
#include <fstream>
#include <sstream>
#include <boost/archive/xml_oarchive.hpp>
#include <boost/archive/xml_iarchive.hpp>
#include <boost/archive/binary_oarchive.hpp>
#include <boost/archive/binary_iarchive.hpp>
#include <boost/serialization/tracking.hpp>
#include <boost/serialization/tracking_enum.hpp>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract_common/types.h>
#include <tesseract_common/serialization_extensions.h>

// Used to replace commas in these macros to avoid them being interpreted as multiple arguments
// Example: TESSERACT_SERIALIZE_SAVE_LOAD_FREE_ARCHIVES_INSTANTIATE(std::variant<std::string COMMA Eigen::Isometry3d>)
#define COMMA ,

// Use this macro for serialization defined using the invasive method inside the class
#define TESSERACT_SERIALIZE_ARCHIVES_INSTANTIATE(Type)                                                                 \
  template void Type::serialize(boost::archive::xml_oarchive& ar, const unsigned int version);                         \
  template void Type::serialize(boost::archive::xml_iarchive& ar, const unsigned int version);                         \
  template void Type::serialize(boost::archive::binary_oarchive& ar, const unsigned int version);                      \
  template void Type::serialize(boost::archive::binary_iarchive& ar, const unsigned int version);

// Use this macro for serialization defined using the invasive method inside the class with custom load/save functions
#define TESSERACT_SERIALIZE_SAVE_LOAD_ARCHIVES_INSTANTIATE(Type)                                                       \
  template void Type::serialize(boost::archive::xml_oarchive& ar, const unsigned int version);                         \
  template void Type::serialize(boost::archive::xml_iarchive& ar, const unsigned int version);                         \
  template void Type::serialize(boost::archive::binary_oarchive& ar, const unsigned int version);                      \
  template void Type::serialize(boost::archive::binary_iarchive& ar, const unsigned int version);                      \
  template void Type::save(boost::archive::xml_oarchive&, const unsigned int version) const;                           \
  template void Type::load(boost::archive::xml_iarchive& ar, const unsigned int version);                              \
  template void Type::save(boost::archive::binary_oarchive&, const unsigned int version) const;                        \
  template void Type::load(boost::archive::binary_iarchive& ar, const unsigned int version);

// Use this macro for serialization defined using the non-invasive free function method outside the class
#define TESSERACT_SERIALIZE_SAVE_LOAD_FREE_ARCHIVES_INSTANTIATE(Type)                                                  \
  template void boost::serialization::serialize(                                                                       \
      boost::archive::xml_oarchive& ar, Type& g, const unsigned int version); /* NOLINT */                             \
  template void boost::serialization::serialize(                                                                       \
      boost::archive::xml_iarchive& ar, Type& g, const unsigned int version); /* NOLINT */                             \
  template void boost::serialization::serialize(                                                                       \
      boost::archive::binary_oarchive& ar, Type& g, const unsigned int version); /* NOLINT */                          \
  template void boost::serialization::serialize(                                                                       \
      boost::archive::binary_iarchive& ar, Type& g, const unsigned int version); /* NOLINT */                          \
  template void boost::serialization::save(                                                                            \
      boost::archive::xml_oarchive&, const Type& g, const unsigned int version); /* NOLINT */                          \
  template void boost::serialization::load(                                                                            \
      boost::archive::xml_iarchive& ar, Type& g, const unsigned int version); /* NOLINT */                             \
  template void boost::serialization::save(                                                                            \
      boost::archive::binary_oarchive&, const Type& g, const unsigned int version); /* NOLINT */                       \
  template void boost::serialization::load(                                                                            \
      boost::archive::binary_iarchive& ar, Type& g, const unsigned int version); /* NOLINT */

namespace tesseract_common
{
struct Serialization
{
  template <typename SerializableType>
  static std::string toArchiveStringXML(const SerializableType& archive_type, const std::string& name = "")
  {
    std::stringstream ss;
    {  // Must be scoped because all data is not written until the oost::archive::xml_oarchive goes out of scope
      boost::archive::xml_oarchive oa(ss);

      // Boost uses the same function for serialization and deserialization so it requires a non-const reference
      // Because we are only serializing here it is safe to cast away const
      if (name.empty())
        oa << boost::serialization::make_nvp<SerializableType>("archive_type",
                                                               const_cast<SerializableType&>(archive_type));  // NOLINT
      else
        oa << boost::serialization::make_nvp<SerializableType>(name.c_str(),
                                                               const_cast<SerializableType&>(archive_type));  // NOLINT
    }

    return ss.str();
  }

  template <typename SerializableType>
  static bool toArchiveFileXML(const SerializableType& archive_type,
                               const std::string& file_path,
                               const std::string& name = "")
  {
    fs::path fp(file_path);
    if (!fp.has_extension())
      fp.append(".").append(serialization::xml::extension<SerializableType>::value);

    std::ofstream os(fp.string());
    {  // Must be scoped because all data is not written until the oost::archive::xml_oarchive goes out of scope
      boost::archive::xml_oarchive oa(os);
      // Boost uses the same function for serialization and deserialization so it requires a non-const reference
      // Because we are only serializing here it is safe to cast away const
      if (name.empty())
        oa << boost::serialization::make_nvp<SerializableType>("archive_type",
                                                               const_cast<SerializableType&>(archive_type));  // NOLINT
      else
        oa << boost::serialization::make_nvp<SerializableType>(name.c_str(),
                                                               const_cast<SerializableType&>(archive_type));  // NOLINT
    }

    return true;
  }

  template <typename SerializableType>
  static bool toArchiveFileBinary(const SerializableType& archive_type,
                                  const std::string& file_path,
                                  const std::string& name = "")
  {
    fs::path fp(file_path);
    if (!fp.has_extension())
      fp.append(".").append(serialization::binary::extension<SerializableType>::value);

    std::ofstream os(fp.string(), std::ios_base::binary);
    {  // Must be scoped because all data is not written until the oost::archive::xml_oarchive goes out of scope
      boost::archive::binary_oarchive oa(os);
      // Boost uses the same function for serialization and deserialization so it requires a non-const reference
      // Because we are only serializing here it is safe to cast away const
      if (name.empty())
        oa << boost::serialization::make_nvp<SerializableType>("archive_type",
                                                               const_cast<SerializableType&>(archive_type));  // NOLINT
      else
        oa << boost::serialization::make_nvp<SerializableType>(name.c_str(),
                                                               const_cast<SerializableType&>(archive_type));  // NOLINT
    }

    return true;
  }

  template <typename SerializableType>
  static SerializableType fromArchiveStringXML(const std::string& archive_xml)
  {
    SerializableType archive_type;

    {  // Must be scoped because all data is not written until the oost::archive::xml_oarchive goes out of scope
      std::stringstream ss(archive_xml);
      boost::archive::xml_iarchive ia(ss);
      ia >> BOOST_SERIALIZATION_NVP(archive_type);
    }

    return archive_type;
  }

  template <typename SerializableType>
  static SerializableType fromArchiveFileXML(const std::string& file_path)
  {
    SerializableType archive_type;

    {  // Must be scoped because all data is not written until the oost::archive::xml_oarchive goes out of scope
      std::ifstream ifs(file_path);
      assert(ifs.good());
      boost::archive::xml_iarchive ia(ifs);
      ia >> BOOST_SERIALIZATION_NVP(archive_type);
    }

    return archive_type;
  }

  template <typename SerializableType>
  static SerializableType fromArchiveFileBinary(const std::string& file_path)
  {
    SerializableType archive_type;

    {  // Must be scoped because all data is not written until the oost::archive::xml_oarchive goes out of scope
      std::ifstream ifs(file_path, std::ios_base::binary);
      assert(ifs.good());
      boost::archive::binary_iarchive ia(ifs);
      ia >> BOOST_SERIALIZATION_NVP(archive_type);
    }

    return archive_type;
  }
};
}  // namespace tesseract_common
#endif  // TESSERACT_COMMON_SERIALIZATION_H
