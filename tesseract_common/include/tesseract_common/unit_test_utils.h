/**
 * @file unit_test_utils.h
 * @brief Common Tesseract unit test utilities
 *
 * @author Levi Armstrong
 * @author Matthew Powelson
 * @date March 16, 2022
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
#ifndef TESSERACT_COMMON_UNIT_TEST_UTILS_H
#define TESSERACT_COMMON_UNIT_TEST_UTILS_H

#include <tesseract_common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <gtest/gtest.h>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract_common/serialization.h>
#include <tesseract_common/any_poly.h>
#include <tesseract_common/utils.h>

namespace tesseract::common
{
template <typename SerializableType>
using TestSerializationCompareFn = std::function<bool(const SerializableType&, const SerializableType&)>;

// Using != because it call == for code coverage
template <typename SerializableType>
bool testSerializationCompareEqual(const SerializableType& a, const SerializableType& b)
{
  return !(a != b);
}

template <typename SerializableType>
bool testSerializationComparePtrEqual(const SerializableType& a, const SerializableType& b)
{
  return !(*a != *b);
}

/**
 * @brief Tests serialization for a serializable type
 * @details Serializes the type to XML file, binary file, and XML string. It then deserializes it and calls the equality
 * operator on the results
 * @param object Object to be serialized
 * @param typename_string Prefix used for filepaths. Serialized files are put in /tmp/<typename_string>.<extension>
 */
template <typename SerializableType>
void testSerialization(
    const SerializableType& object,
    const std::string& typename_string,
    TestSerializationCompareFn<SerializableType> compare = testSerializationCompareEqual<SerializableType>)
{
  {  // Archive program to XML file
    std::string file_path = tesseract::common::getTempPath() + typename_string + ".xml";
    EXPECT_TRUE(tesseract::common::Serialization::toArchiveFileXML<SerializableType>(object, file_path));

    SerializableType nobject{ tesseract::common::Serialization::fromArchiveFileXML<SerializableType>(file_path) };
    EXPECT_TRUE(compare(object, nobject));
  }

  {  // Archive program to XML file with name
    std::string file_path = tesseract::common::getTempPath() + typename_string + ".xml";
    EXPECT_TRUE(
        tesseract::common::Serialization::toArchiveFileXML<SerializableType>(object, file_path, typename_string));

    SerializableType nobject{ tesseract::common::Serialization::fromArchiveFileXML<SerializableType>(file_path,
                                                                                                     typename_string) };
    EXPECT_TRUE(compare(object, nobject));
  }

  {  // Archive program to JSON file
    std::string file_path = tesseract::common::getTempPath() + typename_string + ".json";
    EXPECT_TRUE(tesseract::common::Serialization::toArchiveFileJSON<SerializableType>(object, file_path));

    SerializableType nobject{ tesseract::common::Serialization::fromArchiveFileJSON<SerializableType>(file_path) };
    EXPECT_TRUE(compare(object, nobject));
  }

  {  // Archive program to JSON file with name
    std::string file_path = tesseract::common::getTempPath() + typename_string + ".json";
    EXPECT_TRUE(
        tesseract::common::Serialization::toArchiveFileJSON<SerializableType>(object, file_path, typename_string));

    SerializableType nobject{ tesseract::common::Serialization::fromArchiveFileJSON<SerializableType>(
        file_path, typename_string) };
    EXPECT_TRUE(compare(object, nobject));
  }

  {  // Archive program to binary file
    std::string file_path = tesseract::common::getTempPath() + typename_string + ".binary";
    EXPECT_TRUE(tesseract::common::Serialization::toArchiveFileBinary<SerializableType>(object, file_path));

    SerializableType nobject{ tesseract::common::Serialization::fromArchiveFileBinary<SerializableType>(file_path) };
    EXPECT_TRUE(compare(object, nobject));
  }

  {  // Archive program to binary file with name
    std::string file_path = tesseract::common::getTempPath() + typename_string + ".binary";
    EXPECT_TRUE(
        tesseract::common::Serialization::toArchiveFileBinary<SerializableType>(object, file_path, typename_string));

    SerializableType nobject{ tesseract::common::Serialization::fromArchiveFileBinary<SerializableType>(
        file_path, typename_string) };
    EXPECT_TRUE(compare(object, nobject));
  }

  {  // Archive program to string
    std::string object_string = tesseract::common::Serialization::toArchiveStringXML<SerializableType>(object);
    EXPECT_FALSE(object_string.empty());

    SerializableType nobject{ tesseract::common::Serialization::fromArchiveStringXML<SerializableType>(object_string) };
    EXPECT_TRUE(compare(object, nobject));
  }

  {  // Archive program to string with name
    std::string object_string =
        tesseract::common::Serialization::toArchiveStringXML<SerializableType>(object, typename_string);
    EXPECT_FALSE(object_string.empty());

    SerializableType nobject{ tesseract::common::Serialization::fromArchiveStringXML<SerializableType>(
        object_string, typename_string) };
    EXPECT_TRUE(compare(object, nobject));
  }

  {  // Archive program to string
    std::string object_string = tesseract::common::Serialization::toArchiveStringJSON<SerializableType>(object);
    EXPECT_FALSE(object_string.empty());

    SerializableType nobject{ tesseract::common::Serialization::fromArchiveStringJSON<SerializableType>(
        object_string) };
    EXPECT_TRUE(compare(object, nobject));
  }

  {  // Archive program to string with name
    std::string object_string =
        tesseract::common::Serialization::toArchiveStringJSON<SerializableType>(object, typename_string);
    EXPECT_FALSE(object_string.empty());

    SerializableType nobject{ tesseract::common::Serialization::fromArchiveStringJSON<SerializableType>(
        object_string, typename_string) };
    EXPECT_TRUE(compare(object, nobject));
  }

  {  // Archive program to binary data
    std::vector<std::uint8_t> object_data =
        tesseract::common::Serialization::toArchiveBinaryData<SerializableType>(object);
    EXPECT_FALSE(object_data.empty());

    SerializableType nobject{ tesseract::common::Serialization::fromArchiveBinaryData<SerializableType>(object_data) };
    EXPECT_TRUE(compare(object, nobject));
  }

  {  // Archive program to binary data with name
    std::vector<std::uint8_t> object_data =
        tesseract::common::Serialization::toArchiveBinaryData<SerializableType>(object, typename_string);
    EXPECT_FALSE(object_data.empty());

    SerializableType nobject{ tesseract::common::Serialization::fromArchiveBinaryData<SerializableType>(
        object_data, typename_string) };
    EXPECT_TRUE(compare(object, nobject));
  }
}

/**
 * @brief Tests serialization for a serializable derived type
 * @details Serializes the type to XML file, binary file, and XML string using the base type. It then deserializes it,
 * casts it to the derived type, and calls the equality operator on the results
 * @param object Base class pointer to the object to be serialized
 * @param typename_string Prefix used for filepaths. Serialized files are put in /tmp/<typename_string>.<extension>
 */
template <typename SerializableTypeBase, typename SerializableTypeDerived>
void testSerializationDerivedClass(const std::shared_ptr<SerializableTypeBase>& object,
                                   const std::string& typename_string)
{
  {  // Archive program to XML file
    std::string file_path = tesseract::common::getTempPath() + typename_string + ".xml";
    EXPECT_TRUE(
        tesseract::common::Serialization::toArchiveFileXML<std::shared_ptr<SerializableTypeBase>>(object, file_path));

    auto nobject =
        tesseract::common::Serialization::fromArchiveFileXML<std::shared_ptr<SerializableTypeBase>>(file_path);
    auto nobject_derived = std::dynamic_pointer_cast<SerializableTypeDerived>(nobject);

    auto object_derived = std::dynamic_pointer_cast<SerializableTypeDerived>(object);
    EXPECT_FALSE(*object_derived != *nobject_derived);  // Using != because it call == for code coverage
  }

  {  // Archive program to JSON file
    std::string file_path = tesseract::common::getTempPath() + typename_string + ".json";
    EXPECT_TRUE(
        tesseract::common::Serialization::toArchiveFileJSON<std::shared_ptr<SerializableTypeBase>>(object, file_path));

    auto nobject =
        tesseract::common::Serialization::fromArchiveFileJSON<std::shared_ptr<SerializableTypeBase>>(file_path);
    auto nobject_derived = std::dynamic_pointer_cast<SerializableTypeDerived>(nobject);

    auto object_derived = std::dynamic_pointer_cast<SerializableTypeDerived>(object);
    EXPECT_FALSE(*object_derived != *nobject_derived);  // Using != because it call == for code coverage
  }

  {  // Archive program to binary file
    std::string file_path = tesseract::common::getTempPath() + typename_string + ".binary";
    EXPECT_TRUE(tesseract::common::Serialization::toArchiveFileBinary<std::shared_ptr<SerializableTypeBase>>(
        object, file_path));

    auto nobject =
        tesseract::common::Serialization::fromArchiveFileBinary<std::shared_ptr<SerializableTypeBase>>(file_path);
    auto nobject_derived = std::dynamic_pointer_cast<SerializableTypeDerived>(nobject);

    auto object_derived = std::dynamic_pointer_cast<SerializableTypeDerived>(object);
    EXPECT_FALSE(*object_derived != *nobject_derived);  // Using != because it call == for code coverage
  }

  {  // Archive program to string
    std::string object_string =
        tesseract::common::Serialization::toArchiveStringXML<std::shared_ptr<SerializableTypeBase>>(object,
                                                                                                    typename_string);
    EXPECT_FALSE(object_string.empty());

    auto nobject = tesseract::common::Serialization::fromArchiveStringXML<std::shared_ptr<SerializableTypeBase>>(
        object_string, typename_string);
    auto nobject_derived = std::dynamic_pointer_cast<SerializableTypeDerived>(nobject);

    auto object_derived = std::dynamic_pointer_cast<SerializableTypeDerived>(object);
    EXPECT_FALSE(*object_derived != *nobject_derived);  // Using != because it call == for code coverage
  }

  {  // Archive program to string
    std::string object_string =
        tesseract::common::Serialization::toArchiveStringJSON<std::shared_ptr<SerializableTypeBase>>(object,
                                                                                                     typename_string);
    EXPECT_FALSE(object_string.empty());

    auto nobject = tesseract::common::Serialization::fromArchiveStringJSON<std::shared_ptr<SerializableTypeBase>>(
        object_string, typename_string);
    auto nobject_derived = std::dynamic_pointer_cast<SerializableTypeDerived>(nobject);

    auto object_derived = std::dynamic_pointer_cast<SerializableTypeDerived>(object);
    EXPECT_FALSE(*object_derived != *nobject_derived);  // Using != because it call == for code coverage
  }

  {  // Archive program to binary data
    std::vector<std::uint8_t> object_data =
        tesseract::common::Serialization::toArchiveBinaryData<std::shared_ptr<SerializableTypeBase>>(object,
                                                                                                     typename_string);
    EXPECT_FALSE(object_data.empty());

    auto nobject = tesseract::common::Serialization::fromArchiveBinaryData<std::shared_ptr<SerializableTypeBase>>(
        object_data, typename_string);
    auto nobject_derived = std::dynamic_pointer_cast<SerializableTypeDerived>(nobject);

    auto object_derived = std::dynamic_pointer_cast<SerializableTypeDerived>(object);
    EXPECT_FALSE(*object_derived != *nobject_derived);  // Using != because it call == for code coverage
  }
}

/**
 * @brief Tests serialization for a serializable any poly type
 * @details Serializes the type to XML file, binary file, and XML string using the base type. It then deserializes it,
 * casts it to the derived type, and calls the equality operator on the results
 * @param object Base class pointer to the object to be serialized
 * @param typename_string Prefix used for filepaths. Serialized files are put in /tmp/<typename_string>.<extension>
 */
template <typename SerializableTypeStored>
void testSerializationAnyPoly(
    const tesseract::common::AnyPoly& object,
    const std::string& typename_string,
    TestSerializationCompareFn<SerializableTypeStored> compare = testSerializationCompareEqual<SerializableTypeStored>)
{
  {  // Archive program to XML file
    std::string file_path = tesseract::common::getTempPath() + typename_string + ".xml";
    EXPECT_TRUE(tesseract::common::Serialization::toArchiveFileXML<tesseract::common::AnyPoly>(object, file_path));

    auto nobject = tesseract::common::Serialization::fromArchiveFileXML<tesseract::common::AnyPoly>(file_path);
    const auto& nobject_stored = nobject.as<SerializableTypeStored>();
    EXPECT_TRUE(nobject.getType() == std::type_index(typeid(SerializableTypeStored)));

    const auto& object_stored = object.as<SerializableTypeStored>();
    EXPECT_TRUE(compare(object_stored, nobject_stored));
  }

  {  // Archive program to XML file
    std::string file_path = tesseract::common::getTempPath() + typename_string + ".json";
    EXPECT_TRUE(tesseract::common::Serialization::toArchiveFileJSON<tesseract::common::AnyPoly>(object, file_path));

    auto nobject = tesseract::common::Serialization::fromArchiveFileJSON<tesseract::common::AnyPoly>(file_path);
    const auto& nobject_stored = nobject.as<SerializableTypeStored>();
    EXPECT_TRUE(nobject.getType() == std::type_index(typeid(SerializableTypeStored)));

    const auto& object_stored = object.as<SerializableTypeStored>();
    EXPECT_TRUE(compare(object_stored, nobject_stored));
  }

  {  // Archive program to binary file
    std::string file_path = tesseract::common::getTempPath() + typename_string + ".binary";
    EXPECT_TRUE(tesseract::common::Serialization::toArchiveFileBinary<tesseract::common::AnyPoly>(object, file_path));

    auto nobject = tesseract::common::Serialization::fromArchiveFileBinary<tesseract::common::AnyPoly>(file_path);
    const auto& nobject_stored = nobject.as<SerializableTypeStored>();
    EXPECT_TRUE(nobject.getType() == std::type_index(typeid(SerializableTypeStored)));

    const auto& object_stored = object.as<SerializableTypeStored>();
    EXPECT_TRUE(compare(object_stored, nobject_stored));
  }

  {  // Archive program to string
    std::string object_string =
        tesseract::common::Serialization::toArchiveStringXML<tesseract::common::AnyPoly>(object, typename_string);
    EXPECT_FALSE(object_string.empty());

    auto nobject = tesseract::common::Serialization::fromArchiveStringXML<tesseract::common::AnyPoly>(object_string,
                                                                                                      typename_string);
    const auto& nobject_stored = nobject.as<SerializableTypeStored>();
    EXPECT_TRUE(nobject.getType() == std::type_index(typeid(SerializableTypeStored)));

    const auto& object_stored = object.as<SerializableTypeStored>();
    EXPECT_TRUE(compare(object_stored, nobject_stored));
  }

  {  // Archive program to string
    std::string object_string =
        tesseract::common::Serialization::toArchiveStringJSON<tesseract::common::AnyPoly>(object, typename_string);
    EXPECT_FALSE(object_string.empty());

    auto nobject = tesseract::common::Serialization::fromArchiveStringJSON<tesseract::common::AnyPoly>(object_string,
                                                                                                       typename_string);
    const auto& nobject_stored = nobject.as<SerializableTypeStored>();
    EXPECT_TRUE(nobject.getType() == std::type_index(typeid(SerializableTypeStored)));

    const auto& object_stored = object.as<SerializableTypeStored>();
    EXPECT_TRUE(compare(object_stored, nobject_stored));
  }

  {  // Archive program to binary data
    std::vector<std::uint8_t> object_data =
        tesseract::common::Serialization::toArchiveBinaryData<tesseract::common::AnyPoly>(object, typename_string);
    EXPECT_FALSE(object_data.empty());

    auto nobject = tesseract::common::Serialization::fromArchiveBinaryData<tesseract::common::AnyPoly>(object_data,
                                                                                                       typename_string);
    const auto& nobject_stored = nobject.as<SerializableTypeStored>();
    EXPECT_TRUE(nobject.getType() == std::type_index(typeid(SerializableTypeStored)));

    const auto& object_stored = object.as<SerializableTypeStored>();
    EXPECT_TRUE(compare(object_stored, nobject_stored));
  }
}
}  // namespace tesseract::common
#endif  // TESSERACT_COMMON_UTILS_H
