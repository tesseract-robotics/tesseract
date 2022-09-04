/**
 * @file unit_test_utils.h
 * @brief Common Tesseract unit test utilities
 *
 * @author Levi Armstrong
 * @author Matthew Powelson
 * @date March 16, 2022
 * @version TODO
 * @bug No known bugs
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
#include <boost/serialization/shared_ptr.hpp>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract_common/serialization.h>
#include <tesseract_common/utils.h>

namespace tesseract_common
{
/**
 * @brief Tests Boost serialization for a serializable type
 * @details Serializes the type to XML file, binary file, and XML string. It then deserializes it and calls the equality
 * operator on the results
 * @param object Object to be serialized
 * @param typename_string Prefix used for filepaths. Serialized files are put in /tmp/<typename_string>.<extension>
 */
template <typename SerializableType>
void testSerialization(const SerializableType& object, const std::string& typename_string)
{
  {  // Archive program to XML file
    std::string file_path = tesseract_common::getTempPath() + typename_string + ".xml";
    EXPECT_TRUE(tesseract_common::Serialization::toArchiveFileXML<SerializableType>(object, file_path));

    SerializableType nobject{ tesseract_common::Serialization::fromArchiveFileXML<SerializableType>(file_path) };
    EXPECT_FALSE(object != nobject);  // Using != because it call == for code coverage
  }

  {  // Archive program to binary file
    std::string file_path = tesseract_common::getTempPath() + typename_string + ".binary";
    EXPECT_TRUE(tesseract_common::Serialization::toArchiveFileBinary<SerializableType>(object, file_path));

    SerializableType nobject{ tesseract_common::Serialization::fromArchiveFileBinary<SerializableType>(file_path) };
    EXPECT_FALSE(object != nobject);  // Using != because it call == for code coverage
  }

  {  // Archive program to string
    std::string object_string =
        tesseract_common::Serialization::toArchiveStringXML<SerializableType>(object, typename_string);
    EXPECT_FALSE(object_string.empty());

    SerializableType nobject{ tesseract_common::Serialization::fromArchiveStringXML<SerializableType>(object_string) };
    EXPECT_FALSE(object != nobject);  // Using != because it call == for code coverage
  }
}

/**
 * @brief Tests Boost serialization of shared pointer for a serializable type
 * @details Serializes the type to XML file, binary file, and XML string. It then deserializes it and calls the equality
 * operator on the results
 * @param object Object to be serialized
 * @param typename_string Prefix used for filepaths. Serialized files are put in /tmp/<typename_string>.<extension>
 */
template <typename SerializableType>
void testSerializationPtr(const std::shared_ptr<SerializableType>& object, const std::string& typename_string)
{
  {  // Archive program to XML file
    std::string file_path = tesseract_common::getTempPath() + typename_string + ".xml";
    EXPECT_TRUE(
        tesseract_common::Serialization::toArchiveFileXML<std::shared_ptr<SerializableType>>(object, file_path));

    auto nobject = tesseract_common::Serialization::fromArchiveFileXML<std::shared_ptr<SerializableType>>(file_path);
    EXPECT_FALSE(*object != *nobject);  // Using != because it call == for code coverage
  }

  {  // Archive program to binary file
    std::string file_path = tesseract_common::getTempPath() + typename_string + ".binary";
    EXPECT_TRUE(
        tesseract_common::Serialization::toArchiveFileBinary<std::shared_ptr<SerializableType>>(object, file_path));

    auto nobject = tesseract_common::Serialization::fromArchiveFileBinary<std::shared_ptr<SerializableType>>(file_path);
    EXPECT_FALSE(*object != *nobject);  // Using != because it call == for code coverage
  }

  {  // Archive program to string
    std::string object_string =
        tesseract_common::Serialization::toArchiveStringXML<std::shared_ptr<SerializableType>>(object, typename_string);
    EXPECT_FALSE(object_string.empty());

    auto nobject =
        tesseract_common::Serialization::fromArchiveStringXML<std::shared_ptr<SerializableType>>(object_string);
    EXPECT_FALSE(*object != *nobject);  // Using != because it call == for code coverage
  }
}

/**
 * @brief Tests Boost serialization for a serializable derived type
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
    std::string file_path = tesseract_common::getTempPath() + typename_string + ".xml";
    EXPECT_TRUE(
        tesseract_common::Serialization::toArchiveFileXML<std::shared_ptr<SerializableTypeBase>>(object, file_path));

    auto nobject =
        tesseract_common::Serialization::fromArchiveFileXML<std::shared_ptr<SerializableTypeBase>>(file_path);
    auto nobject_derived = std::dynamic_pointer_cast<SerializableTypeDerived>(nobject);

    auto object_derived = std::dynamic_pointer_cast<SerializableTypeDerived>(object);
    EXPECT_FALSE(*object_derived != *nobject_derived);  // Using != because it call == for code coverage
  }

  {  // Archive program to binary file
    std::string file_path = tesseract_common::getTempPath() + typename_string + ".binary";
    EXPECT_TRUE(
        tesseract_common::Serialization::toArchiveFileBinary<std::shared_ptr<SerializableTypeBase>>(object, file_path));

    auto nobject =
        tesseract_common::Serialization::fromArchiveFileBinary<std::shared_ptr<SerializableTypeBase>>(file_path);
    auto nobject_derived = std::dynamic_pointer_cast<SerializableTypeDerived>(nobject);

    auto object_derived = std::dynamic_pointer_cast<SerializableTypeDerived>(object);
    EXPECT_FALSE(*object_derived != *nobject_derived);  // Using != because it call == for code coverage
  }

  {  // Archive program to string
    std::string object_string =
        tesseract_common::Serialization::toArchiveStringXML<std::shared_ptr<SerializableTypeBase>>(object,
                                                                                                   typename_string);
    EXPECT_FALSE(object_string.empty());

    auto nobject =
        tesseract_common::Serialization::fromArchiveStringXML<std::shared_ptr<SerializableTypeBase>>(object_string);
    auto nobject_derived = std::dynamic_pointer_cast<SerializableTypeDerived>(nobject);

    auto object_derived = std::dynamic_pointer_cast<SerializableTypeDerived>(object);
    EXPECT_FALSE(*object_derived != *nobject_derived);  // Using != because it call == for code coverage
  }
}
}  // namespace tesseract_common
#endif  // TESSERACT_COMMON_UTILS_H
