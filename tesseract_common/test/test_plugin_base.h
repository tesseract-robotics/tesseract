/**
 * @file test_plugin_base.h
 * @brief Plugin Test plugin base class
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
#ifndef TESSERACT_COMMON_TEST_PLUGIN_BASE_H
#define TESSERACT_COMMON_TEST_PLUGIN_BASE_H

namespace tesseract_common
{
class TestPluginBase
{
public:
  TestPluginBase() = default;
  virtual ~TestPluginBase() = default;
  TestPluginBase(const TestPluginBase&) = default;
  TestPluginBase& operator=(const TestPluginBase&) = default;
  TestPluginBase(TestPluginBase&&) = default;
  TestPluginBase& operator=(TestPluginBase&&) = default;
  virtual double multiply(double x, double y) = 0;
};
}  // namespace tesseract_common

#endif  // TESSERACT_COMMON_TEST_PLUGIN_BASE_H
