/**
 * @file test_plugin_multiply.h
 * @brief Plugin Test plugin class
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
#ifndef TESSERACT_COMMON_TEST_PLUGIN_SUM_H
#define TESSERACT_COMMON_TEST_PLUGIN_SUM_H

#include "test_plugin_base.h"

namespace tesseract_common
{
class TestPluginMultiply : public TestPluginBase
{
public:
  TestPluginMultiply() = default;
  ~TestPluginMultiply() override = default;
  TestPluginMultiply(const TestPluginMultiply&) = default;
  TestPluginMultiply& operator=(const TestPluginMultiply&) = default;
  TestPluginMultiply(TestPluginMultiply&&) = default;
  TestPluginMultiply& operator=(TestPluginMultiply&&) = default;
  double multiply(double x, double y) override;
};
}  // namespace tesseract_common

#endif  // TESSERACT_COMMON_TEST_PLUGIN_SUM_H
