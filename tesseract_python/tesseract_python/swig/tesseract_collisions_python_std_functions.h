/**
 * @file tesseract_collisions_python_std_functions.h
 * @brief Callback directors for tesseract_collision_python module
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

#include <tesseract_collision/core/types.h>
#include <tesseract_collision/core/continuous_contact_manager_factory.h>
#include <tesseract_collision/core/discrete_contact_manager_factory.h>

#pragma once

class IsContactAllowedFnBase
{
public:
  virtual bool call(const std::string& a, const std::string& b) = 0;
  virtual ~IsContactAllowedFnBase() {}
};

class IsContactValidFnBase
{
public:
  virtual bool call(const tesseract_collision::ContactResult& a) = 0;
  virtual ~IsContactValidFnBase() {}
};

class ContinuousContactManagerFactoryCreateMethodBase
{
public:
  virtual tesseract_collision::ContinuousContactManager::Ptr call() = 0;
  virtual ~ContinuousContactManagerFactoryCreateMethodBase() {}
};

class DiscreteContactManagerFactoryCreateMethodBase
{
public:
  virtual tesseract_collision::DiscreteContactManager::Ptr call() = 0;
  virtual ~DiscreteContactManagerFactoryCreateMethodBase() {}
};