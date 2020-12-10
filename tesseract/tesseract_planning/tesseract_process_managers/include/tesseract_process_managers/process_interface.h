/**
 * @file process_interface.h
 * @brief Process Inteface
 *
 * @author Levi Armstrong
 * @date December 8. 2020
 * @version TODO
 * @bug No known bugs
 *
 * @copyright Copyright (c) 2020, Southwest Research Institute
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
#ifndef TESSERACT_PROCESS_MANAGERS_PROCESS_INTERFACE_H
#define TESSERACT_PROCESS_MANAGERS_PROCESS_INTERFACE_H

#include <tesseract_common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <memory>
#include <atomic>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

namespace tesseract_planning
{
/**
 * @brief This is a thread safe class used for aborting a process along with checking if a process was succesful
 * @details If a process failed then the process has been abort by some child process
 */
class ProcessInterface
{
public:
  using Ptr = std::shared_ptr<ProcessInterface>;

  /**
   * @brief Check if the process was aborted
   * @return True if aborted, otherwise false
   */
  bool isAborted() const;

  /**
   * @brief Check if the process finished without error
   * @return True if the process was not aborted, otherwise false
   */
  bool isSuccessful() const;

  /** @brief Abort the process associated with this interface */
  void abort();

protected:
  std::atomic<bool> abort_{ false };
};

}  // namespace tesseract_planning

#endif  // TESSERACT_PROCESS_MANAGERS_PROCESS_INTERFACE_H
