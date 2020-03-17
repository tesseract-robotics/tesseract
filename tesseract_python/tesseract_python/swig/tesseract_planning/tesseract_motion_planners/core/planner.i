/**
 * @file planner.i
 * @brief SWIG interface file for tesseract_planning/core/planner.h
 *
 * @author John Wason
 * @date December 10, 2019
 * @version TODO
 * @bug No known bugs
 *
 * @copyright Copyright (c) 2019, Wason Technology, LLC
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

%{
#include <tesseract_motion_planners/core/planner.h>
%}

namespace tesseract_motion_planners
{
class MotionPlanner
{
public:
  MotionPlanner(std::string name);
  virtual ~MotionPlanner();

  const std::string& getName();

  const PlannerRequest& getRequest() const;

  void setRequest(const PlannerRequest& request);

  %rename(_solve) solve;
  virtual tesseract_common::StatusCode solve(PlannerResponse& res,
                                             PostPlanCheckType check_type = PostPlanCheckType::DISCRETE_CONTINUOUS_COLLISION,
                                             bool verbose = false) = 0;
  %pythoncode %{
  def solve(self, check_type=PostPlanCheckType_DISCRETE_CONTINUOUS_COLLISION, verbose=False):
      response = PlannerResponse()
      status_code = self._solve(response, verbose)
      return status_code, response
  %}
  virtual tesseract_common::StatusCode isConfigured() const = 0;

  virtual bool terminate() = 0;

  virtual void clear() = 0;

};
}  // namespace tesseract_motion_planners
