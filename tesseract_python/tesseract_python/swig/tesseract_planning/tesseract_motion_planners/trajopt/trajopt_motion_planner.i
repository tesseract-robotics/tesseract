/**
 * @file trajopt_motion_planner.i
 * @brief SWIG interface file for tesseract_planning/trajopt/config/trajopt_motion_planner.h
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
#include <tesseract_motion_planners/trajopt/trajopt_motion_planner.h>
%}

%shared_ptr(tesseract_motion_planners::TrajOptMotionPlannerStatusCategory)

namespace tesseract_motion_planners
{
class TrajOptMotionPlannerStatusCategory;

class TrajOptMotionPlanner : public MotionPlanner
{
public:

  TrajOptMotionPlanner(std::string name = "TRAJOPT");

  ~TrajOptMotionPlanner() {}

  //bool setConfiguration(const TrajOptPlannerConfig::Ptr config);
  %extend {
    void setConfiguration(const TrajOptPlannerConfig::Ptr config)
  {
    if (!$self->setConfiguration(config))
      {
       throw std::runtime_error("setConfiguration failed");
      }
  }
  }

  %rename(_solve) solve;
  tesseract_common::StatusCode solve(PlannerResponse& response,
                                     PostPlanCheckType check_type = PostPlanhCheckType::DISCRETE_CONTINUOUS_COLLISION,
                                     bool verbose = false) override;

  bool terminate() override;

  void clear() override;

  tesseract_common::StatusCode isConfigured() const override;
};

class TrajOptMotionPlannerStatusCategory : public tesseract_common::StatusCategory
{
public:
  TrajOptMotionPlannerStatusCategory(std::string name);
  const std::string& name() const noexcept override;
  std::string message(int code) const override;

  enum
  {
    IsConfigured = 1,
    SolutionFound = 0,
    IsNotConfigured = -1,
    FailedToParseConfig = -2,
    FailedToFindValidSolution = -3,
    FoundValidSolutionInCollision = -4
  };
};

}  // namespace tesseract_motion_planners
