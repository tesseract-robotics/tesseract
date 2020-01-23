/**
 * @file ompl_motion_planner.cpp
 * @brief Tesseract OMPL motion planner
 *
 * @author Levi Armstrong
 * @date April 18, 2018
 * @version TODO
 * @bug No known bugs
 *
 * @copyright Copyright (c) 2017, Southwest Research Institute
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

#include <tesseract_common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <ompl/geometric/planners/sbl/SBL.h>
#include <ompl/geometric/planners/est/EST.h>
#include <ompl/geometric/planners/kpiece/LBKPIECE1.h>
#include <ompl/geometric/planners/kpiece/BKPIECE1.h>
#include <ompl/geometric/planners/kpiece/KPIECE1.h>
#include <ompl/geometric/planners/rrt/RRT.h>
#include <ompl/geometric/planners/rrt/RRTConnect.h>
#include <ompl/geometric/planners/rrt/RRTstar.h>
#include <ompl/geometric/planners/rrt/TRRT.h>
#include <ompl/geometric/planners/prm/PRM.h>
#include <ompl/geometric/planners/prm/PRMstar.h>
#include <ompl/geometric/planners/prm/LazyPRMstar.h>
#include <ompl/geometric/planners/prm/SPARS.h>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract_motion_planners/ompl/impl/ompl_motion_planner.hpp>
#include <tesseract_motion_planners/ompl/ompl_settings.h>

namespace tesseract_motion_planners
{
// Explicit template instantiation
template class OMPLMotionPlanner<ompl::geometric::SBL>;
template class OMPLMotionPlanner<ompl::geometric::EST>;
template class OMPLMotionPlanner<ompl::geometric::LBKPIECE1>;
template class OMPLMotionPlanner<ompl::geometric::BKPIECE1>;
template class OMPLMotionPlanner<ompl::geometric::KPIECE1>;
template class OMPLMotionPlanner<ompl::geometric::RRT>;
template class OMPLMotionPlanner<ompl::geometric::RRTConnect>;
template class OMPLMotionPlanner<ompl::geometric::RRTstar>;
template class OMPLMotionPlanner<ompl::geometric::TRRT>;
template class OMPLMotionPlanner<ompl::geometric::PRM>;
template class OMPLMotionPlanner<ompl::geometric::PRMstar>;
template class OMPLMotionPlanner<ompl::geometric::LazyPRMstar>;
template class OMPLMotionPlanner<ompl::geometric::SPARS>;

}  // namespace tesseract_motion_planners
