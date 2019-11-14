/**
 * @file ompl_trajopt_freespace_planner.cpp
 * @brief Tesseract OMPL TrajOpt Freespace Planner
 *
 * @author Michael Ripperger
 * @date October 3, 2019
 * @version TODO
 * @bug No known bugs
 *
 * @copyright Copyright (c) 2019, Southwest Research Institute
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
#include <tesseract_motion_planners/hybrid/impl/ompl_trajopt_freespace_planner.hpp>

namespace ompl
{
namespace geometric
{
class SBL;
class EST;
class LBKPIECE1;
class BKPIECE1;
class KPIECE1;
class RRT;
class RRTConnect;
class RRTstar;
class TRRT;
class PRM;
class PRMstar;
class LazyPRMstar;
class SPARS;
}  // namespace geometric
}  // namespace ompl

namespace tesseract_motion_planners
{
// Explicit template instantiation
template class OMPLTrajOptFreespacePlanner<ompl::geometric::SBL>;
template class OMPLTrajOptFreespacePlanner<ompl::geometric::EST>;
template class OMPLTrajOptFreespacePlanner<ompl::geometric::LBKPIECE1>;
template class OMPLTrajOptFreespacePlanner<ompl::geometric::BKPIECE1>;
template class OMPLTrajOptFreespacePlanner<ompl::geometric::KPIECE1>;
template class OMPLTrajOptFreespacePlanner<ompl::geometric::RRT>;
template class OMPLTrajOptFreespacePlanner<ompl::geometric::RRTConnect>;
template class OMPLTrajOptFreespacePlanner<ompl::geometric::RRTstar>;
template class OMPLTrajOptFreespacePlanner<ompl::geometric::TRRT>;
template class OMPLTrajOptFreespacePlanner<ompl::geometric::PRM>;
template class OMPLTrajOptFreespacePlanner<ompl::geometric::PRMstar>;
template class OMPLTrajOptFreespacePlanner<ompl::geometric::LazyPRMstar>;
template class OMPLTrajOptFreespacePlanner<ompl::geometric::SPARS>;

}  // namespace tesseract_motion_planners
