/**
 * @file fwd.h
 * @brief Tesseract Scene Graph Forward Declarations
 *
 * @author Levi Armstrong
 * @date February 18, 2024
 *
 * @copyright Copyright (c) 2024, Levi Armstrong
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

#ifndef TESSERACT_SCENE_GRAPH_FWD_H
#define TESSERACT_SCENE_GRAPH_FWD_H

#include <cstdint>

namespace tesseract::scene_graph
{
// joint.h
class JointDynamics;
class JointLimits;
class JointSafety;
class JointCalibration;
class JointMimic;
enum class JointType : std::uint8_t;
class Joint;

// link.h
class Material;
class Inertial;
class Visual;
class Collision;
class Link;

// graph.h
struct ShortestPath;
class SceneGraph;

// scene_state.h
struct SceneState;
}  // namespace tesseract::scene_graph

#endif  // TESSERACT_SCENE_GRAPH_FWD_H
