/**
 * @file utils.h
 * @brief Tesseract Environment Utility Functions.
 *
 * @author Levi Armstrong
 * @date Dec 18, 2017
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
#ifndef TESSERACT_ENVIRONMENT_UTILS_H
#define TESSERACT_ENVIRONMENT_UTILS_H
#include <tesseract_environment/core/macros.h>
TESSERACT_ENVIRONMENT_IGNORE_WARNINGS_PUSH
#include <tesseract_scene_graph/graph.h>
#include <tesseract_scene_graph/parser/srdf_parser.h>
#include <utility>
#include <unordered_map>
TESSERACT_ENVIRONMENT_IGNORE_WARNINGS_POP

#include <tesseract_environment/core/types.h>

namespace tesseract_environment
{
  /**
   * @brief Get the active Link Names Recursively
   *
   *        This currently only works for graphs that are trees. Need to create a generic method using boost::visitor
   *
   * @param active_links
   * @param scene_graph
   * @param current_link
   * @param active
   */
  inline void getActiveLinkNamesRecursive(std::vector<std::string>& active_links,
                                          const tesseract_scene_graph::SceneGraphConstPtr& scene_graph,
                                          const std::string& current_link,
                                          bool active)
  {
    // recursively get all active links
    assert(scene_graph->isTree());
    if (active)
    {
      active_links.push_back(current_link);
      for (const auto& child_link : scene_graph->getAdjacentLinkNames(current_link))
        getActiveLinkNamesRecursive(active_links, scene_graph, child_link, active);
    }
    else
    {
      for (const auto& child_link : scene_graph->getAdjacentLinkNames(current_link))
        if (scene_graph->getInboundJoints(child_link)[0]->type != tesseract_scene_graph::JointType::FIXED)
          getActiveLinkNamesRecursive(active_links, scene_graph, child_link, true);
        else
          getActiveLinkNamesRecursive(active_links, scene_graph, child_link, active);
    }
  }

  /**  @brief Store a AdjacencyMap <LinkA, Pair<LinkB, Transfrom B to A> > */
  typedef std::unordered_map<std::string, std::pair<std::string, Eigen::Isometry3d>> AdjacencyMap;
  typedef std::shared_ptr<AdjacencyMap> AdjacencyMapPtr;
  typedef std::shared_ptr<const AdjacencyMap> AdjacencyMapConstPtr;

  /**
   * @brief Create a adjacency map provided map_links and nearst parent in the active_links.
   *
   *        If a map_link does not have a parent in the list of active links it is not added the map
   *        Note: This currently only support tree structures.
   *
   * @param scene_graph
   * @param active_links
   * @param map_links
   * @param state
   * @return
   */
  inline AdjacencyMapPtr getAdjacencyMap(const tesseract_scene_graph::SceneGraphConstPtr& scene_graph,
                                         const std::vector<std::string>& active_links,
                                         const std::vector<std::string>& map_links,
                                         const TransformMap& state)
  {
     assert(scene_graph->isTree());
     AdjacencyMapPtr adj_map(new AdjacencyMap());
     AdjacencyMap& adjacency_map = *adj_map;
     for (const auto& ml : map_links)
     {

       if (std::find(active_links.begin(), active_links.end(), ml) != active_links.end())
       {
         Eigen::Isometry3d tf;
         tf.setIdentity();
         adjacency_map[ml] = std::make_pair(ml, tf);
         continue;
       }

       std::vector<std::string> inv_adj_links = scene_graph->getInvAdjacentLinkNames(ml);
       while (!inv_adj_links.empty())
       {
         assert(inv_adj_links.size() == 1);

         const std::string& ial = inv_adj_links[0];
         std::vector<std::string>::const_iterator it = std::find(active_links.begin(), active_links.end(), ial);
         if (it != active_links.end())
         {
           Eigen::Isometry3d tf = state.at(ial).inverse() * state.at(ml);
           adjacency_map[ml] = std::make_pair(ial, tf);
           break;
         }

         inv_adj_links = scene_graph->getInvAdjacentLinkNames(ial);
       }
     }

    return adj_map;
  }

  inline AllowedCollisionMatrixPtr getAllowedCollisionMatrix(const tesseract_scene_graph::SRDFModel& srdf_model)
  {
    AllowedCollisionMatrixPtr allowed_collision_matrix(new AllowedCollisionMatrix());
    for (const auto& pair : srdf_model.getDisabledCollisionPairs())
    {
      allowed_collision_matrix->addAllowedCollision(pair.link1_, pair.link2_, pair.reason_);
    }
    return allowed_collision_matrix;
  }

//  /**
//   * @brief continuousCollisionCheckTrajectory Should perform a continuous collision check over the trajectory
//   * and stop on first collision.
//   * @param manager A continuous contact manager
//   * @param env The environment
//   * @param joint_names JointNames corresponding to the values in traj (must be in same order)
//   * @param link_names Name of the links to calculate collision data for.
//   * @param traj The joint values at each time step
//   * @param contacts A vector of vector of ContactMap where each indicie corrisponds to a timestep
//   * @param first_only Indicates if it should return on first contact
//   * @return True if collision was found, otherwise false.
//   */
//  inline bool continuousCollisionCheckTrajectory(tesseract_collision::ContinuousContactManager& manager,
//                                                 const tesseract_environment::Environment& env,
//                                                 const tesseract_kinematics::ForwardKinematics& kin,
//                                                 const Eigen::Ref<const TrajArray>& traj,
//                                                 std::vector<tesseract_collision::ContactResultMap>& contacts,
//                                                 bool first_only = true)
//  {
//    bool found = false;
//    const std::vector<std::string>& joint_names = kin.getJointNames();
//    const std::vector<std::string>& link_names = kin.getLinkNames();

//    contacts.reserve(static_cast<size_t>(traj.rows() - 1));
//    for (int iStep = 0; iStep < traj.rows() - 1; ++iStep)
//    {
//      tesseract_collision::ContactResultMap collisions;

//      EnvStatePtr state0 = env.getState(joint_names, traj.row(iStep));
//      EnvStatePtr state1 = env.getState(joint_names, traj.row(iStep + 1));

//      for (const auto& link_name : link_names)
//        manager.setCollisionObjectsTransform(link_name, state0->transforms[link_name], state1->transforms[link_name]);

//      manager.contactTest(collisions, tesseract_collision::ContactTestTypes::FIRST);

//      if (collisions.size() > 0)
//        found = true;

//      contacts.push_back(collisions);

//      if (found && first_only)
//        break;
//    }

//    return found;
//  }
}
#endif // TESSERACT_ENVIRONMENT_UTILS_H
