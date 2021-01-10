#ifndef TESSERACT_SCENE_GRAPH_UTILS_H
#define TESSERACT_SCENE_GRAPH_UTILS_H

#include <tesseract_scene_graph/allowed_collision_matrix.h>
#include <tesseract_scene_graph/graph.h>
#include <tesseract_scene_graph/srdf_model.h>
#include <functional>

namespace tesseract_scene_graph
{
inline void processSRDFAllowedCollisions(SceneGraph& scene_graph, const tesseract_scene_graph::SRDFModel& srdf_model)
{
  for (const auto& pair : srdf_model.getAllowedCollisionMatrix().getAllAllowedCollisions())
    scene_graph.addAllowedCollision(pair.first.first, pair.first.second, pair.second);
}

/**
 * @brief Gets allowed collisions for a set of link names.
 * @param link_names Vector of link names for which we want the allowed collisions
 * @param acm_entries Entries in the ACM. Get this with AllowedCollisionMatrix::getAllAllowedCollisions()
 * @param remove_duplicates If true, duplicates will be removed. Default: true
 * @return vector of links that are allowed to collide with links given
 */
inline std::vector<std::string> getAllowedCollisions(const std::vector<std::string>& link_names,
                                                     const AllowedCollisionEntries& acm_entries,
                                                     bool remove_duplicates = true)
{
  std::vector<std::string> results;
  results.reserve(acm_entries.size());

  for (const auto& entry : acm_entries)
  {
    const std::string link_1 = entry.first.first;
    const std::string link_2 = entry.first.second;

    // If the first entry is one of the links we were looking for
    if (std::find(link_names.begin(), link_names.end(), link_1) != link_names.end())
      // If it hasn't already been added or remove_duplicates is disabled
      if (!remove_duplicates || (std::find(results.begin(), results.end(), link_2) == results.end()))
        results.push_back(link_2);

    if (std::find(link_names.begin(), link_names.end(), link_2) != link_names.end())
      if (!remove_duplicates || (std::find(results.begin(), results.end(), link_1) == results.end()))
        results.push_back(link_1);
  }
  return results;
}

}  // namespace tesseract_scene_graph

#endif  // TESSERACT_SCENE_GRAPH_UTILS_H
