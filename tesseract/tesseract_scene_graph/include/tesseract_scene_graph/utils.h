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

}  // namespace tesseract_scene_graph

#endif  // TESSERACT_SCENE_GRAPH_UTILS_H
