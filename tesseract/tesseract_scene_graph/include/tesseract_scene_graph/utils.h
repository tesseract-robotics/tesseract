#ifndef TESSERACT_SCENE_GRAPH_UTILS_H
#define TESSERACT_SCENE_GRAPH_UTILS_H

#include <tesseract_scene_graph/allowed_collision_matrix.h>
#include <tesseract_scene_graph/graph.h>
#include <tesseract_scene_graph/parser/srdf_parser.h>
#include <functional>

namespace tesseract_scene_graph
{
inline void processSRDFAllowedCollisions(SceneGraph& scene_graph, const tesseract_scene_graph::SRDFModel& srdf_model)
{
  for (const auto& pair : srdf_model.getDisabledCollisionPairs())
    scene_graph.addAllowedCollision(pair.link1_, pair.link2_, pair.reason_);
}

}  // namespace tesseract_scene_graph

#endif  // TESSERACT_SCENE_GRAPH_UTILS_H
