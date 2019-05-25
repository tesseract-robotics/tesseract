#ifndef TESSERACT_SCENE_GRAPH_UTILS_H
#define TESSERACT_SCENE_GRAPH_UTILS_H

#include <tesseract_scene_graph/allowed_collision_matrix.h>
#include <tesseract_scene_graph/graph.h>
#include <tesseract_scene_graph/parser/urdf_parser.h>
#include <tesseract_scene_graph/parser/srdf_parser.h>
#include <console_bridge/console.h>


namespace tesseract_scene_graph
{

inline void processSRDFAllowedCollisions(SceneGraph& scene_graph, const tesseract_scene_graph::SRDFModel& srdf_model)
{
  for (const auto& pair : srdf_model.getDisabledCollisionPairs())
    scene_graph.addAllowedCollision(pair.link1_, pair.link2_, pair.reason_);
}

inline std::pair<SceneGraphPtr, SRDFModelPtr> createSceneGraphFromFiles(const std::string& urdf_file_path, const std::string& srdf_file_path, ResourceLocatorFn locator)
{
  SceneGraphPtr scene_graph = parseURDFFile(urdf_file_path, locator);
  if (scene_graph == nullptr)
  {
    CONSOLE_BRIDGE_logError("Failed to parse URDF.");
    return std::make_pair(nullptr, nullptr);
  }

  tesseract_scene_graph::SRDFModelPtr srdf = std::make_shared<tesseract_scene_graph::SRDFModel>();
  bool success = srdf->initFile(*scene_graph, srdf_file_path);
  if (!success)
  {
    CONSOLE_BRIDGE_logError("Failed to parse SRDF.");
    return std::make_pair(nullptr, nullptr);;
  }

  processSRDFAllowedCollisions(*scene_graph, *srdf);

  return std::make_pair(scene_graph, srdf);
}

inline std::pair<SceneGraphPtr, SRDFModelPtr> createSceneGraphFromStrings(const std::string& urdf_xml_string, const std::string& srdf_xml_string, ResourceLocatorFn locator)
{
  SceneGraphPtr scene_graph = parseURDFString(urdf_xml_string, locator);
  if (scene_graph == nullptr)
  {
    CONSOLE_BRIDGE_logError("Failed to parse URDF.");
    return std::make_pair(nullptr, nullptr);
  }

  tesseract_scene_graph::SRDFModelPtr srdf = std::make_shared<tesseract_scene_graph::SRDFModel>();
  bool success = srdf->initString(*scene_graph, srdf_xml_string);
  if (!success)
  {
    CONSOLE_BRIDGE_logError("Failed to parse SRDF.");
    return std::make_pair(nullptr, nullptr);;
  }

  processSRDFAllowedCollisions(*scene_graph, *srdf);

  return std::make_pair(scene_graph, srdf);
}

}

#endif // TESSERACT_SCENE_GRAPH_UTILS_H
