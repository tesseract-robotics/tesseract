#ifndef TESSERACT_SCENE_GRAPH_UTILS_H
#define TESSERACT_SCENE_GRAPH_UTILS_H
#include <tesseract_scene_graph/graph.h>
#include <boost/graph/dijkstra_shortest_paths.hpp>
#include <boost/graph/depth_first_search.hpp>
#include <fstream>

namespace tesseract
{
namespace graph
{
/**
 * @brief Holds the shortest path information.
 *
 * The first vector is a list of links along the shortest path
 * The second vector is a list of joints along the shortest path
 */
typedef std::pair<std::vector<std::string>, std::vector<std::string>> Path;

inline std::ostream& operator<<(std::ostream& os, const Path& path)
{
  os << "Links:" << std::endl;
  for (auto l : path.first)
    os << "  " << l << std::endl;

  os << "Joints:" << std::endl;
  for (auto j : path.second)
    os << "  " << j << std::endl;

  return os;
}

namespace utils
{

static inline Path getShortestPath(const std::string& root, const std::string& tip, const Graph& graph)
{
  std::vector<Vertex> p(boost::num_vertices(graph));
  std::vector<double> d(boost::num_vertices(graph));
  Vertex s = getVertex(root, graph);
  dijkstra_shortest_paths(graph, s, boost::predecessor_map(&p[0]).distance_map(&d[0]));

  std::vector<std::string> links;
  std::vector<std::string> joints;
  Vertex v = getVertex(tip, graph);; // We want to start at the destination and work our way back to the source
  for(Vertex u = p[v]; // Start by setting 'u' to the destintaion node's predecessor
      u != v; // Keep tracking the path until we get to the source
      v = u, u = p[v]) // Set the current vertex to the current predecessor, and the predecessor to one level up
  {
    links.push_back(boost::get(boost::vertex_link, graph)[v]->getName());
    joints.push_back(boost::get(boost::edge_joint, graph)[boost::edge(u, v, graph).first]->getName());
  }
  links.push_back(root);
  std::reverse(links.begin(), links.end());
  std::reverse(joints.begin(), joints.end());

#ifdef NDEBUG
  std::cout << "distances and parents:" << std::endl;
  Graph::vertex_iterator vi, vend;
  for (boost::tie(vi, vend) = boost::vertices(g); vi != vend; ++vi)
  {
    std::cout << "distance(" << boost::get(boost::vertex_link, g)[*vi]->getName() << ") = " << d[*vi] << ", ";
    std::cout << "parent(" << boost::get(boost::vertex_link, g)[*vi]->getName() << ") = " << boost::get(boost::vertex_link, g)[p[*vi]]->getName() << std::endl;
  }
  std::cout << std::endl;
#endif
  return Path(links, joints);
}

struct cycle_detector : public boost::dfs_visitor<>
{
  cycle_detector( bool& ascyclic)
    : ascyclic_(ascyclic) { }

  template <class e, class g>
  void back_edge(e, g&) {
    ascyclic_ = false;
  }
protected:
  bool& ascyclic_;
};

/**
 * @brief Determine if the graph contains cycles
 * @param graph The graph to check
 * @return True if graph is acyclic otherwise false
 */
static inline bool isAcyclic(const Graph& graph)
{
  bool acyclic = true;
  cycle_detector vis(acyclic);
  boost::depth_first_search(graph, boost::visitor(vis));
  return acyclic;
}

/**
 * @brief Saves Graph as Graph Description Language (DOT)
 * @param path The file path
 */
static inline void saveDOT(std::string path, const Graph& graph)
{
  std::ofstream dot_file(path);

  dot_file << "digraph D {\n"
    << "  rankdir=LR\n"
    << "  size=\"4,3\"\n"
    << "  ratio=\"fill\"\n"
    << "  edge[style=\"bold\"]\n" << "  node[shape=\"circle\"]\n";

  Graph::edge_iterator ei, ei_end;
  for (boost::tie(ei, ei_end) = boost::edges(graph); ei != ei_end; ++ei)
  {
    Edge e = *ei;
    Vertex u = boost::source(e, graph);
    Vertex v = boost::target(e, graph);
    JointConstPtr joint = boost::get(boost::edge_joint, graph)[e];

    dot_file << boost::get(boost::vertex_link, graph)[u]->getName()
             << " -> " << boost::get(boost::vertex_link, graph)[v]->getName()
             << "[label=\"" << joint->getName() << "\n(" << joint->type << ")\", color=\"black\"]";

  }
  dot_file << "}";
}



} // namespace utils
} // namespace graph
} // namespace tesseract


#endif // TESSERACT_GRAPH_UTILS_H
