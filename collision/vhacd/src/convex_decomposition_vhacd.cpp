#include <tesseract/common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <console_bridge/console.h>
#include <iomanip>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract/collision/bullet/convex_hull_utils.h>
#include <tesseract/collision/vhacd/convex_decomposition_vhacd.h>
#include <tesseract/geometry/impl/convex_mesh.h>

namespace tesseract::collision
{
class ProgressCallback : public VHACD::IVHACD::IUserCallback
{
public:
  ProgressCallback() = default;
  ~ProgressCallback() override = default;
  ProgressCallback(bool verbose) : verbose_(verbose) {}
  ProgressCallback(const ProgressCallback&) = default;
  ProgressCallback& operator=(const ProgressCallback&) = default;
  ProgressCallback(ProgressCallback&&) = default;
  ProgressCallback& operator=(ProgressCallback&&) = default;

  void Update(const double overallProgress,
              const double stageProgress,
              const char* const stage,
              const char* operation) override
  {
    if (!verbose_)
      return;
    std::cout << std::setfill(' ') << std::setw(3) << ceil(overallProgress) << "% " << "[ " << stage << " "
              << std::setfill(' ') << std::setw(3) << ceil(stageProgress) << "% ] " << operation << "\n";
  }

private:
  bool verbose_{ true };
};

ConvexDecompositionVHACD::ConvexDecompositionVHACD(const VHACDParameters& params) : params_(params) {}

std::vector<std::shared_ptr<tesseract::geometry::ConvexMesh> >
ConvexDecompositionVHACD::compute(const tesseract::common::VectorVector3d& vertices,
                                  const Eigen::VectorXi& faces,
                                  bool verbose) const
{
  if (verbose)
    params_.print();

  std::vector<double> points_local;
  points_local.reserve(vertices.size() * 3);
  for (const auto& v : vertices)
  {
    points_local.push_back(v.x());
    points_local.push_back(v.y());
    points_local.push_back(v.z());
  }

  std::vector<unsigned int> triangles_local;
  triangles_local.reserve(static_cast<std::size_t>(faces.size()) / 4);
  for (Eigen::Index i = 0; i < faces.rows();)
  {
    int face_vertice_cnt = faces(i++);
    if (face_vertice_cnt != 3)
      throw std::runtime_error("Currently only supports triangle meshes");

    triangles_local.push_back(static_cast<unsigned int>(faces(i++)));
    triangles_local.push_back(static_cast<unsigned int>(faces(i++)));
    triangles_local.push_back(static_cast<unsigned int>(faces(i++)));
  }

  // run V-HACD
  VHACD::IVHACD* interfaceVHACD = VHACD::CreateVHACD();

  ProgressCallback progress_callback(verbose);
  VHACD::IVHACD::Parameters par;
  par.m_maxConvexHulls = params_.max_convex_hulls;
  par.m_resolution = params_.resolution;
  par.m_minimumVolumePercentErrorAllowed = params_.minimum_volume_percent_error_allowed;
  par.m_maxRecursionDepth = params_.max_recursion_depth;
  par.m_shrinkWrap = params_.shrinkwrap;
  par.m_fillMode = params_.fill_mode;
  par.m_maxNumVerticesPerCH = params_.max_num_vertices_per_ch;
  par.m_asyncACD = params_.async_ACD;
  par.m_minEdgeLength = params_.min_edge_length;
  par.m_findBestPlane = params_.find_best_plane;
  par.m_callback = &progress_callback;

  bool res = interfaceVHACD->Compute(points_local.data(),
                                     static_cast<unsigned int>(points_local.size() / 3),
                                     triangles_local.data(),
                                     static_cast<unsigned int>(triangles_local.size() / 3),
                                     par);

  std::vector<tesseract::geometry::ConvexMesh::Ptr> output;
  if (res)
  {
    unsigned int num_convex_hulls = interfaceVHACD->GetNConvexHulls();
    VHACD::IVHACD::ConvexHull ch{};
    for (unsigned int p = 0; p < num_convex_hulls; ++p)
    {
      interfaceVHACD->GetConvexHull(p, ch);

      auto vhacd_vertices = std::make_shared<tesseract::common::VectorVector3d>();
      vhacd_vertices->reserve(ch.m_points.size());
      for (const auto& m_point : ch.m_points)
      {
        Eigen::Vector3d v(m_point.mX, m_point.mY, m_point.mZ);
        vhacd_vertices->push_back(v);
      }

      auto ch_vertices = std::make_shared<tesseract::common::VectorVector3d>();
      auto ch_faces = std::make_shared<Eigen::VectorXi>();
      int ch_num_faces = tesseract::collision::createConvexHull(*ch_vertices, *ch_faces, *vhacd_vertices);
      output.push_back(std::make_shared<tesseract::geometry::ConvexMesh>(ch_vertices, ch_faces, ch_num_faces));
    }
  }
  else
  {
    CONSOLE_BRIDGE_logError("Decomposition cancelled by user!");
  }

  interfaceVHACD->Clean();
  interfaceVHACD->Release();

  return output;
}

void VHACDParameters::print() const
{
  std::stringstream msg;
  msg << "+ Parameters\n";
  msg << "\t Max number of convex hulls                      " << max_convex_hulls << "\n";
  msg << "\t Voxel resolution                                " << resolution << "\n";
  msg << "\t Volume error allowed as a percentage            " << minimum_volume_percent_error_allowed << "\n";
  msg << "\t Maximum recursion depth                         " << max_recursion_depth << "\n";
  msg << "\t Shrinkwrap output to source mesh                " << shrinkwrap << "\n";
  msg << "\t Fill mode                                       ";
  switch (fill_mode)
  {
    case VHACD::FillMode::FLOOD_FILL:
      msg << "FLOOD_FILL";
      break;
    case VHACD::FillMode::SURFACE_ONLY:
      msg << "SURFACE_ONLY";
      break;
    case VHACD::FillMode::RAYCAST_FILL:
      msg << "RAYCAST_FILL";
      break;
  }
  msg << "\n";
  msg << "\t Maximum number of vertices                      " << max_num_vertices_per_ch << "\n";
  msg << "\t Run asynchronously                              " << async_ACD << "\n";
  msg << "\t Minimum size of a voxel edge                    " << min_edge_length << "\n";
  msg << "\t Attempt to split planes along the best location " << find_best_plane << "\n";

  std::cout << msg.str();
}

}  // namespace tesseract::collision
