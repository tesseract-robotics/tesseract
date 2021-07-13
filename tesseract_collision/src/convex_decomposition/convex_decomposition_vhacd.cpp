#include <tesseract_common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <console_bridge/console.h>
#include <tesseract_collision/vhacd/VHACD.h>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract_collision/core/common.h>
#include <tesseract_collision/convex_decomposition/convex_decomposition_vhacd.h>

namespace tesseract_collision
{
class ProgressCallback : public VHACD::IVHACD::IUserCallback
{
public:
  ProgressCallback() = default;
  ~ProgressCallback() override = default;
  ProgressCallback(const ProgressCallback&) = default;
  ProgressCallback& operator=(const ProgressCallback&) = default;
  ProgressCallback(ProgressCallback&&) = default;
  ProgressCallback& operator=(ProgressCallback&&) = default;

  void Update(double overallProgress,
              double stageProgress,
              double operationProgress,
              const std::string& stage,
              const std::string& operation) override
  {
    std::cout << std::setfill(' ') << std::setw(3) << std::lround(overallProgress + 0.5) << "% "
              << "[ " << stage << " " << std::setfill(' ') << std::setw(3) << lround(stageProgress + 0.5) << "% ] "
              << operation << " " << std::setfill(' ') << std::setw(3) << std::lround(operationProgress + 0.5) << "%"
              << std::endl;
  }
};

ConvexDecompositionVHACD::ConvexDecompositionVHACD(const VHACDParameters& params) : params_(params) {}

std::vector<tesseract_geometry::ConvexMesh::Ptr>
ConvexDecompositionVHACD::compute(const tesseract_common::VectorVector3d& vertices, const Eigen::VectorXi& faces) const
{
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

  ProgressCallback progress_callback;
  VHACD::IVHACD::Parameters p;
  p.m_concavity = params_.concavity;
  p.m_alpha = params_.alpha;
  p.m_beta = params_.beta;
  p.m_minVolumePerCH = params_.min_volume_per_ch;
  p.m_resolution = params_.resolution;
  p.m_maxNumVerticesPerCH = params_.max_num_vertices_per_ch;
  p.m_planeDownsampling = params_.plane_downsampling;
  p.m_convexhullDownsampling = params_.convexhull_downsampling;
  p.m_pca = params_.pca;
  p.m_mode = params_.mode;
  p.m_convexhullApproximation = params_.convexhull_approximation;
  p.m_oclAcceleration = params_.ocl_acceleration;
  p.m_maxConvexHulls = params_.max_convehulls;
  p.m_projectHullVertices = params_.project_hull_vertices;
  p.m_callback = &progress_callback;

  bool res = interfaceVHACD->Compute(&points_local[0],
                                     static_cast<unsigned int>(points_local.size() / 3),
                                     (const uint32_t*)(&triangles_local[0]),
                                     static_cast<unsigned int>(triangles_local.size() / 3),
                                     p);

  std::vector<tesseract_geometry::ConvexMesh::Ptr> output;
  if (res)
  {
    unsigned int num_convex_hulls = interfaceVHACD->GetNConvexHulls();
    CONSOLE_BRIDGE_logError("Convex decomposition generated %lu convex hulls!", num_convex_hulls);
    VHACD::IVHACD::ConvexHull ch;
    for (unsigned int p = 0; p < num_convex_hulls; ++p)
    {
      interfaceVHACD->GetConvexHull(p, ch);

      auto vhacd_vertices = std::make_shared<tesseract_common::VectorVector3d>();
      vhacd_vertices->reserve(ch.m_nPoints);
      for (std::size_t i = 0; i < ch.m_nPoints; ++i)
      {
        Eigen::Vector3d v(ch.m_points[3 * i], ch.m_points[(3 * i) + 1], ch.m_points[(3 * i) + 2]);
        vhacd_vertices->push_back(v);
      }

      auto ch_vertices = std::make_shared<tesseract_common::VectorVector3d>();
      auto ch_faces = std::make_shared<Eigen::VectorXi>();
      int ch_num_faces = tesseract_collision::createConvexHull(*ch_vertices, *ch_faces, *vhacd_vertices);
      output.push_back(std::make_shared<tesseract_geometry::ConvexMesh>(ch_vertices, ch_faces, ch_num_faces));
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
  msg << "+ Parameters" << std::endl;
  msg << "\t resolution                                  " << resolution << std::endl;
  msg << "\t Max number of convex-hulls                  " << max_convehulls << std::endl;
  msg << "\t max. concavity                              " << concavity << std::endl;
  msg << "\t plane down-sampling                         " << plane_downsampling << std::endl;
  msg << "\t convex-hull down-sampling                   " << convexhull_downsampling << std::endl;
  msg << "\t alpha                                       " << alpha << std::endl;
  msg << "\t beta                                        " << beta << std::endl;
  msg << "\t pca                                         " << pca << std::endl;
  msg << "\t mode                                        " << mode << std::endl;
  msg << "\t max. vertices per convex-hull               " << max_num_vertices_per_ch << std::endl;
  msg << "\t min. volume to add vertices to convex-hulls " << min_volume_per_ch << std::endl;
  msg << "\t convex-hull approximation                   " << convexhull_approximation << std::endl;
  msg << "\t OpenCL acceleration                         " << ocl_acceleration << std::endl;
  //  msg << "\t OpenCL platform ID                          " << oclPlatformID << std::endl;
  //  msg << "\t OpenCL device ID                            " << oclDeviceID << std::endl;

  std::cout << msg.str();
}

}  // namespace tesseract_collision
