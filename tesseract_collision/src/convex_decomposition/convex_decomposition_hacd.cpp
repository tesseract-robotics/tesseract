#include <tesseract_common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <console_bridge/console.h>
#include <bullet/HACD/hacdCircularList.h>
#include <bullet/HACD/hacdGraph.h>
#include <bullet/HACD/hacdHACD.h>
#include <bullet/HACD/hacdICHull.h>
#include <bullet/HACD/hacdVector.h>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract_collision/core/common.h>
#include <tesseract_collision/convex_decomposition/convex_decomposition_hacd.h>

namespace tesseract_collision
{
ConvexDecompositionHACD::ConvexDecompositionHACD(const HACDParameters& params) : params_(params) {}

std::vector<tesseract_geometry::ConvexMesh::Ptr>
ConvexDecompositionHACD::compute(const tesseract_common::VectorVector3d& vertices, const Eigen::VectorXi& faces) const
{
  params_.print();

  std::vector<HACD::Vec3<HACD::Real>> points_local;
  points_local.reserve(vertices.size());
  for (const auto& v : vertices)
  {
    points_local.push_back(HACD::Vec3<HACD::Real>(v.x(), v.y(), v.z()));
  }

  std::vector<HACD::Vec3<long>> triangles_local;
  triangles_local.reserve(static_cast<std::size_t>(faces.size()) / 4);
  for (Eigen::Index i = 0; i < faces.rows();)
  {
    int face_vertice_cnt = faces(i++);
    if (face_vertice_cnt != 3)
      throw std::runtime_error("Currently only supports triangle meshes");

    HACD::Vec3<long> triangle;
    //    triangle[0] = faces(i++);
    //    triangle[1] = faces(i++);
    //    triangle[2] = faces(i++);
    triangles_local.push_back(triangle);
  }

  // run HACD
  HACD::HACD my_hacd;
  my_hacd.SetPoints(&points_local[0]);
  my_hacd.SetNPoints(points_local.size());
  my_hacd.SetTriangles(&triangles_local[0]);
  my_hacd.SetNTriangles(triangles_local.size());
  my_hacd.SetCompacityWeight(params_.compacity_weight);
  my_hacd.SetVolumeWeight(params_.volume_weight);
  my_hacd.SetNClusters(params_.min_num_clusters);
  my_hacd.SetNVerticesPerCH(params_.max_num_vertices_per_ch);
  my_hacd.SetConcavity(params_.concavity);
  my_hacd.SetAddExtraDistPoints(params_.add_extra_dist_points);
  my_hacd.SetAddNeighboursDistPoints(params_.add_neighbours_dist_points);
  my_hacd.SetAddFacesPoints(params_.add_faces_points);

  bool res = my_hacd.Compute();

  std::vector<tesseract_geometry::ConvexMesh::Ptr> output;
  if (res)
  {
    std::size_t num_convex_hulls = my_hacd.GetNClusters();
    CONSOLE_BRIDGE_logError("Convex decomposition generated %lu convex hulls!", num_convex_hulls);

    for (unsigned int p = 0; p < num_convex_hulls; ++p)
    {
      // generate convex result
      size_t num_points = my_hacd.GetNPointsCH(p);
      size_t num_triangles = my_hacd.GetNTrianglesCH(p);

      std::vector<HACD::Vec3<HACD::Real>> points_ch(num_points);
      std::vector<HACD::Vec3<long>> triangles_ch(num_triangles);
      my_hacd.GetCH(p, &points_ch[0], &triangles_ch[0]);

      // points
      auto hacd_vertices = std::make_shared<tesseract_common::VectorVector3d>();
      hacd_vertices->reserve(num_points);
      for (size_t v = 0; v < num_points; v++)
      {
        Eigen::Vector3d vert(points_ch[v].X(), points_ch[v].Y(), points_ch[v].Z());
        hacd_vertices->push_back(vert);
      }

      auto ch_vertices = std::make_shared<tesseract_common::VectorVector3d>();
      auto ch_faces = std::make_shared<Eigen::VectorXi>();
      int ch_num_faces = tesseract_collision::createConvexHull(*ch_vertices, *ch_faces, *hacd_vertices);
      output.push_back(std::make_shared<tesseract_geometry::ConvexMesh>(ch_vertices, ch_faces, ch_num_faces));
    }
  }
  else
  {
    CONSOLE_BRIDGE_logError("Decomposition cancelled by user!");
  }

  return output;
}

void HACDParameters::print() const
{
  std::stringstream msg;
  msg << "+ Parameters" << std::endl;
  msg << "\t compacity_weight           " << compacity_weight << std::endl;
  msg << "\t volume_weight              " << volume_weight << std::endl;
  msg << "\t max. concavity             " << concavity << std::endl;
  msg << "\t min number of clusters     " << min_num_clusters << std::endl;
  msg << "\t add extra dist points      " << ((add_extra_dist_points) ? "true" : "false") << std::endl;
  msg << "\t add neighbours dist points " << ((add_neighbours_dist_points) ? "true" : "false") << std::endl;
  msg << "\t add faces points           " << ((add_faces_points) ? "true" : "false") << std::endl;

  std::cout << msg.str();
}

}  // namespace tesseract_collision
