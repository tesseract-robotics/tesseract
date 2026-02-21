/**
 * @file Create a convex decomposition of a mesh
 * @brief This takes an input file and generates a convex decomposition
 *
 * @author Michael Ripperger
 * @date Nov 22, 2023
 *
 * @copyright Copyright (c) 2023, Southwest Research Institute
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
#include <tesseract_common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <console_bridge/console.h>
#include <boost/program_options.hpp>
#include <iostream>
#include <fstream>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract_common/ply_io.h>
#include <tesseract_collision/vhacd/convex_decomposition_vhacd.h>
#include <tesseract_geometry/impl/convex_mesh.h>

namespace
{
const size_t ERROR_IN_COMMAND_LINE = 1;
const size_t SUCCESS = 0;
const size_t ERROR_UNHANDLED_EXCEPTION = 2;

}  // namespace

template <typename T>
void check_range(const T& value, const T& min, const T& max)
{
  if (value < min || value > max)
  {
    std::stringstream ss;
    ss << "Value " << value << " is not in valid range [" << min << ", " << max << "]";
    throw std::runtime_error(ss.str());
  }
}

int main(int argc, char** argv)
{
  std::string input;
  std::string output;
  tesseract::collision::VHACDParameters params;

  // clang-format off
  namespace po = boost::program_options;
  po::options_description desc("Options");
  desc.add_options()
      (
        "help,h",
        "Print help messages"
      )
      (
        "input,i",
        po::value<std::string>(&input)->required(),
        "File path to mesh used to create a convex hull."
      )
      (
        "output,o",
        po::value<std::string>(&output)->required(),
        "File path to save the generated convex hull as a ply."
      )
      (
        "max_convex_hulls,n",
        po::value<unsigned>(&(params.max_convex_hulls))->notifier([](const unsigned& val){check_range(val, 1u, 32u);}),
        "Maximum number of convex hulls"
      )
      (
        "resolution,r",
        po::value<unsigned>(&(params.resolution))->notifier([](const unsigned& val){check_range(val, 1u, std::numeric_limits<unsigned>::max());}),
        "Number of voxels to use to represent the shape"
      )
      (
        "min_volume_percent_error,e",
        po::value<double>(&(params.minimum_volume_percent_error_allowed))->notifier([](const double& val){check_range(val, 0.001, 10.0);}),
        "If the voxels are within this threshold percentage of the volume of the hull, we consider this a close enough approximation"
      )
      (
        "max_recursion_depth,d",
        po::value<unsigned>(&(params.max_recursion_depth)),
        "Maximum recursion depth for convex decomposition improvement"
      )
      (
        "shrinkwrap,s",
        po::value<bool>(&(params.shrinkwrap)),
        "Shrinkwrap the voxel positions to the source mesh on output"
      )
      (
        "max_num_vertices,v",
        po::value<unsigned>(&(params.max_num_vertices_per_ch))->notifier([](const unsigned& val){check_range(val, 4u, std::numeric_limits<unsigned>::max());}),
        "Maximum number of vertices per convex hull"
      )
      (
        "min_edge_length,l",
        po::value<unsigned>(&(params.min_edge_length))->notifier([](const unsigned& val){check_range(val, 1u, std::numeric_limits<unsigned>::max());}),
        "Once a voxel patch has an edge length of less than this value in all 3 dimensions, stop recursing"
      )
      (
        "find_best_plane,p",
        po::value<bool>(&(params.find_best_plane)),
        "Flag for attempting to split planes along best location (experimental)"
      );
  // clang-format on

  po::variables_map vm;
  try
  {
    po::store(po::parse_command_line(argc, argv, desc), vm);  // can throw

    /** --help option */
    if (vm.count("help") != 0U)
    {
      std::cout << "Basic Command Line Parameter App\n" << desc << "\n";
      return SUCCESS;
    }

    po::notify(vm);  // throws on error, so do after help in case
                     // there are any problems
  }
  catch (po::error& e)
  {
    std::cerr << "ERROR: " << e.what() << "\n\n";
    std::cerr << desc << "\n";
    return ERROR_IN_COMMAND_LINE;
  }

  std::ifstream file(input, std::ios::binary | std::ios::ate);
  std::streamsize size = file.tellg();
  if (size < 0)
  {
    CONSOLE_BRIDGE_logError("Failed to locate input file!");
    return ERROR_UNHANDLED_EXCEPTION;
  }

  tesseract::common::VectorVector3d mesh_vertices;
  Eigen::VectorXi mesh_faces;
  int num_faces = tesseract::common::loadSimplePlyFile(input, mesh_vertices, mesh_faces, true);
  if (num_faces < 0)
  {
    CONSOLE_BRIDGE_logError("Failed to read mesh from file!");
    return ERROR_UNHANDLED_EXCEPTION;
  }

  tesseract::collision::ConvexDecompositionVHACD convex_decomp(params);
  std::vector<std::shared_ptr<tesseract::geometry::ConvexMesh>> convex_hulls =
      convex_decomp.compute(mesh_vertices, mesh_faces);

  if (convex_hulls.empty())
  {
    CONSOLE_BRIDGE_logError("Failed to create convex decomposition!");
    return ERROR_UNHANDLED_EXCEPTION;
  }

  for (std::size_t i = 0; i < convex_hulls.size(); ++i)
  {
    auto ch = convex_hulls[i];
    if (!tesseract::common::writeSimplePlyFile(
            std::to_string(i) + "_" + output, *(ch->getVertices()), *(ch->getFaces()), ch->getFaceCount()))
    {
      CONSOLE_BRIDGE_logError("Failed to write convex hull to file!");
      return ERROR_UNHANDLED_EXCEPTION;
    }
  }

  return 0;
}
