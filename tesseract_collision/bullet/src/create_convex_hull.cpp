/**
 * @file create_convex_hull.cpp
 * @brief This takes an input file and generates a convex hull ply file
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
#include <tesseract_common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <console_bridge/console.h>
#include <boost/program_options.hpp>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract_collision/core/common.h>
#include <tesseract_collision/bullet/convex_hull_utils.h>

namespace
{
const size_t ERROR_IN_COMMAND_LINE = 1;
const size_t SUCCESS = 0;
const size_t ERROR_UNHANDLED_EXCEPTION = 2;

}  // namespace

int main(int argc, char** argv)
{
  std::string input;
  std::string output;
  double shrink = -1.0;
  double clamp = -1.0;

  namespace po = boost::program_options;
  po::options_description desc("Options");
  desc.add_options()("help,h", "Print help messages")(
      "input,i", po::value<std::string>(&input)->required(), "File path to mesh used to create a convex hull.")(
      "output,o", po::value<std::string>(&output)->required(), "File path to save the generated convex hull as a ply.")(
      "shrink,s",
      po::value<double>(&shrink),
      "If positive, the convex hull is shrunken by that amount (each face is moved by 'shrink' length units towards "
      "the center along its normal).")("clamp,c",
                                       po::value<double>(&clamp),
                                       "If positive, 'shrink' is clamped to not exceed 'clamp * innerRadius', where "
                                       "'innerRadius' is the minimum distance of a face to the center of the convex "
                                       "hull.");

  po::variables_map vm;
  try
  {
    po::store(po::parse_command_line(argc, argv, desc), vm);  // can throw

    /** --help option */
    if (vm.count("help") != 0U)
    {
      std::cout << "Basic Command Line Parameter App" << std::endl << desc << std::endl;
      return SUCCESS;
    }

    po::notify(vm);  // throws on error, so do after help in case
                     // there are any problems
  }
  catch (po::error& e)
  {
    std::cerr << "ERROR: " << e.what() << std::endl << std::endl;
    std::cerr << desc << std::endl;
    return ERROR_IN_COMMAND_LINE;
  }

  std::ifstream file(input, std::ios::binary | std::ios::ate);
  std::streamsize size = file.tellg();
  if (size < 0)
  {
    CONSOLE_BRIDGE_logError("Failed to locate input file!");
    return ERROR_UNHANDLED_EXCEPTION;
  }

  tesseract_common::VectorVector3d mesh_vertices;
  Eigen::VectorXi mesh_faces;
  int num_faces = tesseract_collision::loadSimplePlyFile(input, mesh_vertices, mesh_faces);
  if (num_faces < 0)
  {
    CONSOLE_BRIDGE_logError("Failed to read mesh from file!");
    return ERROR_UNHANDLED_EXCEPTION;
  }

  tesseract_common::VectorVector3d ch_vertices;
  Eigen::VectorXi ch_faces;
  int ch_num_faces = tesseract_collision::createConvexHull(ch_vertices, ch_faces, mesh_vertices);

  if (ch_num_faces < 0)
  {
    CONSOLE_BRIDGE_logError("Failed to create convex hull!");
    return ERROR_UNHANDLED_EXCEPTION;
  }

  if (!tesseract_collision::writeSimplePlyFile(output, ch_vertices, ch_faces, ch_num_faces))
  {
    CONSOLE_BRIDGE_logError("Failed to write convex hull to file!");
    return ERROR_UNHANDLED_EXCEPTION;
  }

  return 0;
}
