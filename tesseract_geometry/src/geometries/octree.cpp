/**
 * @file Octree.cpp
 * @brief Tesseract Octree Geometry
 *
 * @author Levi Armstrong
 * @date March 16, 2022
 * @version TODO
 * @bug No known bugs
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
#include <boost/serialization/access.hpp>
#include <boost/serialization/base_object.hpp>
#include <boost/serialization/nvp.hpp>
#include <boost/serialization/binary_object.hpp>
#include <boost/serialization/split_member.hpp>
#include <memory>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract_common/utils.h>
#include <tesseract_geometry/impl/octree.h>

namespace tesseract_geometry
{
bool Octree::operator==(const Octree& rhs) const
{
  using namespace tesseract_common;

  bool equal = true;
  equal &= Geometry::operator==(rhs);
  equal &= sub_type_ == rhs.sub_type_;
  equal &= pruned_ == rhs.pruned_;
  equal &= resolution_ == rhs.resolution_;

  // octree_ == rhs.octree_ looks for exact double equality
  equal &= octree_->getTreeDepth() == rhs.octree_->getTreeDepth();                             // tree_depth
  equal &= almostEqualRelativeAndAbs(octree_->getResolution(), rhs.octree_->getResolution());  // resolution
  equal &= octree_->size() == rhs.octree_->size();                                             // tree_size
  equal &= octree_->getNumLeafNodes() == rhs.octree_->getNumLeafNodes();
  equal &= calcNumSubShapes() == rhs.calcNumSubShapes();

  for (auto it = octree_->begin(static_cast<unsigned char>(octree_->getTreeDepth())), end = octree_->end(); it != end;
       ++it)
  {
    const auto coord = it.getCoordinate();
    const auto* node = rhs.octree_->search(coord);

    if (node != nullptr)
    {
      equal &= almostEqualRelativeAndAbs(it->getValue(), node->getValue());
      equal &= almostEqualRelativeAndAbs(it->getOccupancy(), node->getOccupancy());
    }
    else
      return false;
  }

  return equal;
}
bool Octree::operator!=(const Octree& rhs) const { return !operator==(rhs); }

template <class Archive>
void Octree::save(Archive& ar, const unsigned int /*version*/) const
{
  using namespace boost::serialization;
  ar& BOOST_SERIALIZATION_BASE_OBJECT_NVP(Geometry);
  ar& BOOST_SERIALIZATION_NVP(sub_type_);
  ar& BOOST_SERIALIZATION_NVP(resolution_);
  ar& BOOST_SERIALIZATION_NVP(pruned_);
  ar& BOOST_SERIALIZATION_NVP(binary_octree_);

  // Read the data to a stream which does not guarantee contiguous memory
  std::ostringstream s;
  if (binary_octree_)
    octree_->writeBinaryConst(s);
  else
    octree_->write(s);

  // Write it to a string, wich does guarantee contiguous memory
  std::string data_string = s.str();
  std::size_t octree_data_size = data_string.size();
  ar& make_nvp("octree_data_size", octree_data_size);
  ar& make_nvp("octree_data", make_binary_object(data_string.data(), octree_data_size));
}

template <class Archive>
void Octree::load(Archive& ar, const unsigned int /*version*/)
{
  using namespace boost::serialization;
  ar& BOOST_SERIALIZATION_BASE_OBJECT_NVP(Geometry);
  ar& BOOST_SERIALIZATION_NVP(sub_type_);
  ar& BOOST_SERIALIZATION_NVP(resolution_);
  ar& BOOST_SERIALIZATION_NVP(pruned_);
  ar& BOOST_SERIALIZATION_NVP(binary_octree_);

  // Initialize the octree to the right size
  auto local_octree = std::make_shared<octomap::OcTree>(resolution_);

  // Read the data into a string
  std::size_t octree_data_size = 0;
  ar& make_nvp("octree_data_size", octree_data_size);
  std::string data_string;
  data_string.resize(octree_data_size);
  ar& make_nvp("octree_data", make_binary_object(data_string.data(), octree_data_size));

  // Write that data into the stringstream required by octree and load data
  std::stringstream s;
  s.write(data_string.data(), static_cast<std::streamsize>(octree_data_size));

  if (binary_octree_)
    local_octree->readBinary(s);
  else
    local_octree = std::shared_ptr<octomap::OcTree>(dynamic_cast<octomap::OcTree*>(octomap::OcTree::read(s)));

  octree_ = local_octree;
}

template <class Archive>
void Octree::serialize(Archive& ar, const unsigned int version)
{
  boost::serialization::split_member(ar, *this, version);
}
}  // namespace tesseract_geometry

#include <tesseract_common/serialization.h>
TESSERACT_SERIALIZE_SAVE_LOAD_ARCHIVES_INSTANTIATE(tesseract_geometry::Octree)
BOOST_CLASS_EXPORT_IMPLEMENT(tesseract_geometry::Octree)
