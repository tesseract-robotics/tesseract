#ifndef TESSERACT_GEOMETRY_CEREAL_SERIALIZATION_H
#define TESSERACT_GEOMETRY_CEREAL_SERIALIZATION_H

#include <tesseract_geometry/geometry.h>
#include <tesseract_geometry/geometries.h>
#include <tesseract_common/cereal_boost_types.h>
#include <tesseract_common/cereal_eigen_types.h>

#include <octomap/OcTree.h>

#include <cereal/cereal.hpp>
#include <cereal/types/memory.hpp>
#include <cereal/archives/binary.hpp>

namespace tesseract_geometry
{
template <class Archive>
void serialize(Archive& ar, Geometry& obj)
{
  ar(cereal::base_class<Geometry>(&obj));
  ar(cereal::make_nvp("type", obj.type_));
  ar(cereal::make_nvp("uuid", obj.uuid_));
}

template <class Archive>
void serialize(Archive& ar, Box& obj)
{
  ar(cereal::base_class<Geometry>(&obj));
  ar(cereal::make_nvp("x", obj.x_));
  ar(cereal::make_nvp("y", obj.y_));
  ar(cereal::make_nvp("z", obj.z_));
}

template <class Archive>
void serialize(Archive& ar, Capsule& obj)
{
  ar(cereal::base_class<Geometry>(&obj));
  ar(cereal::make_nvp("radius", obj.r_));
  ar(cereal::make_nvp("length", obj.l_));
}

template <class Archive>
void serialize(Archive& ar, Cone& obj)
{
  ar(cereal::base_class<Geometry>(&obj));
  ar(cereal::make_nvp("radius", obj.r_));
  ar(cereal::make_nvp("length", obj.l_));
}

template <class Archive>
void serialize(Archive& ar, Cylinder& obj)
{
  ar(cereal::base_class<Geometry>(&obj));
  ar(cereal::make_nvp("radius", obj.r_));
  ar(cereal::make_nvp("length", obj.l_));
}

template <class Archive>
void serialize(Archive& ar, Plane& obj)
{
  ar(cereal::base_class<Geometry>(&obj));
  ar(cereal::make_nvp("a", obj.a_));
  ar(cereal::make_nvp("b", obj.b_));
  ar(cereal::make_nvp("c", obj.c_));
  ar(cereal::make_nvp("d", obj.d_));
}

template <class Archive>
void serialize(Archive& ar, Sphere& obj)
{
  ar(cereal::base_class<Geometry>(&obj));
  ar(cereal::make_nvp("radius", obj.r_));
}

template <class Archive>
void save(Archive& ar, Octree& obj)
{
  ar(cereal::base_class<Geometry>(&obj));
  ar(cereal::make_nvp("sub_type", obj.sub_type_));
  ar(cereal::make_nvp("resolution", obj.resolution_));
  ar(cereal::make_nvp("pruned", obj.pruned_));
  ar(cereal::make_nvp("binary_octree", obj.binary_octree_));

  // Read the data to a stream which does not guarantee contiguous memory
  std::ostringstream s;
  if (obj.binary_octree_)
    obj.octree_->writeBinaryConst(s);
  else
    obj.octree_->write(s);

  // Write it to a string, wich does guarantee contiguous memory
  std::string data_string = s.str();
  std::size_t octree_data_size = data_string.size();
  ar(cereal::make_nvp("octree_data_size", octree_data_size));
  ar(cereal::make_nvp("octree_data", cereal::binary_data(data_string.data(), octree_data_size)));
}

template <class Archive>
void load(Archive& ar, Octree& obj)
{
  ar(cereal::base_class<Geometry>(&obj));
  ar(cereal::make_nvp("sub_type", obj.sub_type_));
  ar(cereal::make_nvp("resolution", obj.resolution_));
  ar(cereal::make_nvp("pruned", obj.pruned_));
  ar(cereal::make_nvp("binary_octree", obj.binary_octree_));

  // Initialize the octree to the right size
  auto local_octree = std::make_shared<octomap::OcTree>(obj.resolution_);

  // Read the data into a string
  std::size_t octree_data_size = 0;
  ar(cereal::make_nvp("octree_data_size", octree_data_size));
  std::string data_string;
  data_string.resize(octree_data_size);
  ar(cereal::make_nvp("octree_data", cereal::binary_data(data_string.data(), octree_data_size)));

  // Write that data into the stringstream required by octree and load data
  std::stringstream s;
  s.write(data_string.data(), static_cast<std::streamsize>(octree_data_size));

  if (obj.binary_octree_)
    local_octree->readBinary(s);
  else
    local_octree = std::shared_ptr<octomap::OcTree>(dynamic_cast<octomap::OcTree*>(octomap::OcTree::read(s)));

  obj.octree_ = local_octree;
}

template <class Archive>
void serialize(Archive& ar, PolygonMesh& obj)
{
  ar(cereal::base_class<Geometry>(&obj));
  ar(cereal::make_nvp("vertices", obj.vertices_));
  ar(cereal::make_nvp("faces", obj.faces_));
  ar(cereal::make_nvp("vertex_count", obj.vertex_count_));
  ar(cereal::make_nvp("face_count", obj.face_count_));
  ar(cereal::make_nvp("resource", obj.resource_));
  ar(cereal::make_nvp("scale", obj.scale_));
  ar(cereal::make_nvp("normals", obj.normals_));
  ar(cereal::make_nvp("vertex_colors", obj.vertex_colors_));
  /// @todo Serialize mesh materials and textures
  //    ar(cereal::make_nvp("mesh_material", obj.mesh_material_));
  //    ar(cereal::make_nvp("mesh_textures", obj.mesh_textures_));
}

template <class Archive>
void serialize(Archive& ar, ConvexMesh& obj)
{
  ar(cereal::base_class<PolygonMesh>(&obj));
  ar(cereal::make_nvp("creation_method", obj.creation_method_));
}

template <class Archive>
void serialize(Archive& ar, Mesh& obj)
{
  ar(cereal::base_class<PolygonMesh>(&obj));
}

template <class Archive>
void serialize(Archive& ar, SDFMesh& obj)
{
  ar(cereal::base_class<PolygonMesh>(&obj));
}

template <class Archive>
void serialize(Archive& ar, CompoundMesh& obj)
{
  ar(cereal::base_class<Geometry>(&obj));
  ar(cereal::make_nvp("meshes", obj.meshes_));
}

}  // namespace tesseract_geometry

#endif  // TESSERACT_GEOMETRY_CEREAL_SERIALIZATION_H
