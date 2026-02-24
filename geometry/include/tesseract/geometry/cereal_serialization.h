#ifndef TESSERACT_GEOMETRY_CEREAL_SERIALIZATION_H
#define TESSERACT_GEOMETRY_CEREAL_SERIALIZATION_H

#include <tesseract/geometry/geometry.h>
#include <tesseract/geometry/geometries.h>
#include <tesseract/common/cereal_serialization.h>

#include <octomap/octomap.h>

#include <cereal/cereal.hpp>
#include <cereal/types/memory.hpp>
#include <cereal/types/vector.hpp>
#include <cereal/types/polymorphic.hpp>

namespace tesseract::geometry
{
template <class Archive>
void serialize(Archive& ar, Geometry& obj)
{
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
void serialize(Archive& ar, Octree& obj)
{
  ar(cereal::base_class<Geometry>(&obj));
  ar(cereal::make_nvp("sub_type", obj.sub_type_));
  ar(cereal::make_nvp("resolution", obj.resolution_));
  ar(cereal::make_nvp("pruned", obj.pruned_));
  ar(cereal::make_nvp("binary_octree", obj.binary_octree_));
  if (Archive::is_loading::value)
  {
    // Read bytes back
    std::vector<std::uint8_t> octree_data;
    ar(cereal::make_nvp("octree_data", octree_data));

    // Feed them to octomap via a binary stringstream
    auto local_octree = std::make_shared<octomap::OcTree>(obj.resolution_);
    std::stringstream ss(std::ios::in | std::ios::out | std::ios::binary);
    // NOLINTNEXTLINE(cppcoreguidelines-pro-type-reinterpret-cast)
    ss.write(reinterpret_cast<const char*>(octree_data.data()), static_cast<std::streamsize>(octree_data.size()));

    if (obj.binary_octree_)
    {
      local_octree->readBinary(ss);
    }
    else
    {
      // read(...) returns a new OcTree* (ownership transfer)
      local_octree.reset(dynamic_cast<octomap::OcTree*>(octomap::OcTree::read(ss)));
    }

    obj.octree_ = std::move(local_octree);
  }
  else
  {
    // Read the data to a stream which does not guarantee contiguous memory
    std::ostringstream s;
    if (obj.binary_octree_)
      obj.octree_->writeBinaryConst(s);
    else
      obj.octree_->write(s);

    // Copy stream -> byte vector (handles embedded NULs)
    const std::string& blob = s.str();
    std::vector<std::uint8_t> octree_data(blob.begin(), blob.end());

    // Serialize as an array (size is implicit)
    ar(cereal::make_nvp("octree_data", octree_data));
  }
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

}  // namespace tesseract::geometry

#endif  // TESSERACT_GEOMETRY_CEREAL_SERIALIZATION_H
