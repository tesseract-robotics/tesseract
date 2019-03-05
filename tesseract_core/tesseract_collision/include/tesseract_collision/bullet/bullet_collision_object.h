#ifndef TESSERACT_COLLISION_BULLET_COLLISION_OBJECT_H
#define TESSERACT_COLLISION_BULLET_COLLISION_OBJECT_H

#include <tesseract_collision/core/collision_object_base.h>
#include <tesseract_core/macros.h>
#include <tesseract_core/basic_types.h>
TESSERACT_IGNORE_WARNINGS_PUSH
#include <btBulletCollisionCommon.h>
TESSERACT_IGNORE_WARNINGS_POP
namespace tesseract_collision
{
namespace tesseract_collision_bullet
{

class BulletCollisionObject : public CollisionObjectBase
{
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  virtual BulletCollisionObject(const std::string& name) = 0;

  virtual ~BulletCollisionObject() = default;

  addBox(const Eigen::Isometry3d& pose, double x, double y, double z) override
  {
    shapes_.push_back(std::make_shared<btCollisionShape>(new btBoxShape(btVector3(x, y, z))));
    poses_.push_back(pose);
  }

  addSphere(const Eigen::Isometry3d& pose, double r) override
  {
    shapes_.push_back(std::make_shared<btCollisionShape>(new btSphereShape(static_cast<btScalar>(geom->radius))));
    poses_.push_back(pose);
  }

  virtual addCylinder(const Eigen::Isometry3d& pose, double r, double l) override
  {
    shapes_.push_back(std::make_shared<btCollisionShape>(new btCylinderShapeZ(btVector3(r, r, l))));
    poses_.push_back(pose);
  }

  virtual addCone(const Eigen::Isometry3d& pose, double r, double l) override
  {
    shapes_.push_back(std::make_shared<btCollisionShape>(new btConeShapeZ(r, l)));
    poses_.push_back(pose);
  }

  virtual addMesh(const Eigen::Isometry3d& pose, const VectorVector3d& vertices, const std::vector<int>& faces) override
  {
    btCompoundShape* compound =
        new btCompoundShape(BULLET_COMPOUND_USE_DYNAMIC_AABB, static_cast<int>(geom->triangle_count));
    compound->setMargin(BULLET_MARGIN);  // margin: compound. seems to have no
                                         // effect when positive but has an
                                         // effect when negative

//    for (unsigned i = 0; i < geom->triangle_count; ++i)
//    {
//      btVector3 v[3];
//      for (unsigned x = 0; x < 3; ++x)
//      {
//        unsigned idx = geom->triangles[3 * i + x];
//        for (unsigned y = 0; y < 3; ++y)
//        {
//          v[x][y] = static_cast<btScalar>(geom->vertices[3 * idx + y]);
//        }
//      }

//      btCollisionShape* subshape = new btTriangleShapeEx(v[0], v[1], v[2]);
//      if (subshape != nullptr)
//      {
//        cow->manage(subshape);
//        subshape->setMargin(BULLET_MARGIN);
//        btTransform geomTrans;
//        geomTrans.setIdentity();
//        compound->addChildShape(geomTrans, subshape);
//      }
//    }

    shapes_.push_back(std::make_shared<btCollisionShape>(compound));
    poses_.push_back(pose);
  }

  virtual addConvexMesh(const Eigen::Isometry3d& pose, const VectorVector3d& vertices, const std::vector<int>& faces) override
  {
    btConvexHullShape* shape = new btConvexHullShape();
    for (const auto& v : vertices)
      shape->addPoint(btVector3(static_cast<btScalar>(v[0]), static_cast<btScalar>(v[1]), static_cast<btScalar>(v[2])));

    shapes_.push_back(std::make_shared<btCollisionShape>(shape));
    poses_.push_back(pose);

  }

  virtual addPlane(const Eigen::Isometry3d& pose, double a, double b, double c, double d) override
  {
    shapes_.push_back(std::make_shared<btCollisionShape>());
    poses_.push_back(pose);
  }

  virtual addOctree(const Eigen::Isometry3d& pose, const std::shared_ptr<const octomap::OcTree>& t) override
  {
    shapes_.push_back(std::make_shared<btCollisionShape>());
    poses_.push_back(pose);
  }

  virtual addSDF(const Eigen::Isometry3d& pose) override
  {
    shapes_.push_back(std::make_shared<btCollisionShape>());
    poses_.push_back(pose);
  }
private:
  std::vector<std::make_shared<btCollisionShape>> shapes_;
  VectorIsometry3d poses_,

}
typedef std::shared_ptr<BulletCollisionObject> BulletCollisionObjectPtr;
typedef std::shared_ptr<const BulletCollisionObject> BulletCollisionObjectConstPtr;

}
}
#endif // TESSERACT_COLLISION_BULLET_COLLISION_OBJECT_H
