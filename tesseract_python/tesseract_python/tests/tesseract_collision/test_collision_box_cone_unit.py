import numpy as np
import numpy.testing as nptest
from tesseract import tesseract_geometry
from tesseract import tesseract_collision_bullet
from tesseract import tesseract_collision_fcl
from tesseract import tesseract_geometry
from tesseract import tesseract_common
from tesseract import tesseract_collision

def addCollisionObjects(checker):

    # Add static box to checker
    box = tesseract_geometry.Box(1,1,1)
    box_pose = np.eye(4)
    obj1_shapes = tesseract_geometry.GeometriesConst()
    obj1_shapes.append(box)
    obj1_poses = tesseract_common.VectorIsometry3d()
    obj1_poses.append(tesseract_common.Isometry3d(box_pose))

    checker.addCollisionObject("box_link", 0, obj1_shapes, obj1_poses, False)
    checker.enableCollisionObject("box_link")

    # Add thin box to checker which is disabled
    thin_box = tesseract_geometry.Box(0.1,1,1)
    thin_box_pose = np.eye(4)

    obj2_shapes = tesseract_geometry.GeometriesConst()
    obj2_shapes.append(thin_box)
    obj2_poses = tesseract_common.VectorIsometry3d()
    obj2_poses.append(tesseract_common.Isometry3d(thin_box_pose))

    checker.addCollisionObject("thin_box_link", 0, obj2_shapes, obj2_poses)
    checker.disableCollisionObject("thin_box_link")

    # Add cone to checker
    cone = tesseract_geometry.Cone(0.25, 0.25)
    cone_pose = np.eye(4)

    obj3_shapes = tesseract_geometry.GeometriesConst()
    obj3_shapes.append(cone)
    obj3_poses = tesseract_common.VectorIsometry3d()
    obj3_poses.append(tesseract_common.Isometry3d(cone_pose))

    checker.addCollisionObject("cone_link", 0, obj3_shapes, obj3_poses)

    # Add box and remove
    remove_box = tesseract_geometry.Box(0.1,1,1)
    remove_box_pose = np.eye(4)

    obj4_shapes = tesseract_geometry.GeometriesConst()
    obj4_shapes.append(remove_box)
    obj4_poses = tesseract_common.VectorIsometry3d()
    obj4_poses.append(tesseract_common.Isometry3d(remove_box_pose))

    checker.addCollisionObject("remove_box_link", 0, obj4_shapes, obj4_poses)
    assert len(checker.getCollisionObjects()) == 4
    assert checker.hasCollisionObject("remove_box_link")
    checker.removeCollisionObject("remove_box_link")
    assert not checker.hasCollisionObject("remove_box_link")

    # Try functions on a link that does not exist
    assert  not checker.removeCollisionObject("link_does_not_exist")
    assert not checker.enableCollisionObject("link_does_not_exist")
    assert not checker.disableCollisionObject("link_does_not_exist")

    # Try to add empty Collision Object
    assert not checker.addCollisionObject("empty_link",0,tesseract_geometry.GeometriesConst(),tesseract_common.VectorIsometry3d())

    # Check sizes

    assert len(checker.getCollisionObjects()) == 3
    for co in checker.getCollisionObjects():
        assert len(checker.getCollisionObjectGeometries(co)) == 1
        assert len(checker.getCollisionObjectGeometriesTransforms(co)) == 1
        tfs = checker.getCollisionObjectGeometriesTransforms(co)
        for i in range(len(tfs)):
            cgt = tfs[i]
            nptest.assert_almost_equal(cgt.matrix(), np.eye(4))


def run_test(checker):
    
    # Add collision objects
    addCollisionObjects(checker)

    # Test when object is in collision
    checker.setActiveCollisionObjects(["box_link", "cone_link"])
    checker.setCollisionMarginData(tesseract_collision.CollisionMarginData(0.1))
    nptest.assert_almost_equal(checker.getCollisionMarginData().getMaxCollisionMargin(), 0.1)

    # Set the collision object transforms
    location = tesseract_common.TransformMap()
    location["box_link"] = tesseract_common.Isometry3d(np.eye(4))
    cone_link_transform = np.eye(4)
    cone_link_transform[0][3] = 0.2
    location["cone_link"] = tesseract_common.Isometry3d(cone_link_transform)
    checker.setCollisionObjectsTransform(location)

    # Perform collision check
    result = tesseract_collision.ContactResultMap()
    checker.contactTest(result, tesseract_collision.ContactRequest(tesseract_collision.ContactTestType_CLOSEST))
    result_vector = tesseract_collision.ContactResultVector()
    tesseract_collision.flattenResults(result,result_vector)

    assert len(result_vector) > 0
    nptest.assert_almost_equal(result_vector[0].distance, -0.55)
    
    idx = [0,1,1]
    if result_vector[0].link_names[0] != "box_link":
        idx = [1,0,-1]

    if result_vector[0].single_contact_point:
        nptest.assert_almost_equal(result_vector[0].nearest_points[0][0], result_vector[0].nearest_points[1][0])
        # TODO: more checks
    else:
        nptest.assert_almost_equal(result_vector[0].nearest_points[idx[0]][0], 0.5)
        # TODO: more checks

    nptest.assert_almost_equal(result_vector[0].normal[0], idx[2]*1.0 )
    # TODO: more checks

    # Further C++ code not relevant to testing Python wrappers

def test_bullet_discrete_simple():
    checker = tesseract_collision_bullet.BulletDiscreteSimpleManager()
    run_test(checker)

def test_bullet_discrete_bvh():
    checker = tesseract_collision_bullet.BulletDiscreteBVHManager()
    run_test(checker)

def test_fcl_discrete_bvh():
    checker = tesseract_collision_fcl.FCLDiscreteBVHManager()
    run_test(checker)