**************************
Tesseract Collison Package
**************************

Background
==========
This package is a used for performing both discrete and continuous collision checking. It understands nothing about conectivity of the object within. It purely allows for the user to add objects to the checker, set object transforms, enable/disable objects and perform collision checks.

.. image:: ../_static/continuous_first.gif

Features
========

#. Add/Remove collison objects consisting of multiple collision shapes.
#. Enable/Disable collision objects
#. Set collision objects transformation
#. Set contact distance threshold. If two objects are further than this distance they are ignored.
#. Perform Contact Test with various exit conditions

   * Exit on first **tesseract::ContactTestType::FIRST**
   * Store only closets for each collision object **tesseract::ContactTestType::CLOSEST**
   * Store all contacts for each colliison object **tesseract::ContactTestType::ALL**


Discrete Collision Checker Example
==================================

.. code-block:: c++

   // Create Bullet Discrete BVH Manager
   tesseract::tesseract_bullet::BulletDiscreteBVHManager checker;

   // Add box to checker
   shapes::ShapePtr box(new shapes::Box(1, 1, 1));
   Eigen::Isometry3d box_pose;
   box_pose.setIdentity();

   std::vector<shapes::ShapeConstPtr> obj1_shapes;
   tesseract::VectorIsometry3d obj1_poses;
   tesseract::CollisionObjectTypeVector obj1_types;
   obj1_shapes.push_back(box);
   obj1_poses.push_back(box_pose);
   obj1_types.push_back(tesseract::CollisionObjectType::UseShapeType);

   checker.addCollisionObject("box_link", 0, obj1_shapes, obj1_poses, obj1_types);

   // Add thin box to checker which is disabled
   shapes::ShapePtr thin_box(new shapes::Box(0.1, 1, 1));
   Eigen::Isometry3d thin_box_pose;
   thin_box_pose.setIdentity();

   std::vector<shapes::ShapeConstPtr> obj2_shapes;
   tesseract::VectorIsometry3d obj2_poses;
   tesseract::CollisionObjectTypeVector obj2_types;
   obj2_shapes.push_back(thin_box);
   obj2_poses.push_back(thin_box_pose);
   obj2_types.push_back(tesseract::CollisionObjectType::UseShapeType);

   checker.addCollisionObject("thin_box_link", 0, obj2_shapes, obj2_poses, obj2_types, false);

   // Add second box to checker.
   shapes::ShapePtr second_box(new shapes::Box(2, 2, 2));

   Eigen::Isometry3d second_box_pose;
   second_box_pose.setIdentity();

   std::vector<shapes::ShapeConstPtr> obj3_shapes;
   tesseract::VectorIsometry3d obj3_poses;
   tesseract::CollisionObjectTypeVector obj3_types;
   obj3_shapes.push_back(second_box);
   obj3_poses.push_back(second_box_pose);
   obj3_types.push_back(tesseract::CollisionObjectType::UseShapeType);

   checker.addCollisionObject("second_box_link", 0, obj3_shapes, obj3_poses, obj3_types);

   // Test when object is inside another
   checker.setActiveCollisionObjects({"box_link", "second_box_link"});
   checker.setContactDistanceThreshold(0.1);

   // Set the collision object transforms
   tesseract::TransformMap location;
   location["box_link"] = Eigen::Isometry3d::Identity();
   location["box_link"].translation()(0) = 0.2;
   location["box_link"].translation()(1) = 0.1;
   location["second_box_link"] = Eigen::Isometry3d::Identity();

   checker.setCollisionObjectsTransform(location);

   // Perform collision check
   tesseract::ContactResultMap result;
   checker.contactTest(result, tesseract::ContactTestType::CLOSEST);

   tesseract::ContactResultVector result_vector;
   tesseract::moveContactResultsMapToContactResultsVector(result, result_vector);

   // Test object is out side the contact distance
   location["box_link"].translation() = Eigen::Vector3d(1.60, 0, 0);
   result.clear();
   result_vector.clear();

   checker.setCollisionObjectsTransform(location);
   checker.contactTest(result, tesseract::ContactTestType::CLOSEST);
   tesseract::moveContactResultsMapToContactResultsVector(result, result_vector);

   // Test object inside the contact distance
   result.clear();
   result_vector.clear();

   checker.setContactDistanceThreshold(0.25);
   checker.contactTest(result, tesseract::ContactTestType::CLOSEST);
   tesseract::moveContactResultsMapToContactResultsVector(result, result_vector);


Example Explanation
-------------------

Create Contact Checker
^^^^^^^^^^^^^^^^^^^^^^

There are several available contact checkers.

  * Recommended

    * BulletDiscreteBVHManager
    * BulletCastBVHManager

  * Alternative

    * BulletDiscreteSimpleManager
    * BulletCastSimpleManager

  * Beta

    * FCLDiscreteBVHManager


.. code-block:: c++

   tesseract::tesseract_bullet::BulletDiscreteBVHManager checker;


Add Collision Objects to Contact Checker
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^


Add collision object in a enabled state
"""""""""""""""""""""""""""""""""""""""

.. Note::

   A collision object can consist of multiple collision shape.



.. code-block:: c++

   shapes::ShapePtr box(new shapes::Box(1, 1, 1));
   Eigen::Isometry3d box_pose;
   box_pose.setIdentity();

   std::vector<shapes::ShapeConstPtr> obj1_shapes;
   tesseract::VectorIsometry3d obj1_poses;
   tesseract::CollisionObjectTypeVector obj1_types;
   obj1_shapes.push_back(box);
   obj1_poses.push_back(box_pose);
   obj1_types.push_back(tesseract::CollisionObjectType::UseShapeType);

   checker.addCollisionObject("box_link", 0, obj1_shapes, obj1_poses, obj1_types);

Add collision object in a disabled state
""""""""""""""""""""""""""""""""""""""""

.. code-block:: c++

   shapes::ShapePtr thin_box(new shapes::Box(0.1, 1, 1));
   Eigen::Isometry3d thin_box_pose;
   thin_box_pose.setIdentity();

   std::vector<shapes::ShapeConstPtr> obj2_shapes;
   tesseract::VectorIsometry3d obj2_poses;
   tesseract::CollisionObjectTypeVector obj2_types;
   obj2_shapes.push_back(thin_box);
   obj2_poses.push_back(thin_box_pose);
   obj2_types.push_back(tesseract::CollisionObjectType::UseShapeType);

   checker.addCollisionObject("thin_box_link", 0, obj2_shapes, obj2_poses, obj2_types, false);

Add another collision object
""""""""""""""""""""""""""""

.. code-block:: c++

   shapes::ShapePtr second_box(new shapes::Box(2, 2, 2));

   Eigen::Isometry3d second_box_pose;
   second_box_pose.setIdentity();

   std::vector<shapes::ShapeConstPtr> obj3_shapes;
   tesseract::VectorIsometry3d obj3_poses;
   tesseract::CollisionObjectTypeVector obj3_types;
   obj3_shapes.push_back(second_box);
   obj3_poses.push_back(second_box_pose);
   obj3_types.push_back(tesseract::CollisionObjectType::UseShapeType);

   checker.addCollisionObject("second_box_link", 0, obj3_shapes, obj3_poses, obj3_types);

Set the collision object's transform
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

.. code-block:: c++

   tesseract::TransformMap location;
   location["box_link"] = Eigen::Isometry3d::Identity();
   location["box_link"].translation()(0) = 0.2;
   location["box_link"].translation()(1) = 0.1;
   location["second_box_link"] = Eigen::Isometry3d::Identity();

   checker.setCollisionObjectsTransform(location);

Perform collision check
^^^^^^^^^^^^^^^^^^^^^^^

.. code-block:: c++

   tesseract::ContactResultMap result;
   checker.contactTest(result, tesseract::ContactTestType::CLOSEST);

   tesseract::ContactResultVector result_vector;
   tesseract::moveContactResultsMapToContactResultsVector(result, result_vector);
