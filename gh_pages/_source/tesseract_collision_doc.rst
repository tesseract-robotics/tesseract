***************************
Tesseract Collision Package
***************************

Background
==========
This package is used for performing both discrete and continuous collision checking. It understands nothing about connectivity of the object within. It purely allows for the user to add objects to the checker, set object transforms, enable/disable objects, set contact distance per object, and perform collision checks.

.. image:: ../_static/continuous_first.gif

Features
========

#. Add/Remove collision objects consisting of multiple collision shapes.
#. Enable/Disable collision objects
#. Set collision objects transformation
#. Set contact distance threshold. If two objects are further than this distance they are ignored.
#. Perform Contact Test with various exit conditions

   * Exit on first **tesseract::ContactTestType::FIRST**
   * Store only closest for each collision object **tesseract::ContactTestType::CLOSEST**
   * Store all contacts for each collision object **tesseract::ContactTestType::ALL**


Discrete Collision Checker Example
==================================

.. literalinclude:: ../../tesseract_collision/examples/box_box_example.cpp


Example Explanation
-------------------

Create Contact Checker
^^^^^^^^^^^^^^^^^^^^^^

.. literalinclude:: ../../tesseract_collision/examples/box_box_example.cpp
   :language: c++
   :start-after: // Create Collision Manager
   :end-before: // Add box to checker

There are several available contact checkers.

  * Recommended

    * BulletDiscreteBVHManager
    * BulletCastBVHManager

  * Alternative

    * BulletDiscreteSimpleManager
    * BulletCastSimpleManager

  * Beta

    * FCLDiscreteBVHManager


Add Collision Objects to Contact Checker
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^


Add collision object in a enabled state
"""""""""""""""""""""""""""""""""""""""

.. Note::

   A collision object can consist of multiple collision shape.

.. literalinclude:: ../../tesseract_collision/examples/box_box_example.cpp
   :language: c++
   :start-after: // Add box to checker
   :end-before: // Add thin box to checker which is disabled

Add collision object in a disabled state
""""""""""""""""""""""""""""""""""""""""

.. literalinclude:: ../../tesseract_collision/examples/box_box_example.cpp
   :language: c++
   :start-after: // Add thin box to checker which is disabled
   :end-before: // Add second box to checker, but convert to convex hull mesh.

Create convex hull from mesh file
"""""""""""""""""""""""""""""""""

.. literalinclude:: ../../tesseract_collision/examples/box_box_example.cpp
   :language: c++
   :start-after: documentation:start:create_convex_hull
   :end-before: documentation:end:create_convex_hull

Add convex hull collision object
""""""""""""""""""""""""""""""""
.. literalinclude:: ../../tesseract_collision/examples/box_box_example.cpp
   :language: c++
   :start-after: documentation:start:add_convex_hull_collision
   :end-before: documentation:end:add_convex_hull_collision

Set the active collision objects
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

.. literalinclude:: ../../tesseract_collision/examples/box_box_example.cpp
   :language: c++
   :start-after: documentation:start:set_active_collision_object
   :end-before: documentation:end:set_active_collision_object

Set the contact distance threshold
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

.. literalinclude:: ../../tesseract_collision/examples/box_box_example.cpp
   :language: c++
   :start-after: documentation:start:set_contact_distance_threshold
   :end-before: documentation:end:set_contact_distance_threshold

Set the collision object's transform
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

.. literalinclude:: ../../tesseract_collision/examples/box_box_example.cpp
   :language: c++
   :start-after: documentation:start:set_collision_object_transform_1
   :end-before: documentation:end:set_collision_object_transform_1

Perform collision check
^^^^^^^^^^^^^^^^^^^^^^^

.. Note::

   One object is inside another object

.. literalinclude:: ../../tesseract_collision/examples/box_box_example.cpp
   :language: c++
   :start-after: documentation:start:perform_collision_check_1
   :end-before: documentation:end:perform_collision_check_1

Set the collision object's transform
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

.. literalinclude:: ../../tesseract_collision/examples/box_box_example.cpp
   :language: c++
   :start-after: documentation:start:set_collision_object_transform_2
   :end-before: documentation:end:set_collision_object_transform_2

Perform collision check
^^^^^^^^^^^^^^^^^^^^^^^

.. Note::

   The objects are outside the contact threshold

.. literalinclude:: ../../tesseract_collision/examples/box_box_example.cpp
   :language: c++
   :start-after: documentation:start:perform_collision_check_2
   :end-before: documentation:end:perform_collision_check_2

Change contact distance threshold
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

.. literalinclude:: ../../tesseract_collision/examples/box_box_example.cpp
   :language: c++
   :start-after: // Set higher contact distance threshold
   :end-before: // Check for contact with new threshold


Perform collision check
^^^^^^^^^^^^^^^^^^^^^^^

.. Note::

   The objects are inside the contact threshold

.. literalinclude:: ../../tesseract_collision/examples/box_box_example.cpp
   :language: c++
   :start-after: // Check for contact with new threshold
   :end-before: CONSOLE_BRIDGE_logInform("Has collision: %s", toString(result_vector.empty()).c_str());
