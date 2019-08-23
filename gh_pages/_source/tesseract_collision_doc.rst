***************************
Tesseract Collision Package
***************************

Background
==========
This package is a used for performing both discrete and continuous collision checking. It understands nothing about connectivity of the object within. It purely allows for the user to add objects to the checker, set object transforms, enable/disable objects, set contact distance per objects and perform collision checks.

.. image:: ../_static/continuous_first.gif

Features
========

#. Add/Remove collision objects consisting of multiple collision shapes.
#. Enable/Disable collision objects
#. Set collision objects transformation
#. Set contact distance threshold. If two objects are further than this distance they are ignored.
#. Perform Contact Test with various exit conditions

   * Exit on first **tesseract::ContactTestType::FIRST**
   * Store only closets for each collision object **tesseract::ContactTestType::CLOSEST**
   * Store all contacts for each collision object **tesseract::ContactTestType::ALL**


Discrete Collision Checker Example
==================================

.. literalinclude:: ../../tesseract/tesseract_collision/examples/box_box_example.cpp


Example Explanation
-------------------

Create Contact Checker
^^^^^^^^^^^^^^^^^^^^^^

.. literalinclude:: ../../tesseract/tesseract_collision/examples/box_box_example.cpp
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

.. literalinclude:: ../../tesseract/tesseract_collision/examples/box_box_example.cpp
   :language: c++
   :start-after: // Add box to checker
   :end-before: // Add thin box to checker which is disabled

Add collision object in a disabled state
""""""""""""""""""""""""""""""""""""""""

.. literalinclude:: ../../tesseract/tesseract_collision/examples/box_box_example.cpp
   :language: c++
   :start-after: // Add thin box to checker which is disabled
   :end-before: // Add second box to checker, but convert to convex hull mesh.

Add another collision object
""""""""""""""""""""""""""""

.. literalinclude:: ../../tesseract/tesseract_collision/examples/box_box_example.cpp
   :language: c++
   :start-after: // This is required because convex hull cannot have multiple faces on the same plane.
   :end-before: CONSOLE_BRIDGE_logInform("Test when object is inside another");

Set the active collision object's
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

.. literalinclude:: ../../tesseract/tesseract_collision/examples/box_box_example.cpp
   :language: c++
   :start-after: CONSOLE_BRIDGE_logInform("Test when object is inside another");
   :end-before: checker.setContactDistanceThreshold(0.1);

Set the contact distance threshold
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

.. literalinclude:: ../../tesseract/tesseract_collision/examples/box_box_example.cpp
   :language: c++
   :start-after: checker.setActiveCollisionObjects({ "box_link", "second_box_link" });
   :end-before: // Set the collision object transforms

Set the collision object's transform
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

.. literalinclude:: ../../tesseract/tesseract_collision/examples/box_box_example.cpp
   :language: c++
   :start-after: // Set the collision object transforms
   :end-before: // Perform collision check

Perform collision check
^^^^^^^^^^^^^^^^^^^^^^^

.. Note::

   One object is inside another object

.. literalinclude:: ../../tesseract/tesseract_collision/examples/box_box_example.cpp
   :language: c++
   :start-after: // Perform collision check
   :end-before: CONSOLE_BRIDGE_logInform("Has collision: %s", toString(result_vector.empty()).c_str());

Set the collision object's transform
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

.. literalinclude:: ../../tesseract/tesseract_collision/examples/box_box_example.cpp
   :language: c++
   :start-after: CONSOLE_BRIDGE_logInform("Test object is out side the contact distance");
   :end-before: result.clear();

.. literalinclude:: ../../tesseract/tesseract_collision/examples/box_box_example.cpp
   :language: c++
   :start-after: result_vector.clear();
   :end-before: // Check for collision

Perform collision check
^^^^^^^^^^^^^^^^^^^^^^^

.. Note::

   The objects are outside the contact threshold

.. literalinclude:: ../../tesseract/tesseract_collision/examples/box_box_example.cpp
   :language: c++
   :start-after: // Check for collision after moving object
   :end-before: CONSOLE_BRIDGE_logInform("Has collision: %s", toString(result_vector.empty()).c_str());

Change contact distance threshold
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

.. literalinclude:: ../../tesseract/tesseract_collision/examples/box_box_example.cpp
   :language: c++
   :start-after: // Set higher contact distance threshold
   :end-before: // Check for contact with new threshold


Perform collision check
^^^^^^^^^^^^^^^^^^^^^^^

.. Note::

   The objects are inside the contact threshold

.. literalinclude:: ../../tesseract/tesseract_collision/examples/box_box_example.cpp
   :language: c++
   :start-after: // Check for contact with new threshold
   :end-before: CONSOLE_BRIDGE_logInform("Has collision: %s", toString(result_vector.empty()).c_str());
