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
   :language: c++

You can find this example `here <https://github.com/ros-industrial-consortium/tesseract/blob/master/tesseract_collision/examples/box_box_example.cpp>`_.


Example Explanation
-------------------

Create Contact Checker
^^^^^^^^^^^^^^^^^^^^^^

.. literalinclude:: ../../tesseract_collision/examples/box_box_example.cpp
   :language: c++
   :start-after: // documentation:start:1:
   :end-before: // documentation:end:1:

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

Add a collision object in a enabled state
"""""""""""""""""""""""""""""""""""""""

.. literalinclude:: ../../tesseract_collision/examples/box_box_example.cpp
   :language: c++
   :start-after: // documentation:start:2:
   :end-before: // documentation:end:2:

.. Note::
   A collision object can consist of multiple collision shapes.

Add collision object in a disabled state
""""""""""""""""""""""""""""""""""""""""

.. literalinclude:: ../../tesseract_collision/examples/box_box_example.cpp
   :language: c++
   :start-after: // documentation:start:3:
   :end-before: // documentation:end:3:

Create convex hull from mesh file
"""""""""""""""""""""""""""""""""

.. literalinclude:: ../../tesseract_collision/examples/box_box_example.cpp
   :language: c++
   :start-after: documentation:start:4:
   :end-before: documentation:end:4:

Add convex hull collision object
""""""""""""""""""""""""""""""""
.. literalinclude:: ../../tesseract_collision/examples/box_box_example.cpp
   :language: c++
   :start-after: documentation:start:5:
   :end-before: documentation:end:5:

Set the active collision objects
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

.. literalinclude:: ../../tesseract_collision/examples/box_box_example.cpp
   :language: c++
   :start-after: documentation:start:6:
   :end-before: documentation:end:6:

Set the contact distance threshold
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

.. literalinclude:: ../../tesseract_collision/examples/box_box_example.cpp
   :language: c++
   :start-after: documentation:start:7:
   :end-before: documentation:end:7:

Set the collision object's transform
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

.. literalinclude:: ../../tesseract_collision/examples/box_box_example.cpp
   :language: c++
   :start-after: documentation:start:8:
   :end-before: documentation:end:8:

Perform collision check
^^^^^^^^^^^^^^^^^^^^^^^

.. Note::

   One object is inside another object

.. literalinclude:: ../../tesseract_collision/examples/box_box_example.cpp
   :language: c++
   :start-after: documentation:start:9:
   :end-before: documentation:end:9:

Set the collision object's transform
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

.. literalinclude:: ../../tesseract_collision/examples/box_box_example.cpp
   :language: c++
   :start-after: documentation:start:10:
   :end-before: documentation:end:10:

Perform collision check
^^^^^^^^^^^^^^^^^^^^^^^

.. Note::

   The objects are outside the contact threshold

.. literalinclude:: ../../tesseract_collision/examples/box_box_example.cpp
   :language: c++
   :start-after: documentation:start:11:
   :end-before: documentation:end:11:

Change contact distance threshold
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

.. literalinclude:: ../../tesseract_collision/examples/box_box_example.cpp
   :language: c++
   :start-after: // documentation:start:12:
   :end-before: // documentation:end:12:


Perform collision check
^^^^^^^^^^^^^^^^^^^^^^^

.. Note::

   The objects are inside the contact threshold

.. literalinclude:: ../../tesseract_collision/examples/box_box_example.cpp
   :language: c++
   :start-after: // documentation:start:13:
   :end-before: // documentation:end:13:
