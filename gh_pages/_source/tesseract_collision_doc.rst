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
   :lines: 22

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
   :lines: 25-34

Add collision object in a disabled state
""""""""""""""""""""""""""""""""""""""""

.. literalinclude:: ../../tesseract/tesseract_collision/examples/box_box_example.cpp
   :language: c++
   :lines: 37-46

Add another collision object
""""""""""""""""""""""""""""

.. literalinclude:: ../../tesseract/tesseract_collision/examples/box_box_example.cpp
   :language: c++
   :lines: 49-69

Set the active collision object's
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

.. literalinclude:: ../../tesseract/tesseract_collision/examples/box_box_example.cpp
   :language: c++
   :lines: 72

Set the contact distance threshold
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

.. literalinclude:: ../../tesseract/tesseract_collision/examples/box_box_example.cpp
   :language: c++
   :lines: 73

Set the collision object's transform
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

.. literalinclude:: ../../tesseract/tesseract_collision/examples/box_box_example.cpp
   :language: c++
   :lines: 76-82

Perform collision check
^^^^^^^^^^^^^^^^^^^^^^^

.. Note::

   One object is inside another object

.. literalinclude:: ../../tesseract/tesseract_collision/examples/box_box_example.cpp
   :language: c++
   :lines: 85-89

Set the collision object's transform
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

.. literalinclude:: ../../tesseract/tesseract_collision/examples/box_box_example.cpp
   :language: c++
   :lines: 98,102

Perform collision check
^^^^^^^^^^^^^^^^^^^^^^^

.. Note::

   The objects are outside the contact threshold

.. literalinclude:: ../../tesseract/tesseract_collision/examples/box_box_example.cpp
   :language: c++
   :lines: 103

Change contact distance threshold
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

.. literalinclude:: ../../tesseract/tesseract_collision/examples/box_box_example.cpp
   :language: c++
   :lines: 112

Perform collision check
^^^^^^^^^^^^^^^^^^^^^^^

.. Note::

   The objects are inside the contact threshold

.. literalinclude:: ../../tesseract/tesseract_collision/examples/box_box_example.cpp
   :language: c++
   :lines: 113
