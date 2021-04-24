^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package tesseract_environment
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.4.1 (2021-04-24)
------------------

0.4.0 (2021-04-23)
------------------
* Update tesseract_srdf to leverage nested exceptions
* Move srdf code to its own package tesseract_srdf
* Move printNestedException and leverage forward declarations for tesseract_urdf
* Do not catch exception in parseURDFString and parseURDFFile
* Contributors: Levi Armstrong

0.3.1 (2021-04-14)
------------------
* Move tesseract_variables() before any use of custom macros
* Contributors: Levi Armstrong

0.3.0 (2021-04-09)
------------------
* Only enable code coverage if compiler definition is set
* Fix issue in trajectory player setCurrentDuration not handling finished bool
* Fix bullet broadphase when new links are added
* Debug unit test
* Add cmake format
* Add support for defining collision margin data in SRDF (`#573 <https://github.com/ros-industrial-consortium/tesseract/issues/573>`_)
* Use boost targets, add cpack and license file (`#572 <https://github.com/ros-industrial-consortium/tesseract/issues/572>`_)
* Fix the way in which Eigen is included (`#570 <https://github.com/ros-industrial-consortium/tesseract/issues/570>`_)
* Add libomp-dev as test_depend to tesseract_environment and tesseract_collision
* Add multithreaded environment unit test
* Fix mutex locking bug in environment applyCommands
* Add ability to construct ROP and REP kinematic solver with different solver names
* Contributors: Hervé Audren, Levi Armstrong, Matthew Powelson

0.2.0 (2021-02-17)
------------------
* Add ability to replace link and joint pair where the link is the child link of joint
* Improve clone cache unit tests and fix issues with getting clone
* Add manipulator manager unit tests
* Add support for replacing links and joints
* Rename AddCommand to AddLinkCommand
* Update environment to leverage shared mutex
* Improve unit test coverage and registar FCL as an available contact manager
* Update StateSolver init to take a revision number
* Fix mutex dead lock in tesseract environment
* Switch addJoint, addLink, moveLink and addSceneGraph to use const&
* Improve tesseract_environment unit test coverage
* Refactor tesseract_environment to use applyCommands
* tesseract_environement: Improve documentation
* Update cmake_common_scripts to ros_industrial_cmake_boilerplate
* Move all directories in tesseract directory up one level
* Contributors: Levi Armstrong, Thomas Kostas

0.1.0 (2020-12-31)
------------------
* Add tesseract_environment package
* Create tesseract_environment and semi-isolate
* Contributors: Levi Armstrong
