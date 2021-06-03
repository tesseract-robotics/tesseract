*****************************
Tesseract Motion Planning API
*****************************

Overview
========
Tesseract contains an easy to use motion planning API that enables developers to quickly create
custom-tailored motion plans. Tesseract's interface includes plugins for several popular planning
algorithms such as OMPL, TrajOpt, TrajOpt IFOPT and Descartes.

In this tutorial, we will define each of the key components of Tesseract's motion planning API,
then we will step through a simple freespace example.

Components
==========

Environment Class
-----------------

The `Environment` class is responsible for storing all information about the environment. This
includes the robot and collision objects defined by the URDF and SRDF as well as any links/joints that
are dynamically added to the environment.

The typical workflow for defining and initializing a `Environment` object is:

- Create a `Environment` object: ::

    tesseract_environment::Environment::Ptr env = std::make_shared<tesseract_environment::Environment>();

- Initialize the `Environment` object with the URDF and SRDF files: ::

    tesseract_common::fs::path urdf_path("/path/to/urdf/my_robot.urdf");
    tesseract_common::fs::path srdf_path("/path/to/srdf/my_robot.srdf");
    env->init<tesseract_environment::OFKTStateSolver>(urdf_path, srdf_path, locator);


For more information about the `Environment` class see [TODO: add link to Tesseract Environment page].

Instruction Class
-----------------

The `Instruction` class is a base class for all instruction type classes.

PlanInstruction Class
---------------------

The `PlanInstruction` class inherits from the `Instruction` base class. This class allows the user to
specify information about a singular movement in the trajectory. A `PlanInstruction` object holds
information about the target pose and type of movement (`START`, `FREESPACE`, `LINEAR`, or `CIRCULAR`).

CompositeInstruction Class
--------------------------

The `CompositeInstruction` class inherits from the `Instruction` base class. This class allows the user
to combine multiple `Instruction` objects into one `Instruction`. A `CompositeInstruction` object holds
a list of instructions and enum indicating the type of ordering for the instuctions (`ORDERED`, `UNORDERED`,
`ORDERED_AND_REVERABLE`).

ProcessPlanningRequest Class
----------------------------

The `ProcessPlanningRequest` class allows the user to specify information about the process plan that
they would like to solve.

The typical workflow for creating and initializing `ProcessPlanningRequest` objects is:

- Create a `ProcessPlanningRequest` object: ::

    ProcessPlanningRequest request;

- Specify what type of planner to user: ::

    request.name = process_planner_names::FREESPACE_PLANNER_NAME;

- Create `Instruction` and/or `CompositeInstruction` object(s): ::

    CompositeInstruction program = freespaceExampleProgramIIWA();

- Add `Instruction` object to request: ::

    request.instructions = Instruction(program);

ProcessPlanningServer Class
---------------------------

The `ProcessPlanningServer` class is responsible for accepting `ProcessPlanningRequest` objects and returning
the `ProcessPlanningFuture` objects.

The typical workflow for creating a `ProcessPlanningServer` objcet and using it to solve a `ProcessPlanningRequest` is:

- Create a `ProcessPlanningServer` object: ::

    ProcessPlanningServer planning_server(std::make_shared<ProcessEnvironmentCache>(env), 1);

- Load default process planners: ::

    planning_server.loadDefaultProcessPlanners();

- Create a `ProcessPlanningRequest` object (see section above).

- Run the planning server and pass in the `ProcessPlanningRequest` object: ::

    ProcessPlanningFuture response = planning_server.run(request);

- Wait for the planning server to finish solving the process plan: ::

    planning_server.waitForAll();


ProcessPlanningFuture Class
---------------------------

The `ProcessPlanningFuture` class is the type that gets returned when the `ProcessPlanningServer` solves a process
plan. After calling `run()` the `ProcessPlanningServer` asynchonously solves the process plan and initializes the
`ProcessPlanningFuture` object with the process plan results upon completion.

Running the Freespace Example
=============================

* Run the executable: ::

    ./devel/bin/tesseract_process_managers_freespace_manager_example

The Full Freespace Example
==========================

.. rli:: https://raw.githubusercontent.com/ros-industrial-consortium/tesseract_planning/master/tesseract_process_managers/examples/freespace_manager_example.cpp
   :language: c++

Stepping Through the Freespace Example
======================================

Initial Setup
-------------

Define resource locator function:

.. rli:: https://raw.githubusercontent.com/ros-industrial-consortium/tesseract_planning/master/tesseract_process_managers/examples/freespace_manager_example.cpp
   :language: c++
   :lines: 20-45

Create resource locator object:

.. rli:: https://raw.githubusercontent.com/ros-industrial-consortium/tesseract_planning/master/tesseract_process_managers/examples/freespace_manager_example.cpp
   :language: c++
   :lines: 52-53

Create environment object:

.. rli:: https://raw.githubusercontent.com/ros-industrial-consortium/tesseract_planning/master/tesseract_process_managers/examples/freespace_manager_example.cpp
   :language: c++
   :lines: 54

Initialize environment with URDF and SRDF files:

.. rli:: https://raw.githubusercontent.com/ros-industrial-consortium/tesseract_planning/master/tesseract_process_managers/examples/freespace_manager_example.cpp
   :language: c++
   :lines: 55-57

Dynamically load in ignition visualizer if exists:

.. rli:: https://raw.githubusercontent.com/ros-industrial-consortium/tesseract_planning/master/tesseract_process_managers/examples/freespace_manager_example.cpp
   :language: c++
   :lines: 60-61

Visualize the environment:

.. rli:: https://raw.githubusercontent.com/ros-industrial-consortium/tesseract_planning/master/tesseract_process_managers/examples/freespace_manager_example.cpp
   :language: c++
   :lines: 63-68

Defining the Process Plan
-------------------------

Create process planning server:

.. rli:: https://raw.githubusercontent.com/ros-industrial-consortium/tesseract_planning/master/tesseract_process_managers/examples/freespace_manager_example.cpp
   :language: c++
   :lines: 71-72

Create process planning request:

.. rli:: https://raw.githubusercontent.com/ros-industrial-consortium/tesseract_planning/master/tesseract_process_managers/examples/freespace_manager_example.cpp
   :language: c++
   :lines: 75-76

Define the program:

.. rli:: https://raw.githubusercontent.com/ros-industrial-consortium/tesseract_planning/master/tesseract_process_managers/examples/freespace_manager_example.cpp
   :language: c++
   :lines: 79-80

Solving the Process Plan
------------------------

Solve the process plan:

.. rli:: https://raw.githubusercontent.com/ros-industrial-consortium/tesseract_planning/master/tesseract_process_managers/examples/freespace_manager_example.cpp
   :language: c++
   :lines: 86-87

Visualizing Results
-------------------

Plot the process trajectory:

.. rli:: https://raw.githubusercontent.com/ros-industrial-consortium/tesseract_planning/master/tesseract_process_managers/examples/freespace_manager_example.cpp
   :language: c++
   :lines: 90-94
