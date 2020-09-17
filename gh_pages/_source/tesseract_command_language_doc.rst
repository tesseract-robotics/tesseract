Tesseract Command Language
==========================

This is a high level programming language used throughout the Tesseract Planning Framework. The goal is to provide an abstraction between motion commands and motion planner specific configurations similar to programming on an industrial teach pendant.

At the lowest level the command language is a set of instruction where on an industrial teach pendant these include move instructions, set I/O instruction and set Analog instruction. These are all included in the command language with a few additional instruction like plan instruction and environment instruction to start. These instructions are specific to motion planning and environment management.

This high level language allows a novice user to create complex programs without the knowledge of planner specifics which is designed to encode the the whole process including motion planning, motion execution, environment management, sensor signals and much more.

Overview
========

The command language begins by defining your waypoints. The current supported waypoints are CartesianWaypoint, JointWaypoint and StateWaypoint.

  - CartesianWaypoint: This is a Cartesian pose leveraged within the Plan Instruction
  - JointWaypoint: This is a joint space pose leveraged within the Plan Instruction
  - StateWaypoint: This not only includes joint names and positions but also includes velocity, acceleration, and time from start. It is primarily used in the Move Instruction but may be used in the Plan Instruction.

.. code-block:: c++

   Waypoint wp0 = JointWaypoint(fwd_kin->getJointNames(), Eigen::VectorXd::Zero(6));
   Waypoint wp1 = CartesianWaypoint(Eigen::Isometry3d::Identity() * Eigen::Translation3d(0.8, -0.3, 0.8));
   Waypoint wp2 = CartesianWaypoint(Eigen::Isometry3d::Identity() * Eigen::Translation3d(0.8, -0.2, 0.8));
   Waypoint wp3 = CartesianWaypoint(Eigen::Isometry3d::Identity() * Eigen::Translation3d(0.8, -0.1, 0.8));
   Waypoint wp4 = CartesianWaypoint(Eigen::Isometry3d::Identity() * Eigen::Translation3d(0.8, 0.0, 0.8));
   Waypoint wp5 = CartesianWaypoint(Eigen::Isometry3d::Identity() * Eigen::Translation3d(0.8, 0.1, 0.8));
   Waypoint wp6 = CartesianWaypoint(Eigen::Isometry3d::Identity() * Eigen::Translation3d(0.8, 0.2, 0.8));
   Waypoint wp7 = CartesianWaypoint(Eigen::Isometry3d::Identity() * Eigen::Translation3d(0.8, 0.3, 0.8));


After the definitions of the waypoints, the plan instructions need to be defined. As shown below the plan instruction requires three inputs, the first is the waypoint, the second is the type of motion LINEAR, FREESPACE or START and last is the profile name. The profile name corresponds to a set of planner specific parameters on the motion planning side.

.. code-block:: c++

   PlanInstruction plan_f0(wp1, PlanInstructionType::FREESPACE, "FREESPACE");
   PlanInstruction plan_c0(wp2, PlanInstructionType::LINEAR, "RASTER");
   PlanInstruction plan_c1(wp3, PlanInstructionType::LINEAR, "RASTER");
   PlanInstruction plan_c2(wp4, PlanInstructionType::LINEAR, "RASTER");
   PlanInstruction plan_c3(wp5, PlanInstructionType::LINEAR, "RASTER");
   PlanInstruction plan_c4(wp6, PlanInstructionType::LINEAR, "RASTER");
   PlanInstruction plan_c5(wp7, PlanInstructionType::LINEAR, "RASTER");

The last component is to create the program which would be sent to the motion planning server. This involves the use of a Composite Instruction which is a vector of instructions with additional properties to be set. One of those additional parameters is the ManipulatorInfo which includes information about the manipulator along with working frame and tool center point information.

.. code-block:: c++

   CompositeInstruction program("raster_program", CompositeInstructionOrder::ORDERED, ManipulatorInfo("manipulator"));

   PlanInstruction start_instruction(wp0, PlanInstructionType::START);
   program.setStartInstruction(start_instruction);

   program.push_back(plan_f0);
   program.push_back(plan_c0);
   program.push_back(plan_c1);
   program.push_back(plan_c2);
   program.push_back(plan_c3);
   program.push_back(plan_c4);
   program.push_back(plan_c5);

Now that the program has been created additional utility function will be discussed.

One useful feature of the Command Language, is it can be serialized to xml and de-serialized from xml.

.. code-block:: c++

   std::string xml_string = toXMLString(programn);
   Instruction from_xml_string = fromXMLString(xml_string);


The Tesseract Command Language is unique within Tesseract because it leverages Type Erasures instead of inheritance. This was chosen because Type Erasures obey copy semantics allowing for full use of the stl libraries.

In addition, the Instruction and Waypoint Type Erasure provide a cast and cast_const for casting the type to the erased type leveraging the getType() method.

.. code-block:: c++

   if (isComposite(from_xml_string))
   {
    const auto* const_composite = from_xml_string.cast_const<CompositeInstruction>();
    auto* composite = from_xml_string.const<CompositeInstruction>();
   }
