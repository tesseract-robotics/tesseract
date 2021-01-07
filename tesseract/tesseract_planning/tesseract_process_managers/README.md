# Tesseract Process Managers

This package contains process managers for Tesseract. Examples include

* Freespace planning manager
* Raster strip planning manager

## Overview
### TaskInput
This package uses [Taskflow](https://github.com/taskflow/taskflow) to organize and perform robotics tasks. Each of the tasks in the system will take a TaskInput which is just a Tesseract::ConstPtr and references to instructions and the seed.

Each Task will operate on the instructions and the results will get stored in the seed. The TaskInput does not own the instructions in order to keep it lightweight and able to be segmented into sub-TaskInputs (see [] operator). Therefore, the instructions and seed passed into the TaskInput must be kept alive by the user.

It is also worth noting that TaskInput is not inherently thread-safe. Since taskflow will attempt to execute as many tasks in parallel as possible, it is the responsibility of the user to ensure that Tasks that use the same portions of the TaskInput are not running at the same time. In practice, this shouldn't be difficult as many of the planning operations currently implemented have a clear order. However, you should not, for example, try to plan the same segment using two different planners at the same time without first making a copy of the inputs to the TaskInput for each Task.

### Class Structure
The package is divided into several types of classes

* Process Generators - Wrappers around other Tesseract functionality to conveniently form a Task
* Taskflow Generators - Takes several Process Generators and generates a taskflow that implements some desired process flow
* Process Managers - Manages a robotic process. These take instructions in a specific format and uses the taskflow generators given to generate the overal taskflow and execute. This is the thing that you would wrap in a ROS node to be, for example, a planning server.

## Examples
Raster path and freespace examples are given for the two managers. Note that the input instruction set is hardcoded, but this would likely be coming from something like a teach pendant or a process planner that takes a toolpath and converts it to command language with appropriate approaches, departures, and transitions
