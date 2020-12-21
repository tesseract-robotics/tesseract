from tesseract.tesseract_command_language import CompositeInstruction, StateWaypoint, Waypoint, \
    Instruction, MoveInstruction, Instructions, MoveInstructionType_START, MoveInstructionType_FREESPACE, \
    flatten
from tesseract.tesseract_time_parameterization import IterativeSplineParameterization
import numpy as np

def create_straight_trajectory():

    num = 10
    max_ = 2.0
    joint_names = ["joint_1", "joint_2", "joint_3", "joint_4", "joint_5", "joint_6"]

    program = CompositeInstruction()

    for i in range(num):
        p = np.zeros((6,),dtype=np.float64)
        p[0] = i * (max_/num)
        swp = StateWaypoint(joint_names, p)        
        if i == 0:
            program.setStartInstruction(Instruction(MoveInstruction(Waypoint(swp), MoveInstructionType_START)))
        else:
            program.append(Instruction(Instruction(MoveInstruction(Waypoint(swp), MoveInstructionType_FREESPACE))))

    p = np.zeros((6,),dtype=np.float64)
    p[0] = max_
    swp = StateWaypoint(joint_names, p)
    program.append(Instruction(Instruction(MoveInstruction(Waypoint(swp), MoveInstructionType_FREESPACE))))

    return program

def test_time_parameterization():

    time_parameterization = IterativeSplineParameterization(False)

    program = create_straight_trajectory()
    max_velocity = np.array([2.088, 2.082, 3.27, 3.6, 3.3, 3.078],dtype=np.float64)
    max_acceleration = np.array([ 1, 1, 1, 1, 1, 1],dtype=np.float64)
    assert time_parameterization.compute(program, max_velocity, max_acceleration)
    assert program[-1].cast_MoveInstruction().getWaypoint().cast_StateWaypoint().time > 1.0
    assert program[-1].cast_MoveInstruction().getWaypoint().cast_StateWaypoint().time < 5.0

def test_time_parameterization_vec():

    time_parameterization = IterativeSplineParameterization(False)

    program = flatten(create_straight_trajectory())
    max_velocity = np.array([2.088, 2.082, 3.27, 3.6, 3.3, 3.078],dtype=np.float64)
    max_acceleration = np.array([ 1, 1, 1, 1, 1, 1],dtype=np.float64)
    assert time_parameterization.compute(program, max_velocity, max_acceleration)
    assert program[-1].cast_MoveInstruction().getWaypoint().cast_StateWaypoint().time > 1.0
    assert program[-1].cast_MoveInstruction().getWaypoint().cast_StateWaypoint().time < 5.0
