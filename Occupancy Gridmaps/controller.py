"""
============== UniBo: AI and Robotics 2024 ==============
Base code: dummy controller, performs the action decision parts of the main loop
To be customized in an incremental way for the different labs

(c) 2024 Alessandro Saffiotti
"""

from utilities import *
from math import pi

GOAL_POSITION = (0, 0)
ROBOT_STATE = 0
ANGLE_TOL = 0.01
LINEAR_TOL = 0.1
N_SIDES = 4
LEN_OFFSET = 1
ANGLE_OFFSET = pi/2

def init_controls (goal):
    """
    Initialize the controller, setting the global goal position
    Return True if the inizialization was successful
    """
    global GOAL_POSITION, ROBOT_STATE, ANGLE_TOL, LINEAR_TOL
    GOAL_POSITION = goal
    ROBOT_STATE = 0
    ANGLE_TOL = 0.01
    LINEAR_TOL = 0.05
    global N_SIDES, SIDELEN_OFFSET_LENGTH, ANGLE_OFFSET
    N_SIDES = 4
    LEN_OFFSET = 1
    ANGLE_OFFSET = pi/2

    return True

def rotate_to_goal(distance, vel, tollerance):

    """
    Rotate the robot towards the goal angle.
    :param distance: Angular distance of the robot to the goal
    :param vel: Angular velocity of the robot.
    :param tolerance: Tolerance for angle difference to consider the rotation complete.
    :returns: Tuple (vlin, vrot, done): Linear and angular velocity, done flag.
    """
    done = False
    math.copysign(distance, vel)
        
    if abs(distance) < tollerance:
        done = True

    return (0.0, vel, done)

def move_to_goal(distance, vel, tollerance):
    """
    Move linearily the robot towards the goal.
    :param distance: Linear distance of the robot to the goal
    :param vel: Linear velocity of the robot.
    :param tolerance: Tolerance for linear difference to consider the movement complete.
    :returns: Tuple (vlin, vrot, done): Linear and angular velocity, done flag.
    """
    done = False
        
    if distance < tollerance:
        done = True

    return (vel, 0.0, done)

def rotate_then_move(mypose, vlin_limit, vrot_limit):
    """
    Rotates then moves a robot towards a given goal position by controlling its linear and angular velocities.
    :param pose: Tuple containing the (x, y) coordinates of the start point.
    :param vlin_limit: The value assigned to vlin when moving
    :param vrot_limit: The value assigned to vrot when rotating
    :param goal: Tuple containing the (x, y) coordinates of the end point.
    :returns: Tuple containing the linear velocity (vlin), rotational velocity (vrot), and a boolean value indicating 
              whether the movement towards the goal is complete (done).
    """
    global GOAL_POSITION, ROBOT_STATE, ANGLE_TOL, LINEAR_TOL

    distance_lin = compute_linear_distance(mypose, GOAL_POSITION)
    distance_th = compute_angular_distance(mypose, GOAL_POSITION)
    
    done = False

    if ROBOT_STATE == 0:
        vlin = 0.0
        vrot = 0.0
        ROBOT_STATE = 1

    elif ROBOT_STATE == 1: # Rotate towards the goal
        (vlin, vrot, done) = rotate_to_goal(distance_th, vrot_limit, ANGLE_TOL)
        if done:
            ROBOT_STATE = 2
            done = False
        
    elif ROBOT_STATE == 2: # Move forward
        (vlin, vrot, done) = move_to_goal(distance_lin, vlin_limit, LINEAR_TOL)
        if done:
            ROBOT_STATE = 3
            done = False

    elif ROBOT_STATE == 3: # Idle
        vrot = 0.0
        vlin = 0.0
        ROBOT_STATE = 0
        done = True

    return (vlin, vrot, done)

def rotate_and_move(mypose, vlin_limit, vrot_limit):
    """
    Rotates and moves a robot towards a given goal position by controlling its linear and angular velocities.
    :param pose: Tuple containing the (x, y) coordinates of the start point.
    :param vlin_limit: The value assigned to vlin when moving
    :param vrot_limit: The value assigned to vrot when rotating
    :param goal: Tuple containing the (x, y) coordinates of the end point.
    :returns: Tuple containing the linear velocity (vlin), rotational velocity (vrot), and a boolean value indicating 
              whether the movement towards the goal is complete (done).
    """
    global GOAL_POSITION, ROBOT_STATE, ANGLE_TOL, LINEAR_TOL

    distance_lin = compute_linear_distance(mypose, GOAL_POSITION)
    distance_th = compute_angular_distance(mypose, GOAL_POSITION)
    
    done = False

    if ROBOT_STATE == 0:
        vlin = 0.0
        vrot = 0.0
        ROBOT_STATE = 1

    elif ROBOT_STATE == 1: # Rotate towards the goal, up to a window of pi/4 radian
        (vlin, vrot, done) = rotate_to_goal(distance_th, vrot_limit, pi/4)
        if done:
            ROBOT_STATE = 2
            done = False
        
    elif ROBOT_STATE == 2: # Move forward while rotating in the direction of the goal
        done_lin = False
        done_rot = False
        (_, vrot, done_rot) = rotate_to_goal(distance_th, vrot_limit*2, ANGLE_TOL)
        (vlin, _, done_lin) = move_to_goal(distance_lin, vlin_limit, LINEAR_TOL)
        if done_lin and done_rot:
            ROBOT_STATE = 3
            done = False

    elif ROBOT_STATE == 3: # Idle
        vrot = 0.0
        vlin = 0.0
        ROBOT_STATE = 0
        done = True

    return (vlin, vrot, done)

def square_trip(mypose, vlin_limit, vrot_limit):
    """
    Moves a Robot along a computed square path
    :param pose: Tuple containing the (x, y) coordinates of the start point.
    :param vlin_limit: The value assigned to vlin when moving
    :param vrot_limit: The value assigned to vrot when rotating
    :param goal: Tuple containing the (x, y) coordinates of the end point.
    :returns: Tuple containing the linear velocity (vlin), rotational velocity (vrot), and a boolean value indicating 
              whether the movement towards the goal is complete (done).
    """

    global GOAL_POSITION, ROBOT_STATE, ANGLE_TOL, LINEAR_TOL
    global N_SIDES, LEN_OFFSET, ANGLE_OFFSET
    
    done = False

    if ROBOT_STATE == 0: # Computing and resetting the global goal
        GOAL_POSITION = compute_goal(mypose, LEN_OFFSET, ANGLE_OFFSET)
        vlin = 0.0
        vrot = 0.0
        ROBOT_STATE = 1

    elif ROBOT_STATE == 1: # Rotate of SIDE_ANGLE radian
        distance_th = compute_angular_distance(mypose, GOAL_POSITION)
        (vlin, vrot, done) = rotate_to_goal(distance_th, vrot_limit, ANGLE_TOL)
        if done:
            ROBOT_STATE = 2
            done = False
        
    elif ROBOT_STATE == 2: # Move forward SIDE_LENGTH units
        distance_lin = compute_linear_distance(mypose, GOAL_POSITION)
        (vlin, vrot, done) = move_to_goal(distance_lin, vlin_limit, LINEAR_TOL)
        if done:
            ROBOT_STATE = 3
            done = False

    elif ROBOT_STATE == 3: # Loop
        vrot = 0.0
        vlin = 0.0
        ROBOT_STATE = 0
        N_SIDES = N_SIDES -1
        if N_SIDES == 0: done = True

    return (vlin, vrot, done)

def compute_ctr(mypose, mode):
    """
    Computes control values (vlin, vrot) based on the current robot's pose and the specified mode.
    :param mypose: Tuple containing the current (x, y, theta) pose of the robot, where theta is the orientation angle.
    :param mode: A string specifying the control mode. Possible values are 'CONSTANT', 'SQUARE_TRIP', 'ROTATE_THEN_MOVE', 
                 and 'ROTATE_AND_MODE'.
    :returns: Tuple containing the control values (vlin, vrot) as a pair, along with a boolean value indicating whether 
              the movement is complete (done).
    """

    distance_lin = compute_linear_distance(mypose, GOAL_POSITION)
    distance_th = compute_angular_distance(mypose, GOAL_POSITION)

    vlin = 1.0 * distance_lin
    vrot = 1.0 * distance_th
    done = False

    if mode == 'CONSTANT':
        vlin = 0.0
        vrot = 0.0
    elif mode == 'SQUARE_TRIP':
        (vlin, vrot, done) = square_trip(mypose, vlin, vrot)
    elif mode == 'ROTATE_THEN_MOVE':
        (vlin, vrot, done) = rotate_then_move(mypose, vlin, vrot)
    elif mode == 'ROTATE_AND_MODE':
        (vlin, vrot, done) = rotate_and_move(mypose, vlin, vrot)
    else:
        done = True

    return (vlin, vrot, done)
