import math
import matplotlib.pyplot as plt
import pickle
import robot_gwy, robot_map

def quaternion_to_euler(q):
    """
    Convert quaternion to Euler angles (roll, pitch, yaw).
    :param q: Quaternion in w, x, y, z format.
    :return: Euler angles in radians (roll, pitch, yaw).
    """
    # Extract quaternion components
    w, x, y, z = q

    # Roll (x-axis rotation)
    sinr_cosp = 2.0 * (w * x + y * z)
    cosr_cosp = 1.0 - 2.0 * (x * x + y * y)
    roll = math.atan2(sinr_cosp, cosr_cosp)

    # Pitch (y-axis rotation)
    sinp = 2.0 * (w * y - z * x)
    if abs(sinp) >= 1:
        pitch = math.copysign(pi / 2, sinp)  # Use +/-90 degrees if out of range
    else:
        pitch = math.asin(sinp)

    # Yaw (z-axis rotation)
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    yaw = math.atan2(siny_cosp, cosy_cosp)

    return roll, pitch, yaw

def normalize_angle_deg(angle):
    """
    Normalize an angle to be within the range -180 to +180 degrees.
    :param angle: Input angle in degrees.
    :return: Normalized angle in degrees.
    """
    return (angle + 180) % 360 - 180

def normalize_angle_rad(angle):
    """
    Normalize an angle to be within the range -pi to +pi radians.
    :param angle: Input angle in radians.
    :return: Normalized angle in radians.
    """
    return  (angle + math.pi) % (2 * math.pi) - math.pi

def compute_linear_distance(pose, goal):
    """
    Compute the Euclidean distance between two points.
    :param pose: Tuple containing the (x, y) coordinates of the start point.
    :param goal: Tuple containing the (x, y) coordinates of the end point.
    :return: Euclidean distance between the start and end points.
    """
    dx = goal[0] - pose[0]
    dy = goal[1] - pose[1]

    distance = math.sqrt(dx**2 + dy**2)
    
    return distance

def compute_angular_distance(pose, goal):
    """
    Compute the angular distance (in radians) from start to end coordinates.
    :param pose: Tuple containing the (x, y, th) coordinates of the start point.
    :param goal: Tuple containing the (x, y) coordinates of the end point.
    :return: Angular distance (in radians) from start to end.
    """

    th_rad = normalize_angle_rad(pose[2])
    dx = goal[0] - pose[0]
    dy = goal[1] - pose[1]

    angle_rad = math.atan2(dy, dx)
    distance = normalize_angle_rad(angle_rad - th_rad)

    return distance

def compute_goal(pose, distance, dth):
    """
    Computes the goal position based on the current robot pose, a specified distance, and angular deviation.
    :param pose: Tuple containing the current (x, y, th) pose of the robot, where th is the orientation angle.
    :param distance: The distance from the current position to the goal position.
    :param dth: The angular deviation from the current orientation to the desired orientation.
    :returns: Tuple containing the (x, y) coordinates of the goal position relative to the current pose.
    """
    th = normalize_angle_rad(pose[2])

    dx = distance * math.cos(th + dth)
    dy = distance * math.sin(th + dth)
    
    x2 = pose[0] + dx
    y2 = pose[1] + dy
    
    return x2, y2


def save_trajectory(trajectory, computed_pose, real_pose):
    """
    Update and save the trajectory of the robot during its movement.
    :param trajectory: A list containing the trajectory of the robot. It should have the following structure:
    - Index 0: List of computed x positions
    - Index 1: List of computed y positions
    - Index 2: List of real x positions
    - Index 3: List of real y positions
    :param computed_pose: A tuple containing the computed (estimated) pose of the robot.
    :param real_pose: A tuple containing the real (actual) pose of the robot.
    :return: A list containing the updated trajectory of the robot.
    """
    if len(trajectory) == 0:
        trajectory = [[],[],[],[]]

    trajectory[0].append(computed_pose[0])
    trajectory[1].append(computed_pose[1])
    trajectory[2].append(real_pose[0])
    trajectory[3].append(real_pose[1])

    return trajectory

def plot_trajectory(trajectory):
    """
    Plot the computed and real trajectories on the same graph.
    Parameters:
    :param trajectory: A list containing the trajectory of the robot. It should have the following structure:
    - Index 0: List of computed x positions
    - Index 1: List of computed y positions
    - Index 2: List of real x positions
    - Index 3: List of real y positions
    """

    plt.plot(trajectory[0], trajectory[1], label='Computed Trajectory', color='blue')
    plt.plot(trajectory[2], trajectory[3], label='Real Trajectory', color='red')

    plt.xlabel('X Position')
    plt.ylabel('Y Position')
    plt.title('Robot Trajectory')
    plt.legend()

    plt.grid(True)
    plt.show()

def plot_pos_error(error_list):
    """
    Plots a list of the robot position error as a line graph.

    Parameters:
    error_list (list): A list of float values to be plotted.

    Returns:
    None
    """
    plt.figure(figsize=(10, 5))
    plt.plot(error_list, linestyle='-', color='b')
    plt.title('Line Plot of Pos Error Values')
    plt.xlabel('Index')
    plt.ylabel('Value')
    plt.grid(True)
    plt.show()

def save_data_to_file(data, filename):
    """
    Save a list of data to a file using pickle serialization.
    :param data: The list of data to be saved.
    :param filename: The name of the file to save the data to.
    """
    with open(filename, 'wb') as f:
        pickle.dump(data, f)
    print(f"List saved to {filename}.")

def load_data_from_file(filename):
    """
    Load a list of data from a file using pickle deserialization.
    :param filename: The name of the file to load the data from.
    :return: The loaded list of data, or None if the file is not found.
    """
    try:
        with open(filename, 'rb') as f:
            data = pickle.load(f)
        print(f"List loaded from {filename}.")
        return data
    except FileNotFoundError:
        print(f"File '{filename}' not found.")
        return None

def convert_box_pos(box, th, r):
    """
    Convert the box position to the required format.
    :param box: Tuple containing the x and y coordinates of the box.
    :param th: Orientation angle (in radians).
    :param r: Radius value.
    :return: A tuple containing the x, y coordinates, orientation angle, and radius of the box.
    """
    box_location = (float(box[0]), float(box[1]), th, r)
    return box_location

def box_postion_state_update(state, box):
    """
    Given the box name, get the box position and update the state accordingly.
    :param state: The current state of the system.
    :param box: The name of the box.
    :return: The updated state of the system with the new box position.
    """
    state.pos[box] = convert_box_pos(robot_gwy.get_box_position(box), 0.0, 0.6)     
    state.room[box] = robot_map.map.find_room(state.pos[box])
    robot_map.map.locations[box] = state.pos[box]

    return state

def realtime_box_pos_check(box, mypose):
    """
    Check the real-time position of a box and update if necessary.
    :param box: The name of the box.
    :param mypose: The current pose of the robot.
    :return: A tuple containing the position error, the room error status and the distance between the robot and the box.
    """
    box_pos = convert_box_pos(robot_gwy.get_box_position(box), 0.0, 0.6)
    box_room = robot_map.map.find_room(box_pos)
    my_room = robot_map.map.find_room(mypose)
    pos_error = compute_linear_distance(robot_map.map.locations[box], box_pos)
    room_error = my_room != box_room
    box_robot_dist = compute_linear_distance(mypose, box_pos)

    if pos_error: 
        robot_map.map.locations[box] = box_pos

    return pos_error, room_error, box_robot_dist

def remove_door (door):
    """
    Remove a door from the map topology.
    :param door: The name of the door to be removed.
    """
    for room in robot_map.map.topology:
        for obj in robot_map.map.topology[room]:
            robot_map.map.topology[room] = [i for i in robot_map.map.topology[room] if i != door]

def compute_approach_points(doorpose):
    """
    Compute the approach points for a door based on its position.
    :param doorpose: A tuple containing the door's x, y coordinates, orientation angle, and radius.
    :return: A tuple containing two approach points (left and right) relative to the door.
    """
    door_x, door_y, door_th, radius = doorpose
    radius = 2*radius
    door_th = door_th % (2 * math.pi)

    approach_x_left = door_x + (radius * math.cos(door_th))
    approach_y_left = door_y + (radius * math.sin(door_th))
    
    approach_x_right = door_x + (-radius * math.cos(door_th))
    approach_y_right = door_y + (-radius * math.sin(door_th))
    
    return((approach_x_left, approach_y_left), (approach_x_right, approach_y_right))

def find_approach_point (mypose, doorpose):
    """
    Find the appropriate approach point for a door based on the current robot position.
    :param mypose: A tuple containing the current x, y coordinates, and orientation angle of the robot.
    :param doorpose: A tuple containing the door's x, y coordinates, orientation angle, and radius.
    :return: A tuple containing the x, y coordinates, orientation angle, and radius of the chosen approach point, or False if an error occurs.
    """
    robot_x, robot_y, _ = mypose
    approach_left, approach_right = compute_approach_points(doorpose)
    if robot_map.map.find_room(approach_left) == robot_map.map.find_room((robot_x, robot_y)):
        return(approach_left[0], approach_left[1], 0.0, 0.2)
    elif robot_map.map.find_room(approach_right) == robot_map.map.find_room((robot_x, robot_y)):
        return(approach_right[0], approach_right[1], 0.0, 0.2)
    else:
        print('Approach point error')
        return False

def pos_calibration (pos_list, doorpose):
    """
    Calibrate the position of the robot based on a list of positions and a door pose.
    :param pos_list: A list of tuples containing the x, y coordinates of observed positions.
    :param doorpose: A tuple containing the door's x, y coordinates, orientation angle, and radius.
    :return: A tuple containing the delta x and delta y adjustments for the robot position.
    """
    sum_x = 0
    sum_y = 0
    for pos in pos_list:
        sum_x += pos[0]
        sum_y += pos[1]

    num_positions = len(pos_list)
    average_x = sum_x / num_positions
    average_y = sum_y / num_positions
    dx = average_x - float(doorpose[0])
    dy = average_y - float(doorpose[1])
    return (dx, dy)

def pos_adjustment(mypose, dx, dy):
    """
    Adjust the position of the robot based on given deltas.
    :param mypose: A tuple containing the current x, y coordinates, and orientation angle of the robot.
    :param dx: The delta x adjustment.
    :param dy: The delta y adjustment.
    :return: A tuple containing the adjusted x, y coordinates, and orientation angle of the robot.
    """
    x, y, th = mypose
    x = x - dx
    y = y - dy
    return (x, y, th)