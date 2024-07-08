import math
import matplotlib.pyplot as plt

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
