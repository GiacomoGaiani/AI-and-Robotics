"""
============== UniBo: AI and Robotics 2024 ==============
Base code: gateway to the robot (or simulator)
This is a dummy version, most values and funtions are place-holders
It needs to be customized by the students for the different labs

(c) 2024 Alessandro Saffiotti
"""
import rospy
import math
import threading
from sensor_msgs.msg import JointState, Range
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from environment_pkg.srv import BoxPos, DoorOpen, BoxPickUp, BoxPutDown, BoxUpdatePos
from utilities import quaternion_to_euler
import robot_map


def sonar_ring_pose(bearing):
    rho = 0.260
    phi = math.radians(bearing)
    x = rho * math.cos(phi)
    y = rho * math.sin(phi)
    return (x, y, phi)

parameters = {                        # these are for the Tiago robot
    'wheel_radius' : 0.0985,
    'wheel_axis' : 0.4044,
    'sonar_num' : 12,
    'sonar_maxrange' : 3.0,
    'x_init_pos' : 0.0,
    'y_init_pos' : 0.0,
    'th_init_pos' : 0.0,
    'wl_init_pos' : 0.0,
    'wr_init_pos' : 0.0,
    'sonar_delta' : math.radians(25.0),
    'sonar_poses' : [sonar_ring_pose(0.0),
                     sonar_ring_pose(30.0),
                     sonar_ring_pose(60.0),
                     sonar_ring_pose(90.0),
                     sonar_ring_pose(120.0),
                     sonar_ring_pose(150.0),
                     sonar_ring_pose(180.0),
                     sonar_ring_pose(210.0),
                     sonar_ring_pose(240.0),
                     sonar_ring_pose(270.0),
                     sonar_ring_pose(300.0),
                     sonar_ring_pose(330.0)],
    'sonar_topics' : []                  
}

vlin = 0
vrot = 0
_stop_thread = threading.Event()
velocity_publisher = None

def init_robot(simulation = True):
    """
    Initializes the robot, setting up parameters based on whether it is running in simulation or not.
    Calls either `init_robot_sim` or `init_robot_no_sim` based on the `simulation` parameter.
    """

    if simulation:
        parameters = init_robot_sim()
    else:
        parameters = init_robot_no_sim()
    
    return parameters

def init_robot_sim ():
    """
    This should set up the communication channels with the robot (or simulator)
    and perform any initialization needed
    """
    def pose_callback(msg):
        position = msg.pose.pose.position
        orientation = msg.pose.pose.orientation

        orientation = orientation.x, orientation.y, orientation.z, orientation.w

        roll, pitch, yaw = quaternion_to_euler(orientation)

        parameters['x_init_pos'] = position.x
        parameters['y_init_pos'] = position.y
        parameters['th_init_pos'] = roll

    rospy.init_node('robot_node', anonymous=True)
    rospy.Subscriber('/ground_truth_odom', Odometry, pose_callback)
    parameters['wl_init_pos'],  parameters['wr_init_pos'] = get_wheel_encoders()

    global velocity_publisher
    velocity_publisher = rospy.Publisher('/mobile_base_controller/cmd_vel', Twist, queue_size=10)
    vel_thread = threading.Thread(target=publish_velocity)
    vel_thread.start()

    for i in range(parameters['sonar_num']):
        if i < 9 : idx = '0'+str(i+1)
        else : idx = str(i+1)
        parameters['sonar_topics'].append('/sonar' + idx + '_base')

    initialize_doors()

    print("Robot initialized")
    return parameters

def init_robot_no_sim():
    """
    Perrform the inizialization of the parameters outside the robot simulaion
    """
    for i in range(parameters['sonar_num']):
        if i < 9 : idx = '0'+str(i+1)
        else : idx = str(i+1)
        parameters['sonar_topics'].append('/sonar' + idx + '_base')

    print("Robot initialized")
    return parameters


def shutdown_robot ():
    """
    This should perform any finalization needed on the robot,
    and close the communication channels
    """
    stop_thread()
    print("Robot shut")


def get_wheel_encoders():
    """
    Get current values of wheel encoders, which indicate the current position
    of each wheel in radians
    """

    rospy.init_node('robot_node', anonymous=True)
    encoder_values = None

    def callback(msg):
        nonlocal encoder_values
        encoder_values = (msg.position[12], msg.position[13])

    subscriber = rospy.Subscriber("/joint_states", JointState, callback)
    rospy.wait_for_message("/joint_states", JointState)

    return encoder_values

def get_real_pos ():
    """
    Retrieves the current real position of the robot based on ground truth odometry.
    Returns a list [x, y, theta] representing the robot's position (x, y) and orientation theta in radians.
    """

    rospy.init_node('robot_node', anonymous=True)
    realpose = None

    def realpose_callback(msg):
        position = msg.pose.pose.position
        orientation = msg.pose.pose.orientation

        orientation = orientation.x, orientation.y, orientation.z, orientation.w

        roll, pitch, yaw = quaternion_to_euler(orientation)

        nonlocal realpose
        realpose = [position.x, position.y, roll]

    rospy.Subscriber('/ground_truth_odom', Odometry, realpose_callback)
    rospy.wait_for_message('/ground_truth_odom', Odometry)
    

    return realpose


def get_sonar_data ():
    """
    Get current values from the sonar ring
    Returns an array of readings in the form (sonar pose, range)
    Sonar pose is (x, y, th) in robot's base frame
    """

    res = {}
    rospy.init_node('robot_node', anonymous=True)

    threads = []

    for i in range(parameters['sonar_num']):
        thread = threading.Thread(target=subscribe_sonar, args=(i, res))
        thread.start()
        threads.append(thread)

    for thread in threads:
        thread.join()

    return res

def get_front_sonar_data ():
    """
    Get current values from the five sonar sensors facing forward
    Returns an array of readings in the form (sonar pose, range)
    Sonar pose is (x, y, th) in robot's base frame
    """
    res = {}
    rospy.init_node('robot_node', anonymous=True)

    threads = []

    for i in [0,1,2,10,11]:
        thread = threading.Thread(target=subscribe_sonar, args=(i, res))
        thread.start()
        threads.append(thread)

    for thread in threads:
        thread.join()

    return res

def get_lateral_sonar_data ():
    """
    Get current values from the seven sonar sensors facing forward and on the side
    Returns an array of readings in the form (sonar pose, range)
    Sonar pose is (x, y, th) in robot's base frame
    """
    res = {}
    rospy.init_node('robot_node', anonymous=True)

    threads = []

    for i in [0,1,2,3,9,10,11]:
        thread = threading.Thread(target=subscribe_sonar, args=(i, res))
        thread.start()
        threads.append(thread)

    for thread in threads:
        thread.join()

    return res

def subscribe_sonar(i, result_list):
    """
    Subscribes to a specific sonar topic and collects data.
    Args:
    - i (int): Index of the sonar.
    - result_list (list): List to store the collected data.
    """
    def sonar_callback(msg, args):
        index, result_dict = args
        result_dict[str(index)] = msg.range
        
    subscriber = rospy.Subscriber(parameters['sonar_topics'][i], Range, sonar_callback, callback_args=(i, result_list))
    rospy.wait_for_message(parameters['sonar_topics'][i], Range)
    subscriber.unregister()

def publish_velocity():
    """
    Publishes the current linear and angular velocities to the robot at a fixed rate (10 Hz).
    This function runs in a separate thread until `_stop_thread` is set.
    """
    global vlin, vrot, _stop_thread, velocity_publisher
    rate = rospy.Rate(10)  # 10 Hz
    while not _stop_thread.is_set():
        twist = Twist()
        twist.linear.x = vlin
        twist.angular.z = vrot
        velocity_publisher.publish(twist)
        rate.sleep()

def set_vel_values(new_vlin, new_vrot):
    """
    Sets new values for the robot's linear and angular velocities.
    """
    global vlin, vrot
    vlin = new_vlin
    vrot = new_vrot

def stop_thread():
    """
    Signals the velocity publishing thread to stop.
    """
    global _stop_thread
    _stop_thread.set()
    # vel_thread.join()

def get_box_position (box):
    """
    Virtual sensor to read the current (x,y) position of a given box in global frame
    In simulation, this uses a ad-hoc service implemented in Gazebo
    In a real physical setup, this could be implemented by a set of cameras in the environments
    This dummy version always returns global position (0.0)
    """
    box = box.upper()
    service_name = f"{box}/get_position"
    rospy.wait_for_service(service_name)
    try:
        get_position = rospy.ServiceProxy(service_name, BoxPos)
        response = get_position(True)
        x = response.x_pos
        y = response.y_pos
        z = response.z_pos
        
        return (x, y)
    
    except rospy.ServiceException as e:
        rospy.logerr(f"Service call failed: {e}")
        return (0.0, 0.0)

def box_pickup(box):
    """
    Picks up the specified box.
    """
    box = box.upper()
    service_name = f"{box}/pick_up"
    rospy.wait_for_service(service_name)
    try:
        pick_up = rospy.ServiceProxy(service_name, BoxPickUp)
        response = pick_up(True)
        return response.success
    except rospy.ServiceException as e:
        rospy.logerr(f"Service call failed: {e}")
        return False

def box_putdown(box):
    """
    Puts down the specified box.
    """
    box = box.upper()
    service_name = f"{box}/put_down"
    rospy.wait_for_service(service_name)
    try:
        put_down = rospy.ServiceProxy(service_name, BoxPutDown)
        response = put_down(True)
        return response.success
    except rospy.ServiceException as e:
        rospy.logerr(f"Service call failed: {e}")
        return False

def update_box_position(box, x, y):
    """
    Updates the position of the specified box to the given (x, y) coordinates.
    """
    box = box.upper()
    service_name = f"{box}/update_position"
    rospy.wait_for_service(service_name)
    try:
        update_position = rospy.ServiceProxy(service_name, BoxUpdatePos)
        response = update_position(box, x, y, True) 
        return response.success
    except rospy.ServiceException as e:
        rospy.logerr(f"Service call failed: {e}")
        return False

def door_client (door, request):
    """
    Opens or closes the specified door.
    """
    service_name = f"{door}/update_pos"
    rospy.wait_for_service(service_name)
    try:
        door_request = rospy.ServiceProxy(service_name, DoorOpen)
        response = door_request(request)
    
    except rospy.ServiceException as e:
        rospy.logerr(f"Service call failed: {e}")

def initialize_doors():
    """
    Initializes the doors in the robot's environment based on their predefined states in the robot map.
    For each door in the map properties, it constructs the door's name and sends a request to either open or close the door.3
    """
    for door in robot_map.map.properties:
        door_name = door[0] + 'oor' + door[1]
        if robot_map.map.properties[door] == 'open':
            door_client(door_name, True)
        elif robot_map.map.properties[door] == 'close':
            door_client(door_name, False)
        else:
            print('Error in the definition of the state of ', door_name)

    