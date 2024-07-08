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
import asyncio
from sensor_msgs.msg import JointState, Range
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry

from utilities import quaternion_to_euler


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


def init_robot(simulation = True):

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

    rospy.init_node('robot_node')
    rospy.Subscriber('/ground_truth_odom', Odometry, pose_callback)
    parameters['wl_init_pos'],  parameters['wr_init_pos'] = get_wheel_encoders()

    for i in range(parameters['sonar_num']):
        if i < 9 : idx = '0'+str(i+1)
        else : idx = str(i+1)
        parameters['sonar_topics'].append('/sonar' + idx + '_base')

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
    print("Robot shut")


def get_wheel_encoders():
    """
    Get current values of wheel encoders, which indicate the current position
    of each wheel in radians
    """

    rospy.init_node('robot_node')
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

    rospy.init_node('robot_node')
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

    res = []
    rospy.init_node('robot_node')

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
    rospy.init_node('robot_node')

    threads = []

    for i in [0,1,2,10,11]:
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


def set_vel_values (vlin, vrot):
    """
    Set new linear and rotational velocities for the robot's base
    vlin is m/sec vrot is rad/sec
    Returns True if successful
    """

    try:
        rospy.init_node('robot_node')
        velocity_publisher = rospy.Publisher('/mobile_base_controller/cmd_vel', Twist, queue_size=10)
        rate = rospy.Rate(10)  # 10 Hz

        twist = Twist()
        twist.linear.x = vlin
        twist.angular.z = vrot
        velocity_publisher.publish(twist)
        rate.sleep()

        return True
    except rospy.ROSInterruptException:
        return False
