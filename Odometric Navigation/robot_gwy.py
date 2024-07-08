"""
============== UniBo: AI and Robotics 2024 ==============
Base code: gateway to the robot (or simulator)
This is a dummy version, most values and funtions are place-holders
It needs to be customized by the students for the different labs

(c) 2024 Alessandro Saffiotti
"""
import rospy
import math
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry

from utilities import quaternion_to_euler

SIMULATION = True

parameters = {                  # these are for the Tiago robot
    'wheel_radius' : 0.0985,
    'wheel_axis' : 0.4044,
    'x_init_pos' : 0.0,
    'y_init_pos' : 0.0,
    'th_init_pos' : 0.0,
    'wl_init_pos' : 0.0,
    'wr_init_pos' : 0.0
}


def init_robot ():
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

    if SIMULATION:
        rospy.init_node('robot_node')
        rospy.Subscriber('/ground_truth_odom', Odometry, pose_callback)
    
    parameters['wl_init_pos'],  parameters['wr_init_pos'] = get_wheel_encoders()

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

def get_real_pos ():

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
    
    