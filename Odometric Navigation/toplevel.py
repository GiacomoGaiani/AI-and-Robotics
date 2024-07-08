"""
============== UniBo: AI and Robotics 2024 ==============
Base code: top level execution loop
To be customized in an incremental way for the different labs

(c) 2024 Alessandro Saffiotti
"""
import time, math
import robot_gwy
import observer
import controller
from utilities import normalize_angle_deg
from utilities import save_trajectory, plot_trajectory

class TopLevelLoop:
    """
    Top level execution loop
    """

    def __init__(self,
                 mode = 'CONSTANT',     # Type of movimentation of the controller
                 goal = (0, 0),         # goal position
                 tcycle = 0.1,          # cycle time, in sec
                 debug = 1,             # debug level, use to decide to print debug info
                 plot_positions = 1     # test level, use to plot the trajectory of the robot
                 ):
        self.tcycle = tcycle
        self.mode = mode
        self.goal = goal
        self.debug = debug
        self.plot_positions = plot_positions
        self.trajectory = []

    def step(self):
        """
        Execute one step of this instance of the Top Level loop
        debug level can be used to selectively print debug information
        """
        wl, wr = robot_gwy.get_wheel_encoders()             # read sensors (wheel rotations)
        mypose = observer.update_pose(wl, wr)               # estimate state (robot's pose)
        realpose = robot_gwy.get_real_pos()                 # just to check if the odometry calcualtions are correct
        if self.plot_positions > 0:                         # save the postitions at each step
            self.trajectory = save_trajectory(self.trajectory,mypose,realpose)
        vlin, vrot, done = controller.compute_ctr(mypose, self.mode)   # decide action (robot's vel)
        robot_gwy.set_vel_values(vlin, vrot)                # send control action
        if self.debug > 0:
            print('Real pose: \t ({:.2f}, {:.2f}, {:.2f})'.format(realpose[0], realpose[1], math.degrees(realpose[2])))
            print('Pose: \t\t ({:.2f}, {:.2f}, {:.2f})'.format(mypose[0], mypose[1], normalize_angle_deg(math.degrees(mypose[2]))))
        return not done                                     # return False to exit the loop

    def run(self):
        """
        Run this instance of the Top Level loop
        debug level can be used to selectively print debug information
        """
        nstep = 0
        pars = robot_gwy.init_robot()
        observer.init_pose(pars)
        controller.init_controls(self.goal)
        while self.step():
            time.sleep(self.tcycle)
        robot_gwy.shutdown_robot()
        if self.plot_positions > 0:
            plot_trajectory(self.trajectory)

