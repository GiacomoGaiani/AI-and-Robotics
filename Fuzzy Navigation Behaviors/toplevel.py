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
from utilities import save_trajectory, plot_trajectory


class TopLevelLoop:
    """
    Top level execution loop
    """

    def __init__(self,
                 goal = (0, 0),         # goal position
                 tcycle = 0.1 ,         # cycle time, in sec
                 debug = 0,             # debug level, use to decide to print debug info
                 plot_positions = 1     # test level, use to plot the trajectory of the robot
                 ):
        self.tcycle = tcycle
        self.goal = goal
        self.debug = debug
        self.plot_positions = plot_positions
        self.trajectory = []
        
    def step(self):
        """
        Execute one step of this instance of the Top Level loop
        debug level can be used to selectively print debug information
        """

        wl, wr = robot_gwy.get_wheel_encoders()         # read proprioceptive sensors (wheel rotations)
        sdata  = robot_gwy.get_front_sonar_data()       # read exteroceptive sensors (sonars)
        realpose = robot_gwy.get_real_pos()             # just to check if the odometry calcualtions are correct
        if self.debug > 2:
            print('sonars =', ['{:.2f}'.format(s[1]) for s in sdata])
        mypose = observer.update_pose(wl, wr)           # estimate robot state (pose)
        
        if self.plot_positions > 0:                     # save the postitions at each step
            print(mypose,'\n',realpose)
            self.trajectory = save_trajectory(self.trajectory,mypose,realpose)

#       self.gridmap.update_grid(sdata, mypose)         # estimate environment state (occupancy grid)
        state = {'mypose' : mypose, 'sdata' : sdata}    # state passed to the controller
        vlin, vrot, achieved = controller.compute_ctr(state, self.debug)  # decide action (robot's vel)
        robot_gwy.set_vel_values(vlin, vrot)            # send control action
        if self.debug > 0:
            print('pose = ({:.2f}, {:.2f}, {:.2f})\n'.format(mypose[0], mypose[1], math.degrees(mypose[2]))) 
        return  achieved < 0.99                                     # return False to exit the loop

    def run(self, maxsteps = 0):
        """
        Run this instance of the Top Level loop
        debug level can be used to selectively print debug information
        """
        nstep = 1
        pars = robot_gwy.init_robot()
        observer.init_pose(pars)
#       self.gridmap = observer.Gridmap(pars)
#       self.gridmap.init_grid()
        controller.init_controls(self.goal)
        while True:
            print('Step n: ', nstep)
            start_time = time.time()
            if (maxsteps > 0 and nstep >= maxsteps):
                print("Max number of steps reached: exiting")
                break
            else:
                if not self.step():
                    break
                nstep += 1
                end_time = time.time()
                elapsed_time = end_time - start_time
                if elapsed_time < self.tcycle:
                    time.sleep(self.tcycle - elapsed_time)
#       self.gridmap.close_grid()
        robot_gwy.shutdown_robot()
        if self.plot_positions > 0:
            plot_trajectory(self.trajectory)