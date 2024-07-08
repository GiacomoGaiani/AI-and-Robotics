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
from utilities import save_data_to_file, load_data_from_file

class TopLevelLoop:
    """
    Top level execution loop
    """

    def __init__(self,
                 mode = 'CONSTANT',     # Type of movimentation of the controller
                 gridmap_mode = 'NONE', # Type of estimation of the gridmap
                 goal = (0, 0),         # goal position
                 tcycle = 0.1 ,         # cycle time, in sec
                 debug = 0,             # debug level, use to decide to print debug info
                 ):
        self.tcycle = tcycle
        self.mode = mode
        self.gmode = gridmap_mode
        self.goal = goal
        self.debug = debug
        self.data = []

    def step(self):
        """
        Execute one step of this instance of the Top Level loop
        debug level can be used to selectively print debug information
        """
        wl, wr = robot_gwy.get_wheel_encoders()      # read proprioceptive sensors (wheel rotations)
        sdata  = robot_gwy.get_sonar_data()          # read exteroceptive sensors (sonars)
        if self.debug > 2:
            print('sonars = {}'.format([s[1] for s in sdata]))
        realpose = robot_gwy.get_real_pos()          # just to check if the odometry calcualtions are correct
        mypose = observer.update_pose(wl, wr)        # estimate robot state (pose)
        self.gridmap.update_grid(sdata, mypose)      # estimate environment state (occupancy grid)
        vlin, vrot, done = controller.compute_ctr(mypose, self.mode)   # decide action (robot's vel)
        robot_gwy.set_vel_values(vlin, vrot)         # send control action
        if self.debug > 0:
            print('pose = ({:.2f}, {:.2f}, {:.2f})'.format(mypose[0], mypose[1], math.degrees(mypose[2]))) 
        return not done                              # return False to exit the loop

    def save_sonar_real_pos_data(self):
        """
        Execute one step of this instance of the Top Level loop
        Used to collect the sonar and real pose data
        """
        sdata  = robot_gwy.get_sonar_data()
        realpose = robot_gwy.get_real_pos()
        self.data.append((sdata,realpose))
        
        return True

    def save_sonar_odom_pos_data(self):
        """
        Execute one step of this instance of the Top Level loop
        Used to collect the sonar and the omometric pose data
        """
        sdata  = robot_gwy.get_sonar_data()
        wl, wr = robot_gwy.get_wheel_encoders()
        mypose = observer.update_pose(wl, wr)
        self.data.append((sdata,mypose))
        
        return True


    def run(self, maxsteps = 0):
        """
        Executes the main loop based on the gridmap mode:
        - 'DEFAULT': Initializes robot, observer, grid map, and controller. Steps until maxsteps or loop exits.
        - 'SAVE_DATA': Saves sonar and pose data to a file while running until maxsteps or data is saved.
        - 'COMP_GRID_FROM_DATA': Loads data from file, updates grid map, and prints grid layers.
        """

        if self.gmode == 'DEFAULT':
            nstep = 1
            pars = robot_gwy.init_robot()
            observer.init_pose(pars)
            self.gridmap = observer.Gridmap(pars)
            self.gridmap.init_grid()
            controller.init_controls(self.goal)

            while self.step():
                if (maxsteps == 0 or nstep < maxsteps):
                    nstep += 1
                    time.sleep(self.tcycle)
                else:
                    break
            self.gridmap.print_grid(layer = 'ukn')
            self.gridmap.print_grid(layer = 'ept')
            self.gridmap.print_grid(layer = 'occ')
            self.gridmap.close_grid()

        elif self.gmode == 'SAVE_DATA':
            nstep = 1
            pars = robot_gwy.init_robot()
            observer.init_pose(pars)
            self.gridmap = observer.Gridmap(pars)
            self.gridmap.init_grid()
            controller.init_controls(self.goal)

            while True:
                start_time = time.time()
                print("Step number {}".format(nstep))
                if (maxsteps == 0 or nstep < maxsteps):
                    if not self.save_sonar_real_pos_data():
                        break

                    nstep += 1
                    end_time = time.time()
                    elapsed_time = end_time - start_time
                    print("Step time: {}".format(elapsed_time))
                    if elapsed_time < self.tcycle:
                        time.sleep(self.tcycle - elapsed_time)
                else:
                    save_data_to_file(self.data, 'sdata_file.pkl')
                    break

        elif self.gmode == 'COMP_GRID_FROM_DATA':
            nstep = 1
            pars = robot_gwy.init_robot_no_sim()
            self.gridmap = observer.Gridmap(pars)
            self.gridmap.init_grid()

            data = load_data_from_file('sdata_file.pkl')
            for entry in data:
                sdata = entry[0]
                pose = entry[1]
                self.gridmap.update_grid(sdata, pose, self.debug)
                print("Grid update n.{} completed".format(nstep))
                nstep += 1
            self.gridmap.print_grid(layer = 'ukn')
            self.gridmap.print_grid(layer = 'ept')
            self.gridmap.print_grid(layer = 'occ')
            self.gridmap.close_grid()

        robot_gwy.shutdown_robot()

