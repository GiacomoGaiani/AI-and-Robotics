"""
============== UniBo: AI and Robotics 2024 ==============
Base code: top level execution loop
To be customized in an incremental way for the different labs

(c) 2024 Alessandro Saffiotti
"""
import time, math
import robot_gwy, robot_map, observer, controller, pyhop
import utilities

class TopLevelLoop:
    """
    Top level global execution loop
    """

    def __init__(self,
                 goal  = [],            # top level task, passed to the HTN planner
                 mypose = None,         # robot's starting pose, if None take it from the map
                 tcycle = 0.1 ,         # cycle time, in sec
                 debug = 0,             # debug level, use to decide what debug info to print
                                        # (0: none, 1: only outer SPA loop, 2: inner control loop, 3: details)
                 ):
        self.goal = goal
        self.tcycle = tcycle
        self.debug = debug
        self.mypose = mypose            # estimated robot's pose
        self.pars  = {}                 # robot's parameters
        self.ctr = None                 # controller instance
        # Tasks Variables
        self.box_traking = True         # TASK 02 - Real time traking of the boxes
        self.box_traking_step = 1       # TASK 02 - n of steps between one box check and the next
        self.door_locked_error = False  # TASK 03 - tells if the closed doors can be open in case of error
        # Plot Flags
        self.plot_positions = True      # Flag to store and plot the robot trajectory
        self.plot_pos_error = False     # Flag to store and plot the error between the real and the computed robot position
        # Support Variables
        self.lateral_sensors = False    # Flag for the number of sonar sensor used
        self.trajectory = []            # List containing the robot trajectory
        self.pos_error_list = []        # List contaning the robot position error at each step
        self.recalibration = False      # TASK 04 - Variable used for the recalibration of the robot position
        self.recalibration_poses = []   # TASK 04 - List containig the position information used for the recalibration
        self.temp_cali_list = []        # List used to plot the step in which the robot is recalibrating the position
        self.carrying = None            # Task 05 - The obj the robot is currently carrying

    def print_pose(self, mypose):
        print('pose = ({:.2f}, {:.2f}, {:.2f})\n'.format(mypose[0], mypose[1], math.degrees(mypose[2]))) 
        
    def run (self, maxsteps = 0, goal = None):
        """
        Main entry point: run this instance of the Top Level loop
        'maxsteps' can be used as a timeout to abort execution after a certain number of cycles
        """
        if goal:
            self.goal = goal
        if self.mypose == None:
            self.mypose = robot_gwy.get_real_pos()  # set the initial position using the ground truth position
        self.pars = robot_gwy.init_robot()          # get the robot's parameter from the robot gateway
        observer.init_pose(self.pars)               # init the observer
        self.ctr = controller.Controller()          # init the controller   
        self.sense_plan_act(self.goal, maxsteps)    # go into the main SPA loop
        robot_gwy.shutdown_robot()                  # done
        if self.plot_positions:
            utilities.plot_trajectory(self.trajectory)
        if self.plot_pos_error:
            utilities.plot_pos_error(self.pos_error_list)
            utilities.plot_pos_error(self.temp_cali_list)

    def sense_plan_act(self, goal, maxsteps = 0):
        """
        The outer "sense plan act" loop
        """
        from htn_domain import State

        # the 'S' part, fill the state defined in our domain with current data
        state = State()
        self.get_state(state)
        if self.debug > 0:
            print("Planner called from initial state:")
            pyhop.print_state(state)

        # the 'P' part, generate a plan for the given goal in the current state
        plan = pyhop.pyhop(state, goal, verbose=2)
        if self.debug > 0:
            print("Planner returns plan:", plan)
        if plan:
            # the 'A' part, execute the plan action by action
            result, replan = self.execute_plan(plan, maxsteps)       # A = Act
            if self.debug > 0:
                if result:
                    print("Plan execution completed!")
                else:
                    print("Plan execution failed!")
            if replan:
                print('New plan request')
                self.sense_plan_act(self.goal, maxsteps)
            return result
        else:
            if self.debug > 0:
                print("No plan found!")
        return None
    
    def get_state (self, state):
        """
        Fill the given state with the current values, taken from the map
        or from the robot's sensors
        """
        # first we set the static part of the state, taken from the map
        # see in the map which objects (doors) connect rooms to one another
        for room1 in robot_map.map.topology:
            for room2 in robot_map.map.topology:
                if room1 == room2:
                    continue
                for object1 in robot_map.map.topology[room1]:
                    for object2 in robot_map.map.topology[room2]:
                        if object1 == object2:
                            state.connects[object1] = (room1, room2)
        # set their status
        for door in state.connects:
            state.door[door] = robot_map.map.properties[door]
        # see what is the room of each object (except the doors above)
        for room, contents in robot_map.map.topology.items():
            for object in contents:
                if object in state.connects:
                    continue
                state.room[object] = room

        # second we set the dynamic part of the state, updated through the robot's (real or virtual) sensors
        state.visited = set()
        state.path = []
        state.carrying = 'Empty'

        state.room['me'] = robot_map.map.find_room(self.mypose)
        state.pos['me']  = robot_map.map.find_location(self.mypose)

        state = utilities.box_postion_state_update(state, 'box1')
        state = utilities.box_postion_state_update(state, 'box2')
        state = utilities.box_postion_state_update(state, 'box3')

    def execute_plan (self, plan, maxsteps):
        """
        The "Act" part of the SPA loop
        Pop each action in the plan in sequence, and execute it
        Return the tuple results(status, replan):
        - status: True for successful plan execution, False for failure
        - replan: True if a new plan is needed, False otherwise
        """
        if self.debug > 0:
            print("Executing plan")
        for action in plan:
            result, replan = self.execute_action(action = action, maxsteps = maxsteps)
            if result == False:
                break
        return result, replan

    def execute_action(self, action, threshold = 0.9, maxsteps = 0):
        """
        The inner action execution loop
        Execute 'action' until its degree of achievement is greter than 'threshold'
        timeout with failure after 'maxsteps' steps (zero = no timeout)
        Return True for successful execution, False for failure
        """
        if self.debug > 0:
            print("Executing action:", action)
        nsteps = 0

        # set the behavior to be run by the controller
        behavior = action[0]
        if behavior == 'Open' or behavior == 'Close':  # because these call a service using the door's name
            param = action[1]
        elif behavior == 'PickUp' or behavior == 'PutDown':  # because these call a service using the box's name
            param = action[1]
        elif behavior == 'GoTo' and action[1][0] == 'D':
            doorpose =  robot_map.map.locations[action[1]]
            param = utilities.find_approach_point(self.mypose, doorpose)
        else:
            param = robot_map.map.locations[action[1]]
        self.ctr.set_behavior(bname = behavior, bparam = param)

        if behavior == 'Cross': 
            self.lateral_sensors = True
        else:
            self.lateral_sensors = False
        
        if behavior == 'SavePos':
            if self.recalibration:
                self.recalibration = False
                self.recalibration_poses.append(self.mypose)
                dx, dy = utilities.pos_calibration(self.recalibration_poses, param)
                self.mypose = utilities.pos_adjustment(self.mypose, dx, dy)
            else:
                self.recalibration = True
                self.recalibration_poses.append(self.mypose)

        # run the main control pipeline until completion, or failure
        while True:
            # here you should check that ROS is still running
            # if rospy.is_shutdown():                     # ROS was killed
            #   return False
            nsteps += 1
            if (maxsteps > 0 and nsteps < maxsteps):    # timeout
                if self.debug > 0:
                    print("Max number of steps reached: exiting")
                return False
            result, done = self.step()                  # execute control pipeline

            if behavior == 'CheckBox':                  # Box localization failure
                if done < threshold:
                    if self.debug > 0: print("Action", action, "failed")
                    return False, True
            if behavior == 'GoTo' and action[1][:3] == 'box':
                if self.box_traking and nsteps % self.box_traking_step == 0:
                    pos_error, room_error, _ = utilities.realtime_box_pos_check(action[1], self.mypose)
                    if room_error:
                        if self.debug > 0: print("Realtime Box update: Room changed")
                        return False, True
                    elif pos_error > 0.2:
                        if self.debug > 0: print("Realtime Box update: Position changed")
                        self.execute_action(action = action, maxsteps = maxsteps)
            
            if behavior == 'CheckDoor':                  # Door failure
                if done < threshold:
                    if self.debug > 0: print("Action", action, "failed")
                    if self.door_locked_error:
                        if self.debug > 0: print("Door", action[1], "locked")
                        utilities.remove_door(action[1])
                        return False, True 
                    else:
                        if self.debug > 0: print("Door", action[1], "close")
                        robot_map.map.properties[action[1]] = 'close'
                        return False, True
            if self.carrying != None:                   # Box failure while carrying
                if self.box_traking and nsteps % self.box_traking_step == 0:
                    if action[0] != 'PutDown':
                        _, _, robot_box_dist = utilities.realtime_box_pos_check(self.carrying, self.mypose)
                        if robot_box_dist > 2.0:
                            if self.debug > 0: print("Realtime Box update: Carried box missing")
                            self.carrying = None
                            return False, True

            if result == False:                         # behavior failure
                if self.debug > 0:
                    print("Action", action, "failed")
                return False, False                            # action failed
            if done > threshold:                        # behavior completed
                if self.debug > 0:
                    print("Action", action, "completed")
                    print('Robot pos : ', self.mypose)
                    if action[0] == 'PickUp':
                        self.carrying = action[1]
                    if action[0] == 'PutDown':
                        self.carrying = None
                return True, False                             # action completed

    def step (self):
        """
        The basic control pipeline: read sensors, estimate state, decide controls, send controls
        Return True for successful execution, plus the current degree of achievement
        """
        start_time = time.time()
        wl, wr = robot_gwy.get_wheel_encoders()             # read proprioceptive sensors (wheel rotations)
        if self.lateral_sensors:
            sdata  = robot_gwy.get_lateral_sonar_data()     # read exteroceptive sensors (sonars)
        else: 
            sdata  = robot_gwy.get_front_sonar_data()       # read exteroceptive sensors (sonars)
        # if self.debug > 1:
        #     print('sonars =', ' '.join(['{:.2f}'.format(s[1]) for s in sdata]))

        self.mypose = observer.update_pose(wl, wr)          # estimate robot state (pose)
        realpose = robot_gwy.get_real_pos()
        if self.plot_pos_error:
            self.pos_error_list.append(utilities.compute_linear_distance(self.mypose, realpose))
            if self.recalibration:
                self.temp_cali_list.append(1.0)
            else: self.temp_cali_list.append(0.0)
        
        state = {'mypose' : self.mypose, 'sdata' : sdata}   # state passed to the controller

        if self.plot_positions > 0:                     # save the postitions at each step
            self.trajectory = utilities.save_trajectory(self.trajectory,self.mypose,realpose)

        achieved = self.ctr.run(state, self.debug)          # compute controls (robot's vels)
        vlin = self.ctr.get_vlin()                          # retrieve computed controls (vlin)
        vrot = self.ctr.get_vrot()                          # retrieve computed controls (vrot)

        robot_gwy.set_vel_values(vlin, vrot)                # send controls
        if self.debug > 1:
            self.print_pose(self.mypose)

        end_time = time.time()
        elapsed_time = end_time - start_time
        if elapsed_time < self.tcycle:
            time.sleep(self.tcycle - elapsed_time)          # sync on a constant cycle time
        return True, achieved


