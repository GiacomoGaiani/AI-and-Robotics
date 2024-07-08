"""
============== UniBo: AI and Robotics 2024 ==============
Base code: dummy controller, performs the action decision parts of the main loop
To be customized in an incremental way for the different labs

(c) 2024 Alessandro Saffiotti
"""
from fcontrol import Behavior
from fcontrol import ramp_up, ramp_down, triangle
from fcontrol import global_to_local, local_to_global
from math import atan2, degrees, sqrt

target_pos = {
    'D1': (1.0, -5.5),
    'D4': (-3.0, -5.5),
    'stove': (-6.5, -3.0),
}


class GoToTarget (Behavior):
    """
    Instance of the Behavior class, which defines a fuzzy controller
    to navigate to a given target point target = (x,y)
    The setup function is called once when the behavior is created,
    and it creates all the fuzzy predicates, actions and rules that
    define the behavior's control strategy.
    The update_state function is called at every cycle and it sets
    the internal variables based on the passed robot's state
    The run function is also called at every cycle, and it runs
    the fuzzy controller with the behavior's rules and sets the
    output control variables vlin, vrot: this function is inherited
    from the FControl class (superclass of Behavior)
    The values of vlin,vrot are fetched via the get_vlin and get_vrot
    functions, which are inherited from the Behavior class
    """
    def __init__(self, target = (0.0, 0.0)):
        super().__init__()
        self.target = [target[0], target[1], 0.0]   # target point in global coordinates
        self.tlocal = [0, 0, 0]                     # target point in robot's coordinates
    
    def update_state(self, state):
        """
        Update the relevant local state variables (self.state) at every control cycle
        Uses the 'mypose' estimate, passed as part of the state by the top-level loop
        Set the local state variables 'phi' and 'rho'
        """
        mypose = state['mypose']
        global_to_local(self.target, mypose, self.tlocal)
        xt = self.tlocal[0]
        yt = self.tlocal[1]
        self.state['phi'] = degrees(atan2(yt, xt))
        self.state['rho'] = sqrt(xt * xt + yt * yt)

    def setup(self):
        """
        Definition of the rules of our 'go to target' fuzzy controller
        Fuzzy interpretation is built from the local state variables 'phi' and 'rho'
        """

        self.fpreds = {
            # Definition of the fuzzy predicates, used in the rules' LHS
            'TargetLeft'        : (ramp_up(30.0, 60.0), 'phi'),
            'TargetLeftAhead'   : (triangle(0.0, 15.0, 30.0), 'phi'),
            'TargetRight'       : (ramp_down(-60.0, -30.0), 'phi'),
            'TargetRightAhead'  : (triangle(-30.0, -15.0, 0.0), 'phi'),
            'TargetAhead'       : (triangle(-60.0, 0.0, 60.0), 'phi'),
            'TargetHere'        : (ramp_down(0.1, 1.0), 'rho')
        }

        self.flvars = {
            # Definition of the fuzzy linguistic variables, used in the rules' RHS
            'Move' : ({'Fast':0.5, 'Slow':0.1, 'None':0, 'Back':-0.1}, 'Vlin'),
            'Turn' : ({'Left':40, 'MLeft':5, 'None':0, 'MRight':-5, 'Right':-40}, 'Vrot')
        }

        self.frules = {
            # Lastly, definition of the actual fuzzy rules
            'ToMLeft' : ("TargetLeftAhead AND NOT(TargetHere)", 'Turn', 'MLeft'),
            'ToLeft'  : ("TargetLeft AND NOT(TargetHere)", 'Turn', 'Left'),
            'ToMRight' : ("TargetRightAhead AND NOT(TargetHere)", 'Turn', 'MRight'),
            'ToRight' : ("TargetRight AND NOT(TargetHere)", 'Turn', 'Right'),
            'Far'     : ("TargetAhead AND NOT(TargetHere)", 'Move', 'Fast'),
            'Stop'    : ("TargetHere", 'Move', 'None')
        }

        # The degree of achievement is given by the predicate 'TargetHere'
        self.fgoal = "TargetHere"

        # Finally, initialize the fuzzy predicats and fuzzy output sets
        self.init_flsets()
        self.init_fpreds()
            

class Avoid (Behavior):
    """
    A behavior class for avoiding obstacles. 
    This behavior utilizes fuzzy logic controllers to navigate the robot away from obstacles detected by its sensors.
    """
    def __init__(self, target = (0.0, 0.0)):
        super().__init__()
    
    def update_state(self, state):
        """
        Update the relevant local state variables (self.state) at every control cycle
        Uses the 'mypose' estimate, passed as part of the state by the top-level loop
        Set the local state variables 'phi' and 'rho'
        """
        mypose = state['mypose']
        sdata = state['sdata']
    
        self.state['sFront'] = sdata['0']
        self.state['sMLeft'] = sdata['1']
        self.state['sLeft'] = sdata['2']
        self.state['sMRight'] = sdata['11']
        self.state['sRight'] = sdata['10']

    def setup(self):
        """
        Definition of the rules of our 'avoid' fuzzy controller
        Fuzzy interpretation is built from the local state variables sFront, sMLeft, sLeft, sMRight, sRight
        """

        self.fpreds = {
            # Definition of the fuzzy predicates, used in the rules' LHS
            'ObjectAhead'       : (ramp_down(2.0, 3.0), 'sFront'),
            'ObjectMLeft'       : (ramp_down(1.0, 2.0), 'sMLeft'),
            'ObjectLeft'        : (ramp_down(1.0, 2.0), 'sLeft'),
            'ObjectMRight'      : (ramp_down(1.0, 2.0), 'sMRight'),
            'ObjectRight'       : (ramp_down(1.0, 2.0), 'sRight'),
            'ObjectFarMLeft'    : (triangle(1.0, 2.0, 3.0), 'sMLeft'),
            'ObjectFarLeft'     : (triangle(1.0, 2.0, 3.0), 'sLeft'),
            'ObjectFarMRight'   : (triangle(1.0, 2.0, 3.0), 'sMRight'),
            'ObjectFarRight'    : (triangle(1.0, 2.0, 3.0), 'sRight'),
        }

        self.flvars = {
            # Definition of the fuzzy linguistic variables, used in the rules' RHS
            'Move' : ({'Fast':1.0, 'Slow':0.1, 'None':0, 'Back':-0.1}, 'Vlin'),
            'Turn' : ({'Left':40, 'MLeft':10, 'None':0, 'MRight':-10, 'Right':-40}, 'Vrot')
        }

        self.frules = {
            # Lastly, definition of the actual fuzzy rules
            'NoObjects': ("NOT(ObjectAhead) AND NOT(ObjectMLeft) AND NOT(ObjectMRight)", 'Move', 'Fast'),
            'AvoidLeft': ("ObjectLeft OR ObjectMLeft", 'Turn', 'MRight'),
            'AvoidRight': ("ObjectRight OR ObjectMRight AND NOT(ObjectAhead)", 'Turn', 'MLeft'),
            'StayCloseLeft': ("ObjectFarLeft AND NOT(ObjectAhead)", 'Turn', 'MLeft'),
            'AvoidInFront': ("ObjectAhead", 'Turn', 'Right'),  
        }

        # The degree of achievement is given by the predicate ''NOT(ObjectAhead)'
        self.fgoal = 'NOT(ObjectAhead)'

        # Finally, initialize the fuzzy predicats and fuzzy output sets
        self.init_flsets()
        self.init_fpreds()
            
class FollowObject (Behavior):
    """
    A behavior class for following objects. 
    This behavior employs fuzzy logic controllers to track and follow objects detected by the robot's sensors.
    """
    def __init__(self, target = (0.0, 0.0)):
        super().__init__()
    
    def update_state(self, state):
        """
        Update the relevant local state variables (self.state) at every control cycle
        Uses the 'sdata' inforamtions, passed as part of the state by the top-level loop
        Set the local state variables sFront, sMLeft, sLeft, sMRight, sRight
        """
        mypose = state['mypose']
        sdata = state['sdata']
    
        self.state['sFront'] = sdata['0']
        self.state['sMLeft'] = sdata['1']
        self.state['sLeft'] = sdata['2']
        self.state['sMRight'] = sdata['11']
        self.state['sRight'] = sdata['10']

    def setup(self):
        """
        Definition of the rules of our 'avoid' fuzzy controller
        Fuzzy interpretation is built from the local state variables sFront, sMLeft, sLeft, sMRight, sRight
        """

        self.fpreds = {
            # Definition of the fuzzy predicates, used in the rules' LHS
            'ObjectAhead'       : (ramp_down(2.0, 3.0), 'sFront'),
            'ObjectTooClose'    : (ramp_down(1.0, 1.5), 'sFront'),
            'ObjectMLeft'       : (ramp_down(1.0, 2.0), 'sMLeft'),
            'ObjectLeft'        : (ramp_down(1.0, 2.0), 'sLeft'),
            'ObjectMRight'      : (ramp_down(1.0, 2.0), 'sMRight'),
            'ObjectRight'       : (ramp_down(1.0, 2.0), 'sRight'),
        }

        self.flvars = {
            # Definition of the fuzzy linguistic variables, used in the rules' RHS
            'Move' : ({'Fast':1.0, 'Slow':0.1, 'None':0, 'Back':-0.1}, 'Vlin'),
            'Turn' : ({'Left':40, 'MLeft':10, 'None':0, 'MRight':-10, 'Right':-40}, 'Vrot')
        }

        self.frules = {
            # Lastly, definition of the actual fuzzy rules
            'NoObjects': ("NOT(ObjectAhead) AND NOT(ObjectMLeft) AND NOT(ObjectMRight)", 'Move', 'Fast'),
            'FollowInFront': ("ObjectAhead AND NOT(ObjectTooClose)", 'Move', 'Fast'),
            'FollowLeft': ("ObjectLeft OR ObjectMLeft", 'Turn', 'Left'),
            'FollowRight': ("ObjectRight OR ObjectMRight", 'Turn', 'Right'),
            'TooClose' : ("ObjectTooClose", 'Move', 'Back')
               
        }

        # The degree of achievement is given by the predicate 'ObjectAhead'
        self.fgoal = 'ObjectAhead'

        # Finally, initialize the fuzzy predicats and fuzzy output sets
        self.init_flsets()
        self.init_fpreds()

class CrossDoor (Behavior):
    """
    A behavior class for crossing a door. 
    This behavior uses fuzzy logic controllers to guide the robot through a door opening.
    """
    def __init__(self, target = (0.0, 0.0)):
        super().__init__()
    
    def update_state(self, state):
        """
        Update the relevant local state variables (self.state) at every control cycle
        Uses the 'sdata' inforamtions, passed as part of the state by the top-level loop
        Set the local state variables sFront, sMLeft, sLeft, sMRight, sRight
        """
        mypose = state['mypose']
        sdata = state['sdata']
    
        self.state['sFront'] = sdata['0']
        self.state['sMLeft'] = sdata['1']
        self.state['sLeft'] = sdata['2']
        self.state['sMRight'] = sdata['11']
        self.state['sRight'] = sdata['10']
        self.state['sAround'] = float(sum(sdata.values())/len(sdata))

    def setup(self):
        """
        Definition of the rules of our 'avoid' fuzzy controller
        Fuzzy interpretation is built from the local state variables sFront, sMLeft, sLeft, sMRight, sRight
        """

        self.fpreds = {
            # Definition of the fuzzy predicates, used in the rules' LHS
            'ObjectAhead'       : (ramp_down(1.0, 3.0), 'sFront'),
            'ObjectMLeft'       : (ramp_down(1.0, 3.0), 'sMLeft'),
            'ObjectLeft'        : (ramp_down(1.0, 3.0), 'sLeft'),
            'ObjectMRight'      : (ramp_down(1.0, 3.0), 'sMRight'),
            'ObjectRight'       : (ramp_down(1.0, 3.0), 'sRight'),
            'ObjectAround'      : (ramp_down(1.0, 2.0), 'sAround'),
        }

        self.flvars = {
            # Definition of the fuzzy linguistic variables, used in the rules' RHS
            'Move' : ({'Fast':0.5, 'Slow':0.1, 'None':0, 'Back':-0.1}, 'Vlin'),
            'Turn' : ({'Left':40, 'MLeft':10, 'None':0, 'MRight':-10, 'Right':-40}, 'Vrot')
        }

        self.frules = {
            # Lastly, definition of the actual fuzzy rules
            'FacingDoor': ("NOT(ObjectAhead)", 'Move', 'Fast'),
            'PassedDoor': ("NOT(ObjectAround)", 'Move', 'None'),
            'DoorOnLeft': ("NOT(ObjectLeft)", 'Turn', 'MLeft'),
            'DoorOnRight': ("NOT(ObjectRight)", 'Turn', 'MRight'),
            'DoorOnMLeft': ("NOT(ObjectMLeft)", 'Turn', 'Left'),
            'DoorOnMRight': ("NOT(ObjectMRight)", 'Turn', 'Right'),
        }

        # The degree of achievement is given by the predicate 'TargetHere'
        self.fgoal = 'NOT(ObjectAround)'

        # Finally, initialize the fuzzy predicats and fuzzy output sets
        self.init_flsets()
        self.init_fpreds()


behavior = None     # which behavior we are currently executing


def init_controls (goal):
    """
    Initialize the controller, setting the global goal position
    Return True if the inizialization was successful
    """
    global behavior
    behavior = None
    # behavior = GoToTarget(goal)
    # behavior = Avoid()
    # behavior = FollowObject()
    # behavior = CrossDoor()
    return True


def compute_ctr (state, action_plan, debug):
    """
    Action decision. Compute control values (vlin, vrot) given the current robot's pose
    Returns the control values (vlin, vrot), as a pair
    """
    global behavior
    action, target = action_plan

    goal = target_pos[target]

    if action == 'GoTo': behavior = GoToTarget(goal)
    elif action == 'Cross': behavior = CrossDoor()


    achieved = behavior.run(state, debug)
    vlin = behavior.get_vlin()
    vrot = behavior.get_vrot()
    if debug > 0:
        print('Goal achievement: {:.2f}'.format(achieved))
    if debug > 0:
        print('(vlin, vrot)) = ({:.2f}, {:.2f})'.format(vlin, degrees(vrot))) 
    return (vlin, vrot, achieved)
