"""
============== UniBo: AI and Robotics 2024 ==============
Base code: dummy controller, performs the action decision parts of the main loop
To be customized in an incremental way for the different labs

(c) 2024 Alessandro Saffiotti
"""
from fcontrol import Behavior
from fcontrol import ramp_up, ramp_down, triangle, trapezoid
from fcontrol import global_to_local, local_to_global
from math import atan2, degrees, sqrt

from robot_gwy import door_client, box_pickup, box_putdown

class GoTo (Behavior):
    """
    Navigate to a target object at (x,y,th,radius), where 'x,y' is the object's center
    in global frame, and 'radius' is its size. Use sonars for local obstacle avoidance.
    """
    def __init__(self, target = (0.0, 0.0, 0.0, 0.0)):
        self.target = target            # target point in global coordinates
        self.tlocal = [0, 0, 0]         # target point in robot's coordinates
        self.range_min = target[3] + 0.1
        self.range_max = target[3] + 1.0
        super().__init__()
        print("Instantiated GoTo behavior with target =", target)
    
    def update_state(self, state):
        # pose data state
        mypose = state['mypose']
        global_to_local(self.target, mypose, self.tlocal)
        xt = self.tlocal[0]
        yt = self.tlocal[1]
        self.state['phi'] = degrees(atan2(yt, xt))
        self.state['rho'] = sqrt(xt * xt + yt * yt)
        # sonar data state
        sdata = state['sdata']
        self.state['sFront'] = sdata['0']
        self.state['sMLeft'] = sdata['1']
        self.state['sLeft'] = sdata['2']
        self.state['sMRight'] = sdata['11']
        self.state['sRight'] = sdata['10']
        self.state['Danger'] = min(sdata.values())

    def setup(self):
        """
        Definition of the rules of our 'GoTo' fuzzy controller
        Set the local state variables phi, rho, sFront, sMLeft, sLeft, sMRight, sRight, Danger
        """
        self.fpreds = {
            # Definition of the GoToTarget fuzzy predicates, used in the rules' LHS
            'TargetLeft'        : (ramp_up(30.0, 60.0), 'phi'),
            'TargetLeftAhead'   : (triangle(0.0, 15.0, 30.0), 'phi'),
            'TargetRight'       : (ramp_down(-60.0, -30.0), 'phi'),
            'TargetRightAhead'  : (triangle(-30.0, -15.0, 0.0), 'phi'),
            'TargetAhead'       : (triangle(-60.0, 0.0, 60.0), 'phi'),
            'TargetNear'        : (ramp_down(self.range_max+0.1, self.range_max+1.0), 'rho'),
            'TargetHere'        : (ramp_down(self.range_min, self.range_max), 'rho'),
            # Definition of the Avoid fuzzy predicates, used in the rules' LHS
            'ObjectAhead'       : (ramp_down(1.0, 2.0), 'sFront'),
            'ObjectMLeft'       : (ramp_down(1.0, 2.0), 'sMLeft'),
            'ObjectLeft'        : (ramp_down(1.0, 2.0), 'sLeft'),
            'ObjectMRight'      : (ramp_down(1.0, 2.0), 'sMRight'),
            'ObjectRight'       : (ramp_down(1.0, 2.0), 'sRight'),
            # Definition of the switch fuzzy predicate
            'DangerAhead'       : (ramp_down(0.5, 1.0), 'Danger'),
        }

        self.flvars = {
            # Definition of the fuzzy linguistic variables, used in the rules' RHS
            'Move' : ({'Fast':0.35, 'Slow':0.1, 'None':0, 'Back':-0.1}, 'Vlin'),
            'Turn' : ({'Left':30, 'MLeft':5, 'None':0, 'MRight':-5, 'Right':-30}, 'Vrot')
        }

        self.frules = {
            # GoToTarget fuzzy rules
            'ToMLeft'       : ("(((NOT(DangerAhead) AND NOT(ObjectMLeft)) AND TargetLeftAhead) AND NOT(TargetHere))", 'Turn', 'MLeft'),
            'ToLeft'        : ("(((NOT(DangerAhead) AND NOT(ObjectLeft)) AND TargetLeft) AND NOT(TargetHere))", 'Turn', 'Left'),
            'ToMRight'      : ("(((NOT(DangerAhead) AND NOT(ObjectMRight)) AND TargetRightAhead) AND NOT(TargetHere))", 'Turn', 'MRight'),
            'ToRight'       : ("(((NOT(DangerAhead) AND NOT(ObjectRight)) AND TargetRight) AND NOT(TargetHere))", 'Turn', 'Right'),
            'FarAhead'      : ("(((NOT(DangerAhead) AND NOT(ObjectAhead)) AND TargetAhead) AND NOT(TargetHere))", 'Move', 'Fast'),
            'Slow'          : ("((NOT(ObjectAhead) AND DangerAhead) AND NOT(TargetHere))", 'Move', 'Slow'),
            'GoClose'       : ("((DangerAhead AND TargetNear) AND NOT(TargetHere))", 'Move', 'Slow'),
            'GoCloseLeft'   : ("(((TargetLeft OR TargetLeftAhead) AND DangerAhead) AND TargetNear)", 'Turn', 'MLeft'),
            'GoCloseRight'  : ("(((TargetRight OR TargetRightAhead) AND DangerAhead) AND TargetNear)", 'Turn', 'MRight'),
            'Stop'          : ("NOT(DangerAhead) AND TargetHere", 'Move', 'None'),

            # Avoid fuzzy rules
            # 'Back'          : ("((ObjectAhead AND DangerAhead) AND NOT(TargetNear))", 'Move', 'Back'),
            'AvoidLeft'     : ("(((ObjectLeft AND DangerAhead) AND NOT(TargetNear)) AND NOT(ObjectAhead))", 'Turn', 'MRight'),
            'AvoidRight'    : ("(((ObjectRight AND DangerAhead) AND NOT(TargetNear)) AND NOT(ObjectAhead))", 'Turn', 'MLeft'),
            'AvoidMLeft'    : ("(((ObjectMLeft AND DangerAhead) AND NOT(TargetNear)) AND NOT(ObjectAhead))", 'Turn', 'Right'),
            'AvoidMRight'   : ("(((ObjectMRight AND DangerAhead) AND NOT(TargetNear)) AND NOT(ObjectAhead))", 'Turn', 'Left'),
            'Stuck'         : ("((ObjectAhead AND DangerAhead) AND NOT(TargetNear))", 'Turn', 'Right'),
        }

        # The degree of achievement is given by the predicate 'TargetHere'
        self.fgoal = "TargetHere"

        # Finally, initialize the fuzzy predicats and fuzzy output sets
        self.init_flsets()
        self.init_fpreds()


class Cross (Behavior):
    """
    Cross a door at (x,y,th,radius), where 'x,y' is the door's center in global frame,
    and 'radius' is its size. Use sonars for local obstacle avoidance.
    """
    def __init__(self, door = (0.0, 0.0, 0.0, 0.0)):
        print("Instantiated Cross behavior with door =", door)
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
        self.state['sBLeft'] = sdata['3']
        self.state['sBRight'] = sdata['9']
        self.state['sAround'] = min(sdata.values())

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
            'ObjectAround'      : (ramp_down(1.0, 1.5), 'sAround'),
        }

        self.flvars = {
            # Definition of the fuzzy linguistic variables, used in the rules' RHS
            'Move' : ({'Fast':0.2, 'Slow':0.1, 'None':0, 'Back':-0.1}, 'Vlin'),
            'Turn' : ({'Left':10, 'MLeft':5, 'None':0, 'MRight':-5, 'Right':-10}, 'Vrot')
        }

        self.frules = {
            # Lastly, definition of the actual fuzzy rules
            'FacingDoor'    : ("NOT(ObjectAhead)", 'Move', 'Fast'),
            'PassedDoor'    : ("NOT(ObjectAround)", 'Move', 'None'),
            'DoorOnLeft'    : ("NOT(ObjectLeft)", 'Turn', 'MLeft'),
            'DoorOnRight'   : ("NOT(ObjectRight)", 'Turn', 'MRight'),
            'DoorOnMLeft'   : ("NOT(ObjectMLeft)", 'Turn', 'Left'),
            'DoorOnMRight'  : ("NOT(ObjectMRight)", 'Turn', 'Right'),
        }

        # The degree of achievement is given by the predicate 'ObjectAround'
        self.fgoal = 'NOT(ObjectAround)'

        # Finally, initialize the fuzzy predicats and fuzzy output sets
        self.init_flsets()
        self.init_fpreds()


class Open (Behavior):
    """
    Open a door by a given name, just calling the relevant ROS service
    """
    def __init__(self, door):
        print("Instantiated Close behavior with door =", door)
        self.doorname = door
        super().__init__()
    
    def update_state(self, state):
        if self.doorname == 'D1':
            door_client('Door1',True)
        elif self.doorname == 'D2':
            door_client('Door2',True)
        elif self.doorname == 'D3':
            door_client('Door3',True)
        elif self.doorname == 'D4':
            door_client('Door4',True)
        else:
            print("'Open' called with wrong parameter:", self.doorname)

    def setup(self):
        self.fgoal = "True"


class Close (Behavior):
    """
    Close a door by a given name, just calling the relevant ROS service
    """
    def __init__(self, door):
        self.doorname = door
        print("Instantiated Close behavior with door =", door)
        super().__init__()

    def update_state(self, state):
        if self.doorname == 'D1':
            door_client('Door1',False)
        elif self.doorname == 'D2':
            door_client('Door2',False)
        elif self.doorname == 'D3':
            door_client('Door3',False)
        elif self.doorname == 'D4':
            door_client('Door4',False)
        else:
            print("'Close' called with wrong parameter:", self.doorname)

    def setup(self):
        self.fgoal = "True"

class PickUp (Behavior):
    """
    Pick up a box by a given name, just calling the relevant ROS service.
    """
    def __init__(self, box):
        self.boxname = box
        print("Instantiated PickUp behavior with box =", box)
        super().__init__()

    def update_state(self, state):
        if self.boxname[:3] == 'box':
            success = box_pickup(self.boxname)
        else:
            print("'PickUp' called with wrong parameter:", self.boxname)

    def setup(self):
        self.fgoal = "True"

class PutDown (Behavior):
    """
    Put down a box by a given name, just calling the relevant ROS service.
    """
    def __init__(self, box):
        self.boxname = box
        
        print("Instantiated PutDown behavior with box =", box)
        super().__init__()

    def update_state(self, state):
        if self.boxname[:3] == 'box':
            success = box_putdown(self.boxname)
        else:
            print("'PutDown' called with wrong parameter:", self.boxname)

    def setup(self):
        self.fgoal = "True"

class Align (Behavior):
    """
    Align the robot to face a target point at (x,y,th,radius), where 'x,y' is the target's center
    in global frame, and 'radius' is its size.
    """
    def __init__(self, target = (0.0, 0.0, 0.0, 0.0)):
        self.target = target            # target point in global coordinates
        self.tlocal = [0, 0, 0]         # target point in robot's coordinates
        super().__init__()
        print("Instantiated Align behavior with target =", target)
    
    def update_state(self, state):
        # pose data state
        mypose = state['mypose']
        global_to_local(self.target, mypose, self.tlocal)
        xt = self.tlocal[0]
        yt = self.tlocal[1]
        self.state['phi'] = degrees(atan2(yt, xt))

    def setup(self):
        """
        Definition of the rules of our 'Align' fuzzy controller
        Fuzzy interpretation is built from the local state variable 'phi'
        """
        self.fpreds = {
            # Definition of the GoToTarget fuzzy predicates, used in the rules' LHS
            'TargetLeft'        : (ramp_up(15.0, 60.0), 'phi'),
            'TargetLeftAhead'   : (triangle(0.0, 15.0, 30.0), 'phi'),
            'TargetRight'       : (ramp_down(-60.0, -15.0), 'phi'),
            'TargetRightAhead'  : (triangle(-30.0, -15.0, 0.0), 'phi'),
            'TargetAhead'       : (triangle(-15.0, 0.0, 15.0), 'phi'),
        }

        self.flvars = {
            # Definition of the fuzzy linguistic variables, used in the rules' RHS
            'Move' : ({'Fast':0.5, 'Slow':0.2, 'None':0, 'Back':-0.1}, 'Vlin'),
            'Turn' : ({'Left':10, 'MLeft':5, 'None':0, 'MRight':-5, 'Right':-10}, 'Vrot')
        }

        self.frules = {
            # GoToTarget fuzzy rules
            'ToMLeft'       : ("TargetLeftAhead", 'Turn', 'MLeft'),
            'ToLeft'        : ("TargetLeft", 'Turn', 'Left'),
            'ToMRight'      : ("TargetRightAhead", 'Turn', 'MRight'),
            'ToRight'       : ("TargetRight", 'Turn', 'Right'),
        }

        # The degree of achievement is given by the predicate 'TargetAhead'
        self.fgoal = "TargetAhead"

        # Finally, initialize the fuzzy predicats and fuzzy output sets
        self.init_flsets()
        self.init_fpreds()


class CheckBox (Behavior):
    """
    Check the presence of a box at a given location using sonar data.
    """
    def __init__(self, box):
        self.boxname = box
        super().__init__()

    def update_state(self, state):
        """
        Update the relevant local state variables (self.state) at every control cycle.
        Uses the 'sdata' informations, passed as part of the state by the top-level loop.
        Set the local state variable sFront.
        """   
        sdata = state['sdata']
        self.state['sFront'] = sdata['0']
        self.state['sMLeft'] = sdata['1']
        self.state['sMRight'] = sdata['11']
        self.state['ObjectAhead'] = min(sdata['0'],sdata['1'],sdata['11'])
        print(self.state['ObjectAhead'])

    def setup(self):
        """
        Definition of the rules of our 'CheckBox' fuzzy controller
        """
        self.fpreds = {
            # Definition of the fuzzy predicates, used in the rules' LHS
            'BoxAhead'       : (ramp_down(0.4, 1.0), 'ObjectAhead'),
        }

        # The degree of achievement is given by the predicate 'BoxAhead'
        self.fgoal = 'BoxAhead'

        # Finally, initialize the fuzzy predicats and fuzzy output sets
        self.init_flsets()
        self.init_fpreds()

class CheckDoor (Behavior):
    """
    Check if a door is open or closed using sonar data.
    """
    def __init__(self, box):
        self.boxname = box
        super().__init__()

    def update_state(self, state):
        """
        Update the relevant local state variables (self.state) at every control cycle.
        Uses the 'sdata' informations, passed as part of the state by the top-level loop.
        Set the local state variables sFront, sMLeft, sMRight, sOpening.
        """  
        sdata = state['sdata']
        self.state['sFront'] = sdata['0']
        self.state['sMLeft'] = sdata['1']
        self.state['sMRight'] = sdata['11']
        self.state['sOpening'] = max(sdata['0'],sdata['1'],sdata['11'])

    def setup(self):
        """
        Definition of the rules of our 'CheckDoor' fuzzy controller
        """
        self.fpreds = {
            # Definition of the fuzzy predicates, used in the rules' LHS
            'DoorAhead'       : (ramp_down(1.5, 2.0), 'sOpening'),
        }

        # The degree of achievement is given by the predicate 'DoorAhead'
        self.fgoal = 'NOT(DoorAhead)'

        # Finally, initialize the fuzzy predicats and fuzzy output sets
        self.init_flsets()
        self.init_fpreds()

class SavePos (Behavior):
    """
    """
    def __init__(self, door):
        super().__init__()

    def setup(self):
        self.fgoal = "True"

    

class Controller ():
    def __init__(self):
        self.behavior = None        # top-level fuzzy behavior run by the controller
        self.achieved = 0.0         # level of achievement of current behavior
        self.vlin = 0.0             # current value for vlin control variable
        self.vrot = 0.0             # current value for vrot control variable

    def set_behavior (self, bname = None, bparam = None):
        """
        Initialize the controller, setting the behavior to be executed
        Return True if the inizialization is successful
        """
        from robot_map import map
        if bname:
            self.behavior = globals()[bname](bparam)
        return True

    def run (self, state, debug):
        """
        Action decision. Compute control values (vlin, vrot) given the current robot's pose
        by running the current behavior; return the level of achievement of that behavior
        """
        self.achieved = self.behavior.run(state, debug)
        self.vlin = self.behavior.get_vlin()
        self.vrot = self.behavior.get_vrot()
        if debug > 1:
            print('Goal achievement: {:.2f}'.format(self.achieved))
        if debug > 1:
            print('(vlin, vrot)) = ({:.2f}, {:.2f})'.format(self.vlin, degrees(self.vrot))) 
        return self.achieved

    def get_vlin (self):
        return self.vlin
    
    def get_vrot (self):
        return self.vrot
