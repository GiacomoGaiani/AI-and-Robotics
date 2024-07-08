"""
============== UniBo: AI and Robotics 2024 ==============
Base code: HTN planning domain for Pyhop planner
To be customized in an incremental way for the different labs

(c) 2024 Alessandro Saffiotti
"""
import pyhop, robot_map

class State(pyhop.State):
    """
    This is a minimal basis, you may want to add more state variables depending on your needs
    """
    def __init__(self):
        self.__name__ = "s1"
        self.pos = {}           # position of robot or objet: a symbolic name
        self.room = {}          # room of robot or objet
        self.door = {}          # doors' status: closed or open
        self.connects = {}      # doors' connectivity: pair of rooms
        self.arm = None         # arm state: empty or carrying
        self.path = None        # path ot reach the target
        self.visited = None     # list of visited rooms

###############################################################################
# OPERATORS
# First argument is current state, others are the operator's parameters.
###############################################################################

def GoTo (state, target):
    state.pos['me'] = target
    return state

def Cross (state, door):
    if (state.pos['me'] != door):
        return False
    if (state.room['me'] == state.connects[door][0]):
        state.room['me'] = state.connects[door][1]
        return state
    if (state.room['me'] == state.connects[door][1]):
        state.room['me'] = state.connects[door][0]
        return state
    return False

def Open (state, door):
    if (state.pos['me'] != door):
        return False
    if (state.room['me'] == state.connects[door][0]) or  (state.room['me'] == state.connects[door][1]):
        if robot_map.map.properties[door] == 'close':
            robot_map.map.properties[door] = 'open'
            return state
        return state
    return False

def Close (state, door):
    if (state.pos['me'] != door):
        return False
    if (state.room['me'] == state.connects[door][0]) or  (state.room['me'] == state.connects[door][1]):
        if robot_map.map.properties[door] == 'open':
            robot_map.map.properties[door] = 'close'
            return state
        return state
    return False

def PickUp (state, obj):
    if (state.room['me'] == state.room[obj]) and (state.pos['me'][0] == 'D'):
        state.carrying = obj
        return state
    elif (state.pos['me'] != obj) or (state.carrying != 'Empty'):
        return False
    else:
        state.carrying = obj
        return state

def PutDown (state, target):
    if state.carrying != 'Empty':
        state.room[state.carrying] = state.room['me']
        state.carrying = 'Empty'
        return state
    return False

def Align (state, target):
    return state

def CheckBox (state, target):
    return state

def CheckDoor (state, target):
    return state

def SavePos (state, target):
    return state


pyhop.declare_operators(GoTo, Cross, Open, Close)
pyhop.declare_operators(PickUp, PutDown)
pyhop.declare_operators(Align, CheckBox, CheckDoor, SavePos)



###############################################################################
# METHODS
# First argument is current state, others are the method's parameters.
# They may call other methods, or executable operators.
###############################################################################

# Method to navigate when we are already at the target

def move_in_place (state, target):
    if state.pos['me'] == target:
        return []
    else:
        return False

# Method to navigate when the target is in the same room

def move_in_room(state, target):
    if state.room['me'] == state.room[target]:
        return [('GoTo', target)]
    else:
        return False

# Helper function to find connecting doors

def doors_between(state, room1, room2):
    queue = [(room1, [])]

    while queue:
        current_room, path = queue.pop(0)
        state.visited.add(current_room)
        if current_room == room2:
            return path

        for door, (r1, r2) in state.connects.items():
            if r1 == current_room and r2 not in state.visited:
                queue.append((r2, path + [door]))
            elif r2 == current_room and r1 not in state.visited:
                queue.append((r1, path + [door]))

    return []

# Method to navigate when the target is in an adjacent room

def move_across_rooms (state, target):
    room1 = state.room['me']
    room2 = state.room[target]
    if room1 == room2:
        return False
    if state.path == []:
        state.visited = set()
        state.path = doors_between(state, room1, room2)
        if state.path == []:
            return False
    door = state.path[0]
    state.path.pop(0)
    if state.door[door] == 'open':
        return [('GoTo', door), ('Align', door), ('CheckDoor', door), ('Cross', door), ('navigate_to', target)]
    elif state.door[door] == 'close':
        return [('GoTo', door), ('Align', door), ('Open', door), ('CheckDoor', door), ('Cross', door), ('navigate_to', target)] # Leave the door open after crossing it

def move_in_room_to_door(state, door):
    room1 = state.room['me']
    room2, room3 = state.connects[door]
    if room1 == room2 or room1 == room3:
        return [('GoTo', door)]
    else:
        return False

def move_across_rooms_to_door (state, target):
    room1 = state.room['me']
    room2, room3 = state.connects[target]
    if room1 == room2:
        return False
    if room1 == room3:
        return False
    if state.path == []:
        state.visited = set()
        path1 = doors_between(state, room1, room2)
        state.visited = set()
        path2 = doors_between(state, room1, room3)
        state.path = path1 if len(path1) < len(path2) else path2
        if state.path == []:
            return False
    door = state.path[0]
    state.path.pop(0)
    if state.door[door] == 'open':
        return [('GoTo', door), ('Align', door), ('CheckDoor', door), ('Cross', door), ('navigate_to_door', target)]
    elif state.door[door] == 'close':
        return [('GoTo', door), ('Align', door), ('Open', door), ('CheckDoor', door), ('Cross', door), ('navigate_to_door', target)]

def move_and_pickup (state, target):
    return [('navigate_to', target), ('CheckBox', target), ('PickUp', target)]

def move_and_putdown (state, target):
    return [('navigate_to', target), ('PutDown', state.carrying)]

def transport_obj (state, obj, target):
    return [('fetch', obj), ('carry_to', target)]

def calibrate_pos (state, door):
    return [('navigate_to_door', door), ('Align', door), ('Cross', door), ('SavePos', door), ('Align', door), ('Cross', door), ('SavePos', door)]


pyhop.declare_methods('navigate_to',  move_in_place, move_in_room, move_across_rooms)
pyhop.declare_methods('navigate_to_door',  move_in_place, move_in_room_to_door, move_across_rooms_to_door)
pyhop.declare_methods('fetch', move_and_pickup)
pyhop.declare_methods('carry_to', move_and_putdown)

pyhop.declare_methods('transport', transport_obj)
pyhop.declare_methods('calibrate', calibrate_pos)