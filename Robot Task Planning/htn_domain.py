"""
============== UniBo: AI and Robotics 2024 ==============
Base code: HTN planning domain for Pyhop planner
To be customized in an incremental way for the different labs

(c) 2024 Alessandro Saffiotti
"""
import pyhop

class State(pyhop.State):
    def __init__(self):
        self.__name__ = "s1"
        self.pos = {}           # named position of robot ('me') or of an objet
        self.room = {}          # room of robot or of an objet
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
        if state.connects[door][2] == 'Close':
            state.connects[door][2] == 'Open'
            return state
        return state
    return False

def Close (state, door):
    if (state.pos['me'] != door):
        return False
    if (state.room['me'] == state.connects[door][0]) or  (state.room['me'] == state.connects[door][1]):
        if state.connects[door][2] == 'Open':
            state.connects[door][2] == 'Close'
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


pyhop.declare_operators(GoTo, Cross, Open, Close)
pyhop.declare_operators(PickUp, PutDown)


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

        for door, (r1, r2, _) in state.connects.items():
            if r1 == current_room and r2 not in state.visited:
                queue.append((r2, path + [door]))
            elif r2 == current_room and r1 not in state.visited:
                queue.append((r1, path + [door]))

    return []


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
    if state.connects[door][2] == 'Open':
        return [('GoTo', door), ('Cross', door), ('navigate_to', target)]
    elif state.connects[door][2] == 'Close':
        if state.carrying == 'Empty':
            return [('GoTo', door), ('Open', door), ('Cross', door), ('Close', door), ('navigate_to', target)]
        else:
            obj = state.carrying
            return [('GoTo', door), ('PutDown', door), ('Open', door), ('PickUp', obj), ('Cross', door), ('PutDown', door), ('Close', door), ('PickUp', obj), ('navigate_to', target)]

def move_and_pickup (state, target):
    return [('navigate_to', target), ('PickUp', target)]

def move_and_putdown (state, target):
    return [('navigate_to', target), ('PutDown', state.carrying)]

def transport_obj (state, obj, target):
    return [('fetch', obj), ('carry_to', target)]


pyhop.declare_methods('navigate_to',  move_in_place, move_in_room, move_across_rooms)
pyhop.declare_methods('fetch', move_and_pickup)
pyhop.declare_methods('carry_to', move_and_putdown)

pyhop.declare_methods('transport', transport_obj)
