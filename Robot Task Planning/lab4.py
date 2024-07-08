#!/usr/bin/env python3
import toplevel, pyhop, htn_domain


if __name__ == '__main__':
    # Define and solve some planning problems

    # pyhop.print_operators()
    # pyhop.print_methods()

    state1 = htn_domain.State()

    state1.room['bed1'] = 'Room1'
    state1.room['wardrobe'] = 'Room1'
    state1.room['fridge'] = 'Room2'
    state1.room['stove'] = 'Room2'
    state1.room['sink'] = 'Room2'
    state1.room['table1'] = 'Room2'
    state1.room['table2'] = 'Room4'
    state1.room['table3'] = 'Room4'
    state1.room['box1'] = 'Room1'
    state1.room['box2'] = 'Room2'
    state1.room['box3'] = 'Room3'
    state1.room['box4'] = 'Room4'

    state1.connects['D1'] = ('Room3', 'Room4', 'Open')
    state1.connects['D2'] = ('Room1', 'Room4', 'Open')
    state1.connects['D3'] = ('Room1', 'Room3', 'Open')
    state1.connects['D4'] = ('Room2', 'Room3', 'Open')

    state1.visited = set()
    state1.path = []
    state1.carrying = 'Empty'

    state1.pos['me'] = 'table2'
    state1.room['me'] = 'Room4'



    print("Initial state:")
    pyhop.print_state(state1)

    # myplan = pyhop.pyhop(state1, [('navigate_to', 'table2')], verbose=2)
    # myplan = pyhop.pyhop(state1, [('navigate_to', 'table3')], verbose=2)
    # myplan = pyhop.pyhop(state1, [('navigate_to', 'bed1')], verbose=2)
    myplan = pyhop.pyhop(state1, [('navigate_to', 'stove')], verbose=2)
    # myplan = pyhop.pyhop(state1, [('fetch', 'box1')], verbose = 2)
    # myplan = pyhop.pyhop(state1, [('transport', 'box1', 'table2')], verbose = 2)

    print('')
    print("Plan:", myplan)


    toplevel = toplevel.TopLevelLoop(goal = (8.0, -4.0), tcycle = 0.1, debug = 0, plan = myplan)
    toplevel.run_plan()
