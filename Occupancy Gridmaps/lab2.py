#!/usr/bin/env python3
import toplevel


if __name__ == '__main__':
    toplevel = toplevel.TopLevelLoop(goal = (2.0, 1.0), tcycle = 0.1, debug = 0, gridmap_mode = 'COMP_GRID_FROM_DATA')
    toplevel.run(maxsteps = 500)
