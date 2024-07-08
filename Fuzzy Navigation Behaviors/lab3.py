#!/usr/bin/env python3
import toplevel


if __name__ == '__main__':
    toplevel = toplevel.TopLevelLoop(goal = (8.0, -4.0), tcycle = 0.1, debug = 2)
    toplevel.run(maxsteps = 0)
