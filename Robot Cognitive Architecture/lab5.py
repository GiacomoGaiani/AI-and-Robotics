#!/usr/bin/env python3
import toplevel, pyhop, htn_domain
from math import sqrt


if __name__ == '__main__':
    toplevel = toplevel.TopLevelLoop(tcycle = 0.1, debug = 1)
    toplevel.run(maxsteps = 0, goal = [('transport', 'box3', 'wardrobe1')])
