#!/usr/bin/env python3
import toplevel


if __name__ == '__main__':
    toplevel = toplevel.TopLevelLoop(mode ='ROTATE_AND_MODE', goal = (2.0, 2.0), debug = 0, plot_positions = 1)
    toplevel.run()
