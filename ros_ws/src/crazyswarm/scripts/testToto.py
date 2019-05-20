#!/usr/bin/env python

from __future__ import print_function

from pycrazyswarm import *

import time

def main():
    swarm = Crazyswarm()
    timeHelper = swarm.timeHelper
    allcfs = swarm.allcfs
    cf1 = allcfs.crazyflies[0]
    #cf1.takeoff(0.1, 0.5)
    #time.sleep(1)

    targets = [0.1, 0.2, 0.5, 1.0, 0.75, 0.4]

    for pos in targets:
        print("Goal:", pos)
        cf1.goTo([0, 0.5, 0.1], 0, 0,5)
        time.sleep(1)

    cf1.land(0.04, 0.5)


if __name__ == "__main__":
    main()
