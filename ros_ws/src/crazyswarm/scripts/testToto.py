#!/usr/bin/env python

from __future__ import print_function

from pycrazyswarm import *

import time
import numpy as np

def main():
    swarm = Crazyswarm()
    timeHelper = swarm.timeHelper
    allcfs = swarm.allcfs
    cf1 = allcfs.crazyflies[0]
    cf1.takeoff(0.1, 0.5)
    time.sleep(1)

    targets = [0.2, 0.3, 0.5, 1.0, 0.75, 0.4, 0.2]

    for target in targets:
        goal = np.array(cf1.initialPosition) + np.array([0, 0, target])
        cf1.goTo([0, 0.0, 0.1], 0, 1.5)
        time.sleep(3)

    cf1.land(0.1, 0.5)


if __name__ == "__main__":
    main()
