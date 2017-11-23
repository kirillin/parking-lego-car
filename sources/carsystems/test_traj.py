

import time
from Point import Point
from TrajectoryStuff import StraightLine
from TrajectoryController import TrajectroryController

from Localization import Localization

from Robots import LegoCar

if __name__ == '__main__':

    trajectory_controller = TrajectroryController(0)
    traj_line = StraightLine(Point(1, 2), Point(3, 4), 0.3)

    localization = Localization(LegoCar)

    start_time = time.time()
    t = time.time() - start_time
    while True:
        try:
            last_time = t
            t = time.time() - start_time
            dt = t - last_time  # invalid within first iteration but it's ok

            point_on_line = traj_line.getCoordinates(t)
            trajectory_controller.getControls(point_on_line, x, y, dx, dy, theta, dt)


        except KeyboardInterrupt:
            break

