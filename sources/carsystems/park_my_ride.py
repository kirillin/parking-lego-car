#!/usr/bin/env python3
from libs.Robots import LegoCar
from libs.TrajectoryStuff import Point, StraightLine


if __name__ == '__main__':
    car = LegoCar()

    trajectory_line = StraightLine(Point(0, 1), Point(4, 1), 0.2)
    car.move(trajectory=trajectory_line)
