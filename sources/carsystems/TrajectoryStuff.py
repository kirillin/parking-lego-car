import math
from Point import TrajectoryPoint


# class TrajectoryGenerator:
#
#     def __init__(self, trajectory):
#         self.trajectory = trajectory
#
#     def getCoordinates(self, t):
#         return trajectory.getCoordinates(t)

class OnlyPoint:

    def __init__(self, point):
        self.point = point

    def getCoordintes(self, t):
        return TrajectoryPoint(self.point.x, 0, 0, self.point.y, 0, 0)


class StraightLine:

    def __init__(self, point_0, point_1, v):
        self.point_0 = point_0
        self.point_1 = point_1
        self.v = v
        self.gamma = math.atan2(point_1.y - point_0.y,
                                point_1.x - point_0.x)

    def getCoordinates(self, t):
        x_r = self.v * math.cos(self.gamma) * t + self.point_0.x
        vx_r = self.v * math.cos(self.gamma)
        ax_r = 0
        y_r = self.v * math.sin(self.gamma) * t + self.point_0.y
        vy_r = self.v * math.sin(self.gamma)
        ay_r = 0
        return TrajectoryPoint(x_r, vx_r, ax_r, y_r, vy_r, ay_r)


class CircleLine:

    def __init__(self, point_0, point_1, R, v):
        self.point_0, self.point_1 = point_0, point_1
        self.R = R
        self.v = v

    def getCoordinates(self, t):
        alpha = self.v * t / self.R
        x = self.point_0.x + self.R * math.sin(alpha)


class ParkingLine:

    def __init__(self, x0, y0, x1, y1, v):
        self.x0, self.y0 = x0, y0
        self.x1, self.y1 = x1, y1
        self.v = v

    def getCoordinates(self, t):
        pass
