import math


class Point:

    def __init__(self, x, y):
        self.x = x
        self.y = y


class TrajectoryPoint:

    def __init__(self, x, dx, ddx, y, dy, ddy):
        self.x, self.dx, self.ddx = x, dx, ddx
        self.y, self.dy, self.ddy = y, dy, ddy

    def getPoint(self):
        return self.x, self.dx, self.ddx, self.y, self.dy, self.ddy


class TrajectoryLine:

    def __init__(self):
        self.is_end = False

    def isEnd(self):
        return self.is_end


class StraightLine(TrajectoryLine):

    def __init__(self, point_0, point_1, v):
        self.is_end = False
        self.point_0, self.point_1 = point_0, point_1
        self.end_time = math.sqrt((point_1.y - point_0.y)**2 + (point_1.x - point_0.x)**2) / v
        self.gamma = math.atan2(point_1.y - point_0.y,
                                point_1.x - point_0.x)
        self.vx = v * math.cos(self.gamma)
        self.vy = v * math.sin(self.gamma)

    def getCoordinates(self, t):
        if t > self.end_time:
            self.is_end = True
            return TrajectoryPoint(self.point_1.x, 0.0, 0.0, self.point_1.y, 0.0, 0.0)
        else:
            x_r = self.vx * t + self.point_0.x
            y_r = self.vy * t + self.point_0.y
            return TrajectoryPoint(x_r, self.vx, 0.0, y_r, self.vy, 0.0)


class CircleLine(TrajectoryLine):

    def __init__(self, point_c, point_0, point_1, v):
        self.is_end = False
        self.point_c, self.point_0, self.point_1 = point_c, point_0, point_1
        self.angle_0 = math.atan2(point_0.y - point_c.y, point_0.x - point_c.x)
        angle_1 = math.atan2(point_1.y - point_c.y, point_1.x - point_c.x)
        delta_angle = angle_1 - self.angle_0
        if delta_angle > math.pi:
            delta_angle = delta_angle - 2 * math.pi;
        elif delta_angle < -math.pi:
            delta_angle = delta_angle + 2 * math.pi;
        self.R = math.sqrt((point_0.y - point_c.y)**2 + (point_0.x - point_c.x)**2)
        self.omega = math.copysign(1, delta_angle) * v / self.R
        self.end_time = abs(delta_angle * self.R) / v

    def getCoordinates(self, t):
        if t > self.end_time:
            self.is_end = True
            return TrajectoryPoint(self.point_1.x, 0.0, 0.0, self.point_1.y, 0.0, 0.0)
        else:
            phase = self.angle_0 + self.omega * t
            x_r = self.point_c.x + self.R * math.cos(phase)
            y_r = self.point_c.y + self.R * math.sin(phase)
            vx_r = - self.R * self.omega * math.sin(phase)
            vy_r = self.R * self.omega * math.cos(phase)
            ax_r = - self.R * self.omega**2 * math.cos(phase)
            ay_r = - self.R * self.omega**2 * math.sin(phase)
            return TrajectoryPoint(x_r, vx_r, ax_r, y_r, vy_r, ay_r)


class ParkingLine(TrajectoryLine):

    def __init__(self, point_1, point_l, point_r, radius, depth, delta_1, delta_2, v):

        # some obvious calculations
        point_4 = Point(point_r.x - delta_2, point_r.y + delta_2)
        point_6 = Point(point_l.x + delta_1, point_l.y - depth)
        point_7 = Point(point_6.x, point_6.y + radius)

        # some other calculations
        point_5 = findPoint5(point_7, point_4, radius)
        point_8 = findPoint8(point_4, point_5, radius)
        point_2 = Point(point_8.x, point_1.y);
        point_3 = Point(point_8.x - radius * math.cos(alpha), point_8.y + radius * math.sin(alpha))

        # pieces definition
        self.horiz_line = StraightLine(point_1, point_2, v)
        self.arc_first = CircleLine(point_8, point_2, point_3, v)
        self.inclin_lin = StraightLine(point_3, point_5, v)
        self.arc_final = CircleLine(point_7, point_5, point_6, v)

        self.end_time = sum([x.end_time for x in [self.horiz_line, self.arc_first, self.inclin_line]])


    def findPoint5(point_7, point_4, radius):
        """It finds coordinates of point_5 using Newton's method"""
        e = 1
        alpha = pi/4
        while abs(e) > 0.001:
            deriv_e = -(point_4.x - point_7.x) / math.cos(alpha)**2 + radius * math.sin(alpha) / math.cos(alpha)**2
            e = (point_4.y - point_7.y) - (point_4.x - point_7.x) * math.tan(alpha) + radius * math.cos(alpha) + radius * math.tan(alpha) * math.sin(alpha)
            alpha -= e / deriv_e
        return Point(point_7.x + radius * math.sin(alpha), point_7.y - radius * math.cos(alpha))


    def findPoint8(point_1, point_4, point_5, radius):
        k = (point_4.y - point_5.y) / (point_4.x - point_5.x)
        alpha = math.atan(k)
        b = point_4.y - k * point_4.x
        h = radius / math.cos(alpha)
        return Point(1.0 / k * (point_1.y - radius - b + h), point_1.y - radius)


    def getCoordinates(self, t):
        if t > self.end_time:
            self.is_end = True
            return TrajectoryPoint(self.point_1.x, 0.0, 0.0, self.point_1.y, 0.0, 0.0)
        elif t > self.inclin_lin.end_time:
            return self.arc_final.getCoordinates(t - self.inclin_lin.end_time)
        elif t > self.arc_first.end_time:
            return self.inclin_lin.getCoordinates(t - self.arc_first.end_time)
        elif t > self.horiz_line.end_time:
            return self.arc_first.getCoordinates(t - self.horiz_line.end_time)
        else:
            return self.horiz_line.getCoordinates(t - self.horiz_line.end_time)
