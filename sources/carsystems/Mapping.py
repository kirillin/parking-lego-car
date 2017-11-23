# coding: utf-8
from math import cos, sin, hypot
from Point import Point


def matmult(a, b):
    res = list([0, 0, 0, 0])
    for i in range(len(a)):
        for j in range(len(a)):
            res[i] += a[i][j] * b[j]
    return res


def mean(numbers):
    return float(sum(numbers)) / max(len(numbers), 1)


def meanFilter(self, raw_data):
    well_data = []
    mean_data = mean([point.y for point in raw_data])
    self.mean = mean_data
    for point in raw_data:
        if point.y > mean_data:
            well_data.append(Point(point.x, point.y))
    return well_data


class Mapping:

    def __init__(self, car, us_sensor_front, us_sensor_rear):
        self.car = car
        self.us_front = us_sensor_front
        self.us_rear = us_sensor_rear
        self.max_view_distanse = 0.5

        self.the_map = {
            'obstacles': {'front': [], 'rear': []}
        }
        self.mean = 0


    def updateMap(self, x_current, y_current, theta):
        #
        # H = [[cos(theta), sin(theta),   0,  x_current * cos(theta) + y_current * sin(theta)],
        #      [-sin(theta), cos(theta),  0,  - x_current * sin(theta) + y_current * cos(theta)],
        #      [0, 0, 1, 0],
        #      [0, 0, 0, 1]]


        H = [[cos(theta), -sin(theta),   0,  x_current],
             [sin(theta), cos(theta),  0,  y_current],
             [0, 0, 1, 0],
             [0, 0, 0, 1]]

        # cm to meters
        dist_front = self.us_front.distance_centimeters * 0.01
        dist_rear = self.us_rear.distance_centimeters * 0.01

        if dist_front < self.max_view_distanse:
            point_front = [self.car.US_FRONT[0],
                           -(dist_front + self.car.US_FRONT[1]),
                           0,
                           1]
            # точка с дальномера в нулевой системе координат
            point_front_0 = matmult(H, point_front)
            self.the_map['obstacles']['front'].append(Point(point_front_0[0], point_front_0[1]))

        if dist_rear < self.max_view_distanse:
            point_rear = [self.car.US_BACK[0],
                          -(dist_rear + self.car.US_BACK[1]),
                          0,
                          1]
            point_rear_0 = matmult(H, point_rear)
            self.the_map['obstacles']['rear'].append(Point(point_rear_0[0], point_rear_0[1]))

    def filterMap(self):
        print("Quantity points with noize: ".format(self.the_map['obstacles']['front'] + self.the_map['obstacles']['rear']))
        self.the_map['obstacles']['front'] = meanFilter(self.the_map['obstacles']['front'])
        self.the_map['obstacles']['rear'] = meanFilter(self.the_map['obstacles']['rear'])
        print("Quantity points without noize: ".format(self.the_map['obstacles']['front'] + self.the_map['obstacles']['rear']))

    def findParkingPlace(self):
        parking_place = []
        prev_point = self.the_map['obstacles']['front'][0]
        for point in self.the_map['obstacles']['front']:
            if abs(point.x - prev_point.x) > 0.3:
                parking_place.append(prev_point)
                parking_place.append(point)
                break
        return parking_place

    def getBadPoints(self, points):
        bad_points = []
        n = len(points)
        for i in range(n-1):
            pi = points[i]
            min_dist = 999999
            for j in range(i+1, n):
                pj = points[j]
                dist = hypot(pj.x - pi.x, pj.y - pi.y)
                if  dist < min_dist:
                    min_dist = dist
            if min_dist > 0.02:
                bad_points.append(pi)
        return bad_points

    def compressMap(self):
        self.the_map['obstacles']['front'] = list(set(self.the_map['obstacles']['front']).difference(set(self.getBadPoints(self.the_map['obstacles']['front']))))
        self.the_map['obstacles']['rear'] = list(set(self.the_map['obstacles']['rear']).difference(
            set(self.getBadPoints(self.the_map['obstacles']['rear']))))
