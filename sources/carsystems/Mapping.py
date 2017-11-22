# coding: utf-8
from math import cos, sin, hypot


def matmult(a, b):
    res = list([0,0,0,0])
    for i in range(len(a)):
        for j in range(len(a)):
            res[i] += a[i][j] * b[j]
    return res


class Mapping:

    def __init__(self, car, us_sensor_front, us_sensor_rear):
        self.car = car
        self.us_front = us_sensor_front
        self.us_rear = us_sensor_rear

        self.the_map = {
            'obstacles': {'x': [], 'y': []}
        }

        self.file_map_f = open('map_raw_front.txt', 'w')
        self.file_map_f_rf = open('map_raw_front_robot_frame.txt', 'w')


        self.file_map_r = open('map_raw_rear.txt', 'w')

    def __delete__(self, instance):
        self.file_map_f.close()
        self.file_map_f_rf.close()

        self.file_map_r.close()

    def updateMap(self, x_current, y_current, theta):

        H = [[cos(theta), sin(theta),   0,  x_current * cos(theta) + y_current * sin(theta)],
             [-sin(theta), cos(theta),  0,  - x_current * sin(theta) + y_current * cos(theta)],
             [0, 0, 1, 0],
             [0, 0, 0, 1]]

        # cm to meters
        dist_front = self.us_front.distance_centimeters * 0.001
        dist_rear = self.us_rear.distance_centimeters * 0.001

        if dist_front < 0.15:
            self.file_map_f.write("{} {}\n".format(x_current, dist_front))

            point_front = [self.car.US_FRONT[0],
                           -(dist_front + self.car.US_FRONT[1]),
                           0,
                           1]
            # точка с дальномеров в нулевой системе координат
            point_front_0 = matmult(H, point_front)
            self.the_map['obstacles']['x'].append(point_front_0[0])
            self.the_map['obstacles']['y'].append(point_front_0[1])


        # if dist_rear < 0.15:
        #     self.file_map_r.write("{} {}\n".format(x_current, dist_rear))
        #
        #     point_rear = [self.car.US_BACK[0],
        #                   -(dist_rear + self.car.US_BACK[1]),
        #                   0,
        #                   1]
        #
        #     point_rear_0 = matmult(H, point_rear)
        #     self.the_map['obstacles']['x'].append(point_rear_0[0])
        #     self.the_map['obstacles']['y'].append(point_rear_0[1])
