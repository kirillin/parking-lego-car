#!/usr/bin/env python3
# coding: utf-8
import os
import time
from math import pi, tan, cos, sin
from ev3dev.auto import *

from AngleSensor import AngleSensor
from sources.carsystems.Robots import LegoCar
from Odometry import Odometry


def matmult(a, b):
    res = list([0,0,0,0])
    for i in range(len(a)):
        for j in range(len(a)):
            res[i] += a[i][j] * b[j]
    return res


if __name__ == '__main__':

    halt = Button()
    angle = AngleSensor()   # angle of left wheel
    sensor_us_front = UltrasonicSensor('in2')
    sensor_us_back = UltrasonicSensor('in3')

    steer_motor = MediumMotor(OUTPUT_B)
    motor = LargeMotor(OUTPUT_A)

    steer_motor.reset()
    motor.reset()

    odom = Odometry(LegoCar)

    file_map = open('map_1.txt', 'w')
    os.system("beep -f 440 -l 10")

    start_time = time.time()
    last_time = 0
    while True:
        try:
            cur_time = time.time() - start_time
            dt = cur_time - last_time if last_time != 0 else 0
            last_time = cur_time

            phi_1 = angle.getAngle() * pi / 180

            x, y, theta, vx, vy = odom.getData(phi_1, motor.speed, dt)

            motor.run_direct(duty_cycle_sp=-100)

            H = [[cos(theta), sin(theta), 0, x * cos(theta) + y * sin(theta)],
                 [-sin(theta), cos(theta), 0, - x * sin(theta) + y * cos(theta)],
                 [0, 0, 1, 0],
                 [0, 0, 0, 1]]
            # точки, которые возвращают дальномеры
            point_front = [LegoCar.US_FRONT[0],
                           -(sensor_us_front.distance_centimeters / 100 + LegoCar.US_FRONT[1]),
                           0,
                           1]
            point_back = [LegoCar.US_BACK[0],
                          -(sensor_us_back.distance_centimeters / 100 + LegoCar.US_BACK[1]),
                          0,
                          1]

            # точки с дальномеров в системе координат машинки
            point_front_0 = matmult(H, point_front)
            point_back_0 = matmult(H, point_back)

            dist_data = "{:.4f} {:.4f}\n".format(point_front_0[0], point_front_0[1])
            # dist_data = "{:.4f} {:.4f}\n".format(point_front[0], point_front[1])
            file_map.write(dist_data)

            if halt.enter:
                break

        except KeyboardInterrupt:
            break

    file_map.close()

    motor.duty_cycle_sp = 0
    raise SystemExit
