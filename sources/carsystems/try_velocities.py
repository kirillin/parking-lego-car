#!/usr/bin/env python3
from ev3dev.auto import *

import os
import time
from math import pi

from Robots import LegoCar
from VelocityController import VelocityController
from Localization import Localization
# from Mapping import Mapping
from Point import Point
from TrajectoryStuff import StraightLine
from TrajectoryController import TrajectroryController

if __name__ == "__main__":

    # initialization of devices
    halt = Button()
    gyro_sensor = GyroSensor('in1')
    sensor_us_front = UltrasonicSensor('in2')
    sensor_us_rear = UltrasonicSensor('in3')

    rear_motor = LargeMotor(OUTPUT_A)
    steer_motor = LargeMotor(OUTPUT_B)
    rear_motor.reset()
    steer_motor.reset()
    # trick for reset gyro sensor :)
    gyro_sensor.mode = gyro_sensor.modes[1]
    gyro_sensor.mode = gyro_sensor.modes[0]


    # initialization of controllers
    velocity_controller = VelocityController(LegoCar, -0.3, 0.3)

    # initialization of localization
    # localization = Localization(LegoCar)
    # mapping = Mapping(LegoCar, sensor_us_front, sensor_us_rear)

    trajectory_controller = TrajectroryController(0)
    traj_line = StraightLine(Point(1, 2), Point(3, 4), 0.3)

    os.system("beep -f 440 -l 42")
    # file_localization = open('localization.txt', 'w')   # time, x, t, theta

    file_with_speed = open('file_with_speed.txt', 'w')

    start_time = time.time()
    t = time.time() - start_time
    while True:
        try:
            last_time = t
            t = time.time() - start_time
            dt = t - last_time  # invalid within first iteration but it's ok

            theta, omega = [-x for x in gyro_sensor.rate_and_angle]   # !!! returns ANGLE AND RATE :)

            # mapping
            # x, y, vx, vy = localization.getData(theta * pi / 180, rear_motor.speed * pi / 180, dt)
            # mapping.updateMap(x, y, theta * pi / 180)

            # file_localization.write("{} {} {} {}\n".format(t, x, y, theta * pi / 180))

            # point_on_line = traj_line.getCoordinates(t)
            # v_des, omega_des = trajectory_controller.getControls(point_on_line, x, y, vx, vy, theta, dt)
            #
            # velocity_controller.setTargetVelocities(v_des, omega_des)

            # moving forward
            u_v, u_phi = velocity_controller.getControls(rear_motor.speed * pi / 180.0,
                                                        steer_motor.position * pi / 180,
                                                        omega * pi / 180, dt)
            rear_motor.run_direct(duty_cycle_sp=u_v)
            steer_motor.run_direct(duty_cycle_sp=u_phi)

            file_with_speed.write("{} {} {}\n".format(t, omega * pi / 180, rear_motor.speed))

            if halt.enter:
                break

        except KeyboardInterrupt:
            break

    file_with_speed.close()
    # file_localization.close()
    #
    # obstacles = mapping.the_map['obstacles']
    # x = obstacles['x']
    # y = obstacles['y']
    #
    # file_map = open('map.txt', 'w')
    # for i in range(len(x)):
    #     file_map.write("{} {}\n".format(x[i], y[i]))
    # file_map.close()

    # off motors and close file
    rear_motor.duty_cycle_sp = 0
    steer_motor.duty_cycle_sp = 0
    raise SystemExit
