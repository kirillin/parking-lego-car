#!/usr/bin/env python3
"""
    Test module for moving car-robot for trajectories:
    1) straight line trajectory
    2) moving to point
"""
import os, time
from math import pi

from ev3dev.ev3 import Button, GyroSensor, UltrasonicSensor, LargeMotor, OUTPUT_A, OUTPUT_B

from libs.Robots import LegoCar
from libs.Localization import Localization
from libs.VelocityController import VelocityController

from libs.Point import Point
from libs.TrajectoryStuff import OnlyPoint, StraightLine
from libs.TrajectoryController import TrajectroryController


if __name__ == '__main__':

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
    # sometimes don't work :D
    gyro_sensor.mode = gyro_sensor.modes[1]
    gyro_sensor.mode = gyro_sensor.modes[0]

    localization = Localization(LegoCar)
    velocity_controller = VelocityController(LegoCar, -0.3, 0.3)
    trajectory_controller = TrajectroryController(0.1)

    traj_point = OnlyPoint(Point(3, -1))
    traj_line = StraightLine(Point(0, 1), Point(4, 1), 0.2)

    os.system("beep -f 440 -l 42")

    start_time = time.time()
    t = time.time() - start_time

    while True:
        try:
            last_time = t
            t = time.time() - start_time
            dt = t - last_time  # invalid within first iteration but it's ok

            theta, omega = [-x for x in gyro_sensor.rate_and_angle]  # !!! returns ANGLE AND RATE :)

            x, y, vx, vy = localization.getData(theta * pi / 180, rear_motor.speed * pi / 180, dt)

            # point = traj_line.getCoordinates(t)
            point = traj_point.getCoordintes(t)

            v_des, omega_des = trajectory_controller.getControls(point, x, y, vx, vy, theta * pi / 180, dt)
            velocity_controller.setTargetVelocities(v_des, omega_des)

            u_v, u_phi = velocity_controller.getControls(rear_motor.speed * pi / 180.0,
                                                         steer_motor.position * pi / 180,
                                                         omega * pi / 180, dt)
            rear_motor.run_direct(duty_cycle_sp=u_v)
            steer_motor.run_direct(duty_cycle_sp=u_phi)

            if halt.enter:
                break

        except KeyboardInterrupt:
            break

    # off motors and close file
    rear_motor.duty_cycle_sp = 0
    steer_motor.duty_cycle_sp = 0
    raise SystemExit