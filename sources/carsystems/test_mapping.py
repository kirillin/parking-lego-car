#!/usr/bin/env python3
"""
    Module shows you how to work with mapping for car-robot.
"""
import os
import time
from math import pi

from ev3dev.ev3 import Button, GyroSensor, UltrasonicSensor, LargeMotor, OUTPUT_A, OUTPUT_B

from libs.Robots import LegoCar
from libs.Localization import Localization
from libs.Mapping import Mapping
from libs.VelocityController import VelocityController


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
    # sometimes don't work :D
    gyro_sensor.mode = gyro_sensor.modes[1]
    gyro_sensor.mode = gyro_sensor.modes[0]

    localization = Localization(LegoCar)
    mapping = Mapping(LegoCar, sensor_us_front, sensor_us_rear)
    velocity_controller = VelocityController(LegoCar, 0.3, 0)

    os.system("beep -f 440 -l 42")
    file_localization = open('logs/localization.txt', 'w')   # time, x, y, theta

    start_time = time.time()
    t = time.time() - start_time
    while True:
        try:
            last_time = t
            t = time.time() - start_time
            dt = t - last_time  # invalid within first iteration but it's ok

            theta, omega = [-x for x in gyro_sensor.rate_and_angle]   # !!! returns ANGLE AND RATE :)

            # moving forward
            u_v, u_phi = velocity_controller.getControls(rear_motor.speed * pi / 180.0,
                                                        steer_motor.position * pi / 180,
                                                        omega * pi / 180, dt)
            rear_motor.run_direct(duty_cycle_sp=u_v)
            steer_motor.run_direct(duty_cycle_sp=u_phi)

            # mapping
            x, y, vx, vy = localization.getData(theta * pi / 180, rear_motor.speed * pi / 180, dt)
            mapping.updateMap(x, y, theta * pi / 180)

            file_localization.write("{} {} {} {}\n".format(t, x, y, theta * pi / 180))

            if halt.enter:
                break

        except KeyboardInterrupt:
            break

    # off motors and close file
    rear_motor.duty_cycle_sp = 0
    steer_motor.duty_cycle_sp = 0
    file_localization.close()

    ### 1) write raw data from sensors in base frame
    obstacles = mapping.the_map['obstacles']['front']
    file_map_f = open('logs/map_raw_front_sensor.txt', 'w')
    for p in obstacles:
        file_map_f.write("{} {}\n".format(p.x, p.y))
    file_map_f.close()

    obstacles = mapping.the_map['obstacles']['rear']
    file_map_r = open('logs/map_raw_rear_sensor.txt', 'w')
    for p in obstacles:
        file_map_r.write("{} {}\n".format(p.x, p.y))
    file_map_r.close()

    ### 2) filtring map and write filtred data from sensors in base frame
    mapping.filterMap()

    obstacles = mapping.the_map['obstacles']['front']
    file_map_f = open('logs/map_filt_front_sensor.txt', 'w')
    for p in obstacles:
        file_map_f.write("{} {}\n".format(p.x, p.y))
    file_map_f.close()

    obstacles = mapping.the_map['obstacles']['rear']
    file_map_r = open('logs/map_filt_rear_sensor.txt', 'w')
    for p in obstacles:
        file_map_r.write("{} {}\n".format(p.x, p.y))
    file_map_r.close()

    ### 3) finding free space for park our car and write point for it to file
    border_parking_points = mapping.findParkingPlace()

    if len(border_parking_points) > 0:
        file_parking_place = open('logs/map_parking_place.txt', 'w')
        for i in range(2):
            file_parking_place.write("{} {}".format(border_parking_points[i].x, border_parking_points[i].y))
        file_parking_place.close()

    # exit
    raise SystemExit
