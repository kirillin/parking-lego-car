#!/usr/bin/env python3
from ev3dev.auto import *

import os
import time
from math import pi

from Robots import LegoCar
from VelocityController import VelocityController
from Localization import Localization
from Mapping import Mapping

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
    velocity_controller = VelocityController(LegoCar, 0.3, 0)

    # initialization of localization
    localization = Localization(LegoCar)
    mapping = Mapping(LegoCar, sensor_us_front, sensor_us_rear)

    os.system("beep -f 440 -l 42")
    file_localization = open('localization.txt', 'w')   # time, x, t, theta

    start_time = time.time()
    t = time.time() - start_time
    while True:
        try:
            last_time = t
            t = time.time() - start_time
            dt = t - last_time  # invalid within first iteration but it's ok

            theta, omega = [-x for x in gyro_sensor.rate_and_angle]   # !!! returns ANGLE AND RATE :)

            # moving forward
            # u_v, u_phi = velocity_controller.getControls(rear_motor.speed * pi / 180.0,
            #                                             steer_motor.position * pi / 180,
            #                                             omega * pi / 180, dt)
            rear_motor.run_direct(duty_cycle_sp=50)
            # steer_motor.run_direct(duty_cycle_sp=u_phi)

            # mapping
            x, y, vx, vy = localization.getData(theta * pi / 180, rear_motor.speed * pi / 180, dt)
            mapping.updateMap(x, y, theta * pi / 180)

            file_localization.write("{} {} {} {}\n".format(t, x, y, theta * pi / 180))

            if halt.enter:
                break

        except KeyboardInterrupt:
            break

    file_localization.close()

    rear_motor.duty_cycle_sp = 0
    steer_motor.duty_cycle_sp = 0

    mapping.filterMap()
    border_parking_points = mapping.findParkingPlace()



    print("Border parking points {}".format(border_parking_points))
    file_parking_place = open('map_parking_place.txt', 'w')
    for i in range(2):
        file_parking_place.write("{} {}".format(border_parking_points[i].x, border_parking_points[i].y))
    file_parking_place.close()

    obstacles = mapping.the_map['obstacles']['front']
    file_map_f = open('map_front.txt', 'w')
    print('hi')
    for p in obstacles:
        print(p.x)
        file_map_f.write("{} {}\n".format(p.x, p.y))

    file_map_f.close()

    obstacles = mapping.the_map['obstacles']['rear']
    file_map_r = open('map_rear.txt', 'w')

    for p in obstacles:
        file_map_r.write("{} {}\n".format(p.x, p.y))

    file_map_r.close()

    raise SystemExit
