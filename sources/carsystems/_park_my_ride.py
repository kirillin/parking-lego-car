#!/usr/bin/env python3
import time
from math import pi
from ev3dev.auto import *

from AngleSensor import AngleSensor
from _VelocityController import VelocityController
from Robots import LegoCar

if __name__ == "__main__":

    # initialization of devices
    halt = Button()
    angle_sensor = AngleSensor()
    rear_motor = LargeMotor(OUTPUT_A)
    steer_motor = MediumMotor(OUTPUT_B)
    rear_motor.reset()
    steer_motor.reset()

    # initialization of controllers
    velocity_controller = VelocityController(LegoCar, 0.2, 0.3)

    start_time = time.time()
    t = time.time() - start_time
    while True:
        try:
            last_time = t
            t = time.time() - start_time
            dt = t - last_time  # invalid within first iteration but it's ok

            u_v, u_phi = velocity_controller.getControls(rear_motor.speed * pi / 180.0,
                                                         angle_sensor.getAngle() * pi / 180, dt)

            rear_motor.run_direct(duty_cycle_sp=u_v)
            steer_motor.run_direct(duty_cycle_sp=-u_phi)

            if halt.enter:
                break

        except KeyboardInterrupt:
            break

    # off motors and close file
    rear_motor.duty_cycle_sp = 0
    steer_motor.duty_cycle_sp = 0
    raise SystemExit
