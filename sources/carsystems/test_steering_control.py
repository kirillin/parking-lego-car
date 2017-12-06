#!/usr/bin/env python3
"""
    Module for tests with steering control.
    'phi_desired' sets a desired angle for turn front wheels.
"""
import time
from math import pi

from ev3dev.ev3 import Button, GyroSensor, UltrasonicSensor, LargeMotor, OUTPUT_A, OUTPUT_B
from libs.PID import PID


if __name__ == "__main__":

    # initialization of devices
    halt = Button()
    rear_motor = LargeMotor(OUTPUT_A)
    steer_motor = LargeMotor(OUTPUT_B)
    rear_motor.reset()
    steer_motor.reset()

    pid_phi = PID(100.0, 500.0, 5.0, 100, -100)
    file_for_angles = open('angles.txt', 'w')   # time, x, t, theta
    # 20 max
    rear_motor.run_direct(duty_cycle_sp=70)
    start_time = time.time()
    t = time.time() - start_time

    phi_desired = 15

    while t < 6.0:
        try:
            last_time = t
            t = time.time() - start_time
            dt = t - last_time  # invalid within first iteration but it's ok
            if 4 > t > 2:
                phi_desired = -15
            elif t >= 4:
                phi_desired = 0

            phi_current = steer_motor.position
            error_phi = (phi_desired - phi_current) * pi / 180

            u_phi = pid_phi.getControl(error_phi, dt)
            steer_motor.run_direct(duty_cycle_sp=u_phi)

            print("{} {} {}\n".format(phi_current, error_phi, u_phi))
            file_for_angles.write("{} {}\n".format(t, phi_current))

            if halt.enter:
                break

        except KeyboardInterrupt:
            break


    steer_motor.duty_cycle_sp = 0
    rear_motor.duty_cycle_sp = 0

    file_for_angles.close()
    raise SystemExit
