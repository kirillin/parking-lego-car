#!/usr/bin/env python3
from ev3dev.auto import *

import os
import time
from math import pi

from PID import PID

if __name__ == "__main__":

    # initialization of devices
    halt = Button()
    steer_motor = LargeMotor(OUTPUT_B)
    steer_motor.reset()

    pid_phi = PID(100.0, 500.0, 5.0, 100, -100)
    file_for_angles = open('angles.txt', 'w')   # time, x, t, theta
    # 20 max
    start_time = time.time()
    t = time.time() - start_time
    phi_desired = 10
    while t < 10.0:
        try:
            last_time = t
            t = time.time() - start_time
            dt = t - last_time  # invalid within first iteration but it's ok

            phi_current = steer_motor.position

            error_phi = (phi_desired - phi_current) * pi / 180

            # moving forward
            u_phi = pid_phi.getControl(error_phi, dt)
            print("{} {} {}\n".format(phi_current, error_phi, u_phi))
            # steer_motor.run_direct(duty_cycle_sp=u_phi)

            file_for_angles.write("{} {}\n".format(t, phi_current))

            if halt.enter:
                break

        except KeyboardInterrupt:
            break

    # off motors and close file
    steer_motor.duty_cycle_sp = 0
    file_for_angles.close()
    raise SystemExit
