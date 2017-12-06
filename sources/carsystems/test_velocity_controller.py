#!/usr/bin/env python3
import os, time
from math import pi

from ev3dev.ev3 import Button, GyroSensor, UltrasonicSensor, LargeMotor, OUTPUT_A, OUTPUT_B

from libs.Robots import LegoCar
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

    velocity_controller = VelocityController(LegoCar, 0, 0)

    os.system("beep -f 440 -l 50")

    start_time = time.time()
    t = time.time() - start_time
    while True:
        try:
            last_time = t
            t = time.time() - start_time
            dt = t - last_time  # invalid within first iteration but it's ok

            if t <= 3:
                # standing on the same place
                velocity_controller.setTargetVelocities(0, 0)
            elif 3 < t <= 6:
                # moving forward
                velocity_controller.setTargetVelocities(0.3, 0)
            elif 6 < t <= 9:
                # turn forward left
                velocity_controller.setTargetVelocities(0.3, 0.4)
            elif 9 < t <= 12:
                # turn forward right
                velocity_controller.setTargetVelocities(0.3, -0.4)
            elif 12 < t <= 15:
                # TODO: WHY CHANGING SIGN FOR TURN (fwd and bwd)? :(
                # turn backward right
                velocity_controller.setTargetVelocities(-0.3, 0.4)
            elif 15 < t <= 18:
                # turn backward left
                velocity_controller.setTargetVelocities(-0.3, -0.4)
            elif 18 < t <= 21:
                # moving backward
                velocity_controller.setTargetVelocities(-0.3, 0)
            elif 21 < t <= 24:
                # standing on the same place
                velocity_controller.setTargetVelocities(0, 0)
            elif t > 24:
                break

            theta, omega = [-x for x in gyro_sensor.rate_and_angle]   # !!! returns ANGLE AND RATE :)

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
