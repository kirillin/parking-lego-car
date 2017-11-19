#!/usr/bin/env python3
from math import atan
from PID import PID


class VelocityController:

    def __init__(self, robot, v_desired, omega_desired):
        self.robot = robot
        self.v_desired = v_desired
        self.omega_desired = omega_desired
        self.pid_v = PID(2000.0, 5000.0, 0.0, 100, -100)
        self.pid_phi = PID(100.0, 0.0, 0.0, 100, -100)

    def getControls(self, rear_motor_speed, left_wheel_angle, dt):

        # forward motion control
        v_current = rear_motor_speed * self.robot.R
        error_v = self.v_desired - v_current
        u_v = self.pid_v.getControl(error_v, dt)

        # steer control
        if self.omega_desired > self.v_desired / self.robot.L * self.robot.MAX_TAN_PHI:
            print("Desired omega isn't achievable!!!")
            self.omega_desired = v_current / self.robot.L * self.robot.MAX_TAN_PHI
        phi_1_current = left_wheel_angle
        if phi_1_current > 180:
            phi_1_current = phi_1_current - 360
        phi_1_current = phi_1_current * 3.1416 / 180.0
        tan_phi_desired = self.robot.L * self.omega_desired / (v_current+0.01)
        phi_1_desired = atan(self.robot.L * tan_phi_desired / (self.robot.L + self.robot.D/2 * tan_phi_desired))
        error_phi = phi_1_desired - phi_1_current

        u_phi = self.pid_phi.getControl(error_phi, dt)

        return u_v, u_phi

    def setTargetVelocities(self, v_desired, omega_desired):
        self.v_desired = targer_v
        self.omega_desired = omega_desired
