#!/usr/bin/env python3
from math import atan, pi
from PID import PID


class VelocityController:

    def __init__(self, robot, v_desired, omega_desired):
        self.robot = robot
        self.v_desired = v_desired
        self.omega_desired = omega_desired
        self.pid_v = PID(2000.0, 5000.0, 0.0, 100, -100)
        self.pid_omega = PID(100.0, 0.0, 0.0, 18*pi/180, -18*pi/180)
        self.pid_phi = PID(100.0, 500.0, 5.0, 100, -100)

    def getControls(self, rear_motor_speed, phi, omega, dt):
        # forward motion control
        v_current = rear_motor_speed * self.robot.R
        error_v = self.v_desired - v_current
        u_v = self.pid_v.getControl(error_v, dt)

        if 0 <= v_current < 0.01:
            v_current = 0.01
        elif -0.01 < v_current < 0:
            v_current = -0.01
        phi_desired = atan(self.robot.L * self.omega_desired / v_current)
        if phi_desired > self.robot.MAX_PHI:
            phi_desired = self.robot.MAX_PHI
        elif phi_desired < -self.robot.MAX_PHI:
            phi_desired = -self.robot.MAX_PHI
        error_phi = phi_desired - phi
        u_phi = self.pid_phi.getControl(error_phi, dt)

        return u_v, u_phi

    def setTargetVelocities(self, v_desired, omega_desired):
        self.v_desired = v_desired
        self.omega_desired = omega_desired
