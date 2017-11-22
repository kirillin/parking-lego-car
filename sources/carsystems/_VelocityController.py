#!/usr/bin/env python3
from math import atan, pi
from PID import PID


class VelocityController:

    def __init__(self, robot, v_desired, omega_desired):
        self.robot = robot
        self.v_desired = v_desired
        self.omega_desired = omega_desired
        self.pid_v = PID(300.0, 0.0, 0.0, 100, -100)
        self.pid_phi = PID(25.0, 0.0, 0.0, 100, -100)
        self.pid_omega = PID(20.0, 0.0, 0.0, 65*pi/180, -65*pi/180)

    def getControls(self, rear_motor_speed, phi, omega, dt):
        # forward motion control
        v_current = rear_motor_speed * self.robot.R
        error_v = self.v_desired - v_current
        u_v = self.pid_v.getControl(error_v, dt)

        # steer control
        if self.omega_desired > self.v_desired / self.robot.L * self.robot.MAX_TAN_PHI:
            print("Desired omega isn't achievable!!!")
            self.omega_desired = v_current / self.robot.L * self.robot.MAX_TAN_PHI

        error_omega = self.omega_desired - omega
        phi_desired = self.pid_omega.getControl(error_omega, dt)

        # print("Omega: {:.4f}\tDesired: {:.4f}\tPhi:{:.4f}\tDesired: {:.4f}".format(omega, self.omega_desired, phi, phi_desired))

        error_phi = phi_desired - phi
        u_phi = self.pid_phi.getControl(error_phi, dt)

        return u_v, u_phi

    def setTargetVelocities(self, v_desired, omega_desired):
        self.v_desired = v_desired
        self.omega_desired = omega_desired
