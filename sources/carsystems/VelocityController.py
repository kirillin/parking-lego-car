#!/usr/bin/env python3
from PID import PID
from Robots import Robot


class VelocityController:

    def __init__(self, v_desired, omega_desired):
        self.v_desired = v_desired
         self.omega_desired = omega_desired
        self.pid_v = PID(2000.0, 5000.0, 0.0, 100, -100)
        self.pid_phi = PID(100.0, 0.0, 0.0, 100, -100)


    def getControls(self, rear_motor_speed, left_wheel_angle, dt):

        # forward motion control
        v_current = rear_motor_speed * R
        error_v = self.v_desired - v_current
        u_v = pid_v.get_control(error_v, dt)

        # steer control
        if omega_desired > v_desired / L * MAX_TAN_PHI:
            print("Desired omega isn't achievable!!!")
            omega_desired = v_current / L * MAX_TAN_PHI
        phi_1_current = angle.get_angle()
        if phi_1_current > 180:
            phi_1_current = phi_1_current - 360
        phi_1_current = phi_1_current * 3.1416 / 180.0
        tan_phi_desired = L * omega_desired / (v_current+0.01)
        phi_1_desired = math.atan(L * tan_phi_desired / (L + D/2 * tan_phi_desired))
        error_phi = phi_1_desired - phi_1_current

        u_phi = pid_phi.get_control(error_phi, dt)

        return u_v, u_phi


    def setTargetVelocities(self, v_desired, omega_desired):
        self.v_desired = targer_v
        self.omega_desired = omega_desired
