#!/usr/bin/env python3
from math import sin, cos


class TrajectroryController:

    k_1 = 1.0
    k_2 = 5.0
    k_3 = 5.0

    def __init__(self, xi0):
        self.__xi = xi0
        self.error_file = open('error.txt', 'w')

    def __delete__(self):
        self.error_file.close()

    def getControls(self, trajectory_point, x_cur, y_cur, theta, v_ref, omega_ref, theta_ref, dt):
        x_r, dx_r, ddx_r, y_r, dy_r, ddy_r = trajectory_point.getPoint()

        # error calcuations
        x_e = cos(theta) * (x_r - x_cur) + sin(theta) * (y_r - y_cur)
        y_e = -sin(theta) * (x_r - x_cur) + cos(theta) * (y_r - y_cur)
        theta_e = theta_ref - theta
        print("{0} {1} {2}\n".format(x_e, y_e, theta))

        v_des = v_ref * cos(theta_e) + self.k_1 * x_e
        omega_des = omega_ref + v_ref * (self.k_2 * y_e + self.k_3 * sin(theta_e))

        return v_des, omega_des
